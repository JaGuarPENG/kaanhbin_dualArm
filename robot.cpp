#include "robot.hpp"
#include "gravcomp.hpp"
#include <array>


using namespace std;

namespace robot
{

    //example
    struct ModelSetPos::Imp {
		//Switch Model
		int m_;

		double arm1_p_vector[9]{ 0 };
		double arm1_l_vector[9]{ 0 };
		double arm2_p_vector[9]{ 0 };
		double arm2_l_vector[9]{ 0 };

		double arm1_init_force[9]{ 0 };
		double arm2_init_force[9]{ 0 };

		bool init = false;
		bool contact_check = false;

		//Force Buffer
		std::array<double, 10> force_buffer[6] = {};
		int buffer_index[6]{ 0 };

	};
    auto ModelSetPos::prepareNrt()->void
        {
            for (auto& m : motorOptions()) m = aris::plan::Plan::CHECK_NONE |
                aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
        }
    auto ModelSetPos::executeRT()->int
        {
		//test for get force data
		imp_->m_ = int32Param("model");
		GravComp gc;

		//dual transform modelbase into multimodel
		auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
		//at(0) -> Arm1 -> white
		auto& arm1 = dualArm.subModels().at(0);
		//at(1) -> Arm2 -> blue
		auto& arm2 = dualArm.subModels().at(1);

		//transform to model
		auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
		auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);

		//End Effector
		auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
		auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));

 
		auto getForceData = [&](double* data_, int m_, bool init_)
		{

			int raw_force[6]{ 0 };

			for (std::size_t i = 0; i < 6; ++i)
			{
				if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
				{
					mout() << "Force Sensor Error" << std::endl;
				}


				data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

			}

			if (!init_)
			{
				if (m_ == 0)
				{
					mout() << "Compensate Init Force A1" << std::endl;
					mout() << "Init 1 : " << imp_->arm1_init_force[0] << '\t' << imp_->arm1_init_force[1] << '\t' << imp_->arm1_init_force[2] << '\t'
						<< imp_->arm1_init_force[3] << '\t' << imp_->arm1_init_force[4] << '\t' << imp_->arm1_init_force[5] << std::endl;
				}
				else if (m_ == 1)
				{
					mout() << "Compensate Init Force A2" << std::endl;
					mout() << "Init 2 : " << imp_->arm2_init_force[0] << '\t' << imp_->arm2_init_force[1] << '\t' << imp_->arm2_init_force[2] << '\t'
						<< imp_->arm2_init_force[3] << '\t' << imp_->arm2_init_force[4] << '\t' << imp_->arm2_init_force[5] << std::endl;
				}




			}
			else
			{
				if (m_ == 0)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

					}
				}
				else if (m_ == 1)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

					}
				}

			}
		};

		auto forceFilter = [&](double* actual_force_, double* filtered_force_)
		{
			for (int i = 0; i < 6; i++)
			{
				imp_->force_buffer[i][imp_->buffer_index[i]] = actual_force_[i];
				imp_->buffer_index[i] = (imp_->buffer_index[i] + 1) % 10;

				filtered_force_[i] = std::accumulate(imp_->force_buffer[i].begin(), imp_->force_buffer[i].end(), 0.0) / 10;
			}
		};

		if (!imp_->init && !imp_->contact_check)
		{
			getForceData(imp_->arm1_init_force, 0, imp_->init);
			getForceData(imp_->arm2_init_force, 1, imp_->init);

			double current_angle[12] = { 0 };

			for (int i = 0; i < 12; i++)
			{
				current_angle[i] = controller()->motorPool()[i].actualPos();
			}

			dualArm.setInputPos(current_angle);
			if (dualArm.forwardKinematics())
			{
				mout() << "Forward failed" << std::endl;
				return 0;
			}

			imp_->init = true;
			mout() << "Init" << std::endl;

		}
		else if (imp_->init && !imp_->contact_check)
		{
			double raw_force_checker[12]{ 0 };
			double comp_force_checker[12]{ 0 };
			double force_checker[12]{ 0 };

			double a1_pm[16]{ 0 };
			double a2_pm[16]{ 0 };

			eeA1.getMpm(a1_pm);
			eeA2.getMpm(a2_pm);


			//Arm1
			getForceData(raw_force_checker, 0, imp_->init);
			gc.getCompFT(a1_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force_checker);
			//Arm2
			getForceData(raw_force_checker + 6, 1, imp_->init);
			gc.getCompFT(a2_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force_checker + 6);


			for (int i = 0; i < 12; i++)
			{
				force_checker[i] = comp_force_checker[i] + raw_force_checker[i];

				if (abs(force_checker[i]) > 3)
				{
					imp_->contact_check = true;
					mout() << "Contact Check" << std::endl;
					break;
				}
			}
		}
		else
		{
			double current_pm[16]{ 0 };


			if (imp_->m_ == 0)
			{

				eeA1.getMpm(current_pm);

				double raw_force[6]{ 0 };
				double arm1_compf[6]{ 0 };
				double arm1_actual_force[6]{ 0 };
				double arm1_filtered_force[6]{ 0 };


				getForceData(raw_force, 0, imp_->init);
				gc.getCompFT(current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, arm1_compf);

				for (int i = 0; i < 6; i++)
				{
					arm1_actual_force[i] = arm1_compf[i] + raw_force[i];
				}
				
				forceFilter(arm1_actual_force, arm1_filtered_force);

				if (count() % 100 == 0)
				{
					mout() << "A1_Force: " << arm1_filtered_force[0] << '\t' << arm1_filtered_force[1] << '\t' << arm1_filtered_force[2]
						<< '\t' << arm1_filtered_force[3] << '\t' << arm1_filtered_force[4] << '\t' << arm1_filtered_force[5] << std::endl;
				}


			}
			else if (imp_->m_ == 1)
			{

				eeA2.getMpm(current_pm);

				double raw_force[6]{ 0 };
				double arm2_compf[6]{ 0 };
				double arm2_actual_force[6]{ 0 };
				double arm2_filtered_force[6]{ 0 };


				getForceData(raw_force, 1, imp_->init);
				gc.getCompFT(current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, arm2_compf);

				for (int i = 0; i < 6; i++)
				{
					arm2_actual_force[i] = arm2_compf[i] + raw_force[i];
				}

				forceFilter(arm2_actual_force, arm2_filtered_force);

				if (count() % 100 == 0)
				{
					mout() << "A2_Force: " << arm2_filtered_force[0] << '\t' << arm2_filtered_force[1] << '\t' << arm2_filtered_force[2]
						<< '\t' << arm2_filtered_force[3] << '\t' << arm2_filtered_force[4] << '\t' << arm2_filtered_force[5] << std::endl;
				}
			}
			else
			{
				mout() << "Model Error" << std::endl;
				return 0;
			}
		}


		return 30000 - count();
       

        return 0;

        }
    ModelSetPos::ModelSetPos(const std::string& name)
        {
            aris::core::fromXmlString(command(),
            "<Command name=\"m_set\">"
            "	<GroupParam>"
            "	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
            "	</GroupParam>"
            "</Command>");
        }
    ModelSetPos::~ModelSetPos() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(ModelSetPos)


	
    auto ModelInit::prepareNrt()->void {

        for (auto& m : motorOptions()) m =
            aris::plan::Plan::CHECK_NONE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;


    }
    auto ModelInit::executeRT()->int {


        double eePos[12] = { 0 };

        static double move = 0.0001;
        static double tolerance = 0.00009;

        static double init_pos[12] =
        { 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0,
        0, 0, -5 * PI / 6, PI / 3, PI / 2, 0 };


        modelBase()->setInputPos(init_pos);

        if (modelBase()->forwardDynamics())
        {
            throw std::runtime_error("Forward Kinematics Position Failed!");
        }


        double current_angle[12] = { 0 };

        for (int i = 0; i < 12; i++)
        {
            current_angle[i] = controller()->motorPool()[i].targetPos();
        }


        auto motorsPositionCheck = [=]()
        {
            for(int i = 0; i < 12; i++)
            {
                if(std::fabs(current_angle[i]-init_pos[i])>=move)
                {
                    return false;
                }
            }

            return true;
        };



        for (int i = 0; i < 12; i++)
        {
            if (current_angle[i] <= init_pos[i] - move)
            {
                controller()->motorPool()[i].setTargetPos(current_angle[i] + move);
            }
            else if (current_angle[i] >= init_pos[i] + move)
            {
                controller()->motorPool()[i].setTargetPos(current_angle[i] - move);
            }
        }





        if (count() % 1000 == 0)
        {
            mout()<<current_angle[0]<< "\t"<<current_angle[1]<< "\t"<<current_angle[2]<< "\t"<<current_angle[3]<< "\t"<<current_angle[4]<< "\t"
            <<current_angle[5]<< "\t"<<current_angle[6]<< "\t"<<current_angle[7]<< "\t"<<current_angle[8]<< "\t"<<current_angle[9]<< "\t"
            <<current_angle[10]<< "\t"<<current_angle[11]<<std::endl;
        }


        if (motorsPositionCheck())
        {

            mout()<<"Back to Init Position"<<std::endl;
            modelBase()->setInputPos(current_angle);
            if (modelBase()->forwardKinematics())std::cout << "forward failed" << std::endl;

            mout()<<"current angle: \n"<<current_angle[0]<< "\t"<<current_angle[1]<< "\t"<<current_angle[2]<< "\t"<<current_angle[3]<< "\t"<<current_angle[4]<< "\t"
            <<current_angle[5]<< "\t"<<current_angle[6]<< "\t"<<current_angle[7]<< "\t"<<current_angle[8]<< "\t"<<current_angle[9]<< "\t"
            <<current_angle[10]<< "\t"<<current_angle[11]<<std::endl;

            modelBase()->getOutputPos(eePos);
            mout() << "current end position: \n" <<eePos[0]<< "\t"<<eePos[1]<< "\t"<<eePos[2]<< "\t"<<eePos[3]<< "\t"<<eePos[4]<< "\t"<<eePos[5]<< "\t"
            <<eePos[6]<< "\t"<<eePos[7]<< "\t"<<eePos[8]<< "\t"<<eePos[9]<< "\t"<<eePos[10]<< "\t"<<eePos[11]<<std::endl;


            return 0;
        }
        else
        {
            if(count()==28000)
            {
                mout()<<"Over Time"<<std::endl;
            }

            return 28000 - count();
        }
    }
    ModelInit::ModelInit(const std::string& name)
    {
        aris::core::fromXmlString(command(),
            "<Command name=\"m_init\"/>");
    }
    ModelInit::~ModelInit() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(ModelInit)


    struct Arm2Init::Imp {

        //Flag
        bool init = false;

        //Arm1
        double arm1_init_force[6]{ 0 };
        double arm1_p_vector[6]{ 0 };
        double arm1_l_vector[6]{ 0 };

        //Arm2
        double arm2_init_force[6]{ 0 };
        double arm2_p_vector[6]{ 0 };
        double arm2_l_vector[6]{ 0 };

    };
    auto Arm2Init::prepareNrt()->void {

        for (auto& m : motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;


    }
    auto Arm2Init::executeRT()->int {

        //dual transform modelbase into multimodel
        auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
        //at(0) -> Arm1 -> white
        auto& arm1 = dualArm.subModels().at(0);
        //at(1) -> Arm2 -> blue
        auto& arm2 = dualArm.subModels().at(1);

        //transform to model
        auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
        auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);

        //End Effector
        auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
        auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));


        GravComp gc;

        static double d_pos = 0.00001;
        static double tolerance = 0.00005;

        double current_angle[12]{ 0 };
        double current_sa_angle[6]{ 0 };

        double comp_force[6]{ 0 };
        double current_pm[16]{ 0 };
        double current_pos[6]{ 0 };
        double actual_force[6]{ 0 };
        double transform_force[6]{ 0 };



        auto getForceData = [&](double* data_, int m_, bool init_)
        {

            int raw_force[6]{ 0 };

            for (std::size_t i = 0; i < 6; ++i)
            {
                if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
                    mout() << "error" << std::endl;

                data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

            }

            if (!init_)
            {
                mout() << "Compensate Init Force" << std::endl;
            }
            else
            {
                if (m_ == 0)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

                    }
                }
                else if (m_ == 1)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

                    }
                }
                else
                {
                    mout() << "Wrong Model" << std::endl;
                }

            }

        };

        auto saMove = [&](double* pos_, aris::dynamic::Model& model_, int type_) {

            model_.setOutputPos(pos_);

            if (model_.inverseKinematics())
            {
                throw std::runtime_error("Inverse Kinematics Position Failed!");
            }


            double x_joint[6]{ 0 };

            model_.getInputPos(x_joint);

            if (type_ == 0)
            {
                for (std::size_t i = 0; i < 6; ++i)
                {
                    controller()->motorPool()[i].setTargetPos(x_joint[i]);
                }
            }
            else if (type_ == 1)
            {
                for (std::size_t i = 0; i < 6; ++i)
                {
                    controller()->motorPool()[i + 6].setTargetPos(x_joint[i]);
                }
            }
            else
            {
                throw std::runtime_error("Arm Type Error");
            }
        };

        auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
        {
            for (int i = 0; i < dim_; i++)
            {
                if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
                {
                    return false;
                }
            }

            return true;
        };

        //single arm move 1-->white 2-->blue
        auto saJointMove = [&](double target_mp_[6], int m_)
        {
            double current_angle[12] = { 0 };
            double move = 0.00005;

            for (int i = 0; i < 12; i++)
            {
                current_angle[i] = controller()->motorPool()[i].targetPos();
            }

            for (int i = 0; i < 6; i++)
            {
                if (current_angle[i+6*m_] <= target_mp_[i] - move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] + move);
                }
                else if (current_angle[i+6*m_] >= target_mp_[i] + move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] - move);
                }
            }
        };


        for (int i = 0; i < 12; i++)
        {
            current_angle[i] = controller()->motorPool()[i].actualPos();
        }


        std::copy(current_angle + 6, current_angle + 12, current_sa_angle);





        double assem_pos[6]{ -0.726, 0.009930, 0.289672, PI / 4, -PI / 2, - PI / 4 };
        double init_angle[6]{0};

        model_a2.setOutputPos(assem_pos);
        if(model_a2.inverseKinematics())
        {
            mout()<<"Error"<<std::endl;
        }

        model_a2.getInputPos(init_angle);

        if(count()%1000 == 0)
        {
            mout()<<"current angle: "<<current_sa_angle[0]<<'\t'<<current_sa_angle[1]<<'\t'<<current_sa_angle[2]<<'\t'
                    <<current_sa_angle[3]<<'\t'<<current_sa_angle[4]<<'\t'<<current_sa_angle[5]<<std::endl;
        }

        saJointMove(init_angle, 1);

        if(motorsPositionCheck(current_sa_angle, init_angle, 6))
        {
            getForceData(imp_->arm1_init_force, 0, imp_->init);
            getForceData(imp_->arm2_init_force, 1, imp_->init);

            gc.saveInitForce(imp_->arm1_init_force, imp_->arm2_init_force);

            mout()<<"Init Complete"<<std::endl;

            imp_->init = true;
            return 0;
        }



        if(count() == 20000)
        {
            mout()<<"Over Time"<<std::endl;
        }

        return 20000 - count();




    }
    Arm2Init::Arm2Init(const std::string& name)
    {
        aris::core::fromXmlString(command(),
            "<Command name=\"2_back\"/>");
    }
    Arm2Init::~Arm2Init() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(Arm2Init)


    struct ModelGet::Imp {
        //Switch Model
        int m_;

        double arm1_p_vector[9]{0};
        double arm1_l_vector[9]{0};
        double arm2_p_vector[9]{0};
        double arm2_l_vector[9]{0};

        double arm1_init_force[9]{0};
        double arm2_init_force[9]{0};

        bool init = false;

		//Force Buffer
		std::array<double, 10> force_buffer[6] = {};
		int buffer_index[6]{ 0 };

    };
	auto ModelGet::prepareNrt() -> void
	{
        for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE;
        GravComp gc;
        gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
        gc.loadInitForce(imp_->arm1_init_force, imp_->arm2_init_force);
        imp_->init = true;
	}
	auto ModelGet::executeRT() -> int
	{
		//test for get force data
        imp_->m_ = int32Param("model");
        GravComp gc;

        //dual transform modelbase into multimodel
        auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
        //at(0) -> Arm1 -> white
        auto& arm1 = dualArm.subModels().at(0);
        //at(1) -> Arm2 -> blue
        auto& arm2 = dualArm.subModels().at(1);

        //transform to model
        auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
        auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);

        //End Effector
        auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
        auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));


        auto getForceData = [&](double* data_, int m_, bool init_)
        {

            int raw_force[6]{ 0 };

            for (std::size_t i = 0; i < 6; ++i)
            {
                if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
                {
                    mout() << "Force Sensor Error" << std::endl;
                }


                data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

            }

            if (!init_)
            {
                if(m_ == 0)
                {
                     mout() << "Compensate Init Force A1" << std::endl;
                     mout()<<"Init 1 : "<<imp_->arm1_init_force[0]<<'\t'<<imp_->arm1_init_force[1]<<'\t'<<imp_->arm1_init_force[2]<<'\t'
                             <<imp_->arm1_init_force[3]<<'\t'<<imp_->arm1_init_force[4]<<'\t'<<imp_->arm1_init_force[5]<<std::endl;
                }
                else if (m_ == 1)
                {
                    mout() << "Compensate Init Force A2" << std::endl;
                    mout()<<"Init 2 : "<<imp_->arm2_init_force[0]<<'\t'<<imp_->arm2_init_force[1]<<'\t'<<imp_->arm2_init_force[2]<<'\t'
                            <<imp_->arm2_init_force[3]<<'\t'<<imp_->arm2_init_force[4]<<'\t'<<imp_->arm2_init_force[5]<<std::endl;
                }




            }
            else
            {
                if (m_ == 0)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

                    }
                }
                else if (m_ == 1)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

                    }
                }

            }
        };

		auto forceFilter = [&](double* actual_force_, double* filtered_force_)
		{
			for (int i = 0; i < 6; i++)
			{
				imp_->force_buffer[i][imp_->buffer_index[i]] = actual_force_[i];
				imp_->buffer_index[i] = (imp_->buffer_index[i] + 1) % 10;

				filtered_force_[i] = std::accumulate(imp_->force_buffer[i].begin(), imp_->force_buffer[i].end(), 0.0) / 10;
			}
		};

		auto forceDeadZone = [&](double* actual_force_, double* area_)
		{
			for (int i = 0; i < 6; i++)
			{
				if (abs(actual_force_[i]) < area_[i])
				{
					actual_force_[i] = 0;
				}
			}
		};

		auto forceTransform = [&](double* actual_force_, double* transform_force_, int m_)
		{
			if (m_ == 0)
			{
                transform_force_[0] = actual_force_[2];
                transform_force_[1] = -actual_force_[1];
                transform_force_[2] = actual_force_[0];

                transform_force_[3] = actual_force_[5];
                transform_force_[4] = -actual_force_[4];
                transform_force_[5] = actual_force_[3];
			}
			else if (m_ == 1)
			{
                transform_force_[0] = -actual_force_[2];
                transform_force_[1] = actual_force_[1];
                transform_force_[2] = actual_force_[0];

                transform_force_[3] = -actual_force_[5];
                transform_force_[4] = actual_force_[4];
                transform_force_[5] = actual_force_[3];
			}
			else
			{
				mout() << "Error Model In Force Transform" << std::endl;
			}
		};

      
        double current_pm[16]{0};


        if(imp_->m_ == 0)
        {

            eeA1.getMpm(current_pm);

            double raw_force[6]{0};
            double arm1_compf[6]{0};
            double arm1_actual_force[6]{0};
            double filtered_force[6]{ 0 };
            double transform_force[6]{ 0 };

            getForceData(raw_force, 0, imp_->init);
            gc.getCompFT(current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, arm1_compf);

            for(int i = 0; i<6; i++)
            {
                arm1_actual_force[i] = arm1_compf[i] + raw_force[i];
            }

            forceFilter(arm1_actual_force, filtered_force);
            forceTransform(filtered_force, transform_force, imp_->m_);

            if(count()%100 == 0)
            {
                mout()<<"A1_Force: "<< transform_force[0]<<'\t'<< transform_force[1]<<'\t'<< transform_force[2]
                     <<'\t'<< transform_force[3]<<'\t'<< transform_force[4]<<'\t'<< transform_force[5]<<std::endl;
            }


        }
        else if(imp_->m_ == 1)
        {

            eeA2.getMpm(current_pm);

            double raw_force[6]{0};
            double arm2_compf[6]{0};
            double arm2_actual_force[6]{0};
            double filtered_force[6]{ 0 };
            double transform_force[6]{ 0 };

            getForceData(raw_force, 1, imp_->init);
            gc.getCompFT(current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, arm2_compf);



            for(int i = 0; i<6; i++)
            {
                arm2_actual_force[i] = arm2_compf[i] + raw_force[i];
            }

            forceFilter(arm2_actual_force, filtered_force);
            forceTransform(filtered_force, transform_force, imp_->m_);
            if(count()%100 == 0)
            {
                mout()<<"A2_Force: "<< transform_force[0]<<'\t'<< transform_force[1]<<'\t'<< transform_force[2]
                     <<'\t'<< transform_force[3]<<'\t'<< transform_force[4]<<'\t'<< transform_force[5]<<std::endl;
            }
        }
        else if (imp_->m_ == 2)
        {
            double a1_comp_force[6]{ 0 };
            double a1_current_pm[16]{ 0 };
            double a1_current_force[6]{ 0 };
            double a1_actual_force[6]{ 0 };
            double a1_transform_force[6]{ 0 };

            double a2_comp_force[6]{ 0 };
            double a2_current_pm[16]{ 0 };
            double a2_current_force[6]{ 0 };
            double a2_actual_force[6]{ 0 };
            double a2_transform_force[6]{ 0 };


            GravComp gc;

            getForceData(a1_current_force, 0, imp_->init);
            eeA1.getMpm(a1_current_pm);
            gc.getCompFT(a1_current_pm,imp_->arm1_l_vector,imp_->arm1_p_vector, a1_comp_force);

            for(int i = 0; i<6; i++)
            {
                a1_actual_force[i] = a1_current_force[i] + a1_comp_force[i];
            }

            forceTransform(a1_actual_force, a1_transform_force, 0);

            getForceData(a2_current_force, 1, imp_->init);
            eeA2.getMpm(a2_current_pm);
            gc.getCompFT(a2_current_pm,imp_->arm2_l_vector,imp_->arm2_p_vector, a2_comp_force);

            for(int i = 0; i<6; i++)
            {
                a2_actual_force[i] = a2_current_force[i] + a2_comp_force[i];
            }

            forceTransform(a2_actual_force, a2_transform_force, 1);

            if(count() % 50 == 0)
            {
                mout()<<"A1_Force"<<"\t"<<a1_transform_force[0]<<"\t"<<a1_transform_force[1]<<"\t"<<a1_transform_force[2]<<"\t"
                        <<a1_transform_force[3]<<"\t"<<a1_transform_force[4]<<"\t"<<a1_transform_force[5]<<"\t"<<"\t"
                       <<"A2_Force"<<"\t"<<a2_transform_force[0]<<"\t"<<a2_transform_force[1]<<"\t"<<a2_transform_force[2]<<"\t"
                       <<a2_transform_force[3]<<"\t"<<a2_transform_force[4]<<"\t"<<a2_transform_force[5]<<std::endl;
            }

        }

        else
        {
            mout()<<"Model Error"<<std::endl;
            return 0;
        }


        return 1500-count();
	}
	ModelGet::ModelGet(const std::string& name)
	{
        aris::core::fromXmlString(command(),
        "<Command name=\"m_get\">"
        "	<GroupParam>"
        "	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
        "	</GroupParam>"
        "</Command>");
	}
	ModelGet::~ModelGet() = default;
	KAANH_DEFINE_BIG_FOUR_CPP(ModelGet)


	auto ModelForward::prepareNrt()->void
	{

		for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
	}
	auto ModelForward::executeRT()->int
	{
		TCurve2 s1(1.0, 3.0, 20.0);
		s1.getCurveParam();
		static double init_pos[12]{};
		double input_angle[12]{};
		double ee_pos[12]{};

		if (count() == 1)
		{

			double begin_angle[12]{ 0 };

			for (int i = 0; i < 12; i++)
			{
				begin_angle[i] = controller()->motorPool()[i].targetPos();
			}

			aris::dynamic::dsp(1, 12, begin_angle);

			//this->master()->logFileRawName("move");
			mout() << "read init angle" << std::endl;

			modelBase()->setInputPos(begin_angle);
			if (modelBase()->forwardKinematics())std::cout << "forward failed" << std::endl;
			mout() << "input" << std::endl;

			modelBase()->getOutputPos(init_pos);

			aris::dynamic::dsp(1, 12, init_pos);

		}



		ee_pos[0] = init_pos[0] + 0.01 * s1.getTCurve(count());
		ee_pos[1] = init_pos[1] - 0.01 * s1.getTCurve(count());

		ee_pos[2] = init_pos[2];
		ee_pos[3] = init_pos[3];
		ee_pos[4] = init_pos[4];
		ee_pos[5] = init_pos[5];

		ee_pos[6] = init_pos[6];
		ee_pos[7] = init_pos[7];
		ee_pos[8] = init_pos[8];
		ee_pos[9] = init_pos[9];
		ee_pos[10] = init_pos[10];
		ee_pos[11] = init_pos[11];


		modelBase()->setOutputPos(ee_pos);
		if (modelBase()->inverseKinematics())
		{
			throw std::runtime_error("Inverse Kinematics Position Failed!");
		}



		modelBase()->getInputPos(input_angle);

		for (int i = 0; i < 12; i++)
		{
			controller()->motorPool()[i].setTargetPos(input_angle[i]);
		}

		if (count() % 100 == 0)
		{
			mout() << "Arm1:" << input_angle[0] * 180 / PI << "\t" << input_angle[1] * 180 / PI << "\t" << input_angle[2] * 180 / PI << "\t"
				<< input_angle[3] * 180 / PI << "\t" << input_angle[4] * 180 / PI << "\t" << input_angle[5] * 180 / PI << "\n"
				<< "Arm2:" << input_angle[6] * 180 / PI << "\t" << input_angle[7] * 180 / PI << "\t" << input_angle[8] * 180 / PI
				<< input_angle[9] * 180 / PI << "\t" << input_angle[10] * 180 / PI << "\t" << input_angle[11] * 180 / PI << "\t" << count() << std::endl;
		}



		return 10000 - count();

	}
	ModelForward::ModelForward(const std::string& name)
	{

		aris::core::fromXmlString(command(),
			"<Command name=\"m_forward\">"
			"</Command>");
	}
	ModelForward::~ModelForward() = default;
	KAANH_DEFINE_BIG_FOUR_CPP(ModelForward)


    struct ModelTest::Imp {

        //Flag
        bool init = false;

        //Arm1
        double arm1_init_force[6]{ 0 };
        double arm1_p_vector[6]{ 0 };
        double arm1_l_vector[6]{ 0 };

        //Arm2
        double arm2_init_force[6]{ 0 };
        double arm2_p_vector[6]{ 0 };
        double arm2_l_vector[6]{ 0 };

        //Switch Model
        int m_;

    };
    auto ModelTest::prepareNrt()->void
	{

        for (auto& m : motorOptions()) m =
            aris::plan::Plan::CHECK_NONE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;


        GravComp gc;
        auto getForceData = [&](double* data_, int m_, bool init_)
        {

            int raw_force[6]{ 0 };

            for (std::size_t i = 0; i < 6; ++i)
            {
                if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
                    mout() << "error" << std::endl;

                data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

            }

            if (!init_)
            {
                mout() << "Compensate Init Force" << std::endl;
            }
            else
            {
                if (m_ == 0)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

                    }
                }
                else if (m_ == 1)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

                    }
                }
                else
                {
                    mout() << "Wrong Model" << std::endl;
                }

            }

        };


        gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
        mout() << "Load P & L Vector" << std::endl;

        getForceData(imp_->arm1_init_force, 0, imp_->init);
        getForceData(imp_->arm2_init_force, 1, imp_->init);

        imp_->init = true;

        mout()<<"Get Init Force!"<<std::endl;


	}
    auto ModelTest::executeRT()->int
	{

        //dual transform modelbase into multimodel
        auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
        //at(0) -> Arm1 -> white
        auto& arm1 = dualArm.subModels().at(0);
        //at(1) -> Arm2 -> blue
        auto& arm2 = dualArm.subModels().at(1);

        //transform to model
        auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
        auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);

        //End Effector
        auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
        auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));


        auto getForceData = [&](double* data_, int m_, bool init_)
        {

            int raw_force[6]{ 0 };

            for (std::size_t i = 0; i < 6; ++i)
            {
                if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
                    mout() << "error" << std::endl;

                data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

            }

            if (!init_)
            {
                mout() << "Compensate Init Force" << std::endl;
            }
            else
            {
                if (m_ == 0)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

                    }
                }
                else if (m_ == 1)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

                    }
                }
                else
                {
                    mout() << "Wrong Model" << std::endl;
                }

            }

        };

        auto forceTransform = [&](double* actual_force_, double* transform_force_, int m_)
        {
            if (m_ == 0)
            {
                transform_force_[0] = actual_force_[2];
                transform_force_[1] = -actual_force_[1];
                transform_force_[2] = actual_force_[0];

                transform_force_[3] = actual_force_[5];
                transform_force_[4] = -actual_force_[4];
                transform_force_[5] = actual_force_[3];
            }
            else if (m_ == 1)
            {
                transform_force_[0] = -actual_force_[2];
                transform_force_[1] = actual_force_[1];
                transform_force_[2] = actual_force_[0];

                transform_force_[3] = -actual_force_[5];
                transform_force_[4] = actual_force_[4];
                transform_force_[5] = actual_force_[3];
            }
            else
            {
                mout() << "Error Model In Force Transform" << std::endl;
            }
        };


        double a1_comp_force[6]{ 0 };
        double a1_current_pm[16]{ 0 };
        double a1_current_force[6]{ 0 };
        double a1_actual_force[6]{ 0 };
        double a1_transform_force[6]{ 0 };

        double a2_comp_force[6]{ 0 };
        double a2_current_pm[16]{ 0 };
        double a2_current_force[6]{ 0 };
        double a2_actual_force[6]{ 0 };
        double a2_transform_force[6]{ 0 };


        GravComp gc;

        getForceData(a1_current_force, 0, imp_->init);
        eeA1.getMpm(a1_current_pm);
        gc.getCompFT(a1_current_pm,imp_->arm1_l_vector,imp_->arm1_p_vector, a1_comp_force);

        for(int i = 0; i<6; i++)
        {
            a1_actual_force[i] = a1_current_force[i] + a1_comp_force[i];
        }

        forceTransform(a1_actual_force, a1_transform_force, 0);

        getForceData(a2_current_force, 1, imp_->init);
        eeA2.getMpm(a2_current_pm);
        gc.getCompFT(a2_current_pm,imp_->arm2_l_vector,imp_->arm2_p_vector, a2_comp_force);

        for(int i = 0; i<6; i++)
        {
            a2_actual_force[i] = a2_current_force[i] + a2_comp_force[i];
        }

        forceTransform(a2_actual_force, a2_transform_force, 1);

        if(count() % 50 == 0)
        {
            mout()<<"A1_Force"<<"\t"<<a1_transform_force[0]<<"\t"<<a1_transform_force[1]<<"\t"<<a1_transform_force[2]<<"\t"
                    <<a1_transform_force[3]<<"\t"<<a1_transform_force[4]<<"\t"<<a1_transform_force[5]<<"\t"<<"\t"
                   <<"A2_Force"<<"\t"<<a2_transform_force[0]<<"\t"<<a2_transform_force[1]<<"\t"<<a2_transform_force[2]<<"\t"
                   <<a2_transform_force[3]<<"\t"<<a2_transform_force[4]<<"\t"<<a2_transform_force[5]<<std::endl;
        }




        return 1500 - count();

	}
    ModelTest::ModelTest(const std::string& name)
	{

        aris::core::fromXmlString(command(),
            "<Command name=\"m_t\">"
            "	<GroupParam>"
            "	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
            "	</GroupParam>"
            "</Command>");
	}
    ModelTest::~ModelTest() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(ModelTest)



	struct ModelComP::Imp {
		bool target1_reached = false;
		bool target2_reached = false;
		bool target3_reached = false;
		bool target4_reached = false;

		bool init = false;

		bool stop_flag = false;
		int stop_count = 0;
		int stop_time = 2200;
		int current_stop_time = 0;

		int accumulation_count = 0;

		//temp data to stroage 10 times of actual force


		// For Arm 1
		double arm1_p_vector[6]{ 0 };
		double arm1_l_vector[6]{ 0 };

		double arm1_temp_force_1[6] = { 0 };
		double arm1_temp_force_2[6] = { 0 };
		double arm1_temp_force_3[6] = { 0 };

		double arm1_force_data_1[6] = { 0 };
		double arm1_force_data_2[6] = { 0 };
		double arm1_force_data_3[6] = { 0 };

		double arm1_ee_pm_1[16]{ 0 };
		double arm1_ee_pm_2[16]{ 0 };
		double arm1_ee_pm_3[16]{ 0 };

		double arm1_comp_f[6]{ 0 };
		double arm1_init_force[6]{ 0 };


		//For Arm 2
		double arm2_p_vector[6]{ 0 };
		double arm2_l_vector[6]{ 0 };

		double arm2_temp_force_1[6] = { 0 };
		double arm2_temp_force_2[6] = { 0 };
		double arm2_temp_force_3[6] = { 0 };

		double arm2_force_data_1[6] = { 0 };
		double arm2_force_data_2[6] = { 0 };
		double arm2_force_data_3[6] = { 0 };

		double arm2_ee_pm_1[16]{ 0 };
		double arm2_ee_pm_2[16]{ 0 };
		double arm2_ee_pm_3[16]{ 0 };

		double arm2_comp_f[6]{ 0 };
		double arm2_init_force[6]{ 0 };

	};
	auto ModelComP::prepareNrt()->void
	{

		for (auto& m : motorOptions()) m =
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

       std::cout<<"init"<<std::endl;



	}
	auto ModelComP::executeRT()->int
	{

        GravComp gc;
		static double tolerance = 0.0001;

        //dual transform modelbase into multimodel
        auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
        //at(0) -> Arm1 -> white
        auto& arm1 = dualArm.subModels().at(0);
        //at(1) -> Arm2 -> blue
        auto& arm2 = dualArm.subModels().at(1);

        //transform to model
        auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
        auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);

        //End Effector
        auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
        auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));


		// Only One Arm Move Each Command
        auto saJointMove = [&](double target_mp_[6], int m_)
        {
            double current_angle[12] = { 0 };
            double move = 0.00005;

            for (int i = 0; i < 12; i++)
            {
                current_angle[i] = controller()->motorPool()[i].targetPos();
            }

            for (int i = 0; i < 6; i++)
            {
                if (current_angle[i+6*m_] <= target_mp_[i] - move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] + move);
                }
                else if (current_angle[i+6*m_] >= target_mp_[i] + move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] - move);
                }
            }
        };


        auto daJointMove = [&](double target_mp_[12])
        {
            double current_angle[12] = { 0 };
            double move = 0.00008;

            for (int i = 0; i < 12; i++)
            {
                current_angle[i] = controller()->motorPool()[i].targetPos();
            }

            for (int i = 0; i < 12; i++)
            {
                if (current_angle[i] <= target_mp_[i] - move)
                {
                    controller()->motorPool()[i].setTargetPos(current_angle[i] + move);
                }
                else if (current_angle[i] >= target_mp_[i] + move)
                {
                    controller()->motorPool()[i].setTargetPos(current_angle[i] - move);
                }
            }
        };

		//Ethercat Warning
		auto getForceData = [&](double* data_, int m_, bool init_)
		{

			int raw_force[6]{ 0 };

            for (int i = 0; i < 6; ++i)
			{
                if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
				{
                    mout() << "Force Sensor Error" << std::endl;
				}
					

				data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

			}
			if (!init_)
			{
                if(m_ == 0)
                {
                     mout() << "Compensate Init Force A1" << std::endl;
                     mout()<<"Init 1 : "<<imp_->arm1_init_force[0]<<'\t'<<imp_->arm1_init_force[1]<<'\t'<<imp_->arm1_init_force[2]<<'\t'
                             <<imp_->arm1_init_force[3]<<'\t'<<imp_->arm1_init_force[4]<<'\t'<<imp_->arm1_init_force[5]<<std::endl;
                }
                else if (m_ == 1)
                {
                    mout() << "Compensate Init Force A2" << std::endl;
                    mout()<<"Init 2 : "<<imp_->arm2_init_force[0]<<'\t'<<imp_->arm2_init_force[1]<<'\t'<<imp_->arm2_init_force[2]<<'\t'
                            <<imp_->arm2_init_force[3]<<'\t'<<imp_->arm2_init_force[4]<<'\t'<<imp_->arm2_init_force[5]<<std::endl;
                }
			}
			else
            {
				if (m_ == 0)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

					}
				}
				else if (m_ == 1)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

					}
				}
                else
                {
                    mout()<<"Wrong Model"<<std::endl;
                }
			}
		};


		auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
		{
			for (int i = 0; i < dim_; i++)
			{
				if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
				{
					return false;
				}
			}

			return true;
		};

		auto caculateAvgForce = [=](double arm1_force_data_[6], double arm2_force_data_[6], double arm1_temp_force_[6], double arm2_temp_force_[6], int count_)
		{
			if (count() < imp_->current_stop_time + imp_->stop_time)
			{

				if (count() % 200 == 0 && imp_->accumulation_count < 10)
				{
					double temp1[6]{ 0 };
					double temp2[6]{ 0 };

					getForceData(temp1, 0, imp_->init);
					getForceData(temp2, 1, imp_->init);

					for (int i = 0; i < 6; i++)
					{
						arm1_temp_force_[i] += temp1[i];
						arm2_temp_force_[i] += temp2[i];
					}


					imp_->accumulation_count = imp_->accumulation_count + 1;
					mout() << imp_->accumulation_count << std::endl;
				}

			}
			else if (count() == imp_->current_stop_time + imp_->stop_time)
			{
				mout() << "stop! " << "count(): " << count() << std::endl;
				imp_->accumulation_count = 0;
				for (int i = 0; i < 6; i++)
				{
					arm1_force_data_[i] = arm1_temp_force_[i] / 10.0;
					arm2_force_data_[i] = arm2_temp_force_[i] / 10.0;
				}
				mout() << "Arm 1 Force Data " << count_ << '\n' << arm1_force_data_[0] << '\t' << arm1_force_data_[1] << '\t' << arm1_force_data_[2] << '\t'
					<< arm1_force_data_[3] << '\t' << arm1_force_data_[4] << '\t' << arm1_force_data_[5] << std::endl;
				mout() << "Arm 2 Force Data " << count_ << '\n' << arm2_force_data_[0] << '\t' << arm2_force_data_[1] << '\t' << arm2_force_data_[2] << '\t'
					<< arm2_force_data_[3] << '\t' << arm2_force_data_[4] << '\t' << arm2_force_data_[5] << std::endl;
			}
			else
			{
				mout() << "Flag Change " << count_ << std::endl;
				imp_->stop_flag = false;
			}
		};


		double current_angle[12] = { 0 };

		for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].actualPos();
		}


		if (imp_->stop_flag)
		{
			if (imp_->stop_count == 1)
			{

				caculateAvgForce(imp_->arm1_force_data_1, imp_->arm2_force_data_1, imp_->arm1_temp_force_1, imp_->arm2_temp_force_1, 1);


			}
			else if (imp_->stop_count == 2)
			{

				caculateAvgForce(imp_->arm1_force_data_2, imp_->arm2_force_data_2, imp_->arm1_temp_force_2, imp_->arm2_temp_force_2, 2);

			}
			else if (imp_->stop_count == 3)
			{

				caculateAvgForce(imp_->arm1_force_data_3, imp_->arm2_force_data_3, imp_->arm1_temp_force_3, imp_->arm2_temp_force_3, 3);

			}
			else
			{
				mout() << "Stop Count Wrong: " << imp_->stop_count << " stop flag: " << imp_->stop_flag << std::endl;
				return 0;
			}

			return 80000 - count();
		}
		else
		{
			static double init_angle[12] =
			{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0,
			0, 0, -5 * PI / 6, PI / 3, PI / 2, 0 };

			static double angle1[12] =
			{ 0, 0, 5 * PI / 6, -17 * PI / 18, -PI / 2, 0 ,
			0, 0, -5 * PI / 6, 17 * PI / 18, PI / 2, 0 };

			static double angle2[12] =
			{ 0, 0, 5 * PI / 6, -PI / 2, -PI / 3, 0 ,
			0, 0, -5 * PI / 6, PI / 2, PI / 3, 0 };

			static double angle3[12] =
			{ 0, 0, 5 * PI / 6, -2 * PI / 3, -2 * PI / 3, 0 ,
			0, 0, -5 * PI / 6, 2 * PI / 3, 2 * PI / 3, 0 };


			//// Arm 1 Angle
			//static double init_angle1[6]{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0 };

			//static double angle1_1[6]{ 0, 0, 5 * PI / 6, -17 * PI / 18, -PI / 2, 0 };

			//static double angle1_2[6]{ 0, 0, 5 * PI / 6, -PI / 2, -PI / 2, 0 };

			//static double angle1_3[6]{ 0, 0, 5 * PI / 6, -2 * PI / 3, -2 * PI / 3, 0 };




			//// Arm 2 Angle
			//static double init_angle2[6]{ 0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };

			//static double angle2_1[6]{ 0, 0, -5 * PI / 6, 17 * PI / 18, PI / 2, 0 };

			//static double angle2_2[6]{ 0, 0, -5 * PI / 6, PI / 2, PI / 3, 0 };

			//static double angle2_3[6]{ 0, 0, -5 * PI / 6, 2 * PI / 3, 2 * PI / 3, 0 };

			if (!imp_->init)
			{

                dualArm.setInputPos(init_angle);
                if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;

				daJointMove(init_angle);

				if (motorsPositionCheck(current_angle, init_angle, 12))
				{

                    getForceData(imp_->arm1_init_force, 0, imp_->init);

					getForceData(imp_->arm2_init_force, 1, imp_->init);
					mout() << "Init Complete" << std::endl;

					imp_->init = true;
				}

			}

			if (!imp_->target1_reached && imp_->init)
			{
                dualArm.setInputPos(angle1);
                if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;

				daJointMove(angle1);

				if (motorsPositionCheck(current_angle, angle1, 12))
				{
					mout() << "Target 1 Reached" << std::endl;

					eeA1.getMpm(imp_->arm1_ee_pm_1);
					eeA2.getMpm(imp_->arm2_ee_pm_1);

					imp_->target1_reached = true;
					imp_->stop_count = 1;
					imp_->current_stop_time = count();
					imp_->stop_flag = true;
					mout() << "current stop time: " << imp_->current_stop_time << std::endl;
				}

			}
			else if (imp_->target1_reached && !imp_->target2_reached)
			{
                dualArm.setInputPos(angle2);
                if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;

				daJointMove(angle2);

				if (motorsPositionCheck(current_angle, angle2, 12))
				{
					mout() << "Target 2 Reached" << std::endl;

					eeA1.getMpm(imp_->arm1_ee_pm_2);
					eeA2.getMpm(imp_->arm2_ee_pm_2);

					imp_->target2_reached = true;
					imp_->stop_count = 2;
					imp_->current_stop_time = count();
					imp_->stop_flag = true;
					mout() << "current stop time: " << imp_->current_stop_time << std::endl;
				}
			}
			else if (imp_->target2_reached && !imp_->target3_reached)
			{
                dualArm.setInputPos(angle3);
                if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;

				daJointMove(angle3);

				if (motorsPositionCheck(current_angle, angle3, 12))
				{
					mout() << "Target 3 Reached" << std::endl;

					eeA1.getMpm(imp_->arm1_ee_pm_3);
					eeA2.getMpm(imp_->arm2_ee_pm_3);

					imp_->target3_reached = true;
					imp_->stop_count = 3;
					imp_->current_stop_time = count();
					imp_->stop_flag = true;
					mout() << "current stop time: " << imp_->current_stop_time << std::endl;

				}
			}
			else if (imp_->target3_reached && !imp_->target4_reached)
			{
				// Back To Init
                dualArm.setInputPos(init_angle);
                if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;

				daJointMove(init_angle);

				if (motorsPositionCheck(current_angle, init_angle, 12))
				{
					mout() << "Back To Init Pos" << std::endl;
					imp_->target4_reached = true;

				}

			}
			else if (imp_->target1_reached && imp_->target2_reached && imp_->target3_reached && imp_->target4_reached)
			{
				//Arm 1
				double arm1_t_vector[9]{ 0 };
				double arm1_f_vector[9]{ 0 };

				double arm1_f_matrix[54]{ 0 };
				double arm1_r_matrix[54]{ 0 };


				double arm1_ee_rm_1[9]{ 0 };
				double arm1_ee_rm_2[9]{ 0 };
				double arm1_ee_rm_3[9]{ 0 };

				double arm1_current_force[6]{ 0 };

				aris::dynamic::s_pm2rm(imp_->arm1_ee_pm_1, arm1_ee_rm_1);
				aris::dynamic::s_pm2rm(imp_->arm1_ee_pm_2, arm1_ee_rm_2);
				aris::dynamic::s_pm2rm(imp_->arm1_ee_pm_3, arm1_ee_rm_3);



				gc.getTorqueVector(imp_->arm1_force_data_1, imp_->arm1_force_data_2, imp_->arm1_force_data_3, arm1_t_vector);
				gc.getForceVector(imp_->arm1_force_data_1, imp_->arm1_force_data_2, imp_->arm1_force_data_3, arm1_f_vector);

				gc.getFMatrix(imp_->arm1_force_data_1, imp_->arm1_force_data_2, imp_->arm1_force_data_3, arm1_f_matrix);
				gc.getRMatrix(arm1_ee_rm_1, arm1_ee_rm_2, arm1_ee_rm_3, arm1_r_matrix);

				gc.getPLMatrix(arm1_f_matrix, arm1_t_vector, imp_->arm1_p_vector);
				gc.getPLMatrix(arm1_r_matrix, arm1_f_vector, imp_->arm1_l_vector);

				double arm1_current_ee_pm[16]{ 0 };
                double arm1_compf[6]{0};


                eeA1.getMpm(arm1_current_ee_pm);



                gc.getCompFT(arm1_current_ee_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, arm1_compf);

				getForceData(arm1_current_force, 0, imp_->init);


				//Arm 2
				double arm2_t_vector[9]{ 0 };
				double arm2_f_vector[9]{ 0 };

				double arm2_f_matrix[54]{ 0 };
				double arm2_r_matrix[54]{ 0 };


				double arm2_ee_rm_1[9]{ 0 };
				double arm2_ee_rm_2[9]{ 0 };
				double arm2_ee_rm_3[9]{ 0 };

				double arm2_current_force[6]{ 0 };

				aris::dynamic::s_pm2rm(imp_->arm2_ee_pm_1, arm2_ee_rm_1);
				aris::dynamic::s_pm2rm(imp_->arm2_ee_pm_2, arm2_ee_rm_2);
				aris::dynamic::s_pm2rm(imp_->arm2_ee_pm_3, arm2_ee_rm_3);



				gc.getTorqueVector(imp_->arm2_force_data_1, imp_->arm2_force_data_2, imp_->arm2_force_data_3, arm2_t_vector);
				gc.getForceVector(imp_->arm2_force_data_1, imp_->arm2_force_data_2, imp_->arm2_force_data_3, arm2_f_vector);

				gc.getFMatrix(imp_->arm2_force_data_1, imp_->arm2_force_data_2, imp_->arm2_force_data_3, arm2_f_matrix);
				gc.getRMatrix(arm2_ee_rm_1, arm2_ee_rm_2, arm2_ee_rm_3, arm2_r_matrix);

				gc.getPLMatrix(arm2_f_matrix, arm2_t_vector, imp_->arm2_p_vector);
				gc.getPLMatrix(arm2_r_matrix, arm2_f_vector, imp_->arm2_l_vector);

				double arm2_current_ee_pm[16]{ 0 };
                double arm2_compf[6]{0};


				eeA2.getMpm(arm2_current_ee_pm);

                gc.getCompFT(arm2_current_ee_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, arm2_compf);

                getForceData(arm2_current_force, 1, imp_->init);

                gc.savePLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
                gc.saveInitForce(imp_->arm1_init_force, imp_->arm2_init_force);


                mout() << "Current Arm1 Force:" << '\n' << arm1_current_force[0] << '\t' << arm1_current_force[1] << '\t'
                    << arm1_current_force[2] << '\t' << arm1_current_force[3] << '\t'
                    << arm1_current_force[4] << '\t' << arm1_current_force[5] << std::endl;

                mout() << "Current Arm2 Force:" << '\n' << arm2_current_force[0]  << '\t' << arm2_current_force[1]  << '\t'
                    << arm2_current_force[2] << '\t' << arm2_current_force[3]  << '\t'
                    << arm2_current_force[4] << '\t' << arm2_current_force[5] << std::endl;



                mout() << "Current Arm1 Force After Compensation:" << '\n' << arm1_current_force[0] + arm1_compf[0] << '\t' << arm1_current_force[1] + arm1_compf[1] << '\t'
                    << arm1_current_force[2] + arm1_compf[2] << '\t' << arm1_current_force[3] + arm1_compf[3] << '\t'
                    << arm1_current_force[4] + arm1_compf[4] << '\t' << arm1_current_force[5] + arm1_compf[5] << std::endl;

                mout() << "Current Arm2 Force After Compensation:" << '\n' << arm2_current_force[0] + arm2_compf[0] << '\t' << arm2_current_force[1] + arm2_compf[1] << '\t'
                    << arm2_current_force[2] + arm2_compf[2] << '\t' << arm2_current_force[3] + arm2_compf[3] << '\t'
                    << arm2_current_force[4] + arm2_compf[4] << '\t' << arm2_current_force[5] + arm2_compf[5] << std::endl;




                return 0;

			}

			if (count() == 100000)
			{


				mout() << "Over Time" << std::endl;


			}

			return 100000 - count();

		}
	}
	ModelComP::ModelComP(const std::string& name) : imp_(new Imp)
	{

		aris::core::fromXmlString(command(),
             "<Command name=\"m_comp\"/>");
	}
	ModelComP::~ModelComP() = default;
	KAANH_DEFINE_BIG_FOUR_CPP(ModelComP)



	struct ForceAlign::Imp {

		bool init = false;
		bool contact_check = false;

		double comp_f[6]{ 0 };

		double arm1_init_force[6]{ 0 };
		double arm2_init_force[6]{ 0 };

		double arm1_p_vector[6]{ 0 };
		double arm1_l_vector[6]{ 0 };

		double arm2_p_vector[6]{ 0 };
		double arm2_l_vector[6]{ 0 };

		double x_d;

		double Ke = 220000;
		double K = 3;

		double B[6]{ 0.25,0.25,0.7,0.0,0.0,0.0 };
		double M[6]{ 0.1,0.1,0.1,0.1,0.1,0.1 };

		double desired_force = -5;
		int contact_count;

		//Switch Model
		int m_;

	};
	auto ForceAlign::prepareNrt()->void
	{

		for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

		GravComp gc;
		gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
		mout() << "Load P & L Vector" << std::endl;

	}
	auto ForceAlign::executeRT()->int
	{

		static double tolerance = 0.0001;
		static double init_angle[12] =
		{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0 ,
		0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };

        static double max_vel[6]{ 0.2,0.2,0.2,0.0005,0.0005,0.0001 };
        static double trigger_force[6]{ 0.5,0.5,0.5,0.001,0.001,0.001 };
        static double max_force[6]{ 10,10,10,5,5,5 };
        static double trigger_vel[6]{ 0.0001,0.0001,0.0001,0.0001,0.0001,0.0001 };

		imp_->m_ = int32Param("model");

		//dual transform modelbase into multimodel
		auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
		//at(0) -> Arm1 -> white
		auto& arm1 = dualArm.subModels().at(0);
		//at(1) -> Arm2 -> blue
		auto& arm2 = dualArm.subModels().at(1);

		//transform to model
		auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
		auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);

		//End Effector
		auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
		auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));


		GravComp gc;



		double current_vel[6]{ 0 };
		double current_pos[6]{ 0 };

		double current_force[6]{ 0 };
		double current_angle[12]{ 0 };
		double current_sa_angle[6]{ 0 };
		double current_pm[16]{ 0 };

		double comp_force[6]{ 0 };
		double actual_force[6]{ 0 };

		auto getForceData = [&](double* data_, int m_, bool init_)
		{

			int raw_force[6]{ 0 };

			for (std::size_t i = 0; i < 6; ++i)
			{
                if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
					mout() << "error" << std::endl;

				data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

			}

			if (!init_)
			{
				mout() << "Compensate Init Force" << std::endl;
			}
			else
			{
				if (m_ == 0)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

					}
				}
				else if (m_ == 1)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

					}
				}

			}

			if (m_ == 0)
			{
				//				data_[0] = -data_[0];
				//				data_[1] = -data_[1];

				//				data_[3] = -data_[3];
				//				data_[4] = -data_[4];
			}
			else if (m_ == 1)
			{
				data_[0] = -data_[0];
				data_[1] = -data_[1];

				data_[3] = -data_[3];
				data_[4] = -data_[4];

			}
			else
			{
				mout() << "Wrong Model" << std::endl;
			}


		};


        auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
        {
            for (int i = 0; i < dim_; i++)
            {
                if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
                {
                    return false;
                }
            }

            return true;
        };
		auto daJointMove = [&](double target_mp_[12])
		{
			double current_angle[12] = { 0 };
			double move = 0.00005;

			for (int i = 0; i < 12; i++)
			{
				current_angle[i] = controller()->motorPool()[i].targetPos();
			}

			for (int i = 0; i < 12; i++)
			{
				if (current_angle[i] <= target_mp_[i] - move)
				{
					controller()->motorPool()[i].setTargetPos(current_angle[i] + move);
				}
				else if (current_angle[i] >= target_mp_[i] + move)
				{
					controller()->motorPool()[i].setTargetPos(current_angle[i] - move);
				}
			}
		};

		//single arm move 1-->white 2-->blue
		auto saMove = [&](double* pos_, aris::dynamic::Model& model_, int type_) {

			model_.setOutputPos(pos_);

			if (model_.inverseKinematics())
			{
				throw std::runtime_error("Inverse Kinematics Position Failed!");
			}


			double x_joint[6]{ 0 };

			model_.getInputPos(x_joint);

			if (type_ == 0)
			{
				for (std::size_t i = 0; i < 6; ++i)
				{
					controller()->motorPool()[i].setTargetPos(x_joint[i]);
				}
			}
			else if (type_ == 1)
			{
				for (std::size_t i = 0; i < 6; ++i)
				{
					controller()->motorPool()[i + 6].setTargetPos(x_joint[i]);
				}
			}
			else
			{
				throw std::runtime_error("Arm Type Error");
			}
		};


		for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].actualPos();
		}



		if (!imp_->init)
		{
			dualArm.setInputPos(init_angle);
			if (dualArm.forwardKinematics())
			{
				mout() << "forward fail" << std::endl;
			}

			daJointMove(init_angle);
			if (motorsPositionCheck(current_angle, init_angle, 12))
			{
				getForceData(imp_->arm1_init_force, 0, imp_->init);
				getForceData(imp_->arm2_init_force, 1, imp_->init);
				mout() << "Back To Init" << std::endl;

				imp_->init = true;
			}
		}
		else
		{
			//Arm1
			if (imp_->m_ == 0)
			{
				//Update Statue
				eeA1.getV(current_vel);
				eeA1.getP(current_pos);
				eeA1.getMpm(current_pm);

				std::copy(current_angle, current_angle + 6, current_sa_angle);

				if (count() % 100 == 0)
				{
					mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
						<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
				}



				//Contact Check



				//Get Actual Force
				getForceData(current_force, imp_->m_, imp_->init);
				gc.getCompFT(current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force);
				for (int i = 0; i < 6; i++)
				{
					actual_force[i] = comp_force[i] + current_force[i];
				}

				//Dead Zone of Force
				for (int i = 0; i < 6; i++)
				{
					if (abs(actual_force[i]) < trigger_force[i])
					{
						actual_force[i] = 0;
					}
					if (actual_force[i] > max_force[i])
					{
						actual_force[i] = max_force[i];
					}
					if (actual_force[i] < -max_force[i])
					{
						actual_force[i] = -max_force[i];
					}

				}

				if (!imp_->contact_check)
				{
					if (abs(actual_force[2]) > 2)
					{
						imp_->contact_check = true;
						imp_->contact_count = count();
						// Set Disred Pos 
						imp_->x_d = current_pos[0];
						mout() << "Contacted! Disred X Pos: " << imp_->x_d << std::endl;
					}
					else
					{

						current_pos[0] -= 0.00003;
						saMove(current_pos, model_a1, 0);

					}

				}
				else if (imp_->contact_check)
				{
					double x_r = imp_->x_d - (imp_->desired_force / imp_->Ke);
					double a = (imp_->desired_force - actual_force[2] - imp_->B[2] * current_vel[0] - imp_->K * (current_pos[0] - x_r)) / imp_->M[2];
					double x = current_pos[0] + 0.5 * a * 0.001 * 0.001 + current_vel[0] * 0.001;

					current_pos[0] = x;
					saMove(current_pos, model_a1, 0);

					if (count() % 100 == 0)
					{
						//mout() << "count(): " << count() << std::endl;
						mout() << "force: " << actual_force[0] << '\t' << actual_force[1] << '\t' << actual_force[2] << '\t'
							<< actual_force[3] << '\t' << actual_force[4] << '\t' << actual_force[5] << std::endl;

					}

				}
			}
			//Arm2
			else if (imp_->m_ == 1)
			{
				//Update Statue
				eeA2.getV(current_vel);
				eeA2.getP(current_pos);
				eeA2.getMpm(current_pm);

				std::copy(current_angle + 6, current_angle + 12, current_sa_angle);


				if (count() % 100 == 0)
				{
					mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
						<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
				}


				//Contact Check

                //Get Actual Force
				getForceData(current_force, imp_->m_, imp_->init);
				gc.getCompFT(current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force);
				for (int i = 0; i < 6; i++)
				{
					actual_force[i] = comp_force[i] + current_force[i];
				}

				//Dead Zone of Force
				for (int i = 0; i < 6; i++)
				{
					if (abs(actual_force[i]) < trigger_force[i])
					{
						actual_force[i] = 0;
					}
					if (actual_force[i] > max_force[i])
					{
						actual_force[i] = max_force[i];
					}
					if (actual_force[i] < -max_force[i])
					{
						actual_force[i] = -max_force[i];
					}

				}

				if (!imp_->contact_check)
				{
					if (abs(actual_force[2]) > 2)
					{
						imp_->contact_check = true;
						imp_->contact_count = count();
						// Set Disred Pos 
						imp_->x_d = current_pos[0];
						mout() << "Contacted! Disred X Pos: " << imp_->x_d << std::endl;
					}
					else
					{

						current_pos[0] -= 0.00003;
						saMove(current_pos, model_a2, 1);

					}

				}
				else if (imp_->contact_check)
				{
					double x_r = imp_->x_d - (imp_->desired_force / imp_->Ke);
					double a = (imp_->desired_force - actual_force[2] - imp_->B[2] * current_vel[0] - imp_->K * (current_pos[0] - x_r)) / imp_->M[2];
					double x = current_pos[0] + 0.5 * a * 0.001 * 0.001 + current_vel[0] * 0.001;

					current_pos[0] = x;
					saMove(current_pos, model_a2, 1);

					if (count() % 1000 == 0)
					{
						//mout() << "count(): " << count() << std::endl;
                        mout() << "force: " << actual_force[0] << '\t' << actual_force[1] << '\t' << actual_force[2] << '\t'
                            << actual_force[3] << '\t' << actual_force[4] << '\t' << actual_force[5] << std::endl;

					}

				}
			}
			//Error
			else
			{
				mout() << "Wrong Model" << std::endl;
				return 0;
			}

		}
		//Over Time Exit
		if (count() == 800000)
		{
			mout() << "Over Time" << std::endl;
		}

		return 800000 - count();

	}
	ForceAlign::ForceAlign(const std::string& name) : imp_(new Imp)
	{

		aris::core::fromXmlString(command(),
			"<Command name=\"m_fa\">"
			"	<GroupParam>"
			"	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	ForceAlign::~ForceAlign() = default;
	KAANH_DEFINE_BIG_FOUR_CPP(ForceAlign)


	struct ForceKeep::Imp {

		//Flag
		bool init = false;
		bool contact_check = false;

		//Force Compensation Parameter
		double comp_f[6]{ 0 };

		//Arm1
		double arm1_init_force[6]{ 0 };
		double arm1_p_vector[6]{ 0 };
		double arm1_l_vector[6]{ 0 };

		//Arm2
		double arm2_init_force[6]{ 0 };
		double arm2_p_vector[6]{ 0 };
		double arm2_l_vector[6]{ 0 };

		//Desired Pos, Vel, Acc, Foc
		double arm1_x_d[6]{ 0 };
		double arm2_x_d[6]{ 0 };

		double v_d[6]{ 0 };
		double a_d[6]{ 0 };
		double f_d[6]{ 0 };

		//Current Vel
		double v_c[6]{ 0 };

		//Impedence Parameter
		double a1_K[6]{ 100,100,100,5,5,5 };
		double a1_B[6]{ 100,100,100,5,5,5 };
		double a1_M[6]{ 1,1,1,2,2,2 };

		double a2_K[6]{ 100,100,100,15,15,15 };
		double a2_B[6]{ 100,100,100,15,15,15 };
		double a2_M[6]{ 1,1,1,10,10,10 };

		double Ke[6]{ 220000,220000,220000,220000,220000,220000 };

		//Counter
        int contact_count = 0;

		//Switch Model
		int m_;

        //Force Buffer
        std::array<double, 10> force_buffer[6] = {};
        int buffer_index[6]{ 0 };
	};
	auto ForceKeep::prepareNrt() -> void
	{
		for (auto& m : motorOptions()) m =
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

		GravComp gc;
		gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
		mout() << "Load P & L Vector" << std::endl;
	}
	auto ForceKeep::executeRT() -> int
	{
		//dual transform modelbase into multimodel
		auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
		//at(0) -> Arm1 -> white
		auto& arm1 = dualArm.subModels().at(0);
		//at(1) -> Arm2 -> blue
		auto& arm2 = dualArm.subModels().at(1);

		//transform to model
		auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
		auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);

		//End Effector
		auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
		auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));

		//ver 1.0 not limit on vel, only limit force
		static double tolerance = 0.0001;
		static double init_angle[12] =
		{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0 ,
		0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };
		static double max_vel[6]{ 0.2,0.2,0.2,0.0005,0.0005,0.0001 };
		static double trigger_force[6]{ 0.5,0.5,0.5,0.001,0.001,0.001 };
		static double max_force[6]{ 10,10,10,5,5,5 };
		static double trigger_vel[6]{ 0.0001,0.0001,0.0001,0.0001,0.0001,0.0001 };

		GravComp gc;

        double current_angle[12]{ 0 };
        double current_sa_angle[6]{ 0 };

        double comp_force[6]{ 0 };
        double current_pm[16]{ 0 };
        double current_pos[6]{ 0 };
        double current_force[6]{ 0 };
        double actual_force[6]{ 0 };
        double filtered_force[6]{0};
        double transform_force[6]{0};

		imp_->m_ = int32Param("model");

		auto getForceData = [&](double* data_, int m_, bool init_)
		{

			int raw_force[6]{ 0 };

			for (std::size_t i = 0; i < 6; ++i)
			{
                if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
					mout() << "error" << std::endl;

				data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

			}

			if (!init_)
			{
				mout() << "Compensate Init Force" << std::endl;
			}
			else
			{
				if (m_ == 0)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

					}
				}
				else if (m_ == 1)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

					}
				}
                else
                {
                    mout() << "Wrong Model" << std::endl;
                }

			}

		};


		auto daJointMove = [&](double target_mp_[12])
		{
			double current_angle[12] = { 0 };
			double move = 0.00005;

			for (int i = 0; i < 12; i++)
			{
				current_angle[i] = controller()->motorPool()[i].targetPos();
			}

			for (int i = 0; i < 12; i++)
			{
				if (current_angle[i] <= target_mp_[i] - move)
				{
					controller()->motorPool()[i].setTargetPos(current_angle[i] + move);
				}
				else if (current_angle[i] >= target_mp_[i] + move)
				{
					controller()->motorPool()[i].setTargetPos(current_angle[i] - move);
				}
			}
		};


		auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
		{
			for (int i = 0; i < dim_; i++)
			{
				if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
				{
					return false;
				}
			}

			return true;
		};
		//single arm move 1-->white 2-->blue


		auto saMove = [&](double* pos_, aris::dynamic::Model& model_, int type_) {

			model_.setOutputPos(pos_);

			if (model_.inverseKinematics())
			{
				throw std::runtime_error("Inverse Kinematics Position Failed!");
			}


			double x_joint[6]{ 0 };

			model_.getInputPos(x_joint);

			if (type_ == 0)
			{
				for (std::size_t i = 0; i < 6; ++i)
				{
					controller()->motorPool()[i].setTargetPos(x_joint[i]);
				}
			}
			else if (type_ == 1)
			{
				for (std::size_t i = 0; i < 6; ++i)
				{
					controller()->motorPool()[i + 6].setTargetPos(x_joint[i]);
				}
			}
			else
			{
				throw std::runtime_error("Arm Type Error");
			}
		};


        auto forceFilter = [&](double* actual_force_, double* filtered_force_)
        {
            for (int i = 0; i < 6; i++)
            {
                imp_->force_buffer[i][imp_->buffer_index[i]] = actual_force_[i];
                imp_->buffer_index[i] = (imp_->buffer_index[i] + 1) % 10;

                filtered_force_[i] = std::accumulate(imp_->force_buffer[i].begin(), imp_->force_buffer[i].end(), 0.0) / 10;
            }
        };


		for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].actualPos();
		}



        if (!imp_->init && !imp_->contact_check)
		{



			dualArm.setInputPos(init_angle);

			if (dualArm.forwardKinematics())
			{
				throw std::runtime_error("Forward Kinematics Position Failed!");
			}


			daJointMove(init_angle);

			if (count() % 1000 == 0)
			{

				mout() << current_angle[0] << '\t' << current_angle[1] << '\t' << current_angle[2] << '\t'
					<< current_angle[3] << '\t' << current_angle[4] << '\t' << current_angle[5] << std::endl;

			}

			if (motorsPositionCheck(current_angle, init_angle, 12))
			{

				eeA1.getP(imp_->arm1_x_d);
				eeA2.getP(imp_->arm2_x_d);

				getForceData(imp_->arm1_init_force, 0, imp_->init);
				getForceData(imp_->arm2_init_force, 1, imp_->init);

				mout() << "Back To Init" << std::endl;
				imp_->init = true;
			}


		}

        else if (imp_->init && !imp_->contact_check)
        {
            double raw_force_checker[12]{ 0 };
            double comp_force_checker[12]{ 0 };
            double force_checker[12]{ 0 };

            double a1_pm[16]{ 0 };
            double a2_pm[16]{ 0 };

            eeA1.getMpm(a1_pm);
            eeA2.getMpm(a2_pm);


            //Arm1
            getForceData(raw_force_checker, 0, imp_->init);
            gc.getCompFT(a1_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force_checker);
            //Arm2
            getForceData(raw_force_checker + 6, 1, imp_->init);
            gc.getCompFT(a2_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force_checker + 6);


            for (int i = 0; i < 12; i++)
            {
                force_checker[i] = comp_force_checker[i] + raw_force_checker[i];

                if (abs(force_checker[i]) > 3.0)
                {
                    imp_->contact_check = true;
                    mout() << "Contact Check" << std::endl;
                    break;
                }
            }
        }
		else
		{
			if (imp_->m_ == 0)
			{
				double current_vel[6]{ 0 };
				eeA1.getP(current_pos);
				eeA1.getV(current_vel);
				eeA1.getMpm(current_pm);

				getForceData(current_force, 0, imp_->init);
				gc.getCompFT(current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force);
				for (int i = 0; i < 6; i++)
				{
					actual_force[i] = comp_force[i] + current_force[i];
				}

				//Dead Zone of Force
				for (int i = 0; i < 6; i++)
				{
					if (abs(actual_force[i]) < trigger_force[i])
					{
						actual_force[i] = 0;
					}
					if (actual_force[i] > max_force[i])
					{
						actual_force[i] = max_force[i];
					}
					if (actual_force[i] < -max_force[i])
					{
						actual_force[i] = -max_force[i];
					}

				}

                //Force Filter
                forceFilter(actual_force,filtered_force);

                //Coordinate Transform Arm1
                transform_force[0] = filtered_force[2];
                transform_force[1] = -filtered_force[1];
                transform_force[2] = filtered_force[0];

                transform_force[3] = filtered_force[5];
                transform_force[4] = -filtered_force[4];
                transform_force[5] = filtered_force[3];



				double acc[3]{ 0 };
				double ome[3]{ 0 };
				double pm[16]{ 0 };
				double dx[3]{ 0 };
				double dth[3]{ 0 };
                double dt = 0.002;

				//position
				for (int i = 0; i < 3; i++)
				{
					// da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                    acc[i] = (-imp_->f_d[i] + transform_force[i] - imp_->a1_B[i] * (imp_->v_c[i] - imp_->v_d[i]) - imp_->a1_K[i] * (current_pos[i] - imp_->arm1_x_d[i])) / imp_->a1_M[i];
				}


				for (int i = 0; i < 3; i++)
				{
					imp_->v_c[i] += acc[i] * dt;
					dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
					current_pos[i] = dx[i] + current_pos[i];

				}

				double rm_c[9]{ 0 };
				double rm_d[9]{ 0 };
                double inv_rm_d[9]{0};
				double rm_e[9]{ 0 };
				double pose_error[3]{ 0 };


				//Rm of Desired Pos
				aris::dynamic::s_re2rm(imp_->arm1_x_d + 3, rm_d, "321");

				//Current pe to rm
				aris::dynamic::s_re2rm(current_pos + 3, rm_c, "321");

				//Inverse
                //aris::dynamic::s_rm_dot_inv_rm(rm_c, rm_d, rm_e);
                gc.getInverseRm(rm_d, inv_rm_d);
                aris::dynamic::s_mm(3,3,3,rm_c,inv_rm_d,rm_e);

				//Convert Rm to Ra
				aris::dynamic::s_rm2ra(rm_e, pose_error);



				//pose
				for (int i = 0; i < 3; i++)
				{
					// Caculate Omega
                    ome[i] = (-imp_->f_d[i + 3] + transform_force[i + 3] - imp_->a1_B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3]) - imp_->a1_K[i + 3] * pose_error[i]) / imp_->a1_M[i + 3];
				}


				for (int i = 0; i < 3; i++)
				{
					// Angluar Velocity
					imp_->v_c[i + 3] += ome[i] * dt;
					dth[i] = imp_->v_c[i + 3] * dt;
				}

				double drm[9]{ 0 };
				double rm_target[9]{ 0 };


				//Transform to rm
				aris::dynamic::s_ra2rm(dth, drm);

				//Calcuate Future rm
				aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);

				//Convert rm to pe
				aris::dynamic::s_rm2re(rm_target, current_pos + 3, "321");


				eeA1.setV(imp_->v_c);
				if (model_a1.inverseKinematicsVel()) {
					mout() << "Error" << std::endl;
				}
				saMove(current_pos, model_a1, 0);




				if (count() % 100 == 0)
				{
					// mout() << current_vel[0] << '\t' << current_vel[1] << '\t' << current_vel[2] << '\t'
					// 	<< current_vel[3] << '\t' << current_vel[4] << '\t' << current_vel[5] << std::endl;

					 mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
					 	<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;

					//mout() << "error: " << pose_error[0] << '\t' << pose_error[1] << '\t' << pose_error[2] << std::endl;

				}

			}
			else if (imp_->m_ == 1)
			{

				double current_vel[6]{ 0 };

				eeA2.getP(current_pos);

				eeA2.getV(current_vel);
				eeA2.getMpm(current_pm);

				getForceData(current_force, 1, imp_->init);
				gc.getCompFT(current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force);
				for (int i = 0; i < 6; i++)
				{
					actual_force[i] = comp_force[i] + current_force[i];
				}

				//Dead Zone of Force
				for (int i = 0; i < 6; i++)
				{
					if (abs(actual_force[i]) < trigger_force[i])
					{
						actual_force[i] = 0;
					}
					if (actual_force[i] > max_force[i])
					{
						actual_force[i] = max_force[i];
					}
					if (actual_force[i] < -max_force[i])
					{
						actual_force[i] = -max_force[i];
					}

				}

                //Force Filter
                forceFilter(actual_force, filtered_force);


                //Coordinate Transform Arm2
                transform_force[0] = -filtered_force[2];
                transform_force[1] = filtered_force[1];
                transform_force[2] = filtered_force[0];

                transform_force[3] = -filtered_force[5];
                transform_force[4] = filtered_force[4];
                transform_force[5] = filtered_force[3];


				double acc[3]{ 0 };
				double ome[3]{ 0 };
				double pm[16]{ 0 };
				double dx[3]{ 0 };
				double dth[3]{ 0 };
                double dt = 0.002;

				//position
				for (int i = 0; i < 3; i++)
				{
					// da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M

                    acc[i] = (-imp_->f_d[i] + transform_force[i] - imp_->a2_B[i] * (imp_->v_c[i] - imp_->v_d[i]) - imp_->a2_K[i] * (current_pos[i] - imp_->arm2_x_d[i])) / imp_->a2_M[i];
				}


				for (int i = 0; i < 3; i++)
				{
					imp_->v_c[i] += acc[i] * dt;
					dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
					current_pos[i] = dx[i] + current_pos[i];

				}


				double rm_c[9]{ 0 };
				double rm_d[9]{ 0 };
                double inv_rm_d[9]{0};
				double rm_e[9]{ 0 };
				double pose_error[3]{ 0 };


				//Rm of Desired Pos
				aris::dynamic::s_re2rm(imp_->arm2_x_d + 3, rm_d, "321");

				//Current pe to rm
				aris::dynamic::s_re2rm(current_pos + 3, rm_c, "321");

				//Inverse
                //aris::dynamic::s_rm_dot_inv_rm(rm_c, rm_d, rm_e);
                gc.getInverseRm(rm_d, inv_rm_d);
                aris::dynamic::s_mm(3,3,3,rm_c,inv_rm_d,rm_e);


				//Convert Rm to Ra
				aris::dynamic::s_rm2ra(rm_e, pose_error);




				//pose
				for (int i = 0; i < 3; i++)
				{
					// Caculate Omega
                    ome[i] = (-imp_->f_d[i + 3] + transform_force[i + 3] - imp_->a2_B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3]) - imp_->a2_K[i + 3] * pose_error[i]) / imp_->a2_M[i + 3];
				}


				for (int i = 0; i < 3; i++)
				{
					// Angluar Velocity
					imp_->v_c[i + 3] += ome[i] * dt;
					dth[i] = imp_->v_c[i + 3] * dt;
				}

				double drm[9]{ 0 };
				double rm_target[9]{ 0 };


				//Transform to rm
				aris::dynamic::s_ra2rm(dth, drm);

				//Calcuate Future rm
				aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);

				//Convert rm to pe
				aris::dynamic::s_rm2re(rm_target, current_pos + 3, "321");

				eeA2.setV(imp_->v_c);
				if (model_a2.inverseKinematicsVel()) {
					mout() << "Error" << std::endl;
				}
				saMove(current_pos, model_a2, 1);



				if (count() % 100 == 0)
				{
					// mout() << current_vel[0] << '\t' << current_vel[1] << '\t' << current_vel[2] << '\t'
					// 	<< current_vel[3] << '\t' << current_vel[4] << '\t' << current_vel[5] << std::endl;

					mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
						<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;

				}
			}
			else
			{
				mout() << "Wrong Model" << std::endl;
				return 0;
			}





		}

        return 30000 - count();
	}
	ForceKeep::ForceKeep(const std::string& name)
	{
		aris::core::fromXmlString(command(),
			"<Command name=\"m_fk\">"
			"	<GroupParam>"
			"	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	ForceKeep::~ForceKeep() = default;
	KAANH_DEFINE_BIG_FOUR_CPP(ForceKeep)



	struct ForceDrag::Imp {

		//Flag
		bool init = false;
		bool contact_check = false;

		//Force Compensation Parameter
		double comp_f[6]{ 0 };

		//Arm1
		double arm1_init_force[6]{ 0 };
		double arm1_p_vector[6]{ 0 };
		double arm1_l_vector[6]{ 0 };

		//Arm2
		double arm2_init_force[6]{ 0 };
		double arm2_p_vector[6]{ 0 };
		double arm2_l_vector[6]{ 0 };

		//Desired Pos, Vel, Acc, Foc
		double arm1_x_d[6]{ 0 };
		double arm2_x_d[6]{ 0 };

		double v_d[6]{ 0 };
		double a_d[6]{ 0 };
		double f_d[6]{ 0 };

		//Current Vel
		double v_c[6]{ 0 };

		//Impedence Parameter
		double K[6]{ 100,100,100,15,15,15 };
//        double B[6]{ 300,300,300,15,15,15 };
//        double M[6]{ 3,3,3,10,10,10 };

        double B[6]{ 1000,1000,1000,15,80,80 };
        double M[6]{ 10,10,10,10,5,5 };

		double Ke[6]{ 220000,220000,220000,220000,220000,220000 };


        //Parameters For Compensating rz
        double a_y = -0.0592;
        double b_y = -0.9943;
        double c_y = 0.0132;


		//Counter
		int contact_count = 0;

		//Test
		double actual_force[6]{ 0 };

		//Switch Model
		int m_;

        //Force Buffer
        std::array<double, 10> force_buffer[6] = {};
        int buffer_index[6]{ 0 };
	};
	auto ForceDrag::prepareNrt() -> void
	{
		for (auto& m : motorOptions()) m =
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

		GravComp gc;
		gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
		mout() << "Load P & L Vector" << std::endl;
	}
	auto ForceDrag::executeRT() -> int
	{
		//dual transform modelbase into multimodel
		auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
		//at(0) -> Arm1 -> white
		auto& arm1 = dualArm.subModels().at(0);
		//at(1) -> Arm2 -> blue
		auto& arm2 = dualArm.subModels().at(1);

		//transform to model
		auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
		auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);

		//End Effector
		auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
		auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));

		//ver 1.0 not limit on vel, only limit force
		static double tolerance = 0.0001;
		static double init_angle[12] =
		{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0 ,
		0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };
		static double max_vel[6]{ 0.2,0.2,0.2,0.0005,0.0005,0.0001 };
        //static double trigger_force[6]{ 0.5,0.5,0.5,0.001,0.001,0.001 };
        static double trigger_force[6]{ 3.0,3.0,5.0,0.5,0.5,0.5 };
		static double max_force[6]{ 10,10,10,5,5,5 };
		static double trigger_vel[6]{ 0.0001,0.0001,0.0001,0.0001,0.0001,0.0001 };

        GravComp gc;

        double current_angle[12]{ 0 };
        double current_sa_angle[6]{ 0 };

        double comp_force[6]{ 0 };
        double current_pm[16]{ 0 };
        double current_pos[6]{ 0 };
        double current_force[6]{ 0 };
        double actual_force[6]{ 0 };
        double filtered_force[6]{0};
        double transform_force[6]{0};

        double rz_comp = 0;
        double rz_comp_force[6]{0};

		imp_->m_ = int32Param("model");

		auto getForceData = [&](double* data_, int m_, bool init_)
		{

			int raw_force[6]{ 0 };

			for (std::size_t i = 0; i < 6; ++i)
			{
                if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
					mout() << "error" << std::endl;

				data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

			}

			if (!init_)
			{
				mout() << "Compensate Init Force" << std::endl;
			}
			else
			{
				if (m_ == 0)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

					}
				}
				else if (m_ == 1)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

					}
				}
                else
                {
                    mout() << "Wrong Model" << std::endl;
                }
			}

		};


		auto daJointMove = [&](double target_mp_[12])
		{
			double current_angle[12] = { 0 };
			double move = 0.00005;

			for (int i = 0; i < 12; i++)
			{
				current_angle[i] = controller()->motorPool()[i].targetPos();
			}

			for (int i = 0; i < 12; i++)
			{
				if (current_angle[i] <= target_mp_[i] - move)
				{
					controller()->motorPool()[i].setTargetPos(current_angle[i] + move);
				}
				else if (current_angle[i] >= target_mp_[i] + move)
				{
					controller()->motorPool()[i].setTargetPos(current_angle[i] - move);
				}
			}
		};


		auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
		{
			for (int i = 0; i < dim_; i++)
			{
				if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
				{
					return false;
				}
			}

			return true;
		};
		//single arm move 1-->white 2-->blue


		auto saMove = [&](double* pos_, aris::dynamic::Model& model_, int type_) {

			model_.setOutputPos(pos_);

			if (model_.inverseKinematics())
			{
				throw std::runtime_error("Inverse Kinematics Position Failed!");
			}


			double x_joint[6]{ 0 };

			model_.getInputPos(x_joint);

			if (type_ == 0)
			{
				for (std::size_t i = 0; i < 6; ++i)
				{
					controller()->motorPool()[i].setTargetPos(x_joint[i]);
				}
			}
			else if (type_ == 1)
			{
				for (std::size_t i = 0; i < 6; ++i)
				{
					controller()->motorPool()[i + 6].setTargetPos(x_joint[i]);
				}
			}
			else
			{
				throw std::runtime_error("Arm Type Error");
			}
		};

        auto forceFilter = [&](double* actual_force_, double* filtered_force_)
        {
            for (int i = 0; i < 6; i++)
            {
                imp_->force_buffer[i][imp_->buffer_index[i]] = actual_force_[i];
                imp_->buffer_index[i] = (imp_->buffer_index[i] + 1) % 10;

                filtered_force_[i] = std::accumulate(imp_->force_buffer[i].begin(), imp_->force_buffer[i].end(), 0.0) / 10;
            }
        };




		for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].actualPos();
		}



        if (!imp_->init && !imp_->contact_check)
		{



//			dualArm.setInputPos(init_angle);

//			if (dualArm.forwardKinematics())
//			{
//				throw std::runtime_error("Forward Kinematics Position Failed!");
//			}



//			daJointMove(init_angle);

//			if (count() % 1000 == 0)
//			{

//				mout() << current_angle[0] << '\t' << current_angle[1] << '\t' << current_angle[2] << '\t'
//					<< current_angle[3] << '\t' << current_angle[4] << '\t' << current_angle[5] << std::endl;

//			}

//			if (motorsPositionCheck(current_angle, init_angle, 12))
//			{

//				eeA1.getP(imp_->arm1_x_d);
//				eeA2.getP(imp_->arm2_x_d);

//				mout() << imp_->arm1_x_d[0] << '\t' << imp_->arm1_x_d[1] << '\t' << imp_->arm1_x_d[2] << '\t'
//					<< imp_->arm1_x_d[3] << '\t' << imp_->arm1_x_d[4] << '\t' << imp_->arm1_x_d[5] << std::endl;

//				getForceData(imp_->arm1_init_force, 0, imp_->init);
//				getForceData(imp_->arm2_init_force, 1, imp_->init);

//				mout() << "Back To Init" << std::endl;
//				imp_->init = true;
//			}

            getForceData(imp_->arm1_init_force, 0, imp_->init);
            getForceData(imp_->arm2_init_force, 1, imp_->init);
            master()->logFileRawName("ForceDrag");
            imp_->init = true;

		}
        else if (imp_->init && !imp_->contact_check)
        {
            double raw_force_checker[12]{ 0 };
            double comp_force_checker[12]{ 0 };
            double force_checker[12]{ 0 };

            double a1_pm[16]{ 0 };
            double a2_pm[16]{ 0 };

            eeA1.getMpm(a1_pm);
            eeA2.getMpm(a2_pm);


            //Arm1
            getForceData(raw_force_checker, 0, imp_->init);
            gc.getCompFT(a1_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force_checker);
            //Arm2
            getForceData(raw_force_checker + 6, 1, imp_->init);
            gc.getCompFT(a2_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force_checker + 6);


            for (int i = 0; i < 12; i++)
            {
                force_checker[i] = comp_force_checker[i] + raw_force_checker[i];

                if (abs(force_checker[i]) > 3.0)
                {
                    imp_->contact_check = true;
                    mout() << "Contact Check" << std::endl;
                    break;
                }
            }
        }
		else
		{



			if (imp_->m_ == 0)
			{
				double current_vel[6]{ 0 };

				eeA1.getP(current_pos);
				eeA1.getV(current_vel);
				eeA1.getMpm(current_pm);

				getForceData(current_force, 0, imp_->init);
				gc.getCompFT(current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force);
				for (int i = 0; i < 6; i++)
				{
					actual_force[i] = comp_force[i] + current_force[i];
				}

				//Dead Zone of Force
				for (int i = 0; i < 6; i++)
				{
					if (abs(actual_force[i]) < trigger_force[i])
					{
						actual_force[i] = 0;
					}
					if (actual_force[i] > max_force[i])
					{
						actual_force[i] = max_force[i];
					}
					if (actual_force[i] < -max_force[i])
					{
						actual_force[i] = -max_force[i];
					}

				}

                //Force Filter
                forceFilter(actual_force,filtered_force);

                //Coordinate Transform Arm1
                transform_force[0] = filtered_force[2];
                transform_force[1] = -filtered_force[1];
                transform_force[2] = filtered_force[0];

                transform_force[3] = filtered_force[5];
                transform_force[4] = -filtered_force[4];
                transform_force[5] = filtered_force[3];



				double acc[3]{ 0 };
				double ome[3]{ 0 };
				double pm[16]{ 0 };
				double dx[3]{ 0 };
				double dth[3]{ 0 };
                double dt = 0.002;

				//position
				for (int i = 0; i < 3; i++)
				{
					// da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                    acc[i] = (-imp_->f_d[i] + transform_force[i] - imp_->B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->M[i];
				}


				for (int i = 0; i < 3; i++)
				{
					imp_->v_c[i] += acc[i] * dt;
					dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
					current_pos[i] = dx[i] + current_pos[i];

				}


				//pose
                for (int i = 0; i < 3; i++)
				{
					// Caculate Omega
                    ome[i] = (-imp_->f_d[i + 3] + transform_force[i + 3] - imp_->B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3])) / imp_->M[i + 3];
				}


                for (int i = 0; i < 3; i++)
				{
					// Angluar Velocity
					imp_->v_c[i + 3] += ome[i] * dt;
					dth[i] = imp_->v_c[i + 3] * dt;
				}

				double drm[9]{ 0 };
				double rm_target[9]{ 0 };
				double rm_c[9]{ 0 };




				//Transform to rm
				aris::dynamic::s_ra2rm(dth, drm);

				//Current pe to rm
				aris::dynamic::s_re2rm(current_pos + 3, rm_c, "321");

				//Calcuate Future rm
				aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);

				//Convert rm to pe
				aris::dynamic::s_rm2re(rm_target, current_pos + 3, "321");

				eeA1.setV(imp_->v_c);
				if (model_a1.inverseKinematicsVel()) {
					mout() << "Error" << std::endl;
				}
                //saMove(current_pos, model_a1, 0);


				if (count() % 100 == 0)
				{
					// mout() << current_vel[0] << '\t' << current_vel[1] << '\t' << current_vel[2] << '\t'
					// 	<< current_vel[3] << '\t' << current_vel[4] << '\t' << current_vel[5] << std::endl;

                    mout() <<"Pos: "<<'\t'<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] <<'\t'
                          <<"Force: "<< current_force[0] << '\t' << current_force[1] << '\t' << current_force[2] <<'\t'
                         << current_force[3] << '\t' << current_force[4] << '\t' << current_force[5] <<'\t'<< std::endl;

				}

			}
			else if (imp_->m_ == 1)
			{

				double current_vel[6]{ 0 };

				eeA2.getP(current_pos);

				eeA2.getV(current_vel);
				eeA2.getMpm(current_pm);

				getForceData(current_force, 1, imp_->init);
				gc.getCompFT(current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force);
				for (int i = 0; i < 6; i++)
				{
					actual_force[i] = comp_force[i] + current_force[i];
				}


//                if(count()%10==0)
//                {
//                    lout()<<"raw"<<'\t'<<current_force[0]+imp_->arm2_init_force[0]<<'\t'<<current_force[1]+imp_->arm2_init_force[1]<<'\t'<<current_force[2]+imp_->arm2_init_force[2]<<'\t'
//                            <<"comp"<<'\t'<<actual_force[0]<<'\t'<<actual_force[1]<<'\t'<<actual_force[2]<<std::endl;
//                }

                //Force Filter
                forceFilter(actual_force,filtered_force);

                //Coordinate Transform Arm2
                transform_force[0] = -filtered_force[0];
                transform_force[1] = filtered_force[1];
                transform_force[2] = -filtered_force[2];

                transform_force[3] = -filtered_force[3];
                transform_force[4] = filtered_force[4];
                transform_force[5] = -filtered_force[5];

                if (count() % 50 == 0)
                {
                    // mout() << current_vel[0] << '\t' << current_vel[1] << '\t' << current_vel[2] << '\t'
                    // 	<< current_vel[3] << '\t' << current_vel[4] << '\t' << current_vel[5] << std::endl;

                    mout() <<"Pos: "<<'\t'<< current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] <<'\t'
                          <<"Force: "<< transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] <<'\t'
                         << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] <<'\t'<< std::endl;

                }


                //Dead Zone of Force
                for (int i = 0; i < 6; i++)
                {
                    if (abs(transform_force[i]) < trigger_force[i])
                    {
                        transform_force[i] = 0;
                    }
                    if (transform_force[i] > max_force[i])
                    {
                        transform_force[i] = max_force[i];
                    }
                    if (transform_force[i] < -max_force[i])
                    {
                        transform_force[i] = -max_force[i];
                    }

                }



                //rz Compensation
//                rz_comp = imp_->a_y*transform_force[1]-imp_->b_y*0.0825*transform_force[0]+imp_->c_y;
//                transform_force[5] -= rz_comp;


				double acc[3]{ 0 };
				double ome[3]{ 0 };
				double pm[16]{ 0 };
				double dx[3]{ 0 };
				double dth[3]{ 0 };
                double dt = 0.002;

				//position
                for (int i = 0; i < 3; i++)
				{
					// da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                    acc[i] = (-imp_->f_d[i] + transform_force[i] - imp_->B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->M[i];
				}


                for (int i = 0; i < 3; i++)
				{
					imp_->v_c[i] += acc[i] * dt;
					dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
					current_pos[i] = dx[i] + current_pos[i];

				}


//				//pose
                // for (int i = 1; i < 3; i++)
                // {
                //     // Caculate Omega
                //     ome[i] = (-imp_->f_d[i + 3] + transform_force[i + 3] - imp_->B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3])) / imp_->M[i + 3];
                // }


                // for (int i = 1; i < 3; i++)
                // {
                //     // Angluar Velocity
                //     imp_->v_c[i + 3] += ome[i] * dt;
                //     dth[i] = imp_->v_c[i + 3] * dt;
                // }

                // double drm[9]{ 0 };
                // double rm_target[9]{ 0 };
                // double rm_c[9]{ 0 };




                // //Transform to rm
                // aris::dynamic::s_ra2rm(dth, drm);

                // //Current pe to rm
                // aris::dynamic::s_re2rm(current_pos + 3, rm_c, "321");

                // //Calcuate Future rm
                // aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);

                // //Convert rm to pe
                // aris::dynamic::s_rm2re(rm_target, current_pos + 3, "321");


				eeA2.setV(imp_->v_c);
				if (model_a2.inverseKinematicsVel()) {
					mout() << "Error" << std::endl;
				}
				saMove(current_pos, model_a2, 1);




//                if (count() % 50 == 0)
//                {

//                    mout() <<"force: "<<'\t'<< transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
//                        << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << '\t'
//                        <<"comp: "<<'\t'<< rz_comp <<std::endl;

//                }
			}
			else
			{
				mout() << "Wrong Model" << std::endl;
				return 0;
			}





		}

        return 50000 - count();
	}
	ForceDrag::ForceDrag(const std::string& name)
	{
		aris::core::fromXmlString(command(),
			"<Command name=\"m_fd\">"
			"	<GroupParam>"
            "	<Param name=\"model\" default=\"1\" abbreviation=\"m\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	ForceDrag::~ForceDrag() = default;
	KAANH_DEFINE_BIG_FOUR_CPP(ForceDrag)




    struct Demo::Imp {

        //Flag
        //Phase 1 -> Approach, Phase 2 -> Contact, Phase 3 -> Align, Phase 4 -> Fit, Phase 5 -> Insert
        bool init = false;
        bool phase1 = false;
        bool phase2 = false;
        bool phase3 = false;
        bool phase4 = false;
        bool phase5 = false;

        //Arm1
        double arm1_init_force[6]{ 0 };
        double arm1_start_force[6]{ 0 };
        double arm1_p_vector[6]{ 0 };
        double arm1_l_vector[6]{ 0 };

        double arm1_temp_force[6]{0};

        //Arm2
        double arm2_init_force[6]{ 0 };
        double arm2_start_force[6]{ 0 };
        double arm2_p_vector[6]{ 0 };
        double arm2_l_vector[6]{ 0 };

        double arm2_temp_force[6]{0};

        //Desired Pos, Vel, Acc, Foc
        double arm1_x_d[6]{ 0 };
        double arm2_x_d[6]{ 0 };

        double v_d[6]{ 0 };
        double a_d[6]{ 0 };
        double f_d[6]{ 0 };

        //Desired Force of Each Phase
        double phase2_fd[6]{ 5,0,0,0,0,0 };

        //Current Vel
        double v_c[6]{ 0 };

        //Impedence Parameter
        //double K[6]{ 100,100,100,15,15,15 };
        double phase2_B[6]{ 25000,1500,1500,3,3,3 };
        double phase2_M[6]{ 2000,100,100,2,2,2 };


        //Counter
        int current_count = 0;
        int allign_count = 0;
        int success_count = 0;


        int start_count = 0;
        int complete_count = 0;
        int back_count = 0;

        int accumulation_count = 0;

        //Pos Counter
        int pos_count = 0;
        int pos_success_count = 0;

        double current_pos_checkek[6] = {0};

        //Switch Angle
        double z_;
        double y_;

        //Switch Point
        int p_;

        //Arm1 Force Buffer
        std::array<double, 10> arm1_force_buffer[6] = {};
        int arm1_buffer_index[6]{ 0 };

        //Arm2 Force Buffer
        std::array<double, 10> arm2_force_buffer[6] = {};
        int arm2_buffer_index[6]{ 0 };

        //Parameters for Pos Recoginistion
        double a_y = -0.0567;
        double b_y = -0.9679;
        double c_y = 0.0010;

        double a_z = 0.9745;
        double b_z = -0.0535;
        double c_z = 0.0063;

        double dy = 0;
        double dz = 0;

        double hole_pos[6]{0};
        double hole_angle[6]{0};



    };
    auto Demo::prepareNrt() -> void
    {
        for (auto& m : motorOptions()) m =
            aris::plan::Plan::CHECK_NONE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

        GravComp gc;
        gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
        mout() << "Load P & L Vector" << std::endl;
//		gc.loadInitForce(imp_->arm1_init_force, imp_->arm2_init_force);
//		mout() << "Load Init Force" << std::endl;
//		imp_->init = true;
    }
    auto Demo::executeRT() -> int
    {
        //dual transform modelbase into multimodel
        auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
        //at(0) -> Arm1 -> white
        auto& arm1 = dualArm.subModels().at(0);
        //at(1) -> Arm2 -> blue
        auto& arm2 = dualArm.subModels().at(1);

        //transform to model
        auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
        auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);

        //End Effector
        auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
        auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));

        //ver 1.0 not limit on vel, only limit force
        static double tolerance = 0.00005;
        //ver 2.0 limited vel, 20mm/s, 30deg/s
        static double max_vel[6]{ 0.00002,0.00002,0.00002,0.0005,0.0005,0.0005 };
        static double dead_zone[6]{ 0.1,0.1,0.1,0.0006,0.00920644,0.0006 };


        GravComp gc;

        double current_angle[12]{ 0 };
        double current_sa_angle[6]{ 0 };

        double arm1_comp_force[6]{ 0 };
        double arm1_current_pm[16]{ 0 };
        double arm1_current_pos[6]{ 0 };
        double arm1_actual_force[6]{ 0 };
        double arm1_filtered_force[6]{ 0 };
        double arm1_transform_force[6]{ 0 };
        double arm1_final_force[6]{0};

        double arm2_comp_force[6]{ 0 };
        double arm2_current_pm[16]{ 0 };
        double arm2_current_pos[6]{ 0 };
        double arm2_actual_force[6]{ 0 };
        double arm2_filtered_force[6]{ 0 };
        double arm2_transform_force[6]{ 0 };
        double arm2_final_force[6]{0};


        imp_->z_ = doubleParam("z_degree");

        imp_->y_ = doubleParam("y_degree");

        imp_->p_ = int32Param("point");


        auto getForceData = [&](double* data_, int m_, bool init_)
        {

            int raw_force[6]{ 0 };

            for (std::size_t i = 0; i < 6; ++i)
            {
                if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
                    mout() << "error" << std::endl;

                data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

            }

            if (!init_)
            {
                mout() << "Compensate Init Force" << std::endl;
            }
            else
            {
                if (m_ == 0)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

                    }
                }
                else if (m_ == 1)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

                    }
                }
                else
                {
                    mout() << "Wrong Model" << std::endl;
                }

            }

        };

        auto daJointMove = [&](double target_mp_[12])
        {
            double current_angle[12] = { 0 };
            double move = 0.00005;

            for (int i = 0; i < 12; i++)
            {
                current_angle[i] = controller()->motorPool()[i].targetPos();
            }

            for (int i = 0; i < 12; i++)
            {
                if (current_angle[i] <= target_mp_[i] - move)
                {
                    controller()->motorPool()[i].setTargetPos(current_angle[i] + move);
                }
                else if (current_angle[i] >= target_mp_[i] + move)
                {
                    controller()->motorPool()[i].setTargetPos(current_angle[i] - move);
                }
            }
        };

        auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
        {
            for (int i = 0; i < dim_; i++)
            {
                if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
                {
                    return false;
                }
            }

            return true;
        };

        auto saJointMove = [&](double target_mp_[6], int m_)
        {
            double current_angle[12] = { 0 };
            double move = 0.00005;

            for (int i = 0; i < 12; i++)
            {
                current_angle[i] = controller()->motorPool()[i].targetPos();
            }

            for (int i = 0; i < 6; i++)
            {
                if (current_angle[i+6*m_] <= target_mp_[i] - move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] + move);
                }
                else if (current_angle[i+6*m_] >= target_mp_[i] + move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] - move);
                }
            }
        };

        auto saMove = [&](double* pos_, aris::dynamic::Model& model_, int type_) {

            model_.setOutputPos(pos_);

            if (model_.inverseKinematics())
            {
                throw std::runtime_error("Inverse Kinematics Position Failed!");
            }


            double x_joint[6]{ 0 };

            model_.getInputPos(x_joint);

            if (type_ == 0)
            {
                for (std::size_t i = 0; i < 6; ++i)
                {
                    controller()->motorPool()[i].setTargetPos(x_joint[i]);
                }
            }
            else if (type_ == 1)
            {
                for (std::size_t i = 0; i < 6; ++i)
                {
                    controller()->motorPool()[i + 6].setTargetPos(x_joint[i]);
                }
            }
            else
            {
                throw std::runtime_error("Arm Type Error");
            }
        };

        auto forceFilter = [&](double* actual_force_, double* filtered_force_, int m_)
        {
            if(m_ == 0)
            {
                for (int i = 0; i < 6; i++)
                {
                    imp_->arm1_force_buffer[i][imp_->arm1_buffer_index[i]] = actual_force_[i];
                    imp_->arm1_buffer_index[i] = (imp_->arm1_buffer_index[i] + 1) % 10;

                    filtered_force_[i] = std::accumulate(imp_->arm1_force_buffer[i].begin(), imp_->arm1_force_buffer[i].end(), 0.0) / 10;
                }
            }
            else if(m_ == 1)
            {
                for (int i = 0; i < 6; i++)
                {
                    imp_->arm2_force_buffer[i][imp_->arm2_buffer_index[i]] = actual_force_[i];
                    imp_->arm2_buffer_index[i] = (imp_->arm2_buffer_index[i] + 1) % 10;

                    filtered_force_[i] = std::accumulate(imp_->arm2_force_buffer[i].begin(), imp_->arm2_force_buffer[i].end(), 0.0) / 10;
                }
            }
            else
            {
                mout()<<"Wrong Filter!"<<std::endl;
            }

        };

        auto forceDeadZone = [&](double* actual_force_, double* area_)
        {
            for (int i = 0; i < 6; i++)
            {
                if (abs(actual_force_[i]) < area_[i])
                {
                    actual_force_[i] = 0;
                }
            }
        };

        auto forceTransform = [&](double* actual_force_, double* transform_force_, int m_)
        {
            if (m_ == 0)
            {
                transform_force_[0] = actual_force_[2];
                transform_force_[1] = -actual_force_[1];
                transform_force_[2] = actual_force_[0];

                transform_force_[3] = actual_force_[5];
                transform_force_[4] = -actual_force_[4];
                transform_force_[5] = actual_force_[3];
            }
            else if (m_ == 1)
            {
                transform_force_[0] = -actual_force_[2];
                transform_force_[1] = actual_force_[1];
                transform_force_[2] = actual_force_[0];

                transform_force_[3] = -actual_force_[5];
                transform_force_[4] = actual_force_[4];
                transform_force_[5] = actual_force_[3];
            }
            else
            {
                mout() << "Error Model In Force Transform" << std::endl;
            }
        };

        auto forceCheck = [&](double* current_force_, double* force_check_, int count_)
        {

            bool isValid = true;
            for (int i = 0; i < 3; i++)
            {
                if (abs(current_force_[i]) > force_check_[i])
                {
                    isValid = false;
                    break;
                }
            }

            if (isValid)
            {
                if (imp_->allign_count == 0) {
                    imp_->allign_count = count();
                }

                if ((count() - imp_->allign_count) % 30 == 0) {
                    imp_->success_count++;
                    mout() << "Check " << imp_->success_count << '\t' << "Current Count: " << count() << std::endl;
                    if (imp_->success_count >= count_)
                    {
                        return true;
                    }
                }
            }
            else
            {
                imp_->success_count = 0;
            }

            return false;
        };

        auto posCheck = [&](double* current_pos_, int count_)
        {
            if(count()%500 == 0)
            {
                std::copy(current_pos_, current_pos_+6, imp_->current_pos_checkek);
                //mout()<<"Save Current Pos: "<<imp_->current_pos_checkek[0]<<std::endl;
            }

            bool isValid = true;

            for(int i = 0; i < 3; i++)
            {
                if (abs(current_pos_[i] - imp_->current_pos_checkek[i]) >= 0.000005)
                {
                    isValid = false;
                    break;
                }
            }

            if(isValid)
            {
                if(imp_->pos_count == 0)
                {
                    imp_->pos_count = count();
                }

                if((count() - imp_->pos_count) % 50 == 0)
                {
                    imp_->pos_success_count ++;
                    //mout() << "Check " << imp_->pos_success_count << '\t' << "Current Count: " << count() << std::endl;
                    if(imp_->pos_success_count >= count_)
                    {
                        imp_->pos_count = 0;
                        imp_->pos_success_count = 0;
                        return true;
                    }
                }
            }
            else
            {
                imp_->pos_success_count = 0;
            }

            return false;
        };

        auto velDeadZone = [&](double vel_, double vel_limit_)
        {
            if (vel_ >= vel_limit_)
            {
                vel_ = vel_limit_;
            }
            else if (vel_ <= -vel_limit_)
            {
                vel_ = -vel_limit_;
            }

        };



        for (int i = 0; i < 12; i++)
        {
            current_angle[i] = controller()->motorPool()[i].actualPos();
        }


        std::copy(current_angle + 6, current_angle + 12, current_sa_angle);




        if(!imp_->init)
        {

            getForceData(imp_->arm1_init_force, 0, imp_->init);
            getForceData(imp_->arm2_init_force, 1, imp_->init);
            imp_->start_count = count();
            imp_->init = true;

        }
        else
        {

            eeA1.getP(arm1_current_pos);
            eeA1.getMpm(arm1_current_pm);

            eeA2.getP(arm2_current_pos);
            eeA2.getMpm(arm2_current_pm);

            //Force Comp, Filtered, Transform
            getForceData(arm1_actual_force, 0, imp_->init);
            gc.getCompFT(arm1_current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, arm1_comp_force);

            getForceData(arm2_actual_force, 1, imp_->init);
            gc.getCompFT(arm2_current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, arm2_comp_force);

            for (size_t i = 0; i < 6; i++)
            {
                arm1_comp_force[i] = arm1_actual_force[i] + arm1_comp_force[i];
                arm2_comp_force[i] = arm2_actual_force[i] + arm2_comp_force[i];
            }

            forceFilter(arm1_comp_force, arm1_filtered_force, 0);
            forceFilter(arm2_comp_force, arm2_filtered_force, 1);

            //forceDeadZone(arm1_filtered_force, arm1_dead_zone);
            //forceDeadZone(arm2_filtered_force, arm2_dead_zone);

            forceTransform(arm1_filtered_force, arm1_transform_force, 0);
            forceTransform(arm2_filtered_force, arm2_transform_force, 1);

            for (size_t i = 0; i < 6; i++)
            {

                arm1_final_force[i] = arm1_transform_force[i] - imp_->arm1_start_force[i];
                arm2_final_force[i] = arm2_transform_force[i] - imp_->arm2_start_force[i];
            }


            //Phase 1 Contact
            if (!imp_->phase1)
            {

                if(count() == imp_->start_count + 2000)
                {


                    for (size_t i = 0; i < 6; i++)
                    {
                        imp_->arm1_start_force[i] = arm1_transform_force[i];
                        imp_->arm2_start_force[i] = arm2_transform_force[i];
                    }
                    mout()<<"Start Force Comp"<<std::endl;
                    mout()<<"A2 Start Force: "<<imp_->arm2_start_force[0]<<'\t'<<imp_->arm2_start_force[1]<<'\t'<<imp_->arm2_start_force[2]<<'\t'
                            <<imp_->arm2_start_force[3]<<'\t'<<imp_->arm2_start_force[4]<<'\t'<<imp_->arm2_start_force[5]<<std::endl;


                }


                double raw_force_checker[6]{ 0 };
                double comp_force_checker[6]{ 0 };
                double force_checker[6]{ 0 };

                double a2_pm[16]{ 0 };
                eeA2.getMpm(a2_pm);
                eeA2.getP(arm2_current_pos);

                if (count() % 1000 == 0)
                {
                    mout() << "force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                        << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                }


                //Arm1
                getForceData(raw_force_checker, 1, imp_->init);
                gc.getCompFT(a2_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force_checker);
                if (count() % 1000 == 0)
                {
                    mout() << "pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                }

                for (int i = 0; i < 6; i++)
                {
                    force_checker[i] = comp_force_checker[i] + raw_force_checker[i];
                    if (abs(force_checker[i]) >= 0.5)
                    {
                        imp_->phase1 = true;
                        imp_->start_count = 0;
                        mout() << "Contact Check" << std::endl;
                        mout() << "Contact Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                            << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                        mout() << "Contact force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                               << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
                        break;

		//Counter
        int contact_count = 0;
        int current_count = 0;

                    }

                }
                if (!imp_->phase1)
                {
                    arm2_current_pos[0] -= 0.0000045;
                    saMove(arm2_current_pos, model_a2, 1);
                }

            }
            //Phase 2 Hold Pos To Get 1s Data
            else if (imp_->phase1 && !imp_->phase2)
            {

                double acc[3]{ 0 };
                double dx[3]{ 0 };
                double dt = 0.001;

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(arm2_final_force[i]) > 25.0)
                    {
                        mout() << "Emergency Brake" << std::endl;
                        mout() << "Brake force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                               << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
                        return 0;
                    }
                }
                if (count() % 1000 == 0)
                {
                    mout() << "force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << "pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << std::endl;
                }

                if (posCheck(arm2_current_pos, 8))
                {
                    imp_->phase2 = true;
                    mout() << "Pos Complete" << std::endl;
                    mout() << "Complete Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                    mout() << "Complete force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                    imp_->complete_count = count();

                }
                else
                {

                    if(arm2_final_force[0] >= (imp_->phase2_fd[0] - 0.01) && arm2_final_force[0] <= (imp_->phase2_fd[0] + 2))
                    {
                        arm2_final_force[0] = imp_->phase2_fd[0];
                    }


                    //Impedence Controller
                    for (int i = 0; i < 1; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase2_fd[i] + arm2_final_force[i] - imp_->phase2_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase2_M[i];
                    }


                    for (int i = 0; i < 1; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        arm2_current_pos[i] = dx[i] + arm2_current_pos[i];

                    }


                    saMove(arm2_current_pos, model_a2, 1);
                }
            }
            //Phase 3 Get 1s Data to Rough Search
            else if (imp_->phase2 && !imp_->phase3)
            {
                if(count() <= imp_->complete_count + 1050)
                {
                    if(count() % 50 == 0 && imp_->accumulation_count < 20)
                    {
                        mout()<<"A1_Force"<<"\t"<<arm1_final_force[0]<<"\t"<<arm1_final_force[1]<<"\t"<<arm1_final_force[2]<<"\t"
                                <<arm1_final_force[3]<<"\t"<<arm1_final_force[4]<<"\t"<<arm1_final_force[5]<<"\t"<<"\t"<<"\t"<<"\t"
                               <<"A2_Force"<<"\t"<< arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                              << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;


                        for(int i = 0; i<6; i++)
                        {
                            imp_->arm1_temp_force[i] += arm1_final_force[i];
                            imp_->arm2_temp_force[i] += arm2_final_force[i];

                        }

                          imp_->accumulation_count ++;

                        //mout()<<"count: "<<imp_->accumulation_count<<std::endl;

                    }
                }
                else
                {
                    double arm1_avg_force[6]{0};
                    double arm2_avg_force[6]{0};

                    for(int i = 0; i<6; i++)
                    {
                        arm1_avg_force[i] = imp_->arm1_temp_force[i] / 30.0;
                        arm2_avg_force[i] = imp_->arm2_temp_force[i] / 30.0;
                    }


                    imp_-> dy = -0.0825 + ((imp_->a_y * arm2_avg_force[1] - arm2_avg_force[5] + imp_->c_y) / (imp_->b_y * arm2_avg_force[0]));
                    imp_-> dz = 0.0 - ((arm2_avg_force[4] - imp_->c_z + imp_->b_z * arm2_avg_force[2]) / (imp_->a_z * arm2_avg_force[0]));




                    mout()<<"Data Acquired! Current Point: "<< imp_->p_ << '\t' <<"Current Z Angle: "<< imp_->z_ << '\t' <<"Current Y Angle: "<< imp_->y_ <<std::endl;

                    mout()<<"A1_Force"<<"\t"<<arm1_avg_force[0]<<"\t"<<arm1_avg_force[1]<<"\t"<<arm1_avg_force[2]<<"\t"
                            <<arm1_avg_force[3]<<"\t"<<arm1_avg_force[4]<<"\t"<<arm1_avg_force[5]<<"\t"<<"\t"<<"\t"<<"\t"
                           <<"A2_Force"<<"\t"<<arm2_avg_force[0]<<"\t"<<arm2_avg_force[1]<<"\t"<<arm2_avg_force[2]<<"\t"
                           <<arm2_avg_force[3]<<"\t"<<arm2_avg_force[4]<<"\t"<<arm2_avg_force[5]<<std::endl;

                     mout()<<"Calculated Pos: " << imp_->dy << '\t' <<imp_->dz << std::endl;

                    imp_->back_count = count();
                    imp_->accumulation_count = 0;
                    imp_->phase3 = true;
                }

            }
            //Phase 4 Back & Move to Hole
            else if (imp_->phase3 && !imp_->phase4)
            {

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(arm2_final_force[i]) > 10.0)
                    {
                        mout() << "Emergency Brake" << std::endl;
                        mout() << "Brake force: " << arm2_transform_force[0] << '\t' << arm2_transform_force[1] << '\t' << arm2_transform_force[2] << '\t'
                            << arm2_transform_force[3] << '\t' << arm2_transform_force[4] << '\t' << arm2_transform_force[5] << std::endl;
                        return 0;
                    }
                }
                if (count() % 1000 == 0)
                {
                    mout()<<"Curret Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t'<< arm2_current_pos[2] << '\t'
                         << arm2_current_pos[3] << '\t'<< arm2_current_pos[4] << '\t'<< arm2_current_pos[5] << '\t'<< std::endl;
                }


                if(count() <= imp_->back_count + 1000)
                {
                    arm2_current_pos[0] += 0.00001;
                    saMove(arm2_current_pos, model_a2, 1);

                }
                else if (count() == imp_->back_count + 1001)
                {
                    eeA2.getP(imp_->hole_pos);
                    imp_->hole_pos[1] -= imp_->dy;
                    imp_->hole_pos[2] -= imp_->dz;

                    mout()<<"Hole Pos: " << imp_->hole_pos[0] << '\t' << imp_->hole_pos[1] << '\t'<< imp_->hole_pos[2] << '\t'
                         << imp_->hole_pos[3] << '\t'<< imp_->hole_pos[4] << '\t'<< imp_->hole_pos[5] << '\t'<< std::endl;

                    eeA2.setP(imp_->hole_pos);
                    if(model_a2.inverseKinematics())
                    {
                        mout()<<"Inverse Failed"<<std::endl;
                        return 0;
                    }
                    model_a2.getInputPos(imp_->hole_angle);

                }
                else if(count() > imp_->back_count + 1001)
                {

                    saJointMove(imp_->hole_angle, 1);
                    if (motorsPositionCheck(current_sa_angle, imp_->hole_angle, 6))
                    {

                        imp_->phase4 = true;
                        imp_->start_count = count();
                        mout()<<"Current Pos: "<< arm2_current_pos[0] <<'\t'<< arm2_current_pos[1] <<'\t' << arm2_current_pos[2]<<std::endl;
                        mout() << "Pos Complete !" << std::endl;
                    }
                }
            }
            //Phase 5 Contact
            else if (imp_->phase4 && !imp_->phase5)
            {
                if(count() == imp_->start_count + 200)
                {

                    for (size_t i = 0; i < 6; i++)
                    {
                        imp_->arm1_start_force[i] = arm1_transform_force[i];
                        imp_->arm2_start_force[i] = arm2_transform_force[i];
                    }
                    mout()<<"Start Force Comp"<<std::endl;
                    mout()<<"A2 Start Force: "<<imp_->arm2_start_force[0]<<'\t'<<imp_->arm2_start_force[1]<<'\t'<<imp_->arm2_start_force[2]<<'\t'
                            <<imp_->arm2_start_force[3]<<'\t'<<imp_->arm2_start_force[4]<<'\t'<<imp_->arm2_start_force[5]<<std::endl;

                }


                double raw_force_checker[6]{ 0 };
                double comp_force_checker[6]{ 0 };
                double force_checker[6]{ 0 };

                double a2_pm[16]{ 0 };
                eeA2.getMpm(a2_pm);
                eeA2.getP(arm2_current_pos);

                if (count() % 1000 == 0)
                {
                    mout() << "force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                }


                //Arm2
                getForceData(raw_force_checker, 1, imp_->init);
                gc.getCompFT(a2_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force_checker);
                if (count() % 1000 == 0)
                {
                    mout() << "pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                }

                for (int i = 0; i < 6; i++)
                {
                    force_checker[i] = comp_force_checker[i] + raw_force_checker[i];
                    if (abs(force_checker[i]) >= 0.5)
                    {
                        imp_->phase5 = true;
                        imp_->start_count = 0;
                        mout() << "Contact Check" << std::endl;
                        mout() << "Contact Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                            << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                        mout() << "Contact force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                            << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                        return 0;

                        break;

                    }

                }
                if (!imp_->phase5)
                {
                    arm2_current_pos[0] -= 0.0000045;
                    saMove(arm2_current_pos, model_a2, 1);
                }
            }


        }


        return 60000 - count();
    }
    Demo::Demo(const std::string& name)
{
        aris::core::fromXmlString(command(),
         "<Command name=\"demo\">"
         "	<GroupParam>"
         "	<Param name=\"z_degree\" default=\"0\" abbreviation=\"z\"/>"
         "	<Param name=\"y_degree\" default=\"0\" abbreviation=\"y\"/>"
         "	<Param name=\"point\" default=\"0\" abbreviation=\"p\"/>"
         "	</GroupParam>"
         "</Command>");
}
    Demo::~Demo() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(Demo)









    struct PegOutHole::Imp {

        //Flag
        bool init = false;

        //Arm1
        double arm1_init_force[6]{ 0 };
        double arm1_p_vector[6]{ 0 };
        double arm1_l_vector[6]{ 0 };

        //Arm2
        double arm2_init_force[6]{ 0 };
        double arm2_p_vector[6]{ 0 };
        double arm2_l_vector[6]{ 0 };

        //Switch Model
        int m_;

        //Current Vel
        double v_c[6]{ 0 };

        //Impedence Parameter
        double B[6]{ 2500,3500,3500,0,0,0 };
        double M[6]{ 100,100,100,0,0,0 };

        //Arm1 Force Buffer
        std::array<double, 10> arm1_force_buffer[6] = {};
        int arm1_buffer_index[6]{ 0 };

        //Arm2 Force Buffer
        std::array<double, 10> arm2_force_buffer[6] = {};
        int arm2_buffer_index[6]{ 0 };

        //Deadzone
        double deadzone[6]{3,3,3,0,0,0};

        //Desired Force
        double f_d[6]{0,0,0,0,0,0};
        double v_d[6]{0};

    };
    auto PegOutHole::prepareNrt() -> void
    {
        for (auto& m : motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

        GravComp gc;
        gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
        mout() << "Load P & L Vector" << std::endl;
        gc.loadInitForce(imp_->arm1_init_force, imp_->arm2_init_force);
        mout()<<"Load Init Force"<<std::endl;
    }
    auto PegOutHole::executeRT() -> int
    {
        //dual transform modelbase into multimodel
        auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
        //at(0) -> Arm1 -> white
        auto& arm1 = dualArm.subModels().at(0);
        //at(1) -> Arm2 -> blue
        auto& arm2 = dualArm.subModels().at(1);

		//transform to model
		auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
		auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);

		//End Effector
		auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
		auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));

		//ver 1.0 not limit on vel, only limit force
		static double tolerance = 0.00005;
		static double init_angle[12] =
		{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0 ,
		0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };
		//ver 2.0 limited vel, 20mm/s, 30deg/s
		static double max_vel[6]{ 0.00002,0.00002,0.00002,0.0005,0.0005,0.0005 };

        static double trigger_force[6]{ 0.5,0.5,0.5,0.0015,0.0015,0.0015 };

		static double trigger_vel[6]{ 0.000001,0.000001,0.000001,0.0001,0.0001,0.0001 };
        static double dead_zone[6]{ 0.1,0.1,0.1,0.0015,0.011,0.0015 };
        static double limit_area[6]{17,8,8,0.6,0.6,0.6};

        double p3_deadzone[6]{0,10,10,0,0,0};
        double p4_deadzone[6]{0,0,0,0.2,0.2,0.2};
        double p5_deadzone[6]{0,0,0,0.1,0.1,0.1};


		GravComp gc;

		double current_angle[12]{ 0 };
		double current_sa_angle[6]{ 0 };

		double comp_force[6]{ 0 };
		double current_pm[16]{ 0 };
		double current_pos[6]{ 0 };
		double current_force[6]{ 0 };
		double actual_force[6]{ 0 };
		double filtered_force[6]{ 0 };
		double transform_force[6]{ 0 };
        double limited_force[6]{ 0 };


		auto getForceData = [&](double* data_, int m_, bool init_)
		{

			int raw_force[6]{ 0 };

			for (std::size_t i = 0; i < 6; ++i)
			{
                if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
					mout() << "error" << std::endl;

				data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

			}

        GravComp gc;

        static double d_pos = 0.00001;

        double current_angle[12]{ 0 };

        double comp_force[6]{ 0 };
        double current_pm[16]{ 0 };
        double current_pos[6]{ 0 };
        double actual_force[6]{ 0 };
        double transform_force[6]{ 0 };
        static double limit_area[6]{15,15,15,0.6,0.6,0.6};
        static double max_vel[6]{ 0.00002,0.00002,0.00002,0.0005,0.0005,0.0005 };


        imp_->m_ = int32Param("model");



        auto getForceData = [&](double* data_, int m_, bool init_)
        {

            int raw_force[6]{ 0 };

            for (std::size_t i = 0; i < 6; ++i)
            {
                if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
                    mout() << "error" << std::endl;

                data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

            }

            if (!init_)
            {
                mout() << "Compensate Init Force" << std::endl;
            }
            else
            {
                if (m_ == 0)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

                    }
                }
                else if (m_ == 1)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

                    }
                }
                else
                {
                    mout() << "Wrong Model" << std::endl;
                }

            }

        };

        auto saMove = [&](double* pos_, aris::dynamic::Model& model_, int type_) {

            model_.setOutputPos(pos_);

            if (model_.inverseKinematics())
            {
                throw std::runtime_error("Inverse Kinematics Position Failed!");
            }


            double x_joint[6]{ 0 };

            model_.getInputPos(x_joint);

            if (type_ == 0)
            {
                for (std::size_t i = 0; i < 6; ++i)
                {
                    controller()->motorPool()[i].setTargetPos(x_joint[i]);
                }
            }
            else if (type_ == 1)
            {
                for (std::size_t i = 0; i < 6; ++i)
                {
                    controller()->motorPool()[i + 6].setTargetPos(x_joint[i]);
                }
            }
            else
            {
                throw std::runtime_error("Arm Type Error");
            }
        };

        auto forceTransform = [&](double* actual_force_, double* transform_force_, int m_)
        {
            if (m_ == 0)
            {
                transform_force_[0] = actual_force_[2];
                transform_force_[1] = -actual_force_[1];
                transform_force_[2] = actual_force_[0];

                transform_force_[3] = actual_force_[5];
                transform_force_[4] = -actual_force_[4];
                transform_force_[5] = actual_force_[3];
            }
            else if (m_ == 1)
            {
                transform_force_[0] = -actual_force_[2];
                transform_force_[1] = actual_force_[1];
                transform_force_[2] = actual_force_[0];

                transform_force_[3] = -actual_force_[5];
                transform_force_[4] = actual_force_[4];
                transform_force_[5] = actual_force_[3];
            }
            else
            {
                mout() << "Error Model In Force Transform" << std::endl;
            }
        };

        auto forceFilter = [&](double* actual_force_, double* filtered_force_, int m_)
        {
            if(m_ == 0)
            {
                for (int i = 0; i < 6; i++)
                {
                    imp_->arm1_force_buffer[i][imp_->arm1_buffer_index[i]] = actual_force_[i];
                    imp_->arm1_buffer_index[i] = (imp_->arm1_buffer_index[i] + 1) % 10;

                    filtered_force_[i] = std::accumulate(imp_->arm1_force_buffer[i].begin(), imp_->arm1_force_buffer[i].end(), 0.0) / 10;
                }
            }
            else if(m_ == 1)
            {
                for (int i = 0; i < 6; i++)
                {
                    imp_->arm2_force_buffer[i][imp_->arm2_buffer_index[i]] = actual_force_[i];
                    imp_->arm2_buffer_index[i] = (imp_->arm2_buffer_index[i] + 1) % 10;

                    filtered_force_[i] = std::accumulate(imp_->arm2_force_buffer[i].begin(), imp_->arm2_force_buffer[i].end(), 0.0) / 10;
                }
            }
            else
            {
                mout()<<"Wrong Filter!"<<std::endl;
            }

        };

        auto forceDeadZone = [&](double* actual_force_, double* area_)
        {
            for (int i = 0; i < 6; i++)
            {
                if (abs(actual_force_[i]) < area_[i])
                {
                    actual_force_[i] = 0;
                }
            }
        };

        auto forceUpperLimit = [&](double* actual_force_, double* area_)
        {
            for (int i = 0; i < 6; i++)
            {
                if (actual_force_[i] >= area_[i])
                {
                    actual_force_[i] = area_[i];
                }
                else if(actual_force_[i] <= -area_[i])
                {
                    actual_force_[i] = -area_[i];
                }
            }
        };

        auto velDeadZone = [&](double vel_, double vel_limit_)
        {
            if (vel_ >= vel_limit_)
            {
                vel_ = vel_limit_;
            }
            else if (vel_ <= -vel_limit_)
            {
                vel_ = -vel_limit_;
            }

        };


        for(int i = 0; i < 12; i++)
        {
            current_angle[i] = controller()->motorPool()[i].actualPos();
        }

        if(count() == 1)
        {
            dualArm.setInputPos(current_angle);
            if(dualArm.forwardKinematics()){mout()<<"Error"<<std::endl;}
            mout()<<"Init"<<std::endl;
        }

        double acc[3]{0};
        double dx[3]{0};

        double dt = 0.001;



        if(imp_->m_ == 0)
        {
            eeA1.getP(current_pos);
            eeA1.getMpm(current_pm);

             getForceData(actual_force, 0, true);
             gc.getCompFT(current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force);
             for (size_t i = 0; i < 6; i++)
             {
                comp_force[i] = actual_force[i] + comp_force[i];
             }

             forceTransform(comp_force, transform_force, 0);

            //Safety Check
            for (size_t i = 0; i < 3; i++)
            {
                if (abs(transform_force[i]) > 30)
                {
                    mout() << "Emergency Brake" << std::endl;
                    return 0;
                }
            }

            if (count() % 100 == 0)
            {
                mout() << "force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                    << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << '\t'<< "pos: " << current_pos[1] << '\t' << current_pos[2] <<std::endl;
            }


            current_pos[0] -= d_pos;


            forceUpperLimit(transform_force, limit_area);
            forceDeadZone(transform_force, imp_->deadzone);


            //Impedence Controller
            for (int i = 1; i < 3; i++)
            {
                // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                acc[i] = (-imp_->f_d[i] + transform_force[i] - imp_->B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->M[i];
            }


            for (int i = 1; i < 3; i++)
            {
                imp_->v_c[i] += acc[i] * dt;
                velDeadZone(imp_->v_c[i], max_vel[i]);
                dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                current_pos[i] = dx[i] + current_pos[i];

            }




            saMove(current_pos, model_a1, 0);
        }
        else if(imp_->m_ == 1)
        {
            eeA2.getP(current_pos);
            eeA2.getMpm(current_pm);

             getForceData(actual_force, 1, true);
             gc.getCompFT(current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force);
             for (size_t i = 0; i < 6; i++)
             {
                comp_force[i] = actual_force[i] + comp_force[i];
             }

             forceTransform(comp_force, transform_force, 1);

            //Safety Check
            for (size_t i = 0; i < 3; i++)
            {
                if (abs(transform_force[i]) > 30)
                {
                    mout() << "Emergency Brake" << std::endl;
                    return 0;
                }
            }

            if (count() % 100 == 0)
            {
                mout() << "force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                    << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << '\t'<< "pos: " << current_pos[1] << '\t' << current_pos[2] <<std::endl;
            }


            current_pos[0] += d_pos;


            forceUpperLimit(transform_force, limit_area);
            forceDeadZone(transform_force, imp_->deadzone);


            //Impedence Controller
            for (int i = 1; i < 3; i++)
            {
                // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                acc[i] = (-imp_->f_d[i] + transform_force[i] - imp_->B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->M[i];
            }


            for (int i = 1; i < 3; i++)
            {
                imp_->v_c[i] += acc[i] * dt;
                velDeadZone(imp_->v_c[i], max_vel[i]);
                dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                current_pos[i] = dx[i] + current_pos[i];

            }

            saMove(current_pos, model_a2, 1);
        }
        else
        {
            mout()<<"Wrong Input Model"<<std::endl;
            return 0;
        }




        return 5000 - count();
    }
    PegOutHole::PegOutHole(const std::string& name)
    {
        aris::core::fromXmlString(command(),
         "<Command name=\"m_po\">"
         "	<GroupParam>"
         "	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
         "	</GroupParam>"
         "</Command>");
    }
    PegOutHole::~PegOutHole() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(PegOutHole)



    struct Search::Imp {

        //Flag
        //Phase 1 -> Approach, Phase 2 -> Contact, Phase 3 -> Align, Phase 4 -> Fit, Phase 5 -> Insert
        bool init = false;
        bool stop = false;

        //Force Compensation Parameter
        double comp_f[6]{ 0 };

        //Arm1
        double arm1_init_force[6]{ 0 };
        double arm1_start_force[6]{ 0 };
        double arm1_p_vector[6]{ 0 };
        double arm1_l_vector[6]{ 0 };

        //Arm2
        double arm2_init_force[6]{ 0 };
        double arm2_start_force[6]{ 0 };
        double arm2_p_vector[6]{ 0 };
        double arm2_l_vector[6]{ 0 };

        //Desired Pos, Vel, Acc, Foc
        double v_d[6]{ 0 };

        //Desired Force of Each Phase
        double phase4_fd[6]{ 0,0,0,0,0,0 };

        //Current Vel
        double v_c[6]{ 0 };

        //Impedence Parameter
        double phase4_B[6]{ 6000,2500,2500,50,50,50 };
        double phase4_M[6]{ 100,100,100,10,10,10 };

        //Counter
        int search_start_count = 0;

        //Arm1 Force Buffer
        std::array<double, 10> arm1_force_buffer[6] = {};
        int arm1_buffer_index[6]{ 0 };

        //Arm2 Force Buffer
        std::array<double, 10> arm2_force_buffer[6] = {};
        int arm2_buffer_index[6]{ 0 };


        //Search Parameter
        double px[21] = {0,	0.00025, 0.00050, 0.00075, 0.001, 0.00125, 0.00150, 0.00175, 0.002,
        0.00225, 0.0025, 0.00275, 0.003, 0.00325, 0.0035, 0.00375, 0.004, 0.00425, 0.0045, 0.00475, 0.005,};

        double py[21] = {0, -0.0001443, 0.0002887, -0.000433, 0.0005774, -0.0007217, 0.000866, -0.0010104, 0.0011547, -0.001299, 0.0014434, -0.0019485,
        0.0024536, -0.0029587, 0.0034637, -0.0033072, 0.003, -0.0026339, 0.0021794, -0.0015612, 0};

        double force_direction[20]{0};

        double each_count[20] = {194, 300, 432, 570, 711, 854, 996, 1140, 1283, 1427, 1751, 2255, 2759, 3264, 3438, 3206, 2870, 2460, 1925, 841};


        int search_counter = 0;

        //Input Direction
        double y;
        double z;
        double theta;
        double search_pos[2]{0};


    };
    auto Search::prepareNrt() -> void
    {
        for (auto& m : motorOptions()) m =
                aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;


         GravComp gc;
         gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
         mout() << "Load P & L Vector" << std::endl;
    }
    auto Search::executeRT() -> int
    {
        //dual transform modelbase into multimodel
        auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
        //at(0) -> Arm1 -> white
        auto& arm1 = dualArm.subModels().at(0);
        //at(1) -> Arm2 -> blue
        auto& arm2 = dualArm.subModels().at(1);

        //transform to model
        auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
        auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);

        //End Effector
        auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
        auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));

        //ver 1.0 not limit on vel, only limit force
        static double tolerance = 0.00005;
        //ver 2.0 limited vel, 20mm/s, 30deg/s
        static double max_vel[6]{ 0.00002,0.00002,0.00002,0.0005,0.0005,0.0005 };
        static double arm2_dead_zone[6]{1.0, 1.0, 1.0, 0, 0, 0};


        GravComp gc;

        double current_angle[12]{ 0 };
        double current_sa_angle[6]{ 0 };

        double arm1_comp_force[6]{ 0 };
        double arm1_current_pm[16]{ 0 };
        double arm1_current_pos[6]{ 0 };
        double arm1_actual_force[6]{ 0 };
        double arm1_filtered_force[6]{ 0 };
        double arm1_transform_force[6]{ 0 };

        double arm2_comp_force[6]{ 0 };
        double arm2_current_pm[16]{ 0 };
        double arm2_current_pos[6]{ 0 };
        double arm2_actual_force[6]{ 0 };
        double arm2_filtered_force[6]{ 0 };
        double arm2_transform_force[6]{ 0 };


        auto daJointMove = [&](double target_mp_[12])
        {
            double current_angle[12] = { 0 };
            double move = 0.00005;

            for (int i = 0; i < 12; i++)
            {
                current_angle[i] = controller()->motorPool()[i].targetPos();
            }

            for (int i = 0; i < 12; i++)
            {
                if (current_angle[i] <= target_mp_[i] - move)
                {
                    controller()->motorPool()[i].setTargetPos(current_angle[i] + move);
                }
                else if (current_angle[i] >= target_mp_[i] + move)
                {
                    controller()->motorPool()[i].setTargetPos(current_angle[i] - move);
                }
            }
        };

        auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
        {
            for (int i = 0; i < dim_; i++)
            {
                if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
                {
                    return false;
                }
            }

            return true;
        };

        //single arm move 1-->white 2-->blue
        auto saJointMove = [&](double target_mp_[6], int m_)
        {
            double current_angle[12] = { 0 };
            double move = 0.00005;

            for (int i = 0; i < 12; i++)
            {
                current_angle[i] = controller()->motorPool()[i].targetPos();
            }

            for (int i = 0; i < 6; i++)
            {
                if (current_angle[i + 6 * m_] <= target_mp_[i] - move)
                {
                    controller()->motorPool()[i + 6 * m_].setTargetPos(current_angle[i + 6 * m_] + move);
                }
                else if (current_angle[i + 6 * m_] >= target_mp_[i] + move)
                {
                    controller()->motorPool()[i + 6 * m_].setTargetPos(current_angle[i + 6 * m_] - move);
                }
            }
        };

        auto saMove = [&](double* pos_, aris::dynamic::Model& model_, int type_) {

            model_.setOutputPos(pos_);

            if (model_.inverseKinematics())
            {
                mout() << "Pos: " << pos_[0] << '\t' << pos_[1] << '\t' << pos_[2] << '\t' << pos_[3] << '\t' << pos_[4] << '\t' << pos_[5] << std::endl;
                throw std::runtime_error("Inverse Kinematics Position Failed!");
            }


            double x_joint[6]{ 0 };

            model_.getInputPos(x_joint);

            if (type_ == 0)
            {
                for (std::size_t i = 0; i < 6; ++i)
                {
                    controller()->motorPool()[i].setTargetPos(x_joint[i]);
                }
            }
            else if (type_ == 1)
            {
                for (std::size_t i = 0; i < 6; ++i)
                {
                    controller()->motorPool()[i + 6].setTargetPos(x_joint[i]);
                }
            }
            else
            {
                throw std::runtime_error("Arm Type Error");
            }
        };


        auto getForceData = [&](double* data_, int m_, bool init_)
        {

            int raw_force[6]{ 0 };

            for (std::size_t i = 0; i < 6; ++i)
            {
                if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
                    mout() << "error" << std::endl;

                filtered_force_[i] = std::accumulate(imp_->force_buffer[i].begin(), imp_->force_buffer[i].end(), 0.0) / 20;
			}
		};

		auto forceDeadZone = [&](double* actual_force_, double* area_)
		{
			for (int i = 0; i < 6; i++)
			{
				if (abs(actual_force_[i]) < area_[i])
				{
					actual_force_[i] = 0;
				}
			}
		};

            if (!init_)
            {
                mout() << "Compensate Init Force" << std::endl;
            }
            else
            {
                if (m_ == 0)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

                    }
                }
                else if (m_ == 1)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

                    }
                }
                else
                {
                    mout() << "Wrong Model" << std::endl;
                }

            }

        };

        auto forceFilter = [&](double* actual_force_, double* filtered_force_, int m_)
        {
            if(m_ == 0)
            {
                for (int i = 0; i < 6; i++)
                {
                    imp_->arm1_force_buffer[i][imp_->arm1_buffer_index[i]] = actual_force_[i];
                    imp_->arm1_buffer_index[i] = (imp_->arm1_buffer_index[i] + 1) % 10;

                    filtered_force_[i] = std::accumulate(imp_->arm1_force_buffer[i].begin(), imp_->arm1_force_buffer[i].end(), 0.0) / 10;
                }
            }
            else if(m_ == 1)
            {
                for (int i = 0; i < 6; i++)
                {
                    imp_->arm2_force_buffer[i][imp_->arm2_buffer_index[i]] = actual_force_[i];
                    imp_->arm2_buffer_index[i] = (imp_->arm2_buffer_index[i] + 1) % 10;

                    filtered_force_[i] = std::accumulate(imp_->arm2_force_buffer[i].begin(), imp_->arm2_force_buffer[i].end(), 0.0) / 10;
                }
            }
            else
            {
                mout()<<"Wrong Filter!"<<std::endl;
            }

        };

        auto forceDeadZone = [&](double* actual_force_, double* area_)
        {
            for (int i = 0; i < 6; i++)
            {
                if (abs(actual_force_[i]) < area_[i])
                {
                    actual_force_[i] = 0;
                }
            }
        };

        auto forceTransform = [&](double* actual_force_, double* transform_force_, int m_)
        {
            if (m_ == 0)
            {
                transform_force_[0] = actual_force_[2];
                transform_force_[1] = -actual_force_[1];
                transform_force_[2] = actual_force_[0];

                transform_force_[3] = actual_force_[5];
                transform_force_[4] = -actual_force_[4];
                transform_force_[5] = actual_force_[3];
            }
            else if (m_ == 1)
            {
                transform_force_[0] = -actual_force_[2];
                transform_force_[1] = actual_force_[1];
                transform_force_[2] = actual_force_[0];

                transform_force_[3] = -actual_force_[5];
                transform_force_[4] = actual_force_[4];
                transform_force_[5] = actual_force_[3];
            }
            else
            {
                mout() << "Error Model In Force Transform" << std::endl;
            }
        };

        auto velDeadZone = [&](double vel_, double vel_limit_)
        {
            if (vel_ >= vel_limit_)
            {
                vel_ = vel_limit_;
            }
            else if (vel_ <= -vel_limit_)
            {
                vel_ = -vel_limit_;
            }

        };


        imp_->y = doubleParam("y");
        imp_->z = doubleParam("z");

        for (int i = 0; i < 12; i++)
        {
            current_angle[i] = controller()->motorPool()[i].actualPos();
        }


        std::copy(current_angle + 6, current_angle + 12, current_sa_angle);




        if (!imp_->init)
        {
            double assem_pos[6]{ -0.690, 0.012736, 0.289061, PI / 4, -PI / 2, - PI / 4 };
            double init_angle[6]{0};

            model_a2.setOutputPos(assem_pos);
            if(model_a2.inverseKinematics())
            {
                mout()<<"Error"<<std::endl;
            }

            model_a2.getInputPos(init_angle);

            if(count()%1000 == 0)
            {
                mout()<<"Init angle: "<<init_angle[0]<<'\t'<<init_angle[1]<<'\t'<<init_angle[2]<<'\t'
                        <<init_angle[3]<<'\t'<<init_angle[4]<<'\t'<<init_angle[5]<<std::endl;
                mout()<<"current angle: "<<current_sa_angle[0]<<'\t'<<current_sa_angle[1]<<'\t'<<current_sa_angle[2]<<'\t'
                        <<current_sa_angle[3]<<'\t'<<current_sa_angle[4]<<'\t'<<current_sa_angle[5]<<std::endl;
            }

            saJointMove(init_angle, 1);

            if(motorsPositionCheck(current_sa_angle, init_angle, 6))
            {
                getForceData(imp_->arm1_init_force, 0, imp_->init);
                getForceData(imp_->arm2_init_force, 1, imp_->init);

                gc.saveInitForce(imp_->arm1_init_force, imp_->arm2_init_force);
                master()->logFileRawName("Search");


                imp_->theta = std::atan2(imp_->z, imp_->y);

                for(int i = 0; i < 20; i++)
                {
                    imp_->force_direction[i] = std::atan2((imp_->py[i+1] - imp_->py[i]), (imp_->px[i+1] - imp_->px[i])) - imp_->theta;
                }

                mout() << "Init Complete" << std::endl;
                imp_->init = true;
            }


        }
        else
        {

            eeA1.getP(arm1_current_pos);
            eeA1.getMpm(arm1_current_pm);

            eeA2.getP(arm2_current_pos);
            eeA2.getMpm(arm2_current_pm);

            lout()<< "Current Pos: "<< '\t' << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                  << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;


            //Force Comp, Filtered, Transform
            getForceData(arm1_actual_force, 0, imp_->init);
            gc.getCompFT(arm1_current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, arm1_comp_force);

            getForceData(arm2_actual_force, 1, imp_->init);
            gc.getCompFT(arm2_current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, arm2_comp_force);

            for (size_t i = 0; i < 6; i++)
            {
                arm1_comp_force[i] = arm1_actual_force[i] + arm1_comp_force[i];
                arm2_comp_force[i] = arm2_actual_force[i] + arm2_comp_force[i];
            }

            forceFilter(arm1_comp_force, arm1_filtered_force, 0);
            forceFilter(arm2_comp_force, arm2_filtered_force, 1);

            //forceDeadZone(arm1_filtered_force, arm1_dead_zone);
            forceDeadZone(arm2_filtered_force, arm2_dead_zone);

            forceTransform(arm1_filtered_force, arm1_transform_force, 0);
            forceTransform(arm2_filtered_force, arm2_transform_force, 1);


            for (size_t i = 0; i < 3; i++)
            {
                if (abs(arm2_transform_force[i]) > 15.0)
                {
                    mout() << "Emergency Brake" << std::endl;
                    mout() << "Brake force: " << arm2_transform_force[0] << '\t' << arm2_transform_force[1] << '\t' << arm2_transform_force[2] << '\t'
                           << arm2_transform_force[3] << '\t' << arm2_transform_force[4] << '\t' << arm2_transform_force[5] << std::endl;
                    return 0;
                 }
             }

            double real_yz_force[2]{0};

            for(int i = 0; i < 2; i++)
            {
                real_yz_force[i] = arm2_transform_force[i+1];
            }

            double expected_force[2]{-5.0,0};

            eeA2.getP(arm2_current_pos);

            if(count() % 50 == 0)
            {
                mout()<<"Current pos: "<<arm2_current_pos[0]<<'\t'<<arm2_current_pos[1]<<'\t'<<arm2_current_pos[2]<<
                '\t'<<"Current force: "<<arm2_transform_force[0]<<'\t'<<arm2_transform_force[1]<<'\t'<<arm2_transform_force[2]<<std::endl;
            }


            if(imp_->search_counter < 20)
            {
                //Only yz movment
                if(imp_->search_start_count <= imp_->each_count[imp_->search_counter])
                {

                    double trans_z[4] = {cos(imp_->force_direction[imp_->search_counter]), -sin(imp_->force_direction[imp_->search_counter]),
                                        sin(imp_->force_direction[imp_->search_counter]), cos(imp_->force_direction[imp_->search_counter])};

                    double inv_z[4] = {cos(imp_->force_direction[imp_->search_counter]), sin(imp_->force_direction[imp_->search_counter]),
                                        -sin(imp_->force_direction[imp_->search_counter]), cos(imp_->force_direction[imp_->search_counter])};

                    double acc[2]{ 0 };
                    double dx[2]{ 0 };
                    double dt = 0.001;

                    double trans_dx[2]{0};
                    double trans_yz_force[2]{0};

                    aris::dynamic::s_mm(2,1,2,trans_z,real_yz_force,trans_yz_force);

                    //Impedence Controller
                    for (int i = 0; i < 2; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-expected_force[i] + trans_yz_force[i] - imp_->phase4_B[i+1] * (imp_->v_c[i+1] - imp_->v_d[i+1])) / imp_->phase4_M[i+1];
                    }


                    for (int i = 0; i < 2; i++)
                    {
                        imp_->v_c[i+1] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i+1], max_vel[i+1]);
                        dx[i] = imp_->v_c[i+1] * dt + acc[i] * dt * dt;

                    }

                    aris::dynamic::s_mm(2,1,2,inv_z,dx,trans_dx);

                    double acc_x = 0;
                    double dx_x = 0;

                    acc_x = (-imp_->phase4_fd[0] + arm2_transform_force[0] - imp_->phase4_B[0] * (imp_->v_c[0] - imp_->v_d[0])) / imp_->phase4_M[0];
                    imp_->v_c[0] += acc_x * dt;
                    velDeadZone(imp_->v_c[0], max_vel[0]);
                    dx_x = imp_->v_c[0] * dt + acc_x * dt * dt;

                    arm2_current_pos[0] += dx_x;

                    arm2_current_pos[1] += trans_dx[0];
                    arm2_current_pos[2] += trans_dx[1];

                    imp_->search_start_count++;


                }
                else
                {

                    for(int i = 0; i < 2; i++)
                    {
                        imp_->v_c[i+1] = 0;
                    }
                    imp_->search_counter++;
                    imp_->search_start_count = 0;
                    mout()<<"Search Counter: "<<imp_->search_counter<<std::endl;
                }


                saMove(arm2_current_pos, model_a2, 1);

            }
            else
            {
                mout()<<"Finish"<<std::endl;
                return 0;
            }


        }


        return 60000 - count();
    }
    Search::Search(const std::string& name)
    {
        aris::core::fromXmlString(command(),
            "<Command name=\"m_search\">"
            "	<GroupParam>"
            "	<Param name=\"y\" default=\"0\" abbreviation=\"y\"/>"
            "	<Param name=\"z\" default=\"0\" abbreviation=\"z\"/>"
            "	</GroupParam>"
            "</Command>");
    }
    Search::~Search() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(Search)


    struct Arm2PegInHole::Imp {

        //Flag
        //Phase 1 -> Approach, Phase 2 -> Contact, Phase 3 -> Align, Phase 4 -> Fit, Phase 5 -> Insert
        bool init = false;
        bool stop = false;
        bool phase1 = false;
        bool phase2 = false;
        bool phase3 = false;
        bool phase4 = false;
        bool phase5 = false;
        bool phase6 = false;


        //Force Compensation Parameter
        double comp_f[6]{ 0 };

        //Arm1
        double arm1_init_force[6]{ 0 };
        double arm1_start_force[6]{ 0 };
        double arm1_p_vector[6]{ 0 };
        double arm1_l_vector[6]{ 0 };

        double arm1_temp_force[6]{0};
        double arm1_temp_force2[6]{0};

        //Arm2
        double arm2_init_force[6]{ 0 };
        double arm2_start_force[6]{ 0 };
        double arm2_p_vector[6]{ 0 };
        double arm2_l_vector[6]{ 0 };

        double arm2_temp_force[6]{0};
        double arm2_temp_force2[6]{0};


        //Desired Force of Each Phase

        double phase3_fd[6]{ 5.0,0,0,0,0,0 };
        double phase4_fd[6]{ 3.5,0,0,0,0,0 };
        double phase5_fd[6]{ 3.5,0,0,0,0,0 };
        double phase6_fd[6]{ 7.5,0,0,0,0,0 };

        //Current Vel && Desired Vel
        double v_c[6]{ 0 };
        double v_d[6]{ 0 };

        //Impedence Parameter
        double phase3_B[6]{ 3000,2000,2000,0,0,0 };
        double phase3_M[6]{ 200,100,100,0,0,0 };

        //4.5 2.5
        double phase4_B[6]{ 3000,3500,3500,4.5,3.5,3.5 };
        double phase4_M[6]{ 150,100,100,2.5,1.0,1.0 };

        double phase5_B[6]{ 4000,3500,3500,0,0,0 };
        double phase5_M[6]{ 100,100,100,0,0,0 };

        double phase6_B[6]{ 3500,3500,3500,15,80,80 };
        double phase6_M[6]{ 100,100,100,10,5,5 };

        //Force Counter
        int allign_count = 0;
        int success_count = 0;

        //Counter for force comp
        int start_count = 0;

        //Pos Counter
        int pos_count = 0;
        int pos_success_count = 0;

        //Data Collect Counter
        int complete_count = 0;
        int back_count = 0;
        int accumulation_count = 0;

        double current_pos_checkek[6] = {0};

        //Parameters For Compensate
        double a_y = -0.0592;
        double b_y = -0.9943;
        double c_y = 0.0132;

        //Switch Angle
        double z_;
        double y_;

        //Arm1 Force Buffer
        std::array<double, 10> arm1_force_buffer[6] = {};
        int arm1_buffer_index[6]{ 0 };

        //Arm2 Force Buffer
        std::array<double, 10> arm2_force_buffer[6] = {};
        int arm2_buffer_index[6]{ 0 };

        //Dead Zone 8
        double p3_deadzone[6]{0.1,3,3,0,0,0};
        double p4_deadzone[6]{0,0,0,0.05,0.015,0.015};
        double p5_deadzone[6]{0.1,3,3,0,0,0};
        double p6_deadzone[6]{0.1,5,5,0.3,0.3,0.3};

        //2-Points Contact Pos
        double tpcontact_pos[6]{0};



    };
    auto Arm2PegInHole::prepareNrt() -> void
    {
        for (auto& m : motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

        GravComp gc;
        gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
        mout() << "Load P & L Vector" << std::endl;
    }
    auto Arm2PegInHole::executeRT() -> int
    {
        //dual transform modelbase into multimodel
        auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
        //at(0) -> Arm1 -> white
        auto& arm1 = dualArm.subModels().at(0);
        //at(1) -> Arm2 -> blue
        auto& arm2 = dualArm.subModels().at(1);

        //transform to model
        auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
        auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);

        //End Effector
        auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
        auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));

        //ver 1.0 not limit on vel, only limit force
        static double tolerance = 0.00005;
        //ver 2.0 limited vel, 20mm/s, 30deg/s
        static double max_vel[6]{ 0.00002,0.00002,0.00002,0.0005,0.0005,0.0005 };
        static double dead_zone[6]{ 0.1,0.1,0.1,0.0006,0.00920644,0.0006 };
        static double limit_area[6]{15,15,15,0.6,0.6,0.6};

        GravComp gc;

        double current_angle[12]{ 0 };
        double current_sa_angle[6]{ 0 };

        double arm1_comp_force[6]{ 0 };
        double arm1_current_pm[16]{ 0 };
        double arm1_current_pos[6]{ 0 };
        double arm1_actual_force[6]{ 0 };
        double arm1_filtered_force[6]{ 0 };
        double arm1_transform_force[6]{ 0 };
        double arm1_final_force[6]{0};

        double arm2_comp_force[6]{ 0 };
        double arm2_current_pm[16]{ 0 };
        double arm2_current_pos[6]{ 0 };
        double arm2_actual_force[6]{ 0 };
        double arm2_filtered_force[6]{ 0 };
        double arm2_transform_force[6]{ 0 };
        double arm2_final_force[6]{0};

        double mz_comp = 0;


        imp_->z_ = doubleParam("z_degree");

        imp_->y_ = doubleParam("y_degree");



        auto getForceData = [&](double* data_, int m_, bool init_)
        {

            int raw_force[6]{ 0 };

            for (std::size_t i = 0; i < 6; ++i)
            {
                if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
                    mout() << "error" << std::endl;

                data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

            }

            if (!init_)
            {
                mout() << "Compensate Init Force" << std::endl;
            }
            else
            {
                if (m_ == 0)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

                    }
                }
                else if (m_ == 1)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

                    }
                }
                else
                {
                    mout() << "Wrong Model" << std::endl;
                }

            }

            return false;
        };

        auto daJointMove = [&](double target_mp_[12])
        {
            double current_angle[12] = { 0 };
            double move = 0.00005;

            for (int i = 0; i < 12; i++)
            {
                current_angle[i] = controller()->motorPool()[i].targetPos();
            }

            for (int i = 0; i < 12; i++)
            {
                if (current_angle[i] <= target_mp_[i] - move)
                {
                    controller()->motorPool()[i].setTargetPos(current_angle[i] + move);
                }
                else if (current_angle[i] >= target_mp_[i] + move)
                {
                    controller()->motorPool()[i].setTargetPos(current_angle[i] - move);
                }
            }
        };

        auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
        {
            for (int i = 0; i < dim_; i++)
            {
                if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
                {
                    return false;
                }
            }

		};

        auto saJointMove = [&](double target_mp_[6], int m_)
        {
            double current_angle[12] = { 0 };
            double move = 0.00005;

            for (int i = 0; i < 12; i++)
            {
                current_angle[i] = controller()->motorPool()[i].targetPos();
            }

            for (int i = 0; i < 6; i++)
            {
                if (current_angle[i+6*m_] <= target_mp_[i] - move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] + move);
                }
                else if (current_angle[i+6*m_] >= target_mp_[i] + move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] - move);
                }
            }
        };

        auto saMove = [&](double* pos_, aris::dynamic::Model& model_, int type_) {

            model_.setOutputPos(pos_);

            if (model_.inverseKinematics())
            {
                throw std::runtime_error("Inverse Kinematics Position Failed!");
            }


            double x_joint[6]{ 0 };

            model_.getInputPos(x_joint);

            if (type_ == 0)
            {
                for (std::size_t i = 0; i < 6; ++i)
                {
                    controller()->motorPool()[i].setTargetPos(x_joint[i]);
                }
            }
            else if (type_ == 1)
            {
                for (std::size_t i = 0; i < 6; ++i)
                {
                    controller()->motorPool()[i + 6].setTargetPos(x_joint[i]);
                }
            }
            else
            {
                throw std::runtime_error("Arm Type Error");
            }
        };

        auto forceFilter = [&](double* actual_force_, double* filtered_force_, int m_)
        {
            if(m_ == 0)
            {
                for (int i = 0; i < 6; i++)
                {
                    imp_->arm1_force_buffer[i][imp_->arm1_buffer_index[i]] = actual_force_[i];
                    imp_->arm1_buffer_index[i] = (imp_->arm1_buffer_index[i] + 1) % 10;

                    filtered_force_[i] = std::accumulate(imp_->arm1_force_buffer[i].begin(), imp_->arm1_force_buffer[i].end(), 0.0) / 10;
                }
            }
            else if(m_ == 1)
            {
                for (int i = 0; i < 6; i++)
                {
                    imp_->arm2_force_buffer[i][imp_->arm2_buffer_index[i]] = actual_force_[i];
                    imp_->arm2_buffer_index[i] = (imp_->arm2_buffer_index[i] + 1) % 10;

                    filtered_force_[i] = std::accumulate(imp_->arm2_force_buffer[i].begin(), imp_->arm2_force_buffer[i].end(), 0.0) / 10;
                }
            }
            else
            {
                mout()<<"Wrong Filter!"<<std::endl;
            }

        };

        auto forceDeadZone = [&](double* actual_force_, double* area_)
        {
            for (int i = 0; i < 6; i++)
            {
                if (abs(actual_force_[i]) < area_[i])
                {
                    actual_force_[i] = 0;
                }
            }
        };

        auto forceTransform = [&](double* actual_force_, double* transform_force_, int m_)
        {
            if (m_ == 0)
            {
                transform_force_[0] = actual_force_[2];
                transform_force_[1] = -actual_force_[1];
                transform_force_[2] = actual_force_[0];

                transform_force_[3] = actual_force_[5];
                transform_force_[4] = -actual_force_[4];
                transform_force_[5] = actual_force_[3];
            }
            else if (m_ == 1)
            {
                transform_force_[0] = -actual_force_[2];
                transform_force_[1] = actual_force_[1];
                transform_force_[2] = actual_force_[0];

                transform_force_[3] = -actual_force_[5];
                transform_force_[4] = actual_force_[4];
                transform_force_[5] = actual_force_[3];
            }
            else
            {
                mout() << "Error Model In Force Transform" << std::endl;
            }
        };

        auto forceCheck = [&](double* current_force_, double* force_check_, int count_, int num_)
        {

            bool isValid = true;
            for (int i = 0; i < num_; i++)
            {
                if (abs(current_force_[i]) > force_check_[i])
                {
                    isValid = false;
                    break;
                }
            }

            if (isValid)
            {
                if (imp_->allign_count == 0) {
                    imp_->allign_count = count();
                }

                if ((count() - imp_->allign_count) % 50 == 0) {
                    imp_->success_count++;
                    mout() << "Check " << imp_->success_count << '\t' << "Current Count: " << count() << std::endl;
                    if (imp_->success_count >= count_)
                    {
                        imp_->success_count = 0;
                        imp_->allign_count = 0;
                        return true;
                    }
                }
            }
            else
            {
                imp_->success_count = 0;
            }

            return false;
        };

        auto posCheck = [&](double* current_pos_, int count_)
        {
            if(count()%500 == 0)
            {
                std::copy(current_pos_, current_pos_+6, imp_->current_pos_checkek);
                //mout()<<"Save Current Pos: "<<imp_->current_pos_checkek[0]<<std::endl;
            }

            bool isValid = true;

            for(int i = 0; i < 3; i++)
            {
                if (abs(current_pos_[i] - imp_->current_pos_checkek[i]) >= 0.000005)
                {
                    isValid = false;
                    break;
                }
            }

            if(isValid)
            {
                if(imp_->pos_count == 0)
                {
                    imp_->pos_count = count();
                }

                if((count() - imp_->pos_count) % 50 == 0)
                {
                    imp_->pos_success_count ++;
                    //mout() << "Check " << imp_->pos_success_count << '\t' << "Current Count: " << count() << std::endl;
                    if(imp_->pos_success_count >= count_)
                    {
                        imp_->pos_count = 0;
                        imp_->pos_success_count = 0;
                        return true;
                    }
                }
            }
            else
            {
                imp_->pos_success_count = 0;
            }

            return false;
        };

        auto velDeadZone = [&](double vel_, double vel_limit_)
        {
            if (vel_ >= vel_limit_)
            {
                vel_ = vel_limit_;
            }
            else if (vel_ <= -vel_limit_)
            {
                vel_ = -vel_limit_;
            }

        };

        auto forceUpperLimit = [&](double* actual_force_, double* area_)
        {
            for (int i = 0; i < 6; i++)
            {
                if (actual_force_[i] >= area_[i])
                {
                    actual_force_[i] = area_[i];
                }
                else if(actual_force_[i] <= -area_[i])
                {
                    actual_force_[i] = -area_[i];
                }
            }
        };

                double dx[3]{ 0 };
                double dth[3]{ 0 };
                double dt = 0.001;

                double desired_force[6]{ 0.2,0.2,0.2,0,0,0 };

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(transform_force[i]) > 30.0)
                    {
                        mout() << "Emergency Brake" << std::endl;

                        mout() << "Brake force Phase3: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                            << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;

                        return 0;
                    }
                }
                if (count() % 100 == 0)
                {
                    mout() << "force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                        << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << '\t' << '\t' <<"  pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
                        << current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
                }


                if (forceCheck(transform_force, desired_force, 5))
                {
                    imp_->phase4 = true;
                    mout() << "Allign Complete" << std::endl;
                    mout() << "Allign Pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
                        << current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
                    mout() << "Allign force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                        << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;

                }
                else
                {

                    if(limited_force[0] >= (imp_->phase3_fd[0] - 5.0) && limited_force[0] <= (imp_->phase3_fd[0] + 1.5))
                    {
                        limited_force[0] = imp_->phase3_fd[0];
                    }

                    forceDeadZone(limited_force, p4_deadzone);

//                    for(int i = 4; i<6; i++)
//                    {
//                        if(limited_force[i] > 0)
//                        {
//                            limited_force[i] -= 0.1;
//                        }
//                        else if(limited_force[i] < 0)
//                        {
//                            limited_force[i] += 0.1;
//                        }
//                    }


                    //Impedence Controller
                    for (int i = 0; i < 3; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase4_fd[i] + limited_force[i] - imp_->phase4_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase4_M[i];
                    }


        std::copy(current_angle + 6, current_angle + 12, current_sa_angle);




        if(!imp_->init)
        {
            double assem_pos[6]{ -0.690, 0.012466, 0.291196, PI / 4, -PI / 2, - PI / 4 };
            double init_angle[6]{0};

            model_a2.setOutputPos(assem_pos);
            if(model_a2.inverseKinematics())
            {
                mout()<<"Error"<<std::endl;
            }

            model_a2.getInputPos(init_angle);

            if(count()%1000 == 0)
            {
                mout()<<"Init angle: "<<init_angle[0]<<'\t'<<init_angle[1]<<'\t'<<init_angle[2]<<'\t'
                        <<init_angle[3]<<'\t'<<init_angle[4]<<'\t'<<init_angle[5]<<std::endl;
                mout()<<"current angle: "<<current_sa_angle[0]<<'\t'<<current_sa_angle[1]<<'\t'<<current_sa_angle[2]<<'\t'
                        <<current_sa_angle[3]<<'\t'<<current_sa_angle[4]<<'\t'<<current_sa_angle[5]<<std::endl;
            }

            saJointMove(init_angle, 1);

            if(motorsPositionCheck(current_sa_angle, init_angle, 6))
            {
                getForceData(imp_->arm1_init_force, 0, imp_->init);
                getForceData(imp_->arm2_init_force, 1, imp_->init);

                gc.saveInitForce(imp_->arm1_init_force, imp_->arm2_init_force);

                mout()<<"Init Complete"<<std::endl;

                imp_->init = true;
            }
        }
        else
        {

            eeA1.getP(arm1_current_pos);
            eeA1.getMpm(arm1_current_pm);

            eeA2.getP(arm2_current_pos);
            eeA2.getMpm(arm2_current_pm);

            //Force Comp, Filtered, Transform
            getForceData(arm1_actual_force, 0, imp_->init);
            gc.getCompFT(arm1_current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, arm1_comp_force);

            getForceData(arm2_actual_force, 1, imp_->init);
            gc.getCompFT(arm2_current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, arm2_comp_force);

            for (size_t i = 0; i < 6; i++)
            {
                arm1_comp_force[i] = arm1_actual_force[i] + arm1_comp_force[i];
                arm2_comp_force[i] = arm2_actual_force[i] + arm2_comp_force[i];
            }

            forceFilter(arm1_comp_force, arm1_filtered_force, 0);
            forceFilter(arm2_comp_force, arm2_filtered_force, 1);

            //forceDeadZone(arm1_filtered_force, arm1_dead_zone);
            //forceDeadZone(arm2_filtered_force, arm2_dead_zone);


            forceTransform(arm1_filtered_force, arm1_transform_force, 0);
            forceTransform(arm2_filtered_force, arm2_transform_force, 1);

            for (size_t i = 0; i < 6; i++)
            {

                arm1_final_force[i] = arm1_transform_force[i] - imp_->arm1_start_force[i];
                arm2_final_force[i] = arm2_transform_force[i] - imp_->arm2_start_force[i];
            }


            //Phase 1 Approach to The Hole
            if (!imp_->phase1)
            {
                //Tool
                double assem_pos[6]{ -0.690, 0.012466, 0.291196, PI / 4, -PI / 2, - PI / 4 };
                double assem_angle[6]{ 0 };
                double assem_rm[9]{ 0 };

                //Define Initial Rotate displacment

                double rotate_angle[3]{ 0, (imp_->y_)* 2 * PI / 360, (imp_->z_ )* 2 * PI / 360 };
                double rotate_rm[9]{ 0 };
                double desired_rm[9]{ 0 };

                aris::dynamic::s_ra2rm(rotate_angle, rotate_rm);
                aris::dynamic::s_re2rm(assem_pos + 3, assem_rm, "321");
                aris::dynamic::s_mm(3, 3, 3, rotate_rm, assem_rm, desired_rm);
                aris::dynamic::s_rm2re(desired_rm, assem_pos + 3, "321");

                eeA2.setP(assem_pos);


                if (model_a2.inverseKinematics())
                {
                    mout() << "Assem Pos Inverse Failed" << std::endl;
                }

                model_a2.getInputPos(assem_angle);

                saJointMove(assem_angle, 1);
                if (motorsPositionCheck(current_sa_angle, assem_angle, 6))
                {

                    imp_->phase1 = true;
                    imp_->start_count = count();

                    mout()<<"Current Angle: "<< rotate_angle[0] <<'\t'<< rotate_angle[1] <<'\t' << rotate_angle[2]<<std::endl;
                    mout()<<"Current Pos: "<< assem_pos[0] <<'\t'<< assem_pos[1] <<'\t' << assem_pos[2]<<std::endl;
                    mout() << "Assembly Start !" << std::endl;
                }

            }
            //Phase 2 Contact
            else if (imp_->phase1 && !imp_->phase2)
            {

                if(count() == imp_->start_count + 2000)
                {


                    for (size_t i = 0; i < 6; i++)
                    {
                        imp_->arm1_start_force[i] = arm1_transform_force[i];
                        imp_->arm2_start_force[i] = arm2_transform_force[i];
                    }
                    mout()<<"Start Force Comp"<<std::endl;
                    mout()<<"A2 Start Force: "<<imp_->arm2_start_force[0]<<'\t'<<imp_->arm2_start_force[1]<<'\t'<<imp_->arm2_start_force[2]<<'\t'
                            <<imp_->arm2_start_force[3]<<'\t'<<imp_->arm2_start_force[4]<<'\t'<<imp_->arm2_start_force[5]<<std::endl;


                }


                double raw_force_checker[6]{ 0 };
                double comp_force_checker[6]{ 0 };
                double force_checker[6]{ 0 };

                double a2_pm[16]{ 0 };
                eeA2.getMpm(a2_pm);
                eeA2.getP(arm2_current_pos);

                if (count() % 1000 == 0)
                {
                    mout() << "force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                        << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                }


                //Arm1
                getForceData(raw_force_checker, 1, imp_->init);
                gc.getCompFT(a2_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force_checker);
                if (count() % 1000 == 0)
                {
                    mout() << "pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                }

                for (int i = 0; i < 6; i++)
                {
                    force_checker[i] = comp_force_checker[i] + raw_force_checker[i];
                    if (abs(force_checker[i]) >= 0.5)
                    {
                        imp_->phase2 = true;
                        imp_->start_count = 0;
                        mout() << "Contact Check" << std::endl;
                        mout() << "Contact Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                            << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                        mout() << "Contact force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                               << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
                        break;


                    }

                }
                if (!imp_->phase2)
                {
                    arm2_current_pos[0] -= 0.0000035;
                    saMove(arm2_current_pos, model_a2, 1);
                }

            }
            //Phase 3 Maintain 2 Points Contact
            else if (imp_->phase2 && !imp_->phase3)
            {
                double acc[3]{ 0 };
                double dx[3]{ 0 };
                double dt = 0.001;

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(arm2_final_force[i]) > 15.0)
                    {
                        mout() << "Emergency Brake" << std::endl;
                        mout() << "Brake force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                               << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
                        return 0;
                    }
                }
                if (count() % 100 == 0)
                {
                    mout() << "force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << "pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << std::endl;
                }

                if (posCheck(arm2_current_pos, 10))
                {
                    imp_->phase3 = true;
                    mout() << "Pos 3 Complete" << std::endl;
                    mout() << "Complete Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                    mout() << "Complete force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                    imp_->complete_count = count();
                    eeA2.getP(imp_->tpcontact_pos);

                    for(int i = 0; i<6; i++)
                    {
                        imp_->v_c[i] = 0;
                    }

                    //return 0;

                }
                else
                {


                    forceUpperLimit(arm2_final_force, limit_area);
                    forceDeadZone(arm2_final_force, imp_->p3_deadzone);

                    if(arm2_final_force[0] >= (imp_->phase3_fd[0] - 2.5) && arm2_final_force[0] <= (imp_->phase3_fd[0] + 5.0))
                    {
                        arm2_final_force[0] = imp_->phase3_fd[0];
                    }




                    //Impedence Controller
                    for (int i = 0; i < 3; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase3_fd[i] + arm2_final_force[i] - imp_->phase3_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase3_M[i];
                    }


                    for (int i = 0; i < 3; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        arm2_current_pos[i] = dx[i] + arm2_current_pos[i];

                    }


                    saMove(arm2_current_pos, model_a2, 1);
                }

            }
            //Phase 4 allign
            else if (imp_->phase3 && !imp_->phase4)
            {

                double acc[3]{ 0 };
                double ome[3]{ 0 };

                double dx[3]{ 0 };
                double dth[3]{ 0 };
                double dt = 0.001;

                double depth = 0;

                double desired_force[6]{ 1.0,1.0,1.0,0,0,0 };

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(arm2_final_force[i]) > 20.0)
                    {
                        mout() << "Emergency Brake" << std::endl;

                        mout() << "Brake force Phase3: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                            << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                        return 0;
                    }
                }
//                if (count() % 100 == 0)
//                {
//                    mout() << "force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
//                        << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << '\t' << '\t' <<"  pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
//                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
//                }

//                if (count() % 100 == 0)
//                {
//                    mout() << "force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
//                        << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5]  << std::endl;
//                }

                depth = abs(arm2_current_pos[0] - imp_->tpcontact_pos[0]);

                if (depth >= 0.005 && forceCheck(arm2_final_force, desired_force, 5, 3))
                {
                    imp_->phase4 = true;
                    mout() << "Allign Complete" << std::endl;
                    mout() << "Allign Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                    mout() << "Allign force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                        << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                    mout() << "Insert Depth: " << depth << std::endl;


                    //return 0;

                }
                else
                {


                    //rz Compensation


                    mz_comp = imp_->a_y*arm2_final_force[1]-imp_->b_y*0.0825*arm2_final_force[0]+imp_->c_y;
                    arm2_final_force[5] -= mz_comp;

                    if (count() % 100 == 0)
                    {
                        mout() << "compf: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                            << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
                    }

                    forceUpperLimit(arm2_final_force, limit_area);
                    forceDeadZone(arm2_final_force, imp_->p4_deadzone);

//                    if (count() % 100 == 0)
//                    {
//                        mout() << "compf: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
//                            << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
//                    }


                    if(arm2_final_force[0] >= (imp_->phase4_fd[0] - 2) && arm2_final_force[0] <= (imp_->phase4_fd[0] + 5.5))
                    {
                        arm2_final_force[0] = imp_->phase4_fd[0];
                    }


                    //Impedence Controller
                    for (int i = 0; i < 3; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase4_fd[i] + arm2_final_force[i] - imp_->phase4_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase4_M[i];
                    }


                    for (int i = 0; i < 3; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        arm2_current_pos[i] = dx[i] + arm2_current_pos[i];

                    }

                    //pose
                    for (int i = 1; i < 3; i++)
                    {
                        // Caculate Omega
                        ome[i] = (-imp_->phase4_fd[i + 3] + arm2_final_force[i + 3] - imp_->phase4_B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3])) / imp_->phase4_M[i + 3];
                    }

        //Switch Point
        int p_;

                    for (int i = 1; i < 3; i++)
                    {
                        // Angluar Velocity
                        imp_->v_c[i + 3] += ome[i] * dt;
                        velDeadZone(imp_->v_c[i + 3], max_vel[i + 3]);
                        dth[i] = imp_->v_c[i + 3] * dt;
                    }

                    double drm[9]{ 0 };
                    double rm_target[9]{ 0 };
                    double rm_c[9]{ 0 };

                    //Transform to rm
                    aris::dynamic::s_ra2rm(dth, drm);

                    //Current pe to rm
                    aris::dynamic::s_re2rm(arm2_current_pos + 3, rm_c, "321");

                    //Calcuate Future rm
                    aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);

                    //Convert rm to pe
                    aris::dynamic::s_rm2re(rm_target, arm2_current_pos + 3, "321");

        //ver 1.0 not limit on vel, only limit force
        static double tolerance = 0.00005;
        //ver 2.0 limited vel, 20mm/s, 30deg/s
        static double max_vel[6]{ 0.00002,0.00002,0.00002,0.0005,0.0005,0.0005 };
        static double dead_zone[6]{ 0.1,0.1,0.1,0.0006,0.00920644,0.0006 };

//                    mout()<<"desired pos " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
//                         << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;

                    saMove(arm2_current_pos, model_a2, 1);

                }
            }
            //Phase 5 Maintain Contact
            else if(imp_->phase4 && !imp_->phase5)
            {
                double acc[3]{ 0 };
                double dx[3]{ 0 };
                double dt = 0.001;

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(arm2_final_force[i]) > 20.0)
                    {
                        mout() << "Emergency Brake" << std::endl;
                        mout() << "Brake force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                               << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
                        return 0;
                    }
                }
                if (count() % 100 == 0)
                {
                    mout() << "force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << "pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << std::endl;
                }

                if (posCheck(arm2_current_pos, 10))
                {
                    imp_->phase5 = true;
                    mout() << "Pos 5 Complete" << std::endl;
                    mout() << "Complete Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                    mout() << "Complete force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                    imp_->complete_count = count();
                    eeA2.getP(imp_->tpcontact_pos);

                    for(int i = 0; i<6; i++)
                    {
                        imp_->v_c[i] = 0;
                    }

                    //return 0;

                }
                else
                {


                    forceUpperLimit(arm2_final_force, limit_area);
                    forceDeadZone(arm2_final_force, imp_->p5_deadzone);

                    if(arm2_final_force[0] >= (imp_->phase5_fd[0] - 2.0) && arm2_final_force[0] <= (imp_->phase5_fd[0] + 10.0))
                    {
                        arm2_final_force[0] = imp_->phase5_fd[0];
                    }




                    //Impedence Controller
                    for (int i = 0; i < 3; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase5_fd[i] + arm2_final_force[i] - imp_->phase5_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase5_M[i];
                    }


                    for (int i = 0; i < 3; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        arm2_current_pos[i] = dx[i] + arm2_current_pos[i];

                    }


                    saMove(arm2_current_pos, model_a2, 1);
                }
            }
            //Phase 6 Allign
            else if(imp_->phase5 && !imp_->phase6)
            {
                double acc[3]{ 0 };
                double ome[3]{ 0 };

                double dx[3]{ 0 };
                double dth[3]{ 0 };
                double dt = 0.001;

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(arm2_final_force[i]) > 25.0)
                    {
                        mout() << "Emergency Brake" << std::endl;

                        mout() << "Brake force Phase3: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                            << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                        return 0;
                    }
                }

                if (abs(arm2_final_force[4])<0.3 && abs(arm2_final_force[5])<0.3 && posCheck(arm2_current_pos, 10))
                {
                    imp_->phase6 = true;
                    mout() << "Allign Complete" << std::endl;
                    mout() << "Allign Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                    mout() << "Allign force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                        << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                    return 0;

                }
                else
                {

                    if (count() % 50 == 0)
                    {
                        mout() << "force: "<< '\t' << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                            << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << '\t'
                            << "Pos: "<<'\t'<< arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] <<std::endl;
                    }

                    forceUpperLimit(arm2_final_force, limit_area);
                    forceDeadZone(arm2_final_force, imp_->p6_deadzone);


                    if(arm2_final_force[0] >= (imp_->phase6_fd[0] - 2.5) && arm2_final_force[0] <= (imp_->phase6_fd[0] + 5.5))
                    {
                        arm2_final_force[0] = imp_->phase6_fd[0];
                    }


                    //Impedence Controller
                    for (int i = 0; i < 3; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase6_fd[i] + arm2_final_force[i] - imp_->phase6_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase6_M[i];
                    }


                    for (int i = 0; i < 3; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        arm2_current_pos[i] = dx[i] + arm2_current_pos[i];

                    }

                    //pose
                    for (int i = 1; i < 3; i++)
                    {
                        // Caculate Omega
                        ome[i] = (-imp_->phase6_fd[i + 3] + arm2_final_force[i + 3] - imp_->phase6_B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3])) / imp_->phase6_M[i + 3];
                    }


                    for (int i = 1; i < 3; i++)
                    {
                        // Angluar Velocity
                        imp_->v_c[i + 3] += ome[i] * dt;
                        velDeadZone(imp_->v_c[i + 3], max_vel[i + 3]);
                        dth[i] = imp_->v_c[i + 3] * dt;
                    }

                    double drm[9]{ 0 };
                    double rm_target[9]{ 0 };
                    double rm_c[9]{ 0 };

                    //Transform to rm
                    aris::dynamic::s_ra2rm(dth, drm);

                    //Current pe to rm
                    aris::dynamic::s_re2rm(arm2_current_pos + 3, rm_c, "321");

                    //Calcuate Future rm
                    aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);

                    //Convert rm to pe
                    aris::dynamic::s_rm2re(rm_target, arm2_current_pos + 3, "321");

                    saMove(arm2_current_pos, model_a2, 1);

                }

            }

        }


        return 150000 - count();
    }
    Arm2PegInHole::Arm2PegInHole(const std::string& name)
{
        aris::core::fromXmlString(command(),
         "<Command name=\"a2ph\">"
         "	<GroupParam>"
         "	<Param name=\"z_degree\" default=\"0\" abbreviation=\"z\"/>"
         "	<Param name=\"y_degree\" default=\"0\" abbreviation=\"y\"/>"
         "	</GroupParam>"
         "</Command>");
}
    Arm2PegInHole::~Arm2PegInHole() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(Arm2PegInHole)


    struct PlateAllignData::Imp {

        //Flag
        //Phase 1 -> Approach, Phase 2 -> Contact, Phase 3 -> Align, Phase 4 -> Fit, Phase 5 -> Insert
        bool init = false;
        bool stop = false;
        bool phase1 = false;
        bool phase2 = false;
        bool phase3 = false;
        bool phase4 = false;
        bool phase5 = false;


        //Force Compensation Parameter
        double comp_f[6]{ 0 };

        //Arm1
        double arm1_init_force[6]{ 0 };
        double arm1_start_force[6]{ 0 };
        double arm1_p_vector[6]{ 0 };
        double arm1_l_vector[6]{ 0 };

        double arm1_temp_force[6]{0};
        double arm1_temp_force2[6]{0};

        //Arm2
        double arm2_init_force[6]{ 0 };
        double arm2_start_force[6]{ 0 };
        double arm2_p_vector[6]{ 0 };
        double arm2_l_vector[6]{ 0 };

        double arm2_temp_force[6]{0};
        double arm2_temp_force2[6]{0};


        //Desired Force of Each Phase

        double phase3_fd[6]{ 3.5,0,0,0,0,0 };
        double phase4_fd[6]{ 3.5,0,0,0,0,0 };
        double phase5_fd[6]{ 3.5,0,0,0,0,0 };
        double phase6_fd[6]{ 3.5,0,0,0,0,0 };

        //Current Vel && Desired Vel
        double v_c[6]{ 0 };
        double v_d[6]{ 0 };

        //Impedence Parameter
        double phase3_B[6]{ 4000,3300,3300,0,0,0 };
        double phase3_M[6]{ 100,100,100,0,0,0 };

        //4.5 2.5
        double phase4_B[6]{ 3000,3500,3500,4.5,3.5,3.5 };
        double phase4_M[6]{ 150,100,100,2.5,1.0,1.0 };

        double phase5_B[6]{3000, 3300, 3300, 45, 45, 45};
        double phase5_M[6]{100, 100, 100, 25, 25, 25};

        double phase6_B[6]{3000, 3300, 3300, 45, 45, 45};
        double phase6_M[6]{100, 100, 100, 25, 25, 25};

        //Force Counter
        int allign_count = 0;
        int success_count = 0;

        //Counter for force comp
        int start_count = 0;

        //Pos Counter
        int pos_count = 0;
        int pos_success_count = 0;

        //Data Collect Counter
        int complete_count = 0;
        int back_count = 0;
        int accumulation_count = 0;

        double current_pos_checkek[6] = {0};

        //Parameters For Compensate
        double a_y = -0.0592;
        double b_y = -0.9943;
        double c_y = 0.0132;

        //Switch Angle
        double z_;
        double y_;

        //Arm1 Force Buffer
        std::array<double, 10> arm1_force_buffer[6] = {};
        int arm1_buffer_index[6]{ 0 };

        //Arm2 Force Buffer
        std::array<double, 10> arm2_force_buffer[6] = {};
        int arm2_buffer_index[6]{ 0 };

        //Dead Zone 8
        double p3_deadzone[6]{0.1,3,3,0,0,0};
        double p4_deadzone[6]{0,0,0,0.05,0.015,0.015};
        double p5_deadzone[6]{0,0,0,0.05,0.015,0.015};
        double p6_deadzone[6]{0,0,0,0.05,0.015,0.015};

        //2-Points Contact Pos
        double tpcontact_pos[6]{0};



    };
    auto PlateAllignData::prepareNrt() -> void
    {
        for (auto& m : motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

        GravComp gc;
        gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
        mout() << "Load P & L Vector" << std::endl;
    }
    auto PlateAllignData::executeRT() -> int
    {
        //dual transform modelbase into multimodel
        auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
        //at(0) -> Arm1 -> white
        auto& arm1 = dualArm.subModels().at(0);
        //at(1) -> Arm2 -> blue
        auto& arm2 = dualArm.subModels().at(1);

        //transform to model
        auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
        auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);

        //End Effector
        auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
        auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));

        //ver 1.0 not limit on vel, only limit force
        static double tolerance = 0.00005;
        //ver 2.0 limited vel, 20mm/s, 30deg/s
        static double max_vel[6]{ 0.00002,0.00002,0.00002,0.0005,0.0005,0.0005 };
        static double dead_zone[6]{ 0.1,0.1,0.1,0.0006,0.00920644,0.0006 };
        static double limit_area[6]{15,15,15,0.6,0.6,0.6};

        GravComp gc;

        double current_angle[12]{ 0 };
        double current_sa_angle[6]{ 0 };

        double arm1_comp_force[6]{ 0 };
        double arm1_current_pm[16]{ 0 };
        double arm1_current_pos[6]{ 0 };
        double arm1_actual_force[6]{ 0 };
        double arm1_filtered_force[6]{ 0 };
        double arm1_transform_force[6]{ 0 };
        double arm1_final_force[6]{0};

        double arm2_comp_force[6]{ 0 };
        double arm2_current_pm[16]{ 0 };
        double arm2_current_pos[6]{ 0 };
        double arm2_actual_force[6]{ 0 };
        double arm2_filtered_force[6]{ 0 };
        double arm2_transform_force[6]{ 0 };
        double arm2_final_force[6]{0};

        double mz_comp = 0;


        imp_->z_ = doubleParam("z_degree");

        imp_->y_ = doubleParam("y_degree");



        auto getForceData = [&](double* data_, int m_, bool init_)
        {

            int raw_force[6]{ 0 };

            for (std::size_t i = 0; i < 6; ++i)
            {
                if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
                    mout() << "error" << std::endl;

                data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

            }

            if (!init_)
            {
                mout() << "Compensate Init Force" << std::endl;
            }
            else
            {
                if (m_ == 0)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

                    }
                }
                else if (m_ == 1)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

                    }
                }
                else
                {
                    mout() << "Wrong Model" << std::endl;
                }

            }

        };

        auto daJointMove = [&](double target_mp_[12])
        {
            double current_angle[12] = { 0 };
            double move = 0.00005;

            for (int i = 0; i < 12; i++)
            {
                current_angle[i] = controller()->motorPool()[i].targetPos();
            }

            for (int i = 0; i < 12; i++)
            {
                if (current_angle[i] <= target_mp_[i] - move)
                {
                    controller()->motorPool()[i].setTargetPos(current_angle[i] + move);
                }
                else if (current_angle[i] >= target_mp_[i] + move)
                {
                    controller()->motorPool()[i].setTargetPos(current_angle[i] - move);
                }
            }
        };

        auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
        {
            for (int i = 0; i < dim_; i++)
            {
                if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
                {
                    return false;
                }
            }

            return true;
        };

        auto saJointMove = [&](double target_mp_[6], int m_)
        {
            double current_angle[12] = { 0 };
            double move = 0.00005;

            for (int i = 0; i < 12; i++)
            {
                current_angle[i] = controller()->motorPool()[i].targetPos();
            }

            for (int i = 0; i < 6; i++)
            {
                if (current_angle[i+6*m_] <= target_mp_[i] - move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] + move);
                }
                else if (current_angle[i+6*m_] >= target_mp_[i] + move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] - move);
                }
            }
        };

        auto saMove = [&](double* pos_, aris::dynamic::Model& model_, int type_) {

            model_.setOutputPos(pos_);

            if (model_.inverseKinematics())
            {
                throw std::runtime_error("Inverse Kinematics Position Failed!");
            }


            double x_joint[6]{ 0 };

            model_.getInputPos(x_joint);

            if (type_ == 0)
            {
                for (std::size_t i = 0; i < 6; ++i)
                {
                    controller()->motorPool()[i].setTargetPos(x_joint[i]);
                }
            }
            else if (type_ == 1)
            {
                for (std::size_t i = 0; i < 6; ++i)
                {
                    controller()->motorPool()[i + 6].setTargetPos(x_joint[i]);
                }
            }
            else
            {
                throw std::runtime_error("Arm Type Error");
            }
        };

        auto forceFilter = [&](double* actual_force_, double* filtered_force_, int m_)
        {
            if(m_ == 0)
            {
                for (int i = 0; i < 6; i++)
                {
                    imp_->arm1_force_buffer[i][imp_->arm1_buffer_index[i]] = actual_force_[i];
                    imp_->arm1_buffer_index[i] = (imp_->arm1_buffer_index[i] + 1) % 10;

                    filtered_force_[i] = std::accumulate(imp_->arm1_force_buffer[i].begin(), imp_->arm1_force_buffer[i].end(), 0.0) / 10;
                }
            }
            else if(m_ == 1)
            {
                for (int i = 0; i < 6; i++)
                {
                    imp_->arm2_force_buffer[i][imp_->arm2_buffer_index[i]] = actual_force_[i];
                    imp_->arm2_buffer_index[i] = (imp_->arm2_buffer_index[i] + 1) % 10;

                    filtered_force_[i] = std::accumulate(imp_->arm2_force_buffer[i].begin(), imp_->arm2_force_buffer[i].end(), 0.0) / 10;
                }
            }
            else
            {
                mout()<<"Wrong Filter!"<<std::endl;
            }

        };

        auto forceDeadZone = [&](double* actual_force_, double* area_)
        {
            for (int i = 0; i < 6; i++)
            {
                if (abs(actual_force_[i]) < area_[i])
                {
                    actual_force_[i] = 0;
                }
            }
        };

        auto forceTransform = [&](double* actual_force_, double* transform_force_, int m_)
        {
            if (m_ == 0)
            {
                transform_force_[0] = actual_force_[2];
                transform_force_[1] = -actual_force_[1];
                transform_force_[2] = actual_force_[0];

                transform_force_[3] = actual_force_[5];
                transform_force_[4] = -actual_force_[4];
                transform_force_[5] = actual_force_[3];
            }
            else if (m_ == 1)
            {
                transform_force_[0] = -actual_force_[2];
                transform_force_[1] = actual_force_[1];
                transform_force_[2] = actual_force_[0];

                transform_force_[3] = -actual_force_[5];
                transform_force_[4] = actual_force_[4];
                transform_force_[5] = actual_force_[3];
            }
            else
            {
                mout() << "Error Model In Force Transform" << std::endl;
            }
        };

        auto forceCheck = [&](double* current_force_, double* force_check_, int count_, int num_)
        {

            bool isValid = true;
            for (int i = 0; i < num_; i++)
            {
                if (abs(current_force_[i]) > force_check_[i])
                {
                    isValid = false;
                    break;
                }
            }

            if (isValid)
            {
                if (imp_->allign_count == 0) {
                    imp_->allign_count = count();
                }

                if ((count() - imp_->allign_count) % 50 == 0) {
                    imp_->success_count++;
                    mout() << "Check " << imp_->success_count << '\t' << "Current Count: " << count() << std::endl;
                    if (imp_->success_count >= count_)
                    {
                        imp_->success_count = 0;
                        imp_->allign_count = 0;
                        return true;
                    }
                }
            }
            else
            {
                imp_->success_count = 0;
            }

            return false;
        };

        auto posCheck = [&](double* current_pos_, int count_)
        {
            if(count()%500 == 0)
            {
                std::copy(current_pos_, current_pos_+6, imp_->current_pos_checkek);
                //mout()<<"Save Current Pos: "<<imp_->current_pos_checkek[0]<<std::endl;
            }

            bool isValid = true;

            for(int i = 0; i < 3; i++)
            {
                if (abs(current_pos_[i] - imp_->current_pos_checkek[i]) >= 0.000005)
                {
                    isValid = false;
                    break;
                }
            }

            if(isValid)
            {
                if(imp_->pos_count == 0)
                {
                    imp_->pos_count = count();
                }

                if((count() - imp_->pos_count) % 50 == 0)
                {
                    imp_->pos_success_count ++;
                    //mout() << "Check " << imp_->pos_success_count << '\t' << "Current Count: " << count() << std::endl;
                    if(imp_->pos_success_count >= count_)
                    {
                        imp_->pos_count = 0;
                        imp_->pos_success_count = 0;
                        return true;
                    }
                }
            }
            else
            {
                imp_->pos_success_count = 0;
            }

            return false;
        };

        auto velDeadZone = [&](double vel_, double vel_limit_)
        {
            if (vel_ >= vel_limit_)
            {
                vel_ = vel_limit_;
            }
            else if (vel_ <= -vel_limit_)
            {
                vel_ = -vel_limit_;
            }

        };

        auto forceUpperLimit = [&](double* actual_force_, double* area_)
        {
            for (int i = 0; i < 6; i++)
            {
                if (actual_force_[i] >= area_[i])
                {
                    actual_force_[i] = area_[i];
                }
                else if(actual_force_[i] <= -area_[i])
                {
                    actual_force_[i] = -area_[i];
                }
            }
        };

        for (int i = 0; i < 12; i++)
        {
            current_angle[i] = controller()->motorPool()[i].actualPos();
        }


        std::copy(current_angle + 6, current_angle + 12, current_sa_angle);




        if(!imp_->init)
        {
            double assem_pos[6]{ -0.710, 0.012466, 0.291196, 0.0142609, -1.56474, 0.0  };
            double init_angle[6]{0};

            model_a2.setOutputPos(assem_pos);
            if(model_a2.inverseKinematics())
            {
                mout()<<"Error"<<std::endl;
            }

            model_a2.getInputPos(init_angle);

            if(count()%1000 == 0)
            {
                mout()<<"Init angle: "<<init_angle[0]<<'\t'<<init_angle[1]<<'\t'<<init_angle[2]<<'\t'
                        <<init_angle[3]<<'\t'<<init_angle[4]<<'\t'<<init_angle[5]<<std::endl;
                mout()<<"current angle: "<<current_sa_angle[0]<<'\t'<<current_sa_angle[1]<<'\t'<<current_sa_angle[2]<<'\t'
                        <<current_sa_angle[3]<<'\t'<<current_sa_angle[4]<<'\t'<<current_sa_angle[5]<<std::endl;
            }

            saJointMove(init_angle, 1);

            if(motorsPositionCheck(current_sa_angle, init_angle, 6))
            {
                getForceData(imp_->arm1_init_force, 0, imp_->init);
                getForceData(imp_->arm2_init_force, 1, imp_->init);

                gc.saveInitForce(imp_->arm1_init_force, imp_->arm2_init_force);

                mout()<<"Init Complete"<<std::endl;

                imp_->init = true;
            }
        }
        else
        {

            eeA1.getP(arm1_current_pos);
            eeA1.getMpm(arm1_current_pm);

            eeA2.getP(arm2_current_pos);
            eeA2.getMpm(arm2_current_pm);

            //Force Comp, Filtered, Transform
            getForceData(arm1_actual_force, 0, imp_->init);
            gc.getCompFT(arm1_current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, arm1_comp_force);

            getForceData(arm2_actual_force, 1, imp_->init);
            gc.getCompFT(arm2_current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, arm2_comp_force);

            for (size_t i = 0; i < 6; i++)
            {
                arm1_comp_force[i] = arm1_actual_force[i] + arm1_comp_force[i];
                arm2_comp_force[i] = arm2_actual_force[i] + arm2_comp_force[i];
            }

            forceFilter(arm1_comp_force, arm1_filtered_force, 0);
            forceFilter(arm2_comp_force, arm2_filtered_force, 1);

            //forceDeadZone(arm1_filtered_force, arm1_dead_zone);
            //forceDeadZone(arm2_filtered_force, arm2_dead_zone);


            forceTransform(arm1_filtered_force, arm1_transform_force, 0);
            forceTransform(arm2_filtered_force, arm2_transform_force, 1);

            for (size_t i = 0; i < 6; i++)
            {

                arm1_final_force[i] = arm1_transform_force[i] - imp_->arm1_start_force[i];
                arm2_final_force[i] = arm2_transform_force[i] - imp_->arm2_start_force[i];
            }


            //Phase 1 Approach to The Hole
            if (!imp_->phase1)
            {
                //Tool
                double assem_pos[6]{ -0.710, 0.012466, 0.291196, 0.0142609, -1.56474, 0.0 };
                double assem_angle[6]{ 0 };
                double assem_rm[9]{ 0 };

                //Define Initial Rotate displacment

                double rotate_angle[3]{ 0, (imp_->y_)* 2 * PI / 360, (imp_->z_ )* 2 * PI / 360 };
                double rotate_rm[9]{ 0 };
                double desired_rm[9]{ 0 };

                aris::dynamic::s_ra2rm(rotate_angle, rotate_rm);
                aris::dynamic::s_re2rm(assem_pos + 3, assem_rm, "321");
                aris::dynamic::s_mm(3, 3, 3, rotate_rm, assem_rm, desired_rm);
                aris::dynamic::s_rm2re(desired_rm, assem_pos + 3, "321");

                eeA2.setP(assem_pos);


                if (model_a2.inverseKinematics())
                {
                    mout() << "Assem Pos Inverse Failed" << std::endl;
                }

                model_a2.getInputPos(assem_angle);

                saJointMove(assem_angle, 1);
                if (motorsPositionCheck(current_sa_angle, assem_angle, 6))
                {

                    imp_->phase1 = true;
                    imp_->start_count = count();

                    mout()<<"Current Angle: "<< rotate_angle[0] <<'\t'<< rotate_angle[1] <<'\t' << rotate_angle[2]<<std::endl;
                    mout()<<"Current Pos: "<< assem_pos[0] <<'\t'<< assem_pos[1] <<'\t' << assem_pos[2]<<std::endl;
                    mout() << "Assembly Start !" << std::endl;
                }

            getForceData(arm2_actual_force, 1, imp_->init);
            gc.getCompFT(arm2_current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, arm2_comp_force);

            for (size_t i = 0; i < 6; i++)
            {
                arm1_comp_force[i] = arm1_actual_force[i] + arm1_comp_force[i];
                arm2_comp_force[i] = arm2_actual_force[i] + arm2_comp_force[i];
            }
            //Phase 2 Contact
            else if (imp_->phase1 && !imp_->phase2)
            {

                if(count() == imp_->start_count + 2000)
                {


                    for (size_t i = 0; i < 6; i++)
                    {
                        imp_->arm1_start_force[i] = arm1_transform_force[i];
                        imp_->arm2_start_force[i] = arm2_transform_force[i];
                    }
                    mout()<<"Start Force Comp"<<std::endl;
                    mout()<<"A2 Start Force: "<<imp_->arm2_start_force[0]<<'\t'<<imp_->arm2_start_force[1]<<'\t'<<imp_->arm2_start_force[2]<<'\t'
                            <<imp_->arm2_start_force[3]<<'\t'<<imp_->arm2_start_force[4]<<'\t'<<imp_->arm2_start_force[5]<<std::endl;


                }


                double raw_force_checker[6]{ 0 };
                double comp_force_checker[6]{ 0 };
                double force_checker[6]{ 0 };

                double a2_pm[16]{ 0 };
                eeA2.getMpm(a2_pm);
                eeA2.getP(arm2_current_pos);


                //Define Initial Position displacment
                std::array<std::array<double, 3>, 13> points =
                {
                    mout() << "force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                        << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                }


                //Arm1
                getForceData(raw_force_checker, 1, imp_->init);
                gc.getCompFT(a2_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force_checker);
                if (count() % 1000 == 0)
                {
                    mout() << "pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                }

                for (int i = 0; i < 6; i++)
                {
                    force_checker[i] = comp_force_checker[i] + raw_force_checker[i];
                    if (abs(force_checker[i]) >= 0.5)
                    {
                        imp_->phase2 = true;
                        imp_->start_count = 0;
                        mout() << "Contact Check" << std::endl;
                        mout() << "Contact Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                            << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                        mout() << "Contact force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                               << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
                        break;


                    }

                }
                if (!imp_->phase2)
                {
                    arm2_current_pos[0] -= 0.0000035;
                    saMove(arm2_current_pos, model_a2, 1);
                }

            }
            //Phase 3 Maintain 2 Points Contact
            else if (imp_->phase2 && !imp_->phase3)
            {
                double acc[3]{ 0 };
                double dx[3]{ 0 };
                double dt = 0.001;

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(arm2_final_force[i]) > 20.0)
                    {
                        mout() << "Emergency Brake" << std::endl;
                        mout() << "Brake force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                               << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
                        return 0;
                    }
                }
                if (count() % 100 == 0)
                {
                    mout() << "force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << "pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << std::endl;
                }

                if (posCheck(arm2_current_pos, 10))
                {
                    imp_->phase3 = true;
                    mout() << "Pos 3 Complete" << std::endl;
                    mout() << "Complete Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                    mout() << "Complete force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                    imp_->complete_count = count();
                    eeA2.getP(imp_->tpcontact_pos);

                    for(int i = 0; i<6; i++)
                    {
                        imp_->v_c[i] = 0;
                    }

                    //return 0;

                }
                else
                {


                    forceUpperLimit(arm2_final_force, limit_area);
                    forceDeadZone(arm2_final_force, imp_->p3_deadzone);

                    if(arm2_final_force[0] >= (imp_->phase3_fd[0] - 2.0) && arm2_final_force[0] <= (imp_->phase3_fd[0] + 10.0))
                    {
                        arm2_final_force[0] = imp_->phase3_fd[0];
                    }




                    //Impedence Controller
                    for (int i = 0; i < 1; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase3_fd[i] + arm2_final_force[i] - imp_->phase3_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase3_M[i];
                    }


                    for (int i = 0; i < 1; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        arm2_current_pos[i] = dx[i] + arm2_current_pos[i];

                    }


                    saMove(arm2_current_pos, model_a2, 1);
                }

            }
            //Phase 4 Get Data
            else if (imp_->phase3 && !imp_->phase4)
            {
                if(count() <= imp_->complete_count + 1550)
                {
                    if(count() % 50 == 0 && imp_->accumulation_count < 30)
                    {
                        mout()<<"A2_Force"<<"\t"<<arm2_transform_force[0]<<"\t"<<arm2_transform_force[1]<<"\t"<<arm2_transform_force[2]<<"\t"
                               <<arm2_transform_force[3]<<"\t"<<arm2_transform_force[4]<<"\t"<<arm2_transform_force[5]<<std::endl;


                        for(int i = 0; i<6; i++)
                        {
                            imp_->arm1_temp_force[i] += arm1_transform_force[i];
                            imp_->arm2_temp_force[i] += arm2_transform_force[i];

                        }

                          imp_->accumulation_count ++;

                        //mout()<<"count: "<<imp_->accumulation_count<<std::endl;

                    }
                }
                else
                {
                    double arm1_avg_force[6]{0};
                    double arm2_avg_force[6]{0};

                    for(int i = 0; i<6; i++)
                    {
                        arm1_avg_force[i] = imp_->arm1_temp_force[i] / 30.0;
                        arm2_avg_force[i] = imp_->arm2_temp_force[i] / 30.0;
                    }


                    mout()<<"Data Acquired!"  << '\t' <<"Current Z Angle: "<< imp_->z_ << '\t' <<"Current Y Angle: "<< imp_->y_ <<std::endl;

                    mout()<<"A2_Force"<<"\t"<<arm2_avg_force[0]<<"\t"<<arm2_avg_force[1]<<"\t"<<arm2_avg_force[2]<<"\t"
                           <<arm2_avg_force[3]<<"\t"<<arm2_avg_force[4]<<"\t"<<arm2_avg_force[5]<<std::endl;

                    imp_->back_count = count();
                    imp_->accumulation_count = 0;
                    imp_->phase4 = true;
                }

            }
            //Back
            else if(imp_->phase4 && !imp_->phase5)
            {
                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(arm2_transform_force[i]) > 20.0)
                    {
                        mout() << "Emergency Brake" << std::endl;
                        mout() << "Brake force: " << arm2_transform_force[0] << '\t' << arm2_transform_force[1] << '\t' << arm2_transform_force[2] << '\t'
                            << arm2_transform_force[3] << '\t' << arm2_transform_force[4] << '\t' << arm2_transform_force[5] << std::endl;
                        return 0;
                    }
                }
                if (count() % 1000 == 0)
                {
                    mout()<<"Complete!: "
                           <<"A2_Force"<<"\t"<<arm2_transform_force[0]<<"\t"<<arm2_transform_force[1]<<"\t"<<arm2_transform_force[2]<<"\t"
                           <<arm2_transform_force[3]<<"\t"<<arm2_transform_force[4]<<"\t"<<arm2_transform_force[5]<<std::endl;
                }


                if(count() <= imp_->back_count + 4000)
                {
                    arm2_current_pos[0] += 0.00001;
                    saMove(arm2_current_pos, model_a2, 1);
                }
                else
                {
                    mout()<<"Test Complete! "<< '\t' <<"Current Z Angle: "<< imp_->z_ << '\t' <<"Current Y Angle: "<< imp_->y_ <<std::endl;
                    imp_->phase5 = true;
                    return 0;
                }
            }


        }


        return 150000 - count();
    }
    PlateAllignData::PlateAllignData(const std::string& name)
{
        aris::core::fromXmlString(command(),
         "<Command name=\"m_pad\">"
         "	<GroupParam>"
         "	<Param name=\"z_degree\" default=\"0\" abbreviation=\"z\"/>"
         "	<Param name=\"y_degree\" default=\"0\" abbreviation=\"y\"/>"
         "	</GroupParam>"
         "</Command>");
}
    PlateAllignData::~PlateAllignData() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(PlateAllignData)

    struct PlateAllignTest::Imp {

        //Flag
        //Phase 1 -> Approach, Phase 2 -> Contact, Phase 3 -> Align, Phase 4 -> Fit, Phase 5 -> Insert
        bool init = false;
        bool stop = false;
        bool phase1 = false;
        bool phase2 = false;
        bool phase3 = false;
        bool phase4 = false;
        bool phase5 = false;

        //Force Compensation Parameter
        double comp_f[6]{ 0 };

        //Arm1
        double arm1_init_force[6]{ 0 };
        double arm1_start_force[6]{ 0 };
        double arm1_p_vector[6]{ 0 };
        double arm1_l_vector[6]{ 0 };

        double arm1_temp_force[6]{0};
        double arm1_temp_force2[6]{0};

        //Arm2
        double arm2_init_force[6]{ 0 };
        double arm2_start_force[6]{ 0 };
        double arm2_p_vector[6]{ 0 };
        double arm2_l_vector[6]{ 0 };

        double arm2_temp_force[6]{0};
        double arm2_temp_force2[6]{0};


        //Desired Force of Each Phase

        double phase3_fd[6]{ 3.5,0,0,0,0,0 };
        double phase4_fd[6]{ 7.5,0,0,0,0,0 };
        double phase5_fd[6]{ 3.5,0,0,0,0,0 };
        double phase6_fd[6]{ 3.5,0,0,0,0,0 };

        //Current Vel && Desired Vel
        double v_c[6]{ 0 };
        double v_d[6]{ 0 };

        //Impedence Parameter
        double phase3_B[6]{ 4000,3300,3300,0,0,0 };
        double phase3_M[6]{ 100,100,100,0,0,0 };

        //4.5 2.5
        double phase4_B[6]{ 3500,2500,2500,15,80,80 };
        double phase4_M[6]{ 100,100,100,10,5,5 };

        double phase5_B[6]{3000, 3300, 3300, 45, 45, 45};
        double phase5_M[6]{100, 100, 100, 25, 25, 25};

        double phase6_B[6]{3000, 3300, 3300, 45, 45, 45};
        double phase6_M[6]{100, 100, 100, 25, 25, 25};

        //Force Counter
        int allign_count = 0;
        int success_count = 0;

        //Counter for force comp
        int start_count = 0;

        //Pos Counter
        int pos_count = 0;
        int pos_success_count = 0;

        //Data Collect Counter
        int complete_count = 0;
        int back_count = 0;
        int accumulation_count = 0;

        double current_pos_checkek[6] = {0};

        //Parameters For Compensate
        double a_y = -0.0592;
        double b_y = -0.9943;
        double c_y = 0.0132;

        //Switch Angle
        double z_;
        double y_;

        //Arm1 Force Buffer
        std::array<double, 10> arm1_force_buffer[6] = {};
        int arm1_buffer_index[6]{ 0 };

        //Arm2 Force Buffer
        std::array<double, 10> arm2_force_buffer[6] = {};
        int arm2_buffer_index[6]{ 0 };

        //Dead Zone 8
        double p3_deadzone[6]{0.1,3,3,0,0,0};
        double p4_deadzone[6]{0.1,0.1,0.1,0.3,0.3,0.3};
        double p5_deadzone[6]{0,0,0,0.05,0.015,0.015};
        double p6_deadzone[6]{0,0,0,0.05,0.015,0.015};

        //2-Points Contact Pos
        double tpcontact_pos[6]{0};



    };
    auto PlateAllignTest::prepareNrt() -> void
    {
        for (auto& m : motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

        GravComp gc;
        gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
        mout() << "Load P & L Vector" << std::endl;
    }
    auto PlateAllignTest::executeRT() -> int
    {
        //dual transform modelbase into multimodel
        auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
        //at(0) -> Arm1 -> white
        auto& arm1 = dualArm.subModels().at(0);
        //at(1) -> Arm2 -> blue
        auto& arm2 = dualArm.subModels().at(1);

        //transform to model
        auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
        auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);

        //End Effector
        auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
        auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));

        //ver 1.0 not limit on vel, only limit force
        static double tolerance = 0.00005;
        //ver 2.0 limited vel, 20mm/s, 30deg/s
        static double max_vel[6]{ 0.00002,0.00002,0.00002,0.0005,0.0005,0.0005 };
        static double dead_zone[6]{ 0.1,0.1,0.1,0.0006,0.00920644,0.0006 };
        static double limit_area[6]{15,15,15,0.6,0.6,0.6};

        GravComp gc;

        double current_angle[12]{ 0 };
        double current_sa_angle[6]{ 0 };

        double arm1_comp_force[6]{ 0 };
        double arm1_current_pm[16]{ 0 };
        double arm1_current_pos[6]{ 0 };
        double arm1_actual_force[6]{ 0 };
        double arm1_filtered_force[6]{ 0 };
        double arm1_transform_force[6]{ 0 };
        double arm1_final_force[6]{0};

        double arm2_comp_force[6]{ 0 };
        double arm2_current_pm[16]{ 0 };
        double arm2_current_pos[6]{ 0 };
        double arm2_actual_force[6]{ 0 };
        double arm2_filtered_force[6]{ 0 };
        double arm2_transform_force[6]{ 0 };
        double arm2_final_force[6]{0};

        double mz_comp = 0;


        imp_->z_ = doubleParam("z_degree");

        imp_->y_ = doubleParam("y_degree");



        auto getForceData = [&](double* data_, int m_, bool init_)
        {

            int raw_force[6]{ 0 };

            for (std::size_t i = 0; i < 6; ++i)
            {
                if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
                    mout() << "error" << std::endl;

                data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

            }

            if (!init_)
            {
                mout() << "Compensate Init Force" << std::endl;
            }
            else
            {
                if (m_ == 0)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

                    }
                }
                else if (m_ == 1)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

                    }
                }
                else
                {
                    mout() << "Wrong Model" << std::endl;
                }

            }

        };

        auto daJointMove = [&](double target_mp_[12])
        {
            double current_angle[12] = { 0 };
            double move = 0.00005;

            for (int i = 0; i < 12; i++)
            {
                current_angle[i] = controller()->motorPool()[i].targetPos();
            }

            for (int i = 0; i < 12; i++)
            {
                if (current_angle[i] <= target_mp_[i] - move)
                {
                    controller()->motorPool()[i].setTargetPos(current_angle[i] + move);
                }
                else if (current_angle[i] >= target_mp_[i] + move)
                {
                    controller()->motorPool()[i].setTargetPos(current_angle[i] - move);
                }
            }
        };

        auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
        {
            for (int i = 0; i < dim_; i++)
            {
                if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
                {
                    return false;
                }
            }

            return true;
        };

        auto saJointMove = [&](double target_mp_[6], int m_)
        {
            double current_angle[12] = { 0 };
            double move = 0.00005;

            for (int i = 0; i < 12; i++)
            {
                current_angle[i] = controller()->motorPool()[i].targetPos();
            }

            for (int i = 0; i < 6; i++)
            {
                if (current_angle[i+6*m_] <= target_mp_[i] - move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] + move);
                }
                else if (current_angle[i+6*m_] >= target_mp_[i] + move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] - move);
                }
            }
        };

        auto saMove = [&](double* pos_, aris::dynamic::Model& model_, int type_) {

            model_.setOutputPos(pos_);

            if (model_.inverseKinematics())
            {
                throw std::runtime_error("Inverse Kinematics Position Failed!");
            }


            double x_joint[6]{ 0 };

            model_.getInputPos(x_joint);

            if (type_ == 0)
            {
                for (std::size_t i = 0; i < 6; ++i)
                {
                    controller()->motorPool()[i].setTargetPos(x_joint[i]);
                }
            }
            else if (type_ == 1)
            {
                for (std::size_t i = 0; i < 6; ++i)
                {
                    controller()->motorPool()[i + 6].setTargetPos(x_joint[i]);
                }
            }
            else
            {
                throw std::runtime_error("Arm Type Error");
            }
        };

        auto forceFilter = [&](double* actual_force_, double* filtered_force_, int m_)
        {
            if(m_ == 0)
            {
                for (int i = 0; i < 6; i++)
                {
                    imp_->arm1_force_buffer[i][imp_->arm1_buffer_index[i]] = actual_force_[i];
                    imp_->arm1_buffer_index[i] = (imp_->arm1_buffer_index[i] + 1) % 10;

                    filtered_force_[i] = std::accumulate(imp_->arm1_force_buffer[i].begin(), imp_->arm1_force_buffer[i].end(), 0.0) / 10;
                }
            }
            else if(m_ == 1)
            {
                for (int i = 0; i < 6; i++)
                {
                    imp_->arm2_force_buffer[i][imp_->arm2_buffer_index[i]] = actual_force_[i];
                    imp_->arm2_buffer_index[i] = (imp_->arm2_buffer_index[i] + 1) % 10;

                    filtered_force_[i] = std::accumulate(imp_->arm2_force_buffer[i].begin(), imp_->arm2_force_buffer[i].end(), 0.0) / 10;
                }
            }
            else
            {
                mout()<<"Wrong Filter!"<<std::endl;
            }

        };

        auto forceDeadZone = [&](double* actual_force_, double* area_)
        {
            for (int i = 0; i < 6; i++)
            {
                if (abs(actual_force_[i]) < area_[i])
                {
                    actual_force_[i] = 0;
                }
            }
        };

        auto forceTransform = [&](double* actual_force_, double* transform_force_, int m_)
        {
            if (m_ == 0)
            {
                transform_force_[0] = actual_force_[2];
                transform_force_[1] = -actual_force_[1];
                transform_force_[2] = actual_force_[0];

                transform_force_[3] = actual_force_[5];
                transform_force_[4] = -actual_force_[4];
                transform_force_[5] = actual_force_[3];
            }
            else if (m_ == 1)
            {
                transform_force_[0] = -actual_force_[2];
                transform_force_[1] = actual_force_[1];
                transform_force_[2] = actual_force_[0];

                transform_force_[3] = -actual_force_[5];
                transform_force_[4] = actual_force_[4];
                transform_force_[5] = actual_force_[3];
            }
            else
            {
                mout() << "Error Model In Force Transform" << std::endl;
            }
        };

        auto forceCheck = [&](double* current_force_, double* force_check_, int count_, int num_)
        {

            bool isValid = true;
            for (int i = 0; i < num_; i++)
            {
                if (abs(current_force_[i]) > force_check_[i])
                {
                    isValid = false;
                    break;
                }
            }

            if (isValid)
            {
                if (imp_->allign_count == 0) {
                    imp_->allign_count = count();
                }

                if ((count() - imp_->allign_count) % 50 == 0) {
                    imp_->success_count++;
                    mout() << "Check " << imp_->success_count << '\t' << "Current Count: " << count() << std::endl;
                    if (imp_->success_count >= count_)
                    {
                        imp_->success_count = 0;
                        imp_->allign_count = 0;
                        return true;
                    }
                }
            }
            else
            {
                imp_->success_count = 0;
            }

            return false;
        };

        auto posCheck = [&](double* current_pos_, int count_)
        {
            if(count()%500 == 0)
            {
                std::copy(current_pos_, current_pos_+6, imp_->current_pos_checkek);
                //mout()<<"Save Current Pos: "<<imp_->current_pos_checkek[0]<<std::endl;
            }

            bool isValid = true;

            for(int i = 0; i < 3; i++)
            {
                if (abs(current_pos_[i] - imp_->current_pos_checkek[i]) >= 0.000005)
                {
                    isValid = false;
                    break;
                }
            }

            if(isValid)
            {
                if(imp_->pos_count == 0)
                {
                    imp_->pos_count = count();
                }

                if((count() - imp_->pos_count) % 50 == 0)
                {
                    imp_->pos_success_count ++;
                    //mout() << "Check " << imp_->pos_success_count << '\t' << "Current Count: " << count() << std::endl;
                    if(imp_->pos_success_count >= count_)
                    {
                        imp_->pos_count = 0;
                        imp_->pos_success_count = 0;
                        return true;
                    }
                }
            }
            else
            {
                imp_->pos_success_count = 0;
            }

            return false;
        };

        auto velDeadZone = [&](double vel_, double vel_limit_)
        {
            if (vel_ >= vel_limit_)
            {
                vel_ = vel_limit_;
            }
            else if (vel_ <= -vel_limit_)
            {
                vel_ = -vel_limit_;
            }

        };

        auto forceUpperLimit = [&](double* actual_force_, double* area_)
        {
            for (int i = 0; i < 6; i++)
            {
                if (actual_force_[i] >= area_[i])
                {
                    actual_force_[i] = area_[i];
                }
                else if(actual_force_[i] <= -area_[i])
                {
                    actual_force_[i] = -area_[i];
                }
            }
        };

        for (int i = 0; i < 12; i++)
        {
            current_angle[i] = controller()->motorPool()[i].actualPos();
        }


        std::copy(current_angle + 6, current_angle + 12, current_sa_angle);





        if(!imp_->init)
        {
            double assem_pos[6]{ -0.710, 0.012466, 0.291196, 0.0142609, -1.56474, 0.0  };
            double init_angle[6]{0};

            model_a2.setOutputPos(assem_pos);
            if(model_a2.inverseKinematics())
            {
                mout()<<"Error"<<std::endl;
            }

            model_a2.getInputPos(init_angle);

            if(count()%1000 == 0)
            {
                mout()<<"Init angle: "<<init_angle[0]<<'\t'<<init_angle[1]<<'\t'<<init_angle[2]<<'\t'
                        <<init_angle[3]<<'\t'<<init_angle[4]<<'\t'<<init_angle[5]<<std::endl;
                mout()<<"current angle: "<<current_sa_angle[0]<<'\t'<<current_sa_angle[1]<<'\t'<<current_sa_angle[2]<<'\t'
                        <<current_sa_angle[3]<<'\t'<<current_sa_angle[4]<<'\t'<<current_sa_angle[5]<<std::endl;
            }

            saJointMove(init_angle, 1);

            if(motorsPositionCheck(current_sa_angle, init_angle, 6))
            {
                getForceData(imp_->arm1_init_force, 0, imp_->init);
                getForceData(imp_->arm2_init_force, 1, imp_->init);

                gc.saveInitForce(imp_->arm1_init_force, imp_->arm2_init_force);

                mout()<<"Init Complete"<<std::endl;

                imp_->init = true;
            }
        }
        else
        {

            eeA1.getP(arm1_current_pos);
            eeA1.getMpm(arm1_current_pm);

            eeA2.getP(arm2_current_pos);
            eeA2.getMpm(arm2_current_pm);

            //Force Comp, Filtered, Transform
            getForceData(arm1_actual_force, 0, imp_->init);
            gc.getCompFT(arm1_current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, arm1_comp_force);

            getForceData(arm2_actual_force, 1, imp_->init);
            gc.getCompFT(arm2_current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, arm2_comp_force);

            for (size_t i = 0; i < 6; i++)
            {
                arm1_comp_force[i] = arm1_actual_force[i] + arm1_comp_force[i];
                arm2_comp_force[i] = arm2_actual_force[i] + arm2_comp_force[i];
            }

            forceFilter(arm1_comp_force, arm1_filtered_force, 0);
            forceFilter(arm2_comp_force, arm2_filtered_force, 1);

            //forceDeadZone(arm1_filtered_force, arm1_dead_zone);
            //forceDeadZone(arm2_filtered_force, arm2_dead_zone);


            forceTransform(arm1_filtered_force, arm1_transform_force, 0);
            forceTransform(arm2_filtered_force, arm2_transform_force, 1);

            for (size_t i = 0; i < 6; i++)
            {

                arm1_final_force[i] = arm1_transform_force[i] - imp_->arm1_start_force[i];
                arm2_final_force[i] = arm2_transform_force[i] - imp_->arm2_start_force[i];
            }


            //Phase 1 Approach to The Hole
            if (!imp_->phase1)
            {
                //Tool
                double assem_pos[6]{ -0.710, 0.012466, 0.291196, 0.0142609, -1.56474, 0.0 };
                double assem_angle[6]{ 0 };
                double assem_rm[9]{ 0 };

                //Define Initial Rotate displacment

                double rotate_angle[3]{ 0, (imp_->y_)* 2 * PI / 360, (imp_->z_ )* 2 * PI / 360 };
                double rotate_rm[9]{ 0 };
                double desired_rm[9]{ 0 };

                aris::dynamic::s_ra2rm(rotate_angle, rotate_rm);
                aris::dynamic::s_re2rm(assem_pos + 3, assem_rm, "321");
                aris::dynamic::s_mm(3, 3, 3, rotate_rm, assem_rm, desired_rm);
                aris::dynamic::s_rm2re(desired_rm, assem_pos + 3, "321");

                eeA2.setP(assem_pos);


                if (model_a2.inverseKinematics())
                {
                    mout() << "Assem Pos Inverse Failed" << std::endl;
                }

                model_a2.getInputPos(assem_angle);

                saJointMove(assem_angle, 1);
                if (motorsPositionCheck(current_sa_angle, assem_angle, 6))
                {

                    imp_->phase1 = true;
                    imp_->start_count = count();

                    mout()<<"Current Angle: "<< rotate_angle[0] <<'\t'<< rotate_angle[1] <<'\t' << rotate_angle[2]<<std::endl;
                    mout()<<"Current Pos: "<< assem_pos[0] <<'\t'<< assem_pos[1] <<'\t' << assem_pos[2]<<std::endl;
                    mout() << "Assembly Start !" << std::endl;
                }

            }
            //Phase 2 Contact Check
            else if (imp_->phase1 && !imp_->phase2)
            {

                if(count() == imp_->start_count + 2000)
                {


                    for (size_t i = 0; i < 6; i++)
                    {
                        imp_->arm1_start_force[i] = arm1_transform_force[i];
                        imp_->arm2_start_force[i] = arm2_transform_force[i];
                    }
                    mout()<<"Start Force Comp"<<std::endl;
                    mout()<<"A2 Start Force: "<<imp_->arm2_start_force[0]<<'\t'<<imp_->arm2_start_force[1]<<'\t'<<imp_->arm2_start_force[2]<<'\t'
                            <<imp_->arm2_start_force[3]<<'\t'<<imp_->arm2_start_force[4]<<'\t'<<imp_->arm2_start_force[5]<<std::endl;


                }


                double raw_force_checker[6]{ 0 };
                double comp_force_checker[6]{ 0 };
                double force_checker[6]{ 0 };

                double a2_pm[16]{ 0 };
                eeA2.getMpm(a2_pm);
                eeA2.getP(arm2_current_pos);

                if (count() % 1000 == 0)
                {
                    mout() << "force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                        << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                }


                //Arm1
                getForceData(raw_force_checker, 1, imp_->init);
                gc.getCompFT(a2_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force_checker);
                if (count() % 1000 == 0)
                {
                    mout() << "pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                }

                for (int i = 0; i < 6; i++)
                {
                    force_checker[i] = comp_force_checker[i] + raw_force_checker[i];
                    if (abs(force_checker[i]) >= 0.5)
                    {
                        imp_->phase2 = true;
                        imp_->start_count = 0;
                        mout() << "Contact Check" << std::endl;
                        mout() << "Contact Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                            << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                        mout() << "Contact force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                               << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
                        break;


                    }

                }
                if (!imp_->phase2)
                {
                    arm2_current_pos[0] -= 0.0000035;
                    saMove(arm2_current_pos, model_a2, 1);
                }

            }
            //Phase 3 Maintain Contact
            else if (imp_->phase2 && !imp_->phase3)
            {
                double acc[3]{ 0 };
                double dx[3]{ 0 };
                double dt = 0.001;

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(arm2_final_force[i]) > 20.0)
                    {
                        mout() << "Emergency Brake" << std::endl;
                        mout() << "Brake force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                               << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
                        return 0;
                    }
                }
                if (count() % 100 == 0)
                {
                    mout() << "force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << "pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << std::endl;
                }

                if (posCheck(arm2_current_pos, 10))
                {
                    imp_->phase3 = true;
                    mout() << "Pos 3 Complete" << std::endl;
                    mout() << "Complete Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                    mout() << "Complete force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                    imp_->complete_count = count();
                    eeA2.getP(imp_->tpcontact_pos);

                    for(int i = 0; i<6; i++)
                    {
                        imp_->v_c[i] = 0;
                    }

                    //return 0;

                }
                else
                {


                    forceUpperLimit(arm2_final_force, limit_area);
                    forceDeadZone(arm2_final_force, imp_->p3_deadzone);

                    if(arm2_final_force[0] >= (imp_->phase3_fd[0] - 2.0) && arm2_final_force[0] <= (imp_->phase3_fd[0] + 10.0))
                    {
                        arm2_final_force[0] = imp_->phase3_fd[0];
                    }




                    //Impedence Controller
                    for (int i = 0; i < 1; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase3_fd[i] + arm2_final_force[i] - imp_->phase3_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase3_M[i];
                    }


                    for (int i = 0; i < 1; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        arm2_current_pos[i] = dx[i] + arm2_current_pos[i];

                    }


                    saMove(arm2_current_pos, model_a2, 1);
                }

            }
            //Phase 4 Allign
            else if (imp_->phase3 && !imp_->phase4)
            {
                double acc[3]{ 0 };
                double ome[3]{ 0 };

                double dx[3]{ 0 };
                double dth[3]{ 0 };
                double dt = 0.001;

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(arm2_final_force[i]) > 25.0)
                    {
                        mout() << "Emergency Brake" << std::endl;

                        mout() << "Brake force Phase3: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                            << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                        return 0;
                    }
                }
//                if (count() % 100 == 0)
//                {
//                    mout() << "force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
//                        << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << '\t' << '\t' <<"  pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
//                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
//                }

//                if (count() % 100 == 0)
//                {
//                    mout() << "force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
//                        << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5]  << std::endl;
//                }

                if (posCheck(arm2_current_pos, 10))
                {
                    imp_->phase4 = true;
                    mout() << "Allign Complete" << std::endl;
                    mout() << "Allign Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                    mout() << "Allign force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                        << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                    return 0;

                }
                else
                {

                    if (count() % 50 == 0)
                    {
                        mout() << "force: "<< '\t' << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                            << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << '\t'
                            << "Pos: "<<'\t'<< arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] <<std::endl;
                    }

                    forceUpperLimit(arm2_final_force, limit_area);
                    forceDeadZone(arm2_final_force, imp_->p4_deadzone);


                    if(arm2_final_force[0] >= (imp_->phase4_fd[0] - 2.5) && arm2_final_force[0] <= (imp_->phase4_fd[0] + 5.5))
                    {
                        arm2_final_force[0] = imp_->phase4_fd[0];
                    }


                    //Impedence Controller
                    for (int i = 0; i < 1; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase4_fd[i] + arm2_final_force[i] - imp_->phase4_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase4_M[i];
                    }


                    for (int i = 0; i < 1; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        arm2_current_pos[i] = dx[i] + arm2_current_pos[i];

                    }

                    //pose
                    for (int i = 1; i < 3; i++)
                    {
                        // Caculate Omega
                        ome[i] = (-imp_->phase4_fd[i + 3] + arm2_final_force[i + 3] - imp_->phase4_B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3])) / imp_->phase4_M[i + 3];
                    }

            //Safety Check
            for (size_t i = 0; i < 3; i++)
            {
                if (abs(transform_force[i]) > 30)
                {
                    mout() << "Emergency Brake" << std::endl;
                    return 0;
                }
            }

                    for (int i = 1; i < 3; i++)
                    {
                        // Angluar Velocity
                        imp_->v_c[i + 3] += ome[i] * dt;
                        velDeadZone(imp_->v_c[i + 3], max_vel[i + 3]);
                        dth[i] = imp_->v_c[i + 3] * dt;
                    }

                    double drm[9]{ 0 };
                    double rm_target[9]{ 0 };
                    double rm_c[9]{ 0 };

                    //Transform to rm
                    aris::dynamic::s_ra2rm(dth, drm);

                    //Current pe to rm
                    aris::dynamic::s_re2rm(arm2_current_pos + 3, rm_c, "321");

                    //Calcuate Future rm
                    aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);

                    //Convert rm to pe
                    aris::dynamic::s_rm2re(rm_target, arm2_current_pos + 3, "321");

                    saMove(arm2_current_pos, model_a2, 1);

                }
            }
            //Back
            else if(imp_->phase4 && !imp_->phase5)
            {

            }


        }


        return 150000 - count();
    }
    PlateAllignTest::PlateAllignTest(const std::string& name)
{
        aris::core::fromXmlString(command(),
         "<Command name=\"m_pat\">"
         "	<GroupParam>"
         "	<Param name=\"z_degree\" default=\"0\" abbreviation=\"z\"/>"
         "	<Param name=\"y_degree\" default=\"0\" abbreviation=\"y\"/>"
         "	</GroupParam>"
         "</Command>");
}
    PlateAllignTest::~PlateAllignTest() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(PlateAllignTest)



    ARIS_REGISTRATION {
        aris::core::class_<ModelInit>("ModelInit")
            .inherit<aris::plan::Plan>();
        aris::core::class_<ModelForward>("ModelForward")
            .inherit<aris::plan::Plan>();
        aris::core::class_<ModelGet>("ModelGet")
            .inherit<aris::plan::Plan>();
        aris::core::class_<ModelTest>("ModelTest")
            .inherit<aris::plan::Plan>();
        aris::core::class_<ModelSetPos>("ModelSetPos")
            .inherit<aris::plan::Plan>();
        aris::core::class_<ModelComP>("ModelComP")
            .inherit<aris::plan::Plan>();
        aris::core::class_<ForceAlign>("ForceAlign")
            .inherit<aris::plan::Plan>();
        aris::core::class_<ForceKeep>("ForceKeep")
            .inherit<aris::plan::Plan>();
        aris::core::class_<ForceDrag>("ForceDrag")
            .inherit<aris::plan::Plan>();
        aris::core::class_<PegOutHole>("PegOutHole")
            .inherit<aris::plan::Plan>();
        aris::core::class_<Demo>("Demo")
            .inherit<aris::plan::Plan>();
        aris::core::class_<Search>("Search")
            .inherit<aris::plan::Plan>();
        aris::core::class_<Arm2PegInHole>("Arm2PegInHole")
            .inherit<aris::plan::Plan>();
        aris::core::class_<PlateAllignData>("PlateAllignData")
            .inherit<aris::plan::Plan>();
        aris::core::class_<PlateAllignTest>("PlateAllignTest")
            .inherit<aris::plan::Plan>();

    }


}
