#include "robot.hpp"
#include "gravcomp.hpp"


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
        0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };


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

    };
	auto ModelGet::prepareNrt() -> void
	{
        for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE;
        GravComp gc;
        gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);

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

        if(!imp_->init)
        {
            getForceData(imp_->arm1_init_force, 0, imp_->init);
            getForceData(imp_->arm2_init_force, 1, imp_->init);

            double current_angle[12] = { 0 };

            for (int i = 0; i < 12; i++)
            {
                current_angle[i] = controller()->motorPool()[i].actualPos();
            }

            dualArm.setInputPos(current_angle);
            if(dualArm.forwardKinematics())
            {
                mout()<<"Forward failed"<<std::endl;
                return 0;
            }

            imp_->init = true;
            mout()<<"Init"<<std::endl;

        }
        else
        {
            double current_pm[16]{0};


            if(imp_->m_ == 0)
            {

                eeA1.getMpm(current_pm);

                double raw_force[6]{0};
                double arm1_compf[6]{0};
                double arm1_actual_force[6]{0};


                getForceData(raw_force, 0, imp_->init);
                gc.getCompFT(current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, arm1_compf);

                for(int i = 0; i<6; i++)
                {
                    arm1_actual_force[i] = arm1_compf[i] + raw_force[i];
                }

                if(count()%100 == 0)
                {
                    mout()<<"A1_Force: "<<arm1_actual_force[0]<<'\t'<<arm1_actual_force[1]<<'\t'<<arm1_actual_force[2]
                         <<'\t'<<arm1_actual_force[3]<<'\t'<<arm1_actual_force[4]<<'\t'<<arm1_actual_force[5]<<std::endl;
                }


            }
            else if(imp_->m_ == 1)
            {

                eeA2.getMpm(current_pm);

                double raw_force[6]{0};
                double arm2_compf[6]{0};
                double arm2_actual_force[6]{0};


                getForceData(raw_force, 1, imp_->init);
                gc.getCompFT(current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, arm2_compf);

                for(int i = 0; i<6; i++)
                {
                    arm2_actual_force[i] = arm2_compf[i] + raw_force[i];
                }
                if(count()%100 == 0)
                {
                    mout()<<"A2_Force: "<<arm2_actual_force[0]<<'\t'<<arm2_actual_force[1]<<'\t'<<arm2_actual_force[2]
                         <<'\t'<<arm2_actual_force[3]<<'\t'<<arm2_actual_force[4]<<'\t'<<arm2_actual_force[5]<<std::endl;
                }
            }
            else
            {
                mout()<<"Model Error"<<std::endl;
                return 0;
            }
        }


        return 10000-count();
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

	auto ModelMoveX::prepareNrt()->void
	{

		for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
	}
	auto ModelMoveX::executeRT()->int
	{
		m_ = int32Param("model");
		d_ = doubleParam("distance");
		o_ = doubleParam("orientation");


		TCurve2 s1(1.0, 3.0, 20.0);
		s1.getCurveParam();
		static double init_pos[12]{};
		double input_angle[12]{};
		double ee_pos[12]{};
		double current_pos[12]{};
		double current_angle[12]{};

		if (count() == 1)
		{

			double begin_angle[12]{0};

            for (int i = 0; i < 12; i++)
            {
                begin_angle[i] = controller()->motorPool()[i].targetPos();
            }

            mout() << "init angle:" << std::endl;
			aris::dynamic::dsp(1, 12, begin_angle);

			//this->master()->logFileRawName("move");
            mout() << "read init angle" << std::endl;

			modelBase()->setInputPos(begin_angle);
			if (modelBase()->forwardKinematics())std::cout << "forward failed" << std::endl;
			modelBase()->getOutputPos(init_pos);

            mout() << "init position:" << std::endl;
			aris::dynamic::dsp(1, 12, init_pos);

		}


		auto eemove = [&](double* pos_) {
			modelBase()->setOutputPos(pos_);
			if (modelBase()->inverseKinematics())
			{
				throw std::runtime_error("Inverse Kinematics Position Failed!");
			}


			double x_joint[12]{ 0 };


			modelBase()->getInputPos(x_joint);


            for (std::size_t i = 0; i < 12; ++i)
            {
                controller()->motorPool()[i].setTargetPos(x_joint[i]);
            }

		};


		modelBase()->getOutputPos(ee_pos);



		if (m_ == 0)
		{
			ee_pos[0] += 0.00001;

		}
		else if (m_ == 1)
		{
			ee_pos[6] += 0.00001;
		}
		else
		{
            mout() << "model out of range; 0 ---> arm1 (white); 1 ---> arm2 (blue)" << std::endl;
		}


		eemove(ee_pos);

		modelBase()->getInputPos(input_angle);


        if (count() % 10 == 0)
		{
            mout() << "arm1:" << input_angle[0] * 180 / PI << "\t" << input_angle[1] * 180 / PI << "\t" << input_angle[2] * 180 / PI << "\t"
                << input_angle[3] * 180 / PI << "\t" << input_angle[4] * 180 / PI << "\t" << input_angle[5] * 180 / PI << "\n"
                << "arm2:" << input_angle[6] * 180 / PI << "\t" << input_angle[7] * 180 / PI << "\t" << input_angle[8] * 180 / PI
                << input_angle[9] * 180 / PI << "\t" << input_angle[10] * 180 / PI << "\t" << input_angle[11] * 180 / PI << "\t" << std::endl;

//            mout()<<input_angle[0]<< "\t" <<input_angle[1]<< "\t"<<input_angle[2]<< "\t"
//                                  <<input_angle[3]<< "\t"<<input_angle[4]<< "\t"<<input_angle[5]<<std::endl;

		}

		//aris::dynamic::dsp(1,12,ee_pos);
		return (d_ * 100) - count();

	}
	ModelMoveX::ModelMoveX(const std::string& name)
	{

		aris::core::fromXmlString(command(),
			"<Command name=\"m_x\">"
			"	<GroupParam>"
			"	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
			"	<Param name=\"distance\" default=\"10.0\" abbreviation=\"d\"/>"
			"	<Param name=\"orientation\" default=\"1.0\" abbreviation=\"o\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	ModelMoveX::~ModelMoveX() = default;
	KAANH_DEFINE_BIG_FOUR_CPP(ModelMoveX)



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
			aris::plan::Plan::CHECK_NONE |
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
			double mp[12];
			for (std::size_t i = (0 + 6 * m_); i < (6 + 6 * m_); ++i)
			{

				if (controller()->motorPool()[i].actualPos() - target_mp_[i - 6 * m_] < 8 / 180 * PI) {
					if (controller()->motorPool()[i].actualPos() >= target_mp_[i - 6 * m_] + 0.0001)
					{

						mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
					}
					else if (controller()->motorPool()[i].targetPos() <= target_mp_[i - 6 * m_] - 0.0001) {

						mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
					}
					else {
						mp[i] = target_mp_[i - 6 * m_];

					}
				}
				else {
					if (controller()->motorPool()[i].actualPos() >= target_mp_[i - 6 * m_] + 0.0001)
					{

						mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
					}
					else if (controller()->motorPool()[i].targetPos() <= target_mp_[i - 6 * m_] - 0.0001) {

						mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
					}
					else {
						mp[i] = target_mp_[i - 6 * m_];

					}

				}
				controller()->motorPool()[i].setTargetPos(mp[i]);
			}
		};

		auto daJointMove = [&](double target_mp_[12])
		{
			double mp[12];

			for (std::size_t i = 0; i < 12; ++i)
			{

				if (controller()->motorPool()[i].actualPos() - target_mp_[i] < 8 / 180 * PI) {
                    if (controller()->motorPool()[i].actualPos() >= target_mp_[i] + 0.0001)
					{

                        mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
					}
                    else if (controller()->motorPool()[i].targetPos() <= target_mp_[i] - 0.0001) {

                        mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
					}
					else {
						mp[i] = target_mp_[i];

					}
				}
				else {
                    if (controller()->motorPool()[i].actualPos() >= target_mp_[i] + 0.0001)
					{

                        mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
					}
                    else if (controller()->motorPool()[i].targetPos() <= target_mp_[i] - 0.0001) {

                        mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
					}
					else {
						mp[i] = target_mp_[i];

					}

				}
				controller()->motorPool()[i].setTargetPos(mp[i]);
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
			0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };

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


		auto daJointMove = [&](double target_mp_[12])
		{
			double mp[12];

			for (std::size_t i = 0; i < 12; ++i)
			{

				if (controller()->motorPool()[i].actualPos() - target_mp_[i] < 8 / 180 * PI) {
					if (controller()->motorPool()[i].actualPos() >= target_mp_[i] + 0.0001)
					{

						mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
					}
					else if (controller()->motorPool()[i].targetPos() <= target_mp_[i] - 0.0001) {

						mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
					}
					else {
						mp[i] = target_mp_[i];

					}
				}
				else {
					if (controller()->motorPool()[i].actualPos() >= target_mp_[i] + 0.0001)
					{

						mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
					}
					else if (controller()->motorPool()[i].targetPos() <= target_mp_[i] - 0.0001) {

						mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
					}
					else {
						mp[i] = target_mp_[i];

					}

				}
				controller()->motorPool()[i].setTargetPos(mp[i]);
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
			aris::plan::Plan::CHECK_NONE |
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
			double mp[12];

			for (std::size_t i = 0; i < 12; ++i)
			{

				if (controller()->motorPool()[i].actualPos() - target_mp_[i] < 8 / 180 * PI) {
					if (controller()->motorPool()[i].actualPos() >= target_mp_[i] + 0.0001)
					{

						mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
					}
					else if (controller()->motorPool()[i].targetPos() <= target_mp_[i] - 0.0001) {

						mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
					}
					else {
						mp[i] = target_mp_[i];

					}
				}
				else {
					if (controller()->motorPool()[i].actualPos() >= target_mp_[i] + 0.0001)
					{

						mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
					}
					else if (controller()->motorPool()[i].targetPos() <= target_mp_[i] - 0.0001) {

						mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
					}
					else {
						mp[i] = target_mp_[i];

					}

				}
				controller()->motorPool()[i].setTargetPos(mp[i]);
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
		double B[6]{ 100,100,100,15,15,15 };
		double M[6]{ 1,1,1,10,10,10 };

		double Ke[6]{ 220000,220000,220000,220000,220000,220000 };

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
			aris::plan::Plan::CHECK_NONE |
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
			double mp[12];

			for (std::size_t i = 0; i < 12; ++i)
			{

				if (controller()->motorPool()[i].actualPos() - target_mp_[i] < 8 / 180 * PI) {
					if (controller()->motorPool()[i].actualPos() >= target_mp_[i] + 0.0001)
					{

						mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
					}
					else if (controller()->motorPool()[i].targetPos() <= target_mp_[i] - 0.0001) {

						mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
					}
					else {
						mp[i] = target_mp_[i];

					}
				}
				else {
					if (controller()->motorPool()[i].actualPos() >= target_mp_[i] + 0.0001)
					{

						mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
					}
					else if (controller()->motorPool()[i].targetPos() <= target_mp_[i] - 0.0001) {

						mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
					}
					else {
						mp[i] = target_mp_[i];

					}

				}
				controller()->motorPool()[i].setTargetPos(mp[i]);
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

				mout() << imp_->arm1_x_d[0] << '\t' << imp_->arm1_x_d[1] << '\t' << imp_->arm1_x_d[2] << '\t'
					<< imp_->arm1_x_d[3] << '\t' << imp_->arm1_x_d[4] << '\t' << imp_->arm1_x_d[5] << std::endl;

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
				saMove(current_pos, model_a1, 0);


				if (count() % 100 == 0)
				{
					// mout() << current_vel[0] << '\t' << current_vel[1] << '\t' << current_vel[2] << '\t'
					// 	<< current_vel[3] << '\t' << current_vel[4] << '\t' << current_vel[5] << std::endl;

					mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
						<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;

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
                forceFilter(actual_force,filtered_force);

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


				//mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
				//	<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;

				//mout() << dx[0] << '\t' << dx[1] << '\t' << dx[2] << '\t'
				//	<< dx[3] << '\t' << dx[4] << '\t' << dx[5] << std::endl;

				//mout() << acc[0] << '\t' << acc[1] << '\t' << acc[2] << '\t'
				//	<< acc[3] << '\t' << acc[4] << '\t' << acc[5] << std::endl;

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
	ForceDrag::ForceDrag(const std::string& name)
	{
		aris::core::fromXmlString(command(),
			"<Command name=\"m_fd\">"
			"	<GroupParam>"
			"	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	ForceDrag::~ForceDrag() = default;
	KAANH_DEFINE_BIG_FOUR_CPP(ForceDrag)


    struct ModelComP2::Imp {
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
            double temp_force1[6] = {0};
            double temp_force2[6] = {0};
            double temp_force3[6] = {0};

            double force_data_1[6]={0};
            double force_data_2[6]={0};
            double force_data_3[6]={0};

            double init_force[6]{0};


            double init_pm[16]{0};
            double ee_pm_1[16]{0};
            double ee_pm_2[16]{0};
            double ee_pm_3[16]{0};

            double comp_f[6]{0};

            int m_;
        };
    auto ModelComP2::prepareNrt()->void
    {

        for (auto& m : motorOptions()) m =
            aris::plan::Plan::CHECK_NONE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;




    }
    auto ModelComP2::executeRT()->int
    {


        imp_->m_ = int32Param("model");



        static double move = 0.0001;

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
        auto jointMove = [&](double target_mp_[6], int m_)
        {
            double mp[12];
            for (std::size_t i = (0 + 6 * m_); i < (6 + 6 * m_); ++i)
            {

                if (controller()->motorPool()[i].actualPos() - target_mp_[i - 6 * m_] < 8 / 180 * PI) {
                    if (controller()->motorPool()[i].actualPos() >= target_mp_[i - 6 * m_] + 0.0002)
                    {

                        mp[i] = controller()->motorPool()[i].targetPos() - 0.0002;
                    }
                    else if (controller()->motorPool()[i].targetPos() <= target_mp_[i - 6 * m_] - 0.0002) {

                        mp[i] = controller()->motorPool()[i].targetPos() + 0.0002;
                    }
                    else {
                        mp[i] = target_mp_[i - 6 * m_];

                    }
                }
                else {
                    if (controller()->motorPool()[i].actualPos() >= target_mp_[i - 6 * m_] + 0.0002)
                    {

                        mp[i] = controller()->motorPool()[i].targetPos() - 0.0002;
                    }
                    else if (controller()->motorPool()[i].targetPos() <= target_mp_[i - 6 * m_] - 0.0002) {

                        mp[i] = controller()->motorPool()[i].targetPos() + 0.0002;
                    }
                    else {
                        mp[i] = target_mp_[i - 6 * m_];

                    }

                }
                controller()->motorPool()[i].setTargetPos(mp[i]);
            }
        };



        auto getForceData = [&](double* data_, int m_)
        {

            int raw_force[6]{ 0 };
            for (std::size_t i = 0; i < 6; ++i)
            {
                if (ecMaster()->slavePool()[8 + 7*m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
                    mout() << "error" << std::endl;

                data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

            }

            if(m_ == 0)
            {
                data_[0] = -data_[1];
                data_[1] = data_[0];
            }
            else if(m_ == 1)
            {
               data_[0] = -data_[0];
               data_[1] = -data_[1];

               data_[3] = -data_[3];
               data_[4] = -data_[4];

            }
            else
            {
                mout()<<"Wrong Model"<<std::endl;
            }


        };


        auto motorsPositionCheck = [](double current_sa_angle_[6], double target_pos_[6])
        {
            for (int i = 0; i < 6; i++)
            {
                if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= move)
                {
                    return false;
                }
            }

            return true;
        };

        auto caculateAvgForce = [=](double force_data_[6], double temp_force_[6], int count_, int m_)
        {
            if (count() < imp_->current_stop_time + imp_->stop_time)
            {

                if (count() % 200 == 0 && imp_->accumulation_count < 10)
                {
                    double temp2[6]{0};
                    getForceData(temp2, m_);

                    for (int i = 0; i < 6; i++)
                    {
                        temp_force_[i] = temp_force_[i] + temp2[i];
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
                    force_data_[i] = (temp_force_[i] / 10.0);
                }
                mout() << "Force Data " << count_ << '\n' << force_data_[0] << '\t' << force_data_[1] << '\t' << force_data_[2] << '\t'
                    << force_data_[3] << '\t' << force_data_[4] << '\t' << force_data_[5] << std::endl;

            }
            else
            {
                mout() << "Flag Change " << count_ << std::endl;
                imp_->stop_flag = false;
            }
        };




        static double init_pos[12] =
        { 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0,
        0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };

        double current_angle[12] = { 0 };

        for (int i = 0; i < 12; i++)
        {
            current_angle[i] = controller()->motorPool()[i].actualPos();
        }


        if (imp_->stop_flag)
        {
            if (imp_->stop_count == 1)
            {

                caculateAvgForce(imp_->force_data_1, imp_->temp_force1, 1, imp_->m_);

            }
            else if (imp_->stop_count == 2)
            {

                caculateAvgForce(imp_->force_data_2, imp_->temp_force2, 2, imp_->m_);

            }
            else if (imp_->stop_count == 3)
            {

                caculateAvgForce(imp_->force_data_3, imp_->temp_force3, 3, imp_->m_);

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
            // arm 1 white arm
            if (imp_->m_ == 0)
            {
//                static double angle1_1[6]{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0 };

//                static double angle1_2[6]{ 0, 0, 5 * PI / 6, -PI / 2, -PI / 2, 0 };

//                static double angle1_3[6]{ 0, 0, 5 * PI / 6, -2 * PI / 3, -2 * PI / 3, 0 };




                //// Arm 1 Angle
                static double init_angle1[6]{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0 };

                static double angle1_1[6]{ 0, 0, 5 * PI / 6, -17 * PI / 18, -PI / 2, 0 };

                static double angle1_2[6]{ 0, 0, 5 * PI / 6, -PI / 2, -PI / 2, 0 };

                static double angle1_3[6]{ 0, 0, 5 * PI / 6, -2 * PI / 3, -2 * PI / 3, 0 };




                //// Arm 2 Angle
                //static double init_angle2[6]{ 0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };

                //static double angle2_1[6]{ 0, 0, -5 * PI / 6, 17 * PI / 18, PI / 2, 0 };

                //static double angle2_2[6]{ 0, 0, -5 * PI / 6, PI / 2, PI / 3, 0 };

                //static double angle2_3[6]{ 0, 0, -5 * PI / 6, 2 * PI / 3, 2 * PI / 3, 0 };




                double current_sa_angle[6]{ 0 };
                std::copy(current_angle, current_angle + 6, current_sa_angle);

                if (count() % 1000 == 0)
                {
                    mout() << current_sa_angle[0] << '\t' << current_sa_angle[1] << '\t' << current_sa_angle[2] << '\t'
                        << current_sa_angle[3] << '\t' << current_sa_angle[4] << '\t' << current_sa_angle[5] << std::endl;

                }


                if (!imp_->init)
                {
                    model_a1.setInputPos(init_angle1);
                    if (model_a1.forwardKinematics())std::cout << "forward failed" << std::endl;

                    jointMove(init_angle1, 0);

                    if (motorsPositionCheck(current_sa_angle, init_angle1))
                    {
                        mout() << "Back To Init" << std::endl;


                        eeA1.getMpm(imp_->init_pm);
                        getForceData(imp_->init_force, 0);

                        mout()<<"Init Force: "<<imp_->init_force[0]<<'\t'<<imp_->init_force[1]<<'\t'<<imp_->init_force[2]<<'\t'
                                <<imp_->init_force[3]<<'\t'<<imp_->init_force[4]<<'\t'<<imp_->init_force[5]<<std::endl;

                        imp_->init = true;
                    }

                }
                else if (imp_->init && !imp_->target1_reached)
                {
                    model_a1.setInputPos(angle1_1);
                    if (model_a1.forwardKinematics())std::cout << "forward failed" << std::endl;

                    jointMove(angle1_1, 0);

                    if (motorsPositionCheck(current_sa_angle, angle1_1))
                    {
                        mout() << "Target 1 Reached" << std::endl;


                        eeA1.getMpm(imp_->ee_pm_1);

                        imp_->target1_reached = true;
                        imp_->stop_count = 1;
                        imp_->current_stop_time = count();
                        imp_->stop_flag = true;
                        mout() << "current stop time: " << imp_->current_stop_time << std::endl;
                    }

                }
                else if (imp_->target1_reached && !imp_->target2_reached)
                {
                    model_a1.setInputPos(angle1_2);
                    if (model_a1.forwardKinematics())std::cout << "forward failed" << std::endl;

                    jointMove(angle1_2, 0);

                    if (motorsPositionCheck(current_sa_angle, angle1_2))
                    {
                        mout() << "Target 2 Reached" << std::endl;

                        eeA1.getMpm(imp_->ee_pm_2);

                        imp_->target2_reached = true;
                        imp_->stop_count = 2;
                        imp_->current_stop_time = count();
                        imp_->stop_flag = true;
                        mout() << "current stop time: " << imp_->current_stop_time << std::endl;
                    }
                }
                else if (imp_->target2_reached && !imp_->target3_reached)
                {
                    model_a1.setInputPos(angle1_3);
                    if (model_a1.forwardKinematics())std::cout << "forward failed" << std::endl;

                    jointMove(angle1_3, 0);

                    if (motorsPositionCheck(current_sa_angle, angle1_3))
                    {
                        mout() << "Target 3 Reached" << std::endl;

                        eeA1.getMpm(imp_->ee_pm_3);

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
                    model_a1.setInputPos(init_angle1);
                    if (model_a1.forwardKinematics())std::cout << "forward failed" << std::endl;

                    jointMove(init_angle1, 0);

                    if (motorsPositionCheck(current_sa_angle, init_angle1))
                    {
                        mout() << "Back To Init Pos" << std::endl;
                        mout() << "Current Angle: " << current_sa_angle[0] << '\t' << current_sa_angle[1] << '\t' << current_sa_angle[2] << '\t'
                            << current_sa_angle[3] << '\t' << current_sa_angle[4] << '\t' << current_sa_angle[5] << std::endl;

                        imp_->target4_reached = true;

                    }

                }
                else if (imp_->target1_reached && imp_->target2_reached && imp_->target3_reached && imp_->target4_reached)
                {
                    double t_vector[9]{ 0 };
                    double f_vector[9]{ 0 };

                    double f_matrix[54]{ 0 };
                    double r_matrix[54]{ 0 };

                    double p_vector[6]{ 0 };
                    double l_vector[6]{ 0 };

                    double ee_rm_1[9]{ 0 };
                    double ee_rm_2[9]{ 0 };
                    double ee_rm_3[9]{ 0 };

                    double current_force[6]{ 0 };

                    aris::dynamic::s_pm2rm(imp_->ee_pm_1, ee_rm_1);
                    aris::dynamic::s_pm2rm(imp_->ee_pm_2, ee_rm_2);
                    aris::dynamic::s_pm2rm(imp_->ee_pm_3, ee_rm_3);

                    GravComp gc;

                    gc.getTorqueVector(imp_->force_data_1, imp_->force_data_2, imp_->force_data_3, t_vector);
                    gc.getForceVector(imp_->force_data_1, imp_->force_data_2, imp_->force_data_3, f_vector);

                    gc.getFMatrix(imp_->force_data_1, imp_->force_data_2, imp_->force_data_3, f_matrix);
                    gc.getRMatrix(ee_rm_1, ee_rm_3, ee_rm_3, r_matrix);

                    gc.getPLMatrix(f_matrix, t_vector, p_vector);
                    gc.getPLMatrix(r_matrix, f_vector, l_vector);

                    double current_ee_pm[16]{ 0 };
                    eeA1.getMpm(current_ee_pm);

                    gc.getCompFT(current_ee_pm, l_vector, p_vector, imp_->comp_f);
                    getForceData(current_force, 0);

                    mout() << "Current Force After Compensation:" << '\n' << current_force[0] + imp_->comp_f[0] << '\t' << current_force[1] + imp_->comp_f[1]<< '\t'
                        << current_force[2] + imp_->comp_f[2]<< '\t' << current_force[3] + imp_->comp_f[3]<< '\t'
                        << current_force[4] + imp_->comp_f[4]<< '\t' << current_force[5] + imp_->comp_f[5]<< std::endl;


                    return 0;
                }

                if (count() == 80000)
                {

                    mout() << "Over Time" << std::endl;
                }

                return 80000 - count();



            }

            // arm 2 blue arm
            else if (imp_->m_ == 1)
            {
                // 	Init Pos
                static double angle2_1[12]{ 0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, PI / 2 };

                static double angle2_2[12]{ 0, 0, -5 * PI / 6, PI / 2, PI / 2, PI / 2 };

                static double angle2_3[12]{ 0, 0, -5 * PI / 6, 2 * PI / 3, 2 * PI / 3, PI / 2 };

                double current_sa_angle[6]{ 0 };

                std::copy(current_angle + 6, current_angle + 12, current_sa_angle);

                if (count() % 1000 == 0)
                {
                    mout() << current_sa_angle[0] << '\t' << current_sa_angle[1] << '\t' << current_sa_angle[2] << '\t'
                        << current_sa_angle[3] << '\t' << current_sa_angle[4] << '\t' << current_sa_angle[5] << std::endl;

                }




                if (!imp_->target1_reached)
                {
                    model_a2.setInputPos(angle2_1);
                    if (model_a2.forwardKinematics())std::cout << "forward failed" << std::endl;

                    jointMove(angle2_1, 1);

                    if (motorsPositionCheck(current_sa_angle, angle2_1))
                    {
                        mout() << "Target 1 Reached" << std::endl;


                        eeA2.getMpm(imp_->ee_pm_1);

                        imp_->target1_reached = true;
                        imp_->stop_count = 1;
                        imp_->current_stop_time = count();
                        imp_->stop_flag = true;
                        mout() << "current stop time: " << imp_->current_stop_time << std::endl;
                    }

                }
                else if (imp_->target1_reached && !imp_->target2_reached)
                {
                    model_a2.setInputPos(angle2_2);
                    if (model_a2.forwardKinematics())std::cout << "forward failed" << std::endl;

                    jointMove(angle2_2, 1);

                    if (motorsPositionCheck(current_sa_angle, angle2_2))
                    {
                        mout() << "Target 2 Reached" << std::endl;

                        eeA2.getMpm(imp_->ee_pm_2);

                        imp_->target2_reached = true;
                        imp_->stop_count = 2;
                        imp_->current_stop_time = count();
                        imp_->stop_flag = true;
                        mout() << "current stop time: " << imp_->current_stop_time << std::endl;
                    }
                }
                else if (imp_->target2_reached && !imp_->target3_reached)
                {
                    model_a2.setInputPos(angle2_3);
                    if (model_a2.forwardKinematics())std::cout << "forward failed" << std::endl;

                    jointMove(angle2_3, 1);

                    if (motorsPositionCheck(current_sa_angle, angle2_3))
                    {
                        mout() << "Target 3 Reached" << std::endl;

                        eeA2.getMpm(imp_->ee_pm_3);

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
                    model_a2.setInputPos(angle2_1);
                    if (model_a2.forwardKinematics())std::cout << "forward failed" << std::endl;

                    jointMove(angle2_1, 1);

                    if (motorsPositionCheck(current_sa_angle, angle2_1))
                    {
                        mout() << "Back To Init Pos" << std::endl;
                        mout() << "Current Angle: " << current_sa_angle[0] << '\t' << current_sa_angle[1] << '\t' << current_sa_angle[2] << '\t'
                            << current_sa_angle[3] << '\t' << current_sa_angle[4] << '\t' << current_sa_angle[5] << std::endl;

                        imp_->target4_reached = true;

                    }

                }
                else if (imp_->target1_reached && imp_->target2_reached && imp_->target3_reached && imp_->target4_reached)
                {
                    double t_vector[9]{ 0 };
                    double f_vector[9]{ 0 };

                    double f_matrix[54]{ 0 };
                    double r_matrix[54]{ 0 };

                    double p_vector[6]{ 0 };
                    double l_vector[6]{ 0 };

                    double ee_rm_1[9]{ 0 };
                    double ee_rm_2[9]{ 0 };
                    double ee_rm_3[9]{ 0 };

                    double current_force[6]{ 0 };

                    aris::dynamic::s_pm2rm(imp_->ee_pm_1, ee_rm_1);
                    aris::dynamic::s_pm2rm(imp_->ee_pm_2, ee_rm_2);
                    aris::dynamic::s_pm2rm(imp_->ee_pm_3, ee_rm_3);

                    GravComp gc;

                    gc.getTorqueVector(imp_->force_data_1, imp_->force_data_2, imp_->force_data_3, t_vector);
                    gc.getForceVector(imp_->force_data_1, imp_->force_data_2, imp_->force_data_3, f_vector);

                    gc.getFMatrix(imp_->force_data_1, imp_->force_data_2, imp_->force_data_3, f_matrix);
                    gc.getRMatrix(ee_rm_1, ee_rm_2, ee_rm_3, r_matrix);

                    gc.getPLMatrix(f_matrix, t_vector, p_vector);
                    gc.getPLMatrix(r_matrix, f_vector, l_vector);

                    double current_ee_pm[16]{ 0 };
                    eeA2.getMpm(current_ee_pm);

                    gc.getCompFT(current_ee_pm, l_vector, p_vector, imp_->comp_f);
                    getForceData(current_force, 1);

                    mout() << "Current Force After Compensation:" << '\n' << current_force[0] + imp_->comp_f[0] << '\t' << current_force[1] + imp_->comp_f[1] << '\t'
                        << current_force[2] + imp_->comp_f[2] << '\t' << current_force[3] + imp_->comp_f[3] << '\t'
                        << current_force[4] + imp_->comp_f[4] << '\t' << current_force[5] + imp_->comp_f[5] << std::endl;

                    mout() << "Current End Pos:" << '\n' << current_ee_pm[0] << '\t' << current_ee_pm[1] << '\t'
                        << current_ee_pm[2] << '\t' << current_ee_pm[3] << '\t'
                        << current_ee_pm[4] << '\t' << current_ee_pm[5] << std::endl;

                    return 0;
                }

                if (count() == 80000)
                {

                    mout() << "Over Time" << std::endl;
                }

                return 80000 - count();





            }
            // wrong model
            else
            {
                mout() << "Wrong Model" << std::endl;
                throw std::runtime_error("Arm Type Error");
                return 0;
            }
        }












    }
    ModelComP2::ModelComP2(const std::string& name) : imp_(new Imp)
    {

        aris::core::fromXmlString(command(),
            "<Command name=\"m_c2\">"
            "	<GroupParam>"
            "	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
            "	</GroupParam>"
            "</Command>");
    }
    ModelComP2::~ModelComP2() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(ModelComP2)






	ARIS_REGISTRATION {
		aris::core::class_<ModelInit>("ModelInit")
			.inherit<aris::plan::Plan>();
		aris::core::class_<ModelForward>("ModelForward")
			.inherit<aris::plan::Plan>();
		aris::core::class_<ModelGet>("ModelGet")
			.inherit<aris::plan::Plan>();
		aris::core::class_<ModelMoveX>("ModelMoveX")
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
        aris::core::class_<ModelComP2>("ModelComP2")
            .inherit<aris::plan::Plan>();

	}



}
