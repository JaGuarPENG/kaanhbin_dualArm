#include <aris.hpp>

#include "curve.hpp"
#include "kaanh/general/macro.hpp"
#include "kaanh/module/middle_module.hpp"
#include "kaanh/middleware/middleware.hpp"


const double PI = aris::PI;

namespace robot
{


	class ModelInit : public aris::core::CloneObject<ModelInit, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		virtual ~ModelInit();
		explicit ModelInit(const std::string& name = "init");
		KAANH_DECLARE_BIG_FOUR(ModelInit)

	private:

	};


    class Arm2Init : public aris::core::CloneObject<Arm2Init, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        virtual ~Arm2Init();
        explicit Arm2Init(const std::string& name = "Arm2Init");
        KAANH_DECLARE_BIG_FOUR(Arm2Init)

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;

    };




	class ModelForward :public aris::core::CloneObject<ModelForward, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ModelForward();
		explicit ModelForward(const std::string& name = "ModelForward");
		KAANH_DECLARE_BIG_FOUR(ModelForward)

	private:

	};



	class ModelGet : public aris::core::CloneObject<ModelGet, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ModelGet();
		explicit ModelGet(const std::string& name = "ModelGet");
		KAANH_DECLARE_BIG_FOUR(ModelGet)

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;

	};

    class ModelTest :public aris::core::CloneObject<ModelTest, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

        virtual ~ModelTest();
        explicit ModelTest(const std::string& name = "ModelTest");
        KAANH_DECLARE_BIG_FOUR(ModelTest)
	private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;
	};


    class ModelSetPos :public aris::core::CloneObject<ModelSetPos, aris::plan::Plan>
        {
        public:
            auto virtual prepareNrt()->void;
            auto virtual executeRT()->int;

            virtual ~ModelSetPos();
            explicit ModelSetPos(const std::string& name = "model_set_pos");
            KAANH_DECLARE_BIG_FOUR(ModelSetPos)
         private:
			 struct Imp;
			 aris::core::ImpPtr<Imp> imp_;

        };

	class ModelComP :public aris::core::CloneObject<ModelComP, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ModelComP();
		explicit ModelComP(const std::string& name = "ModelComP");
		KAANH_DECLARE_BIG_FOUR(ModelComP)

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

	};

	class ForceAlign :public aris::core::CloneObject<ForceAlign, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ForceAlign();
		explicit ForceAlign(const std::string& name = "ForceAlign");
		KAANH_DECLARE_BIG_FOUR(ForceAlign)

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

	};

	class ForceKeep :public aris::core::CloneObject<ForceKeep, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ForceKeep();
		explicit ForceKeep(const std::string& name = "ForceKeep");
		KAANH_DECLARE_BIG_FOUR(ForceKeep)

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;


	};

	class ForceDrag :public aris::core::CloneObject<ForceDrag, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ForceDrag();
		explicit ForceDrag(const std::string& name = "ForceDrag");
		KAANH_DECLARE_BIG_FOUR(ForceDrag)

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;



	};


	class PegInHole :public aris::core::CloneObject<PegInHole, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~PegInHole();
		explicit PegInHole(const std::string& name = "PegInHole");
        KAANH_DECLARE_BIG_FOUR(PegInHole)

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

		
	};

    class HoleInPeg :public aris::core::CloneObject<HoleInPeg, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~HoleInPeg();
        explicit HoleInPeg(const std::string& name = "HoleInPeg");
        KAANH_DECLARE_BIG_FOUR(HoleInPeg)

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;


    };

	class PegOutHole :public aris::core::CloneObject<PegOutHole, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~PegOutHole();
		explicit PegOutHole(const std::string& name = "PegOutHole");
		KAANH_DECLARE_BIG_FOUR(PegOutHole)
	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

	};


}




