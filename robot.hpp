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

	class ModelMoveX :public aris::core::CloneObject<ModelMoveX, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ModelMoveX();
		explicit ModelMoveX(const std::string& name = "ModelMoveX");
		KAANH_DECLARE_BIG_FOUR(ModelMoveX)
	private:
		int m_;
		double d_;
		double o_;
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

    class ModelComP2 :public aris::core::CloneObject<ModelComP2, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~ModelComP2();
        explicit ModelComP2(const std::string& name = "ModelComP2");
        KAANH_DECLARE_BIG_FOUR(ModelComP2)

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;

    };

}




