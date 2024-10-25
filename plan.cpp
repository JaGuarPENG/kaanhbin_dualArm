#include "plan.hpp"


#include "kaanhbot/utility/kaanh.hpp"
#include "kaanhbot/io/io_plan.hpp"
#include "kaanhbot/functional/function_gather.hpp"
#include "kaanhbot/protocol/modbus_tcp_server.hpp"
#include "kaanhbot/config/external_axis.hpp"
#include "kaanhbot/calibration/calibration.hpp"
#include "kaanh/package/force.hpp"
#include "kaanh/current/current.hpp"
#include "kaanh/multi_motion/jog_plan.hpp"
#include "kaanh/multi_motion/motion_plan.hpp"
#include "kaanhbot/motion/move_find.hpp"
#include "kaanhbot/conveyor/conveyor_motion.hpp"
#include "kaanhbot/conveyor/conveyor_motion.hpp"
#include "kaanh/module/splc_controller.hpp"

auto creatPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>
{
    std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

	// system
	plan_root->planPool().add<kaanhbot::general::Mode>();
	plan_root->planPool().add<kaanhbot::general::Start>();
	plan_root->planPool().add<kaanhbot::general::Stop>();
	plan_root->planPool().add<kaanhbot::general::Clear>();
	plan_root->planPool().add<kaanhbot::general::CancelEStop>();
	plan_root->planPool().add<kaanhbot::general::SaveHome>();
	plan_root->planPool().add<kaanhbot::general::Sleep>();
	plan_root->planPool().add<kaanhbot::general::Recover>();
	plan_root->planPool().add<kaanhbot::general::Reset>();
	plan_root->planPool().add<kaanhbot::general::ManualStop>();
	plan_root->planPool().add<kaanhbot::general::DeleteFile>();
	plan_root->planPool().add<kaanhbot::general::SetDH>();
	plan_root->planPool().add<kaanhbot::general::SaveXml>();
	plan_root->planPool().add<kaanhbot::general::ScanSlave>();
	plan_root->planPool().add<kaanhbot::general::GetXml>();
	plan_root->planPool().add<kaanhbot::general::SetXml>();
	plan_root->planPool().add<kaanhbot::general::SetPgmVel>();
	plan_root->planPool().add<kaanhbot::general::SetJogVel>();
	plan_root->planPool().add<kaanhbot::general::SetTool>();
	plan_root->planPool().add<kaanhbot::general::SetWobj>();
	plan_root->planPool().add<kaanhbot::general::SetJogCoordinate>();
	plan_root->planPool().add<kaanhbot::general::SetJogRunMode>();
	plan_root->planPool().add<kaanhbot::general::SetRobotOpMode>();

	// yw
	plan_root->planPool().add<kaanhbot::config::MoveEa>();
	plan_root->planPool().add<kaanhbot::calibration::CalibT2P>();
	plan_root->planPool().add<kaanhbot::calibration::CalibW3P>();
	plan_root->planPool().add<kaanhbot::calibration::CalibT4P>();
	plan_root->planPool().add<kaanhbot::calibration::CalibT5P>();
	plan_root->planPool().add<kaanhbot::calibration::CalibT6P>();
	plan_root->planPool().add<kaanhbot::calibration::ManualSetTool>();
	plan_root->planPool().add<kaanhbot::calibration::GetTool>();
	plan_root->planPool().add<kaanhbot::calibration::ManualSetWobj>();
	plan_root->planPool().add<kaanhbot::calibration::GetWobj>();
	plan_root->planPool().add<kaanhbot::general::GetPe>();
	plan_root->planPool().add<kaanh::package::CCStop>();
	plan_root->planPool().add<kaanh::package::CalibDynPar>();
	plan_root->planPool().add<kaanh::package::CurrentGuidance>();
	plan_root->planPool().add<kaanh::multi_motion::JogJ>();
	plan_root->planPool().add<kaanh::multi_motion::JogC>();
	plan_root->planPool().add<kaanh::multi_motion::MoveAbsJ>();
//	plan_root->planPool().add<kaanh::multi_motion::MoveAbsJTrig>();
	plan_root->planPool().add<kaanh::multi_motion::MoveL>();
//	plan_root->planPool().add<kaanh::multi_motion::MoveLTrig>();
	plan_root->planPool().add<kaanh::multi_motion::MoveC>();
//	plan_root->planPool().add<kaanh::multi_motion::MoveCTrig>();
	plan_root->planPool().add<kaanh::multi_motion::MoveLRel>();
	plan_root->planPool().add<kaanh::multi_motion::RePlay>();
	plan_root->planPool().add<kaanh::multi_motion::MoveArchP>();
	plan_root->planPool().add<kaanh::multi_motion::MoveArch>();
	plan_root->planPool().add<kaanh::multi_motion::ManualMoveAbsJ>();
	plan_root->planPool().add<kaanh::multi_motion::ManualMoveL>();
	plan_root->planPool().add<kaanhbot::motion::MoveLFind>();
	plan_root->planPool().add<kaanhbot::general::SetPdo>();
	plan_root->planPool().add<kaanhbot::general::GetPdo>();
	plan_root->planPool().add<kaanhbot::general::Reset>();
    plan_root->planPool().add<kaanhbot::general::Reset>();
    plan_root->planPool().add<kaanh::module::pauseProgram>();
	plan_root->planPool().add<kaanhbot::general::SetToolMode>();
    // yw conveyor
    plan_root->planPool().add<kaanhbot::package::ConveyorFastCatch>();
    plan_root->planPool().add<kaanhbot::package::MoveConveyor>();
    plan_root->planPool().add<kaanhbot::package::SyncStop>();
	
	// // xxt


	// lcz
	plan_root->planPool().add<kaanhbot::io::WaitDi>();
	plan_root->planPool().add<kaanhbot::functional::WaitMultiDi>();
	plan_root->planPool().add<kaanhbot::protocol::MtcpRoWaitBool>();
	plan_root->planPool().add<kaanhbot::protocol::MtcpWrWaitBool>();
	plan_root->planPool().add<kaanhbot::io::Pulse>();
	plan_root->planPool().add<kaanhbot::io::ManualSetDo>();
	plan_root->planPool().add<kaanh::package::MoveF>();
	plan_root->planPool().add<kaanh::package::ForceControlInit>();
	plan_root->planPool().add<kaanh::package::AttitudeAdjust>();
	plan_root->planPool().add<kaanh::package::SetDir>();
	plan_root->planPool().add<kaanh::package::FCStop>();
	plan_root->planPool().add<kaanh::package::FindHole>();
	plan_root->planPool().add<kaanh::package::Assemble>();
	plan_root->planPool().add<kaanh::package::ForceGuidanceBeforeRealForce>();
	plan_root->planPool().add<kaanh::package::FCMonitor>();
	plan_root->planPool().add<kaanh::package::SaveProXml>();
	plan_root->planPool().add<kaanh::package::LoadProXml>();
	plan_root->planPool().add<kaanh::package::ResetFCMoitor>();
    plan_root->planPool().add<kaanh::package::ForceAssembleInit>();
    plan_root->planPool().add<kaanh::package::AxisAssemble>();
    plan_root->planPool().add<kaanh::package::SetForceLoadId>();
	plan_root->planPool().add<kaanh::package::SetZero>();
	plan_root->planPool().add<kaanh::package::Kunwei>();
	plan_root->planPool().add<kaanh::package::Ati>();
	plan_root->planPool().add<kaanh::package::Yuli>();
	plan_root->planPool().add<kaanh::package::ForceSensorGuidance>();
	plan_root->planPool().add<kaanh::package::ForceTrace>();
	plan_root->planPool().add<kaanhbot::io::SetDOPlan>();
	plan_root->planPool().add<kaanhbot::io::SetDOsPlan>();

    return plan_root;
}
