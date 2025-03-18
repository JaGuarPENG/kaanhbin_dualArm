#include "aris.hpp"
#include "kaanh/log/log.hpp"
#include "kaanh/robot/robot.hpp"
#include "kaanhbot/robot/robot.hpp"
#include "kaanhbot/config/kaanhconfig.hpp"
#include "aris_version.h"
#include "kaanh_version.h"
#include "kaanhbot_version.h"
#include "kaanhbin_version.h"
#include "aris/core/version.hpp"
#include "kaanh/general/version.hpp"
#include "kaanhbot/system/version.hpp"
#include "kaanhbot/system/about.hpp"
#include "kaanhbot/utility/tool_api.hpp"
#include "plan.hpp"

#include "robot.hpp"
#include "assemcomd.hpp"


#define __S(x) #x
#define _S(x) __S(x)

auto xmlpath = std::filesystem::absolute(".");	//获取当前工程所在的路径
auto logpath = std::filesystem::absolute(".");
const std::string xmlfile = "kaanh.xml";
const std::string logfolder = "log";

int main(int argc, char *argv[]){
	xmlpath = xmlpath / xmlfile;
	logpath = logpath / logfolder;

	auto&cs = aris::server::ControlServer::instance();
	auto port = argc < 2 ? 5866 : std::stoi(argv[1]);
	auto path = argc < 2 ? xmlpath : argv[2];
	auto logp = argc < 2 ? logpath : argv[3];

	std::cout << "port:" << port << std::endl;
	std::cout << "xmlpath:" << xmlpath << std::endl;
	std::cout << "path:" << path << std::endl;
	std::cout << "logfolder:" << logp << std::endl;

	// 重载Aris log接口
	aris::core::setLogMethod([](aris::core::LogData data)->void {
		// switch (data.level) {
		// case aris::core::LogLvl::kError :
		// case aris::core::LogLvl::kFatal :
		// 	ERROR_SYS_NC("Aris -- "+data.msg, data.code);
		// 	break;
		// case aris::core::LogLvl::kDebug :
		// case aris::core::LogLvl::kInfo :
		// default :
		// 	break;
		// } 
		if ((int)(data.level)>(int)(aris::core::LogLvl::kInfo))
			std::cout << "aris-log -- " << data.msg << std::endl;
	});

	aris::core::fromXmlFile(cs, path);

    cs.init();


    // 修改末端杆件位姿,需要重写xml
   {
       auto& model = dynamic_cast<aris::dynamic::Model&>(dynamic_cast<aris::dynamic::MultiModel&>(cs.model()).subModels()[0]);
        double input_pos[7]{0,0,0,0,0,0,0};
        model.setInputPos(input_pos);
        model.forwardKinematics();

       if (model.name() == "PumaModel" && model.findPart("EE")->geometryPool().size())
       {
            auto pm = *model.findPart("EE")->pm();
            auto& geo = dynamic_cast<aris::dynamic::FileGeometry&>(model.findPart("EE")->geometryPool()[0]);
            std::string path = geo.filePath();
            model.findPart("EE")->geometryPool().clear();
            model.findPart("EE")->geometryPool().add<aris::dynamic::FileGeometry>(path,pm);
       }
        else if (model.name() == "UrModel" && model.findPart("L6")->geometryPool().size())
        {
            auto pm = *model.findPart("L6")->pm();
            auto& geo = dynamic_cast<aris::dynamic::FileGeometry&>(model.findPart("L6")->geometryPool()[0]);
            std::string path = geo.filePath();
            model.findPart("L6")->geometryPool().clear();
            model.findPart("L6")->geometryPool().add<aris::dynamic::FileGeometry>(path,pm);
        }
        else if (model.name() == "ScaraModel" && model.findPart("L4")->geometryPool().size())
        {
            auto pm = *model.findPart("L4")->pm();
            auto& geo = dynamic_cast<aris::dynamic::FileGeometry&>(model.findPart("L4")->geometryPool()[0]);
            std::string path = geo.filePath();
            model.findPart("L4")->geometryPool().clear();
            model.findPart("L4")->geometryPool().add<aris::dynamic::FileGeometry>(path,pm);

        }
        else if (model.name() == "DeltaModel")
        {

        }
        else{}
        kaanhbot::utility::saveCs();
   }

	// 重载Aris log接口
	aris::core::setLogMethod([](aris::core::LogData data)->void {
		// switch (data.level) {
		// case aris::core::LogLvl::kError :
		// case aris::core::LogLvl::kFatal :
		// 	ERROR_SYS_NC("Aris -- "+data.msg, data.code);
		// 	break;
		// case aris::core::LogLvl::kDebug :
		// case aris::core::LogLvl::kInfo :
		// default :
		// 	break;
		// } 
		std::cout << "aris-log -- " << data.msg << std::endl;
	});

    std::cout <<"motor size: "<< cs.controller().motorPool().size() << std::endl;
	std::cout <<"model motion size: "<< cs.model().inputPosSize() << std::endl;
    std::cout <<"io size: "<< cs.controller().digitalIoPool().size() << std::endl;
    std::cout <<"fs size: "<< cs.controller().ftSensorPool().size() << std::endl;

	// 设置版本回调
    kaanhbot::system::About::registerCbk("aris", []()->std::pair<std::string, std::string> {
        return std::make_pair(std::string(std::string("v") + _S(ARIS_VERSION_MAJOR) + "." + _S(ARIS_VERSION_MINOR) + "." + _S(ARIS_VERSION_PATCH) + "." + _S(ARIS_VERSION_TIME)), std::string(_S(ARIS_DESCRIPTION)));
        }, aris::core::version);
    kaanhbot::system::About::registerCbk("kaanh", []()->std::pair<std::string, std::string> {
        return std::make_pair(std::string(std::string("v") + _S(KAANH_VERSION_MAJOR) + "." + _S(KAANH_VERSION_MINOR) + "." + _S(KAANH_VERSION_PATCH) + "." + _S(KAANH_VERSION_TIME)), std::string(_S(KAANH_DESCRIPTION)));
        }, kaanh::general::version);
    kaanhbot::system::About::registerCbk("kaanhbot", []()->std::pair<std::string, std::string> {
        return std::make_pair(std::string(std::string("v") + _S(KAANHBOT_VERSION_MAJOR) + "." + _S(KAANHBOT_VERSION_MINOR) + "." + _S(KAANHBOT_VERSION_PATCH) + "." + _S(KAANHBOT_VERSION_TIME)), std::string(_S(KAANHBOT_DESCRIPTION)));
        }, kaanhbot::system::version);
    kaanhbot::system::About::registerCbk("kaanhbin", []()->std::pair<std::string, std::string> {
        return std::make_pair(std::string(std::string("v") + _S(KAANHBIN_VERSION_MAJOR) + "." + _S(KAANHBIN_VERSION_MINOR) + "." + _S(KAANHBIN_VERSION_PATCH) + "." + _S(KAANHBIN_VERSION_TIME)), std::string(_S(KAANHBIN_DESCRIPTION)));
        }, []()->std::pair<std::string, std::string> {
            return std::make_pair(std::string(std::string("v") + _S(KAANHBIN_VERSION_MAJOR) + "." + _S(KAANHBIN_VERSION_MINOR) + "." + _S(KAANHBIN_VERSION_PATCH) + "." + _S(KAANHBIN_VERSION_TIME)), std::string(_S(KAANHBIN_DESCRIPTION)));
        });

	// 防止kaanhbot库被编译器优化
	kaanhbot::robot::Robot::instanceInCs();

	// 重载Aris Rt-log接口
    auto func = [](aris::plan::Plan *p, int error_num, const char *error_msg) {
        auto err_num_abs = std::abs(error_num);
        if((err_num_abs & 0xffff) < 0x1000) RT_ERROR_SYS_NC(error_msg, error_num)

    };
    cs.setRtErrorCallback(func);

    try{
        auto & log = kaanh::log::Sqlite3Log::instance();
        log.initErrorConfig();
        log.createErrorConfig();
    }catch(const std::exception& err){
        WARNING_CFG_NC(LOCALE_SELECT("not find error.txt","未发现错误配置文件"),-1);
    }
    //实时回调函数，每个实时周期调用一次
    cs.setRtPlanPreCallback([](aris::server::ControlServer&cs){
        static int32_t update_counter = 0;
        if(update_counter < 2000){
            update_counter++;
            return;
        }
        kaanh::robot::Robot::instanceInCs().rtUpdate(cs);
    });

    // for(aris::Size i = 0; i < cs.controller().motorPool().size(); i++){
    //     cs.idleMotionCheckOption()[i] |= aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
    //     cs.globalMotionCheckOption()[i] |= aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
    // }

    
    //开启控制器服务
    try {
        cs.start();
        INFO_SYS(LOCALE_SELECT("system starts", "系统启动"), 0)
        #ifdef __aarch64__
        int counts = 60;
        std::string filename="doc/tmp_slave.txt";
        if(!std::filesystem::exists(filename)){
            if((!std::filesystem::is_directory("./doc") )) std::filesystem::create_directory("doc");
            std::fstream fstrm(filename, std::ios::out | std::ios::trunc);
            fstrm.close();
        }
        while(counts-->0){
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            if(system(("ethercat slave > " + filename).c_str())!=0) continue;
            std::ifstream  ifs(filename);
            std::string ethercat_slave_str;
            int i=0;
            bool flag{true};
            while(getline(ifs,ethercat_slave_str)&&i<100){
                // 0  0:0  PREOP + CoolDrive RC
                // 0  0:0  PREOP E CoolDriver
                // 0  0:0  SAFEOP+ERROR E CoolDrive  RC
                std::regex ruler("[0-9]+\\s+[0-9]+:[0-9]+\\s+([[:w:](+)(0-9)(_)]+)\\s+[[:w:](+)(0-9)(_)]+\\s+(.+)");
                std::sregex_iterator pos(ethercat_slave_str.cbegin(),ethercat_slave_str.cend(),ruler);
                std::sregex_iterator end;
                std::string ethercat_status;
		        // std::string ethercat_name;
                if(pos!=end){
                    ethercat_status=pos->str(1);
		            // ethercat_name=pos->str(2);
			        // std::cout << ethercat_status << ":" << ethercat_name << std::endl;
                    if(ethercat_status != "OP"){
                        flag = false;
                    }
                }
                i++;
            }
            if(flag) break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10000));
        #endif
        //电机抱闸设置恢复
//        int driver_num=cs.controller().motorPool().size();
//        for(int i= 0; i < driver_num; i++){
//            auto &motor=cs.controller().motorPool().at(i);
//            motor.setOutputIoNrt(1,0x00);
//            motor.setOutputIoNrt(2,0x00);
//        }

        cs.executeCmd("md");

        cs.executeCmd("rc");
    }
    catch (const std::exception& err){
        kaanh::log::Sqlite3Log::instance().triggerState(0x3509);
    }

    
	//Start Web Socket
	cs.open();

	//Receive Command
	cs.runCmdLine();

	return 0;
}
