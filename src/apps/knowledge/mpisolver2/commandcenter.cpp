#include "commandcenter.h"

// Just for sleep, which might be not needed in future
#include <boost/thread/thread.hpp> 
// To manipulate the files
#include <boost/filesystem.hpp> 
#include <boost/algorithm/string.hpp>

#include <mpi.h>

#include "utils/Paths.h"
#include "common/app_cppcommon.h"
#include "controller/rootchild.h"
#include "controller/SimPack.h"

#include "database.h"
#include "explorer.h"
#include "evaluator.h"
#include "optimizer.h"
#include "dbxmlwriter.h"

// #define INSTRUCTION_DIR "command"
// #define INSTRUCTION_DIR "/nv/hp16/sha9/scratch"
#define COMMAND_FILE (isLocal() ? "command/command.xml" : "/nv/hp16/sha9/scratch/command/command.xml")
#define STATUS_FILE (isLocal() ? "command/status.xml" : "/nv/hp16/sha9/scratch/command/status.xml")

namespace mpisolver2 {
    CommandCenter::CommandCenter(int _nProc, bool _isLocal)
        : MEMBER_INIT_NULL(simpack)
        , MEMBER_INIT(nProc, _nProc)
        , MEMBER_INIT(isPaused, false)
        , MEMBER_INIT(isLocal, _isLocal)
        , MEMBER_INIT_NULL(database)
        , MEMBER_INIT_NULL(explorer)
        , MEMBER_INIT_NULL(evaluator)
        , MEMBER_INIT_NULL(optimizer)
    {
        set_database( new Database() );
        set_simpack( new controller::SimPack() );
        //simpack()->init(RTQL8_DATA_PATH"xml/temp.xml"); // Should be modified
        simpack()->init(); // Should be modified

        set_explorer( new NaiveExplorer(this) );
        // set_explorer( new DartThrowExplorer(this) );
        set_evaluator( new Evaluator(this) );
        set_optimizer( new Optimizer(this) );

        LOG(INFO) << "isLocal = " << isLocal();
        LOG(INFO) << "COMMAND_FILE = " << COMMAND_FILE;
        LOG(INFO) << "STATUS_FILE = " << STATUS_FILE;
        
        boost::filesystem::remove(COMMAND_FILE);
    }
    
    CommandCenter::~CommandCenter() {

    }

    bool CommandCenter::mainloop() {
        LOG(INFO) << FUNCTION_NAME();
        writeStatus("started");
        // explorer()->start();
        while(true) {
            std::string cmd = readCommand(); 
            if (cmd == "") {
                LOG_EVERY_N(INFO, 100) << FUNCTION_NAME() << " is idling: isPaused = " << isPaused();
                boost::this_thread::sleep(
                    boost::posix_time::milliseconds(50) );               

            } else {
                LOG(INFO) << FUNCTION_NAME() << " : command = [" << cmd << "]";
                if (cmd == "stop") {
                    LOG(INFO) << FUNCTION_NAME() << " : stop solver";
                    explorer()->stop();
                    optimizer()->stop();
                    quit();
                    writeStatus("solver terminated OK");
                    break;
                } else if (cmd == "pause") {
                    LOG(INFO) << FUNCTION_NAME() << " : pause solver";
                    set_isPaused(true);
                    writeStatus("solver paused");
                } else if (cmd == "resume") {
                    LOG(INFO) << FUNCTION_NAME() << " : resume solver";
                    if (explorer()->isRunning() == false
                        && optimizer()->isRunning() == false) {
                        LOG(INFO) << FUNCTION_NAME() << " : DO THE EXPLORATION";
                        explorer()->start();
                    }
                    set_isPaused(false);
                    writeStatus("solver resumed");
                } else if (cmd == "evaluate") {
                    LOG(INFO) << FUNCTION_NAME() << " : reevaluate samples";
                    evaluator()->reevaluate();
                    writeStatus("solver reevaluates the samples");

                } else if (cmd == "optimize") {
                    LOG(INFO) << FUNCTION_NAME() << " : start optimization";
                    optimizer()->start();
                    explorer()->stop(); 
                    set_isPaused(false);
                    writeStatus("optimization finished");
                } else if (cmd == "explore") {
                    LOG(INFO) << FUNCTION_NAME() << " : reevaluate samples";
                    explorer()->start();
                    optimizer()->stop();
                    writeStatus("solver starts the exploration");
                } else if (cmd == "newcontroller") {
                    LOG(INFO) << FUNCTION_NAME() << " : newcontroller";
                } else if (cmd == "request") {
                    LOG(INFO) << FUNCTION_NAME() << " : request";
                    DBXMLWriter writer(this);
                    writer.write();
                    if (explorer()->isRunning()) {
                        explorer()->incStartCount();
                    }
                    writeStatus("new solutions reported");

                } else if (cmd == "dump") {
                    LOG(INFO) << FUNCTION_NAME() << " : dump";
                    database()->dump();
                    writeStatus(database()->summary().c_str());
                }
            }

            if (!isPaused() && explorer()->isRunning()) {
                explorer()->mainloop();
            }
            if (!isPaused() && optimizer()->isRunning()) {
                optimizer()->mainloop();
            }
        }
        
        LOG(INFO) << FUNCTION_NAME() << " OK";
    }

    int CommandCenter::dim() {
        return simpack()->con()->dim();
    }

    void CommandCenter::quit() {
        const int START = 1;
        const int BUFF_SIZE = 2048;
        char buff[BUFF_SIZE];
        sprintf(buff, "QUIT");
        for (int id = START; id <= nChildProc(); id++) {
            MPI_Send(&buff, BUFF_SIZE, MPI_PACKED, id, id, MPI_COMM_WORLD);
        }    
        // printf("All QUIT messages sent.\n");
        LOG(INFO) << "All QUIT messages sent.";

        LOG(INFO) << FUNCTION_NAME() << " OK";
    }

    std::string CommandCenter::readCommand() {
        const std::string empty("");
        if (boost::filesystem::exists(COMMAND_FILE) == false) {
            return empty;
        }
        try {
            rapidxml::file<> xmlFile(COMMAND_FILE);
            rapidxml::xml_document<> doc;
            doc.parse<0>(xmlFile.data());
            
            rapidxml::xml_node<>* root = doc.first_node();
            if (!root) {
                return empty;
            }
            std::string name = root->name();
            if (name == "newcontroller") {
                onNewInstruction(root);
            }

            boost::filesystem::remove(COMMAND_FILE);
            return name;
            
        } catch (const std::exception& e) {
            cout << "error = " << e.what() << endl;
        }
        return empty;
    }

    bool CommandCenter::writeStatus(const char* const status) {
        LOG(INFO) << FUNCTION_NAME();
        std::ofstream fout(STATUS_FILE);
        fout << "<status>" << status << "</status>" << endl;
        fout.close();
        cout << status << endl;
        LOG(INFO) << FUNCTION_NAME() << " OK";
    	return true;
    }

    void CommandCenter::onNewInstruction(rapidxml::xml_node<>* root) {
        set_isPaused(true);
        LOG(INFO) << FUNCTION_NAME();
        std::string filename = root->value();
        boost::algorithm::trim(filename);
        LOG(INFO) << FUNCTION_NAME() << boost::format(" filename = [%s]") % filename;

        // Step 1. Replace my controller
        simpack()->initController(filename.c_str());
        LOG(INFO) << FUNCTION_NAME() << " init controller OK";

        // Step 2. Tell the workers to replace their controller
        const int START = 1;
        const int BUFF_SIZE = 2048;

        char buff[BUFF_SIZE];
        sprintf(buff, "newcontroller");
        LOG(INFO) << FUNCTION_NAME() << " send msg to child. buff = [" << buff << "]";
        for (int id = START; id <= nChildProc(); id++) {
            MPI_Send(&buff, BUFF_SIZE, MPI_PACKED, id, id, MPI_COMM_WORLD);
        }

        // Step 3. Tell them the filename to follow
        sprintf(buff, "%s", filename.c_str());
        LOG(INFO) << FUNCTION_NAME() << " send msg to child. buff = [" << buff << "]";
        for (int id = START; id <= nChildProc(); id++) {
            MPI_Send(&buff, BUFF_SIZE, MPI_PACKED, id, id, MPI_COMM_WORLD);
        }

        // Step 4. Update db
        evaluator()->reevaluate();
        
        LOG(INFO) << FUNCTION_NAME() << " OK";

        writeStatus("controller updated");

    }


} // namespace mpisolver2
