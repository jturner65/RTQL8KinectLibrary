#include "GuiKinController.h"
#include "utils/UtilsMath.h"
#include <sstream>
#include "kinect/KinCntrlrXMLReader.h"
#include <boost/algorithm/string.hpp>
#include "common/app_cppcommon.h"
#include "common/render_cppcommon.h"
#include "common/fullbody1.h"

#include "controller/knowledge.h"
#include "controller/simpack.h"
#include "controller/app_composite_controller.h"
#include "controller/rapidxml_helper.h"

#include "application.h"

using namespace rtql8::kinect;

namespace gui {

    GuiKinController::GuiKinController(Application* _app)
        : KinectController()
        , app(_app) {
        rtql8::kinect::KinCntrlrXMLReader config(this);
        config.readFile(RTQL8_DATA_PATH"kinectui.xml");

        buildCustomUI();
        updateCustomUI();
        buildDynamicPage();
        buildDynamicPageForSelect();


        assignKinectControllerToAllUIComponents();
        
        dict.clear();
        KC_flags[_KC_PROC_2D_DGRAB]  = false;

        // resetAllTimers();
        // setFlag(_KC_PROC_IK, true);
        setFlag(_KC_PROC_HANDS_2D, true);

    }//cnstrctr

    void GuiKinController::buildCustomUI() {
        optProgressBar = new KinUIPrgBar(450, 720);
        optProgressBar->setDim(50, 50);
        optProgressBar->setColor(1.0, 0.0, 1.0, 0.4);
        optProgressBar->setIsHoriz(true);
        optProgressBar->setIsLToR(true);
        optProgressBar->setIsIncr(true);
        optProgressBar->setSlideMin(0.0);
        optProgressBar->setSlideVals(100,0,0,300,50);
        optProgressBar->setLabel("Opt");
        optProgressBar->setSlideRng(15.0);
        this->setOptProgressBar(0.0);

        {
            int SIZE = 100;
            KinUIImageButton* btn = new KinUIImageButton(0, 0);
            btn->setLoc(320, 695);
            btn->setDim(SIZE, SIZE);
            btn->setColor(1.0, 0.0, 1.0, 0.9);
            btn->setLabel("OK");
            btn->setHotSpot(30, 30);
            std::string onclick = "";
            btn->setOnClick(onclick);
            btn->kinectController = this;
            std::string path = RTQL8_DATA_PATH;
            btn->setImgSrcFileName(path + "btnpurple.png");          

            terminateOpt = btn;
            manualUICmp.push_back(terminateOpt);
        }

        {
            int SIZE = 100;
            KinUIImageButton* btn = new KinUIImageButton(0, 0);
            btn->setLoc(780, 695);
            btn->setDim(SIZE, SIZE);
            btn->setColor(1.0, 0.0, 1.0, 0.9);
            btn->setLabel("Next");
            btn->setHotSpot(30, 30);
            std::string onclick = "";
            btn->setOnClick(onclick);
            btn->kinectController = this;
            std::string path = RTQL8_DATA_PATH;
            btn->setImgSrcFileName(path + "btnpurple.png");          

            nextStyle = btn;
            manualUICmp.push_back(nextStyle);
        }


    }


    void GuiKinController::updateCustomUI() {
        {
            std::stringstream sout;
            int num = app->solver_num_styles();
            int idx = app->solver_current_style() + 1;
            if (num == 0) idx = 0;
            sout << idx << "/" << num;
            nextStyle->setLabel(sout.str());
        }
    }
    
    void GuiKinController::buildDynamicPage() {//build initial page of commands
        KinUIPage* page = new KinUIPage();
        page->name = "create-prev";
        controller::Knowledge* kn = app->kn();
        std::vector<std::string> names = kn->getCompositeControllerNames();
        double x = 50;
        double y = 50;
        double DX = 150;
        double SX = 100;
        double SY = 100;
        double A = 0.7;
        for (int i = 0; i < names.size(); i++) {
            if (i == 8) {
                x = 50;
                y = 550;
            }

            KinUIImageButton* btn = new KinUIImageButton(0, 0);
            btn->setLoc(x, y);
            btn->setDim(SX, SY);
            double r = rtql8::utils::random(0.0, 1.0);
            double g = rtql8::utils::random(0.0, 1.0);
            double b = rtql8::utils::random(0.0, 1.0);
            btn->setColor(r, g, b, A);
            btn->setLabel(names[i].c_str());
            btn->setHotSpot(30, 30);
            std::string onclick
                = (boost::format("set:prev %s;page:create-terminate") % names[i]).str();
            btn->setOnClick(onclick);
            std::string path = RTQL8_DATA_PATH;
            btn->setImgSrcFileName(path + "btnblue.png");          

            LOG_INFO << FUNCTION_NAME() << " : " << names[i] << ",onclick = " << onclick;
            page->addUIImgBtn(btn);
            x += DX;
        }
        this->addPage(page);
    }

    void GuiKinController::buildDynamicPageForSelect() {
        KinUIPage* page = new KinUIPage();
        page->name = "select";
        controller::Knowledge* kn = app->kn();
        std::vector<std::string> names = kn->getCompositeControllerNames();
        double x = 50;
        double y = 50;
        double DX = 150;
        double SX = 100;
        double SY = 100;
        double A = 0.7;
        for (int i = 0; i < names.size(); i++) {
            if (i == 8) {
                x = 50;
                y = 550;
            }

            KinUIImageButton* btn = new KinUIImageButton(0, 0);
            btn->setLoc(x, y);
            btn->setDim(SX, SY);
            double r = rtql8::utils::random(0.0, 1.0);
            double g = rtql8::utils::random(0.0, 1.0);
            double b = rtql8::utils::random(0.0, 1.0);
            btn->setColor(r, g, b, A);
            btn->setLabel(names[i].c_str());
            btn->setHotSpot(30, 30);
            std::string onclick
                = (boost::format("cmd:select %s;page:start") % names[i]).str();
            btn->setOnClick(onclick);
            std::string path = RTQL8_DATA_PATH;
            btn->setImgSrcFileName(path + "btnblue.png");          
            LOG_INFO << FUNCTION_NAME() << " : " << names[i] << ",onclick = " << onclick;
            page->addUIImgBtn(btn);
            x += DX;
        }
        this->addPage(page);
    }


    GuiKinController::~GuiKinController() {
    }//dstrctr


    int GuiKinController::initIKSkel(){
        int numD = kinSkel->getNumDofs();
        Eigen::VectorXd q(numD);
        //modify skeleton's initial position to face camera and stand a bit more naturally
        kinSkel->getPose(q);
        q[4] = -2.1;
        q[9] = -0.17;
        q[15]= -0.17;
        kinSkel->setPose(q, true, true);
        return 0;
    }//initIKSkel

    int GuiKinController::initKinData(){//should no longer be called - flags should be set via xml settings
        //cout<<"GuiKinController flags being set"<<endl;
        //KC_flags[_KC_DISP_DEPTH] = true;
        //KC_flags[_KC_DISP_RGB	] = true;
        //KC_flags[_KC_DISP_AVATAR] = true;
        //KC_flags[_KC_DISP_MRKRS] = true;
        //KC_flags[_KC_DISP_AVTLOC] = true;
        //KC_flags[_KC_DISP_RAWSK] = true;
        //KC_flags[_KC_DISP_FILTSK] = true;
        //KC_flags[_KC_DISP_SKCLIP] = true;
        //KC_flags[_KC_DISP_UI_2D] = true;
        //KC_flags[_KC_PROC_2D_DGRAB]  = false;
        return 0;
    }//initIKSkel


    void GuiKinController::buildUI(vector<int>& numCmps){

    }//buildUI


///////
//call these from timer loop
//////
//debug info for gui display
    void GuiKinController::onTimerQuery(){
        updateCustomUI();

        if(kinHNDLR->kinSkelHandlerAvail()){
            skelJointStrs =  kinHNDLR->getBestKinSkelJointLocsStrs();
            if(kinSpeechRecog != ""){this->parseSpeechCommand();}
            //	kinSpeechRecog = kinHNDLR->getSpchRecogResult();
        }//if valid skel handler


        for (int i = 0; i < numUITimers(); i++) {
            KinUITimer* tmr = getUITimer(i);
            if (tmr->getIsDone()) {
                LOG_INFO << "Done!! " << tmr->getLabel()<< " onclick = " << tmr->getOnClick();
                //cout<< "CLICKED!! " << btn->getLabel()<< " onclick = " << btn->getOnClick()<<endl;
                std::string onclick = tmr->getOnClick();
                std::vector<std::string> commands;
                boost::split(commands, onclick, boost::is_any_of(";"));
                for (int j = 0; j < commands.size(); j++) {
                    this->onCommand(commands[j]);
                }
                return;
            }
        }

    }//onTimerQuery

///////
///call these from draw
///////

    void GuiKinController::onDrawUpdate(rtql8::renderer::RenderInterface* mRI){
        //KinectController::drawUI();
        glPushMatrix();
        glTranslatef(0, 1.0, -3);
        if(nullptr != kinSkel){
            //cout<<"skel ! null"<<endl;
            // if (KC_flags[_KC_PROC_IK]) {
            if (isIKMode()) {
                glBlock() {
                    glTranslated(0.0, 0.0, 4.0);
                    glPolygonMode(GL_FRONT_AND_BACK,  GL_LINE);
                    if(KC_flags[_KC_DISP_AVATAR]){kinSkel->draw(mRI);}
                    if(KC_flags[_KC_DISP_MRKRS]){kinSkel->drawMarkers(mRI);}
                }
            }
        }
        skelDrawn = true;
        glPopMatrix();
        onDrawUpdate();
    }//onDrawUpdate

//display IK skeleton and various kinect-derived information and images
    void GuiKinController::onDrawUpdate(){
        onTimerQuery();	
        //kinect stuff drawn here 
        if(kinHNDLR->kinValid()) {
            kinectRConnTimer = 90;
            glPushMatrix();
            // glTranslatef(0,-1,-3);

            glTranslatef(0,0,-3);
            if(KC_flags[_KC_DISP_DEPTH] && isIKMode() ){
                if (currentUIPage->name != "ikmode-confirm" &&
                    currentUIPage->name != "poseik-confirm") {
                    drawKinDpthHandImage(true, -4,4,-5,1,1,1);
                    drawKinDpthHandImage(false, 4,4,-5,1,1,1);
                    drawKinImage(true,0,0,-5,1,1,1);
                }
            }//if using depth image
            if(KC_flags[_KC_DISP_RGB] && isIKMode() ) {
                // drawKinImage(false,0,5,-5,.2f,.2f,1);
            }
			
            if(KC_flags[_KC_USE_STRM_SKEL]){
                glPushMatrix();
                glTranslatef(0,1,0);
                if(kinHNDLR->kinSkelHandlerAvail()){
                    if ( isIKMode() ) {
                        if (currentUIPage->name != "ikmode-confirm" &&
                            currentUIPage->name != "poseik-confirm") {
                            drawKinSkelData();
                        }
                    }
                }
                glPopMatrix();
            } //if using skel stream/IK
            glPopMatrix();
        }//if kinect connected
        else if (0 >= kinectRConnTimer) {
            kinHNDLR->checkKinectConn();kinectRConnTimer = 90;
        }
        else{
            --kinectRConnTimer;
        }

        //draw skeleton visualisations
        if((nullptr != kinSkel) && (!skelDrawn) && isIKMode() ){
            // if (currentUIPage->name != "ikmode-confirm" &&
            //     currentUIPage->name != "poseik-confirm") {
                if(KC_flags[_KC_DISP_AVATAR]){kinSkel->draw();}
                if(KC_flags[_KC_DISP_MRKRS]){kinSkel->drawMarkers();}
            // }
        }

        KinectController::onDrawUpdate();

        // optProgressBar->advPrgBarVal(0.1);
        {
            glPushMatrix();
            enable2dUIMode(true);
            glPushMatrix();
            optProgressBar->draw();
            nextStyle->draw();
            terminateOpt->draw();
            glPopMatrix();
            enable2dUIMode(false);
            glPopMatrix();
        }
        skelDrawn = false;
        // setFlag(_KC_PROC_IK, true);
    }//onDrawUpdate

    bool GuiKinController::parseSpeechCommand(){
        bool res = KinectController::parseSpeechCommand();
        if(res){    
            return res;
        } else {//not found in base class
            //add custom phrase handling in here
            clog<<"GuiKinController : Unhandled Recognized Phrase : "<<kinSpeechRecog<<endl;
            clearSpeechRecogVals();
        }
        return res;
    }//parseSpeechCommand 

    bool GuiKinController::checkUIElements(int& x, int& y, int& d, 
                                                bool mouseEvent, int hand, int evnt) {
        bool clickedOn = false;
        //call base class version to handle actual object events
        bool result = KinectController::checkUIElements(x, y, d, mouseEvent, hand, evnt);
        
        
        for (int i = 0; i < numUIButtons(); i++) {
            KinUIButton* btn  = getUIButton(i);
            if (btn->isClicked()) {
                LOG_INFO << "CLICKED!! " << btn->getLabel()<< " onclick = " << btn->getOnClick();
                //cout<< "CLICKED!! " << btn->getLabel()<< " onclick = " << btn->getOnClick()<<endl;
                std::string onclick = btn->getOnClick();
                std::vector<std::string> commands;
                boost::split(commands, onclick, boost::is_any_of(";"));
                for (int j = 0; j < commands.size(); j++) {
                    this->onCommand(commands[j]);
                }
                return result;
            }//if click
        }//for each button
        for (int i = 0; i < numUIImageButtons(); i++) {
            KinUIImageButton* btn  = getUIImageButton(i);
            if (btn->isClicked()) {
                LOG_INFO << "CLICKED!! " << btn->getLabel()<< " onclick = " << btn->getOnClick();
                //cout<< "CLICKED!! " << btn->getLabel()<< " onclick = " << btn->getOnClick()<<endl;
                std::string onclick = btn->getOnClick();
                std::vector<std::string> commands;
                boost::split(commands, onclick, boost::is_any_of(";"));
                for (int j = 0; j < commands.size(); j++) {
                    this->onCommand(commands[j]);
                }
                return result;
            }//if click
        }//for each button

        if (nextStyle->isClicked()) {
            LOG_INFO << "Next Style button is clicked!!!!!";
            // nextStyle->setLabel("HAHA");
            app->solver_request_next_style();
        }
        if (terminateOpt->isClicked()) {
            LOG_INFO << "Terminate button is clicked!!!!";
            // nextStyle->setLabel("HAHA");
            app->solver_request_terminate();
        }

        return result;
    }//checkUIElements

    bool GuiKinController::onCommand(const std::string& command) {
        LOG_INFO << "command = " << command;
        std::vector<std::string> tokens;
        boost::split(tokens, command, boost::is_any_of("\n\t :"));
        int n = tokens.size();
        if (n < 1) {
            LOG_WARNING << "command not detected";
            return false;
        }
        std::string cmd = tokens[0];
        if (cmd == "") {
            return true;
        } else if (cmd == "page" && n == 2) {
            onCommandPage(tokens);
        } else if (cmd == "cmd") {
            onCommandCmd(tokens);
        } else if (cmd == "create") {
            onCommandCreate(tokens);
        } else if (cmd == "set") {
            onCommandSet(tokens);
        } else if (cmd == "hint") {
            onCommandHint(tokens);
        } else if (cmd == "mode") {
            onCommandMode(tokens);
        } else if (cmd == "fixik") {
            onCommandFixIK(tokens);
        } else {
            LOG_FATAL << endl;
            LOG_FATAL << "*****************************************"
                      << "there is a invalid command: [" << cmd << "]";
            LOG_FATAL << endl;
            return false;
        }
        return true;
    }

    void GuiKinController::onCommandSet(std::vector<std::string>& tokens) {
        filterTokens(tokens);
        int n = tokens.size();
        if (n == 2 && tokens[1] == "clear") {
            LOG_INFO << ">>>> dictionary clear";
            dict.clear();
            return;
        } else if (n == 3) {
            std::string var = tokens[1];
            std::string val = tokens[2];
            LOG_INFO << ">>>> dictionary " << var << " <- " << val;
            dict[var] = val;
            return;
        }
    }

    void GuiKinController::filterTokens(std::vector<std::string>& tokens) {
        for (int i = 0; i < tokens.size(); i++) {
            if (tokens[i][0] == '#') {
                tokens[i] = tokens[i].substr(1);
                KinUISlider* sld = this->getUISlider(tokens[i].c_str());
                if (sld == NULL) {
                    LOG_FATAL << "invalid UI: " << tokens[i];
                    exit(0);
                }
                // double v = 1.0 - sld->getSliderPos();
                // double v = sld->getSliderPos();
                double v = sld->getCurValue();
                tokens[i] = boost::lexical_cast<std::string>(v);
            } else if (tokens[i][0] == '$') {
                tokens[i] = tokens[i].substr(1);
                KinUICrank* crk = this->getUICrank(tokens[i].c_str());
                if (crk == NULL) {
                    LOG_FATAL << "invalid UI: " << tokens[i];
                    exit(0);
                }
                // double v = 1.0 - sld->getSliderPos();
                // double v = sld->getSliderPos();
                double v = crk->getCurValue();
                tokens[i] = boost::lexical_cast<std::string>(v);
            } else if (tokens[i] == "@ik") {
                using namespace fullbody1;
                Eigen::VectorXd q;
                kinSkel->getPose(q);
                q.head<6>().setZero();
                q(1) = 0.93;
                q(l_ankle_ez) = -( q(l_thigh_ez) + q(l_knee_ez) );
                q(r_ankle_ez) = -( q(r_thigh_ez) + q(r_knee_ez) );
                std::stringstream sout;
                sout << IO(q);
                std::string str = sout.str();
                boost::erase_all(str, " ");
                tokens[i] = str;
            } else if (tokens[i][0] == '%') {
                std::string key = tokens[i].substr(1);
                if (dict.find(key) == dict.end()) {
                    LOG_FATAL << "invalid key: " << tokens[i];
                    exit(0);
                }
                tokens[i] = dict[key];
            }
        }
    }

    void GuiKinController::onCommandPage(std::vector<std::string>& tokens) {
        std::string nextpage = tokens[1];
        gotoPage(nextpage.c_str());
    }

    void GuiKinController::onCommandCmd(std::vector<std::string>& tokens) {
        std::stringstream sout;
        filterTokens(tokens);
        for (int i = 1; i < tokens.size(); i++) {
            sout << tokens[i] << " ";
        }
        LOG_INFO << ">>>> cmd = [" << sout.str() << "]";
        bool result = app->interpret(sout.str().c_str());
        LOG_INFO << "result = " << result;

        // gotoPage("start");
    }

    void GuiKinController::onCommandCreate(std::vector<std::string>& tokens) {
        LOG_INFO << FUNCTION_NAME();
        using boost::format;
        // 1. Create an empty controller with name
        std::string name = dict["name"];
        LOG_INFO << "xml = " << app->kn()->xmldoc();
        app->kn()->createController(name.c_str());
        app->kn()->select(name.c_str());
        LOG_INFO << "name = " << name;
        LOG_INFO << "controller xml = " << app->kn()->con()->toString();

        // 2. Set the initial pose or previous motion
        if (dict.find("pose") != dict.end()) { // 2.1 Set the initial pose 
            std::string pose = dict["pose"];
            app->interpret(format("init %s") % pose);
            LOG_INFO << "pose = " << pose;
        } else if (dict.find("prev") != dict.end()) { // 2.2 Set the previous
            std::string prev = dict["prev"];
            app->interpret(format("addprev %s") % prev);
            LOG_INFO << "prev = " << prev;
        }
        LOG_INFO << "controller xml = " << app->kn()->con()->toString();

        // 3. Set the terminate condition
        std::string terminate = dict["terminate"];
        LOG_INFO << "terminate = " << terminate;
        if (terminate == "timeout") {
            std::string t = dict["t"];
            LOG_INFO << "t = " << t;
            app->interpret(( format("terminate timeout %s") % t).str().c_str());
        } else if (terminate == "nocontacts") {
            app->interpret("terminate nocontacts");
        } else if (terminate == "anycontacts") {
            app->interpret("terminate anycontacts");
        }
        LOG_INFO << "controller xml = " << app->kn()->con()->toString();

        cout << "################################" << endl;
        cout << app->kn()->saveXMLString() << endl;
        cout << "################################" << endl;

        // Minor. update the dynamic page
        bool deleted = deletePage("create-prev");
        LOG_INFO << "delete dynamic page = " << deleted;
        buildDynamicPage();
        buildDynamicPageForSelect();
    }

    void GuiKinController::onCommandHint(std::vector<std::string>& tokens) {
        std::stringstream sout;
        filterTokens(tokens);
        for (int i = 1; i < tokens.size(); i++) {
            sout << tokens[i] << " ";
        }
        std::string hint = sout.str();
        LOG_INFO << ">>>> hint = [" << hint << "]";

        app->kn()->pushHint(hint.c_str());
    }

    void GuiKinController::onCommandMode(std::vector<std::string>& tokens) {
        std::string modename = tokens[1];
        LOG_INFO << "changeMode = [" << modename << "]";
        if (modename == "ik") {
            setFlag(_KC_PROC_IK, true);
            LOG_INFO << "setFlag IK = true";
        } else if (modename == "ui") {
            setFlag(_KC_PROC_HANDS_2D, true);
            LOG_INFO << "setFlag UI = true";
        }
    }

    void GuiKinController::onCommandFixIK(std::vector<std::string>& tokens) {
        std::string task = tokens[1];
        LOG_INFO << "fixik: task = " << task;
        if (task == "armsymmetry") {
            LOG_INFO << "Make arms symmetry";
            Eigen::VectorXd q;
            kinSkel->getPose(q);
            LOG_INFO << "orig  = " << IO(q);
            using namespace fullbody1;
            for (int i = l_scapula_ex, j = r_scapula_ex;
                 i < l_wrist_ex + 1; i++, j++) {
                if (i == l_elbow_ez) {
                    q(i) = q(j);
                } else {
                    q(i) = -q(j);
                }
            }
            LOG_INFO << "fixed = " << IO(q);
            kinSkel->setPose(q, true, true);
        } else if (task == "legsymmetry") {
            LOG_INFO << "Make legs symmetry";
            Eigen::VectorXd q;
            kinSkel->getPose(q);
            LOG_INFO << "orig  = " << IO(q);
            using namespace fullbody1;
            for (int i = l_thigh_ez, j = r_thigh_ez;
                 i < l_toe_ez + 1; i++, j++) {
                if (i != l_thigh_ex && i != l_ankle_ey) {
                    q(i) = q(j);
                } else {
                    q(i) = -q(j);
                }


            }
            LOG_INFO << "fixed = " << IO(q);
            kinSkel->setPose(q, true, true);
        }
    }

    void GuiKinController::gotoPage(const char* const name) {
        // LOG_INFO << "xml = " << app->kn()->xmldoc();
        LOG_INFO << "Go to page = [" << name << "]";
        KinUIPage* page = setPage(name);
        std::string onload = page->onload;
        if (onload.length() == 0) {
            return;
        }
        LOG_INFO << "page.onload = " << onload;
        std::vector<std::string> commands;
        boost::split(commands, onload, boost::is_any_of(";"));
        for (int j = 0; j < commands.size(); j++) {
            this->onCommand(commands[j]);
        }
        // updateCustomUI();

    }

    bool GuiKinController::isIKMode() {
        // Nasty exception!!!!!
        if (currentUIPage->name == "ikmode-confirm") {
            return true;
        }
        if (currentUIPage->name == "poseik-confirm") {
            return true;
        }
        return KC_flags[_KC_PROC_IK];
    }

//draw various kinect and avatar skeleton and skeleton-related structures
    void GuiKinController::drawKinSkelData(){
        if(KC_flags[_KC_DISP_SKCLIP]) {
            drawKinSkelClipBox(kinSkelDisp[SK_NSY_KIN][0],
                               kinSkelDisp[SK_NSY_KIN][1],
                               kinSkelDisp[SK_NSY_KIN][2]);
        }
        if((KC_flags[_KC_DISP_RAWSK]) || (KC_flags[_KC_DISP_FILTSK])) {
            kinSkelJointState = kinHNDLR->getCtlSkelJointState();
        }
        if(KC_flags[_KC_DISP_AVTLOC]) {
            drawKinSkelFrame(kinSkelDisp[SK_AVTR][0],kinSkelDisp[SK_AVTR][1],
                             kinSkelDisp[SK_AVTR][2],SK_AVTR, avatarIKHandles);
        }
        if((KC_flags[_KC_DISP_RAWSK]) && (kinHNDLR->validCtlSkel())) {
            drawKinSkelFrame(kinSkelDisp[SK_NSY_KIN][0],
                             kinSkelDisp[SK_NSY_KIN][1],
                             kinSkelDisp[SK_NSY_KIN][2],
                             SK_NSY_KIN, kinSkelNoisyMarkers);
        }
        if((KC_flags[_KC_DISP_FILTSK]) && (kinHNDLR->validCtlSkel())){
            drawKinSkelFrame(kinSkelDisp[SK_FLT_KIN][0],
                             kinSkelDisp[SK_FLT_KIN][1],
                             kinSkelDisp[SK_FLT_KIN][2],
                             SK_FLT_KIN, kinSkelMarkerPos);
        }
    }//drawKinSkelData

    void GuiKinController::resetOptProgressBar() {
        optProgressBar->reset();
    }
    
    void GuiKinController::setOptProgressBar(double value) {
        // optProgressBar->setCurrVal(value);
        optProgressBar->reset();
        int v = value;
        if (v > 14.0) {
            v = 14.0;
        }
        optProgressBar->advPrgBarVal(v);
    }


} // namespace gui
