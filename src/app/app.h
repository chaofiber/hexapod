#pragma once

#define EIGEN_STATIC_ASSERT(CONDITION,MSG) eigen_assert((CONDITION) && #MSG);
#include <gui/application.h>
#include <gui/shader.h>
#include <gui/renderer.h>
#include <gui/light.h>
#include <gui/camera.h>
#include <gui/ImGuizmo.h>
#include <utils/logger.h>
#include <robot/Robot.h>
#include <ode/ODERBEngine.h>
#include "IKsolver.h"
#include "GaitControl.h"
#include <eiquadprog/eiquadprog.hpp>
#include "Whole_body_control.h"
#include <utils/mathUtils.h>
/**
    Space: run/pause sim
    Click on robot: highlight the body part, print out the name.
*/

class App : public Basic3DAppWithShadows {
public:
    App(const char* title = "CRL Playground - Creature Locomotion app", std::string iconPath = CRL_DATA_FOLDER"/icons/icon.png")
        : Basic3DAppWithShadows(title, iconPath) {

        camera = TrackingCamera(5);
        camera.aspectRatio = float(width) / height;
        camera.rotAboutUpAxis = -0.75;
        camera.rotAboutRightAxis = 0.5;

        light.s = 0.03f;
        shadowbias = 0.0f;

        glEnable(GL_DEPTH_TEST);

        showConsole = true;
        automanageConsole = true;
        Logger::maxConsoleLineCount = 10;
        consoleHeight = 225;

        odeRbEngine->loadRBsFromFile(CRL_DATA_FOLDER "/environment/Ground.rbs");
        odeRbEngine->loadRBsFromFile(CRL_DATA_FOLDER "/environment/Sphere.rbs");
        robot.showMeshes = false;
        robot.showSkeleton = true;

        robot.setRootState(P3D(0, 0.5, 0), Quaternion::Identity());

        // set all the joints to position mode
        for (int i = 0; i < robot.getJointCount(); i++) {
            robot.getJoint(i)->controlMode = RBJointControlMode::POSITION_MODE;
        }

        this->targetFramerate = 30;
        this->limitFramerate = true;

        for(int i = 0; i < MY_PLOT_N; i++)
            myPlotValues[i] = 0;
    }

    virtual ~App() override {
    }

    virtual void resizeWindow(int width, int height) override {
        camera.aspectRatio = float(width) / height;
        return Application::resizeWindow(width, height);
    }

    bool mouseMove(double xpos, double ypos) override {
        P3D rayOrigin;
        V3D rayDirection;
        camera.getRayFromScreenCoordinates(xpos, ypos, rayOrigin, rayDirection);
        Ray mouseRay(rayOrigin, rayDirection);

        if (mouseState.dragging == false) {
            // this bit of code will check for mouse-ray robot intersections all the time.
            if (selectedRB != NULL)
                selectedRB->rbProps.selected = false;

            P3D selectedPoint;

            selectedRB = robot.getFirstRBHitByRay(mouseRay, selectedPoint, false, true);

            if (selectedRB != NULL) {
                selectedRB->rbProps.selected = true;
            }
        }
        else {
        }

        camera.processMouseMove(mouseState, keyboardState);
        return true;
    }

    bool mouseButtonReleased(int button, int mods) override {
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            if (selectedRB) {
                selectedRB->rbProps.selected = false;
                selectedRB = nullptr;
            }
        }
        return true;
    }

    bool mouseButtonPressed(int button, int mods) override {
        if (button == GLFW_MOUSE_BUTTON_LEFT || button == GLFW_MOUSE_BUTTON_RIGHT) {
            if (selectedRB != NULL) {
                if (selectedRB->pJoint == NULL)
                    Logger::consolePrint("Clicked on BodyLink %s (root)\n", selectedRB->name.c_str());
                else
                    Logger::consolePrint("Clicked on BodyLink %s (parent joint is %s with id %d))\n", selectedRB->name.c_str(), selectedRB->pJoint->name.c_str(), selectedRB->pJoint->jIndex);
            }
        }

        return true;
    }

    bool scrollWheel(double xoffset, double yoffset) override {
        camera.processMouseScroll(xoffset, yoffset);
        return true;
    }

    void computeAndApplyControlInputs(double dt) {
        simTime += dt;

        // let's say we want the robot to go from the zero angle configuration
        // (as it is loaded) to the default pose in 2 seconds:
        double interpVal = simTime / 0.3;
        boundToRange(&interpVal, 0, 1.0);
        P3D goal_pos = P3D(guizmoPosition.x(),guizmoPosition.y(),guizmoPosition.z());
        /*todo: here we define the positions of the obstacles. we need to draw obstacles exactly;
         * the side length of obstacles should match the search range of the robot, which is defined in
         * the GaitControl::initialize_auto_navi_with_obstacles when we need to define one/or more sub_goal(s)
         * in the trajectories. One rule of thumb is ro leave enough space for the robot to pass, which means
         * our strategy should be conservative.*/

        vector<P3D> obstacles{P3D(0.5,0.0,0.5), P3D(0.2,0.0,1.9),P3D(1.8,0.0,0.1)};

        data.dev_x = dev_x; data.dev_y = dev_y; data.dev_z = dev_z; data.yaw = yaw;
        data.pitch = pitch; data.roll = roll; data.f_inv = f_inv; data.v_ref = v_ref;
        data.w_ref = w_ref; data.grad = gradient; data.auto_navi = auto_navi; data.auto_navi_with_obstacles = auto_navi_with_obstacles;
        data.obstacles = obstacles; data.goal_pos = goal_pos; data.with_sphere_terrain = with_sphere_terrain; 
        data.sphere_pos = sphere_pos; data.sphere_r = sphere_r; data.compute_body_pos = compute_body_pos;
        data.st_jump = st_jump;
        gaitControl.get_control_data(data); mode = gaitControl.mode;
        gaitControl.spider_Go(&robot);

        for(const auto joint : robot.jointList) {
          joint->desiredControlSignal = interpVal * joint->defaultJointAngle;
        }


    }

    void process() override {
        if (appIsRunning == false)
            return;

        // we need to do enough work here until the simulation time is caught up
        // with the display time...



        double tmpT = 0;
        while (tmpT < 1.0 / targetFramerate) {
            tmpT += dt;
            if (whole_body_control){
                torqueControl.spider_Go(&robot);
            }
            if (!whole_body_control){
                computeAndApplyControlInputs(dt);
            }

            odeRbEngine->step(dt);
        }

        light.target.x() = robot.root->state.pos.x;
        light.target.z() = robot.root->state.pos.z;

        camera.target.x = robot.root->state.pos.x;
        camera.target.z = robot.root->state.pos.z;
    }

    virtual void drawAuxiliaryInfo() {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        drawFPS();

        drawConsole();

        drawImGui();

        ImGui::EndFrame();
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    }

    // objects drawn with a shadowMapRenderer (during shadow pass) will cast a shadow
    virtual void drawShadowCastingObjects() override {
        robot.draw(shadowMapRenderer);
    }

    // objects drawn with a shadowShader (during the render pass) will have shadows cast on them
    virtual void drawObjectsWithShadows() override {
        ground.draw(shadowShader, V3D(0.6, 0.6, 0.8));
        P3D goal_pos = P3D(guizmoPosition.x(), guizmoPosition.y(), guizmoPosition.z());
        P3D goal_mark;
        goal_mark.x = goal_pos.x;
        goal_mark.y = 0.5;
        goal_mark.z = goal_pos.z;
        Quaternion orient1 = getRotationQuaternion(0,V3D(0,0,1));
        V3D dims1 = V3D(0.1,0.4,0.1);
        if(auto_navi_with_obstacles){
            drawCuboid(P3D(0.55, 0.2, 0.55),orient1,dims1,basicShader, V3D(0, 1, 0));
            drawCuboid(P3D(0.25, 0.2, 1.95),orient1,dims1,basicShader, V3D(0, 1, 0));
            drawCuboid(P3D(1.85, 0.2, 0.15),orient1,dims1,basicShader, V3D(0, 1, 0));
        }
        drawArrow3d(goal_mark, V3D(0, -0.2, 0), 0.05, basicShader, V3D(1, 0, 0));
        if(with_sphere_terrain){
            drawSphere(sphere_pos,2*sphere_r,basicShader);
        }
    }

    // objects drawn with basic shadowShader (during the render pass) will not have shadows cast on them
    virtual void drawObjectsWithoutShadows() override {
        robot.draw(basicShader);
    }

    virtual bool keyPressed(int key, int mods) override {
        if (key == GLFW_KEY_SPACE) {
            appIsRunning = !appIsRunning;
            return true;
        }
        return false;
    }

    // draws the UI using ImGui. you can add your own UI elements (see below)
    virtual void drawImGui(){

        using namespace ImGui;

        SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Once);
        Begin("Main Menu");

        Text("Play:");
        SameLine();
        ToggleButton("Play App", &appIsRunning);

        if (appIsRunning == false) {
            SameLine();
            if (ArrowButton("tmp", ImGuiDir_Right)) {
                appIsRunning = true;
                process();
                appIsRunning = false;
            }
        }

        Checkbox("Draw Console", &showConsole);

        if (TreeNode("Draw options...")) {
            Checkbox("Draw Meshes", &robot.showMeshes);
            Checkbox("Draw Skeleton", &robot.showSkeleton);
            Checkbox("Draw Joint Axes", &robot.showJointAxes);
            Checkbox("Draw Joint Limits", &robot.showJointLimits);
            Checkbox("Draw Collision Primitives", &robot.showCollisionSpheres);
            Checkbox("Draw MOI box", &robot.showMOI);

            TreePop();
        }

        // add your own UI elements for example like this:
        if(CollapsingHeader("my UI")){
            Indent();

            Checkbox("gradient", &gradient);
            if(gradient){
                SameLine();
                Text(" = use gradient descent for IK");
            }

            Checkbox("compute body velocity by Jacobian", &compute_body_pos);
            if(compute_body_pos){
                SameLine();
                Text(" = compute body velocity by Jacobian");
            }

            Checkbox("add sphere terrian", &with_sphere_terrain);
            if(with_sphere_terrain){
                SameLine();
                Text(" = add sphere terrian");
            }

            Checkbox("auto navigation", &auto_navi);
            if(auto_navi){
                SameLine();
                Text(" = start auto navigation");
            }

            Checkbox("auto navigation with obstacles", &auto_navi_with_obstacles);
            if(auto_navi_with_obstacles){
                SameLine();
                Text(" = start auto navigation with obstacles");
            }

            Checkbox("Start jumping", &st_jump);
            if (st_jump) {
                SameLine();
                Text(" = start to jump");
            }

            Checkbox("whole body control (still under test)", &whole_body_control);
            if(whole_body_control){
                SameLine();
                Text(" = use whole body control");
            }

//            InputDouble("yaw", &yaw);
//            InputDouble("pitch",&pitch);
//            InputDouble("roll",&roll);
            InputDouble("myDouble",&myDouble);
            InputInt("mode",&mode);
            // make sure to always unique labels
            // SliderScalar("my double", ... ); // this would not work!
            double min_x = -0.05, max_x = 0.05;
            double min_y = -0.05, max_y = 0.05;
            double min_z = -0.05, max_z = 0.05;
            double min = -PI/6, max = PI/6;
            double min_p = -PI/12, max_p = PI/12;
            double min_v = -0.05, max_v = 0.05;
            double min_w = -0.1, max_w = 0.1;
            if(SliderScalar("deviation in x of robot", ImGuiDataType_Double, &dev_x, &min_x, &max_x))
                Logger::consolePrint("robot changed in x '%f'", dev_x);

            if(SliderScalar("deviation in y of robot", ImGuiDataType_Double, &dev_y, &min_y, &max_y))
                Logger::consolePrint("robot changed in y '%f'", dev_y);

            if(SliderScalar("deviation in z of robot", ImGuiDataType_Double, &dev_z, &min_z, &max_z))
                Logger::consolePrint("robot changed in x '%f'", dev_z);

            if(SliderScalar("yaw angle of robot", ImGuiDataType_Double, &yaw, &min, &max))
                Logger::consolePrint("yaw angle of robot '%f'", yaw);

            if(SliderScalar("pitch angle of robot", ImGuiDataType_Double, &pitch, &min_p, &max_p))
                Logger::consolePrint("pitch angle of robot '%f'", pitch);

            if(SliderScalar("roll angle of robot", ImGuiDataType_Double, &roll, &min_p, &max_p))
                Logger::consolePrint("roll angle of robot '%f'", roll);

            if(SliderScalar("velocity of robot", ImGuiDataType_Double, &v_ref, &min_v, &max_v))
                Logger::consolePrint("velocity of robot '%f'", v_ref);
            
            if(SliderScalar("angular velocity of robot", ImGuiDataType_Double, &w_ref, &min_w, &max_w))
                Logger::consolePrint("angular velocity of robot '%f'", w_ref);
            InputInt("1/f",&f_inv);
            //if(Button("multiply doubles"))
             //   Logger::consolePrint("result is: %f", myDouble*myDouble2);

            // also works with Eigen vectors
            //InputScalarN("my vector", ImGuiDataType_Double, myVector3d.data(), myVector3d.size());

            // make a plot
            PlotLines("my plot", myPlotValues, MY_PLOT_N, myPlotCounter, NULL, FLT_MAX, FLT_MAX, {0, 200});
            // You probably would want to put the following lines in the process() function.
            // I put it here so it's all in one place.
            myPlotValues[myPlotCounter] = myDouble2 * myDouble;
            myPlotCounter = (myPlotCounter+1) % MY_PLOT_N;

            Text("guizmo:");
            Checkbox("enabled", &guizmoEnabled);
            InputScalarN("position", ImGuiDataType_Double, guizmoPosition.data(), 3);

            Unindent();
        }

        // example code on how to use the ImGuizmo
        if(guizmoEnabled)
        {
            ImGuizmo::BeginFrame();
            ImGuiIO& io = ImGui::GetIO();
            ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

            // we use it in translate mode and need to provide the corresponding
            // transformation matrix to ImGuizmo ...
            auto transform = glm::translate(glm::mat4(1.f), toGLM(guizmoPosition));
            ImGuizmo::Manipulate(glm::value_ptr(camera.getViewMatrix()), glm::value_ptr(camera.getProjectionMatrix()), ImGuizmo::TRANSLATE, ImGuizmo::WORLD, glm::value_ptr(transform));

            // ... and thus after manipulation, we extract the changed position
            // from the transformation matrix.
            guizmoPosition = Vector3d(
                transform[3][0],
                transform[3][1],
                transform[3][2]
                );

//            auto obstacle1 = glm::translate(glm::mat4(1.f), toGLM(obstacle_pos));
//            ImGuizmo::Manipulate(glm::value_ptr(camera.getViewMatrix()), glm::value_ptr(camera.getProjectionMatrix()), ImGuizmo::TRANSLATE, ImGuizmo::WORLD, glm::value_ptr(obstacle1));
//
//            // ... and thus after manipulation, we extract the changed position
//            // from the transformation matrix.
//            obstacle_pos = Vector3d(
//                obstacle1[3][0],
//                obstacle1[3][1],
//                obstacle1[3][2]
//            );
        }

        ImGui::End();

        // start a new ImGui window like this:
        // ImGui::Begin("my window");
        // ImGui::Button("hello!");
        // ...
        // ImGui::End();

    }

    virtual bool drop(int count, const char** fileNames) override {
        return true;
    }

public:
    SimpleGroundModel ground;   // model to draw the ground

    // the simulation engine
    crl::sim::ODERBEngine* odeRbEngine = new crl::sim::ODERBEngine();

    // the robot to load. uncomment/comment to change robot model
    Robot robot =
        Robot(odeRbEngine, CRL_DATA_FOLDER"/robots/simple/hex.rbs");
//      Robot(odeRbEngine, CRL_DATA_FOLDER"/robots/simple/dog.rbs");

    RobotRB* selectedRB = NULL; // robot rigid body selected by mouse, = NULL when nothing selected
    bool appIsRunning = false;

    double dt = 1 / 120.0;      // time step for the simulation
    double simTime = 0;         // current simulation time

    // example variables for how to use ImGui
    bool gradient = false;
    bool auto_navi = false;
    bool auto_navi_with_obstacles = false;
    bool with_sphere_terrain = false;
    bool compute_body_pos = true;
    bool st_jump = false;
    bool whole_body_control = false;
    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0;
    double myDouble2 = 2;
    double myDouble = 0;
    double dev_x = 0.0;
    double dev_y = 0.0;
    double dev_z = 0.0;
    double v_ref = 0.006;
    double w_ref = 0.08;
    int f_inv = 6;
    int mode = 0;
    P3D sphere_pos = P3D(-1,-1.9,1); double sphere_r = 2;

    int myPlotCounter = 0;
    const static int MY_PLOT_N = 100;
    float myPlotValues[MY_PLOT_N];

    // example for a 3d guizmo
    bool guizmoEnabled = true;
//    Vector3d guizmoPosition = V3D(1,0.115,1);
    Vector3d guizmoPosition = V3D(-1,0.115,1);
    // Vector3d  obstacle_pos = V3D(0.5,0,0.5);
    GaitControl gaitControl;
    wholeBodyControl torqueControl;
    Data data;

};
