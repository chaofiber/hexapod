//
// Created by chao on 2020-05-17.
//

#ifndef FINAL_PROJECT_GAITCONTROL_H
#define FINAL_PROJECT_GAITCONTROL_H

#endif // FINAL_PROJECT_GAITCONTROL_H
#include <robot/Robot.h>
#include <robot/GeneralizedCoordinatesRobotRepresentation.h>
#include <eiquadprog/eiquadprog.hpp>
#include <optLib/NewtonMinimizer.h>
#include <utils/mathUtils.h>
#include "app.h"

map<string, int> ID_Name = {
    {"rb0_rb1", 0},
    {"rb0_rb4", 1},
    {"rb0_rb7", 2},
    {"rb0_rb10", 3},
    {"rb0_rb13", 4},
    {"rb0_rb16", 5}
};

class GaitControl
{
public:

    /** main part: Go the spider!!!*/
    void spider_Go(Robot *robot){

        vector<P3D> endPos_all;
        hex.leg = &leg;
        RobotRB* rb = (robot->root->cJoints)[0]->child->cJoints[0]->child->cJoints[0]->child;
        if (swith_gait){
            time_switch += dt;
            get_Time_dep_endEffector_pos_vel(robot,3);
            get_Time_dep_body_pos(robot,3,auto_navi);
        }
        else{
            get_robot_initialization(robot); // from this we get the mode of the robot
            switch (mode) {
                case -1: {
                    get_Time_dep_body_pos(robot,mode,auto_navi);
                    break;
                }
                case 0:
                case 1:
                case 2:
                {
                    simTime += dt;
                    get_Time_dep_body_pos(robot,mode,auto_navi);
                    get_Time_dep_endEffector_pos_vel(robot,mode);
                    break;
                }
                case 4:
                {
                    simTime += dt;
                    Go_for_Jumping_end(rb->state.getWorldCoordinates(rb->rbProps.endEffectorPoints[0]).y, (robot->root->state.pos).y);
                    get_Time_dep_endEffector_pos_vel(robot, mode);
                    get_Time_dep_body_pos(robot, mode, auto_navi);
                }
            }
        }

        for(const auto joint: robot->root->cJoints){
            P3D desire_kidney_pos = robot->root->state.getWorldCoordinates(joint->pJPos); //desired kidney position based on the position and orientation of the body
            RobotRB* rb = joint->child->cJoints[0]->child->cJoints[0]->child;
            Vector3d angle(0,0,0);
            P3D endPos;
            V3D kid_to_end;
            // mode(0): standing, test IK; mode(1): walking, the body position is computed by QP
            switch (mode) {
                case -1:{
                    endPos = default_endpos[ID_Name[joint->name]]; // this is just used for inverse kinematic
                    break;
                }
                // this is ther rest mode. used when the spider is in autonomous navigation:
                /** For now, we let the rebot stay where it is*/
                /** todo: we can make the robot go back to the default position*/
                case 0: {
                    endPos = endpos[ID_Name[joint->name]];
                    break;
                }
                case 1:
                case 2:
                {
                    endPos = endpos[ID_Name[joint->name]]; // world coordinate
                    break;
                }
                case 4:
                {
                    endPos = endpos[ID_Name[joint->name]]; // world coordinate
                    break;
                }
            }
            kid_to_end = robot->root->state.getLocalCoordinates(V3D(endPos - desire_kidney_pos));
            if(!gradient){
                angle = iKsolver.anaLytical(kid_to_end, joint);
            }
            else{
                hex.desire_kid_to_end = kid_to_end;
                hex.kidney = joint;
                Eigen::VectorXd x=V3D{joint->defaultJointAngle, joint->child->cJoints[0]->defaultJointAngle, rb->pJoint->defaultJointAngle};
                minimizer.minimize(&hex, x);
                angle = x;
            }
            joint->defaultJointAngle = angle(0);
            joint->child->cJoints[0]->defaultJointAngle = angle(1);
            rb->pJoint->defaultJointAngle = angle(2);
        }
    }
    /** compute the next y of endeffector given next x&z, considering the case with different terrians **/
    void compute_y_given_xz(P3D &endpos, V3D &endVel, bool is_endeff);

    /** return the time dependent endeffector global tranjectory of position & velocity, i.e. p=p(t),v=v(t)*/
    void get_Time_dep_endEffector_pos_vel(Robot *robot, int m);

    /** return the time dependent body global position, only works when in moving mode*/
    void get_Time_dep_body_pos(Robot *robot, int mode, bool auto_navi);

    /** feed all data required*/
    void get_control_data(Data feed_data){
        data = feed_data;
    }

    /** stack the end effeffector velocities */
    dVector get_stacked_end_vel(vector<V3D> vel, Robot *robot);

    /** stack the jacobian, J: dimension: 18*24 */
    void get_stacked_jacobian(Robot *robot, Eigen::MatrixXd &J);

    /** update the body position using jacobian metho, i.e. x_dot = J * q_dot*/
    void get_pos_vel_by_jacob(Robot *robot);

    /** add projection matrix for regularization purpose, (not used in most cases)*/
    void get_project_matrix(Eigen::MatrixXd & proj);

    /** initialize data: mode, goal_pose, Rel_angle, etc.*/
    void get_robot_initialization(Robot *robot);

    /** check if the robot can detect the obstacles at current position*/
    P3D check_collision(Robot *robot);

    /**initialize in the setting: auto navigation with obstacles*/
    void initialize_auto_navi_with_obstacles(Robot *robot, bool find_sphere);

    /**initialize in the setting: auto navigation without obstacles, you can guide the trajectory by changing the goal position*/
    void initialize_auto_navi(Robot *robot, bool find_sphere);

    /** control the robot foot to jump at a fixed point*/
    void Go_for_Jumping_end(double end_pos_y, double body_pos_y);

public:
    Data data;
    double dev_x = 0;
    double dev_y = 0;
    double dev_z = 0;
    double yaw = 0;
    double pitch = 0;
    double roll = 0;
    double simTime = 0;
    double dt = 1.0/30;
    int gait_switch = 0;
    int pre_your_endeffector = 5; // change your body pose when jumping, 0: accelarating; 1: shrink the endeffectors; 2: extend the endeffectors; 3: stop ...
    double init_vel_end = 1;
    int shrink_count = 0;
    int extend_count = 0;
    bool begin_jump = true;
    int mode = 0; // {0:static IK test; 1:triple walk; 2:....
    V3D goal_dirct;
    Quaternion quaterion_body;
    vector<P3D> default_endpos{P3D(-0.197626, 0, 0.184689),
                                P3D(0.197626, 0, 0.184689),
                                P3D(0.241196, 0, 0),
                                P3D(-0.241196, 0, 0),
                                P3D(-0.197626, 0, -0.184689),
                                P3D(0.197626, 0, -0.184689)};
    vector<P3D> endpos = default_endpos; //use to track the motion of the end-effectors
    
    /** default_end_effset: the relative position between the body pos and the endeffector positions;
     * the desired endeffector position should be robot->root.pos(global) + default_end_offset;
     * here for now, we assume the y pos of body doesn't change, but we might need to replace 0.115 with the real-time
     * position, all for the purpose that the endeffector should contact the ground*/
    vector<P3D> default_end_offset{P3D(-0.197626, -0.115, 0.184689),
                               P3D(0.197626, -0.115, 0.184689),
                               P3D(0.241196, -0.115, 0),
                               P3D(-0.241196, -0.115, 0),
                               P3D(-0.197626, -0.115, -0.184689),
                               P3D(0.197626, -0.115, -0.184689)};
    vector<V3D> endVel{V3D{0,0,0},
                       V3D{0,0,0},
                       V3D{0,0,0},
                       V3D{0,0,0},
                       V3D{0,0,0},
                       V3D{0,0,0}};
    vector<V3D> shr_ext_direct{ V3D(-0.197626, 0, 0.184689).normalized(),
                        V3D(0.197626, 0, 0.184689).normalized(),
                        V3D(0.241196, 0, 0).normalized(),
                        V3D(-0.241196, 0, 0).normalized(),
                        V3D(-0.197626, 0, -0.184689).normalized(),
                        V3D(0.197626, 0, -0.184689).normalized() };
    bool gradient=true;
    bool auto_navi;
    bool auto_navi_with_obstacles;
    bool st_jump;
    bool with_sphere_terrain;
    bool compute_body_pos;
    IKsolver iKsolver;
    hexobjective hex;
    Leg leg;
    vel_objective jaco_obj;
    GradientBasedMinimizer minimizer;
//    GradientBasedMinimizer minimizer_;
    NewtonMinimizer minimizer_;
    Eigen::MatrixXd G;
    Eigen::MatrixXd J;
    dVector vel;
    int f_inv = 6;
    double v_ref = 0.006; //vel for test
    double w_ref = 0.08; //angular vel
    V3D current_to_goal;
    double Rel_angle;
    vector<P3D> obstacles;
    P3D sphere_pos;
    double sphere_r;
    vector<P3D> sub_goals;
    bool first_feed = true;
    P3D current_obstacle{-1,-1,-1};
    P3D goal_pos;
    double time_switch = 0;
    bool swith_gait = false;
};

void GaitControl::compute_y_given_xz(P3D &endpos, V3D &endVel, bool is_endeff=true){
    
    endpos.y = 0;
    endVel.y() =0;
    // when the robot is on the sphere terrain
    if(with_sphere_terrain==true && 
        pow(endpos.x-sphere_pos.x,2)+pow(endpos.z-sphere_pos.z,2)<=pow(sphere_r,2)-pow(sphere_pos.y,2)){
        endpos.y = sqrt(pow(sphere_r,2)-pow(endpos.x-sphere_pos.x,2)-pow(endpos.z-sphere_pos.z,2)) + sphere_pos.y;
        endVel.y() = -1./(endpos.y-sphere_pos.y) * ((endpos.x-sphere_pos.x)*endVel.x()+(endpos.z-sphere_pos.z)*endVel.z());
    }
    // when the computing object is endeffector instead of root, add periodic motion on y-axis
    if(is_endeff){
        double s = abs(0.03 * sin(2 * M_PI / f_inv * simTime));
        double d = 0.03*cos(2*M_PI/f_inv*simTime)*2*M_PI/f_inv;
        if (sin(2 * M_PI / f_inv * simTime)<0) d *= -1;
        endpos.y += s;
        endVel.y() += d;
    }
}

/** return the time dependent endeffector global tranjectory of position, i.e. p=p(t)*/
void GaitControl::get_Time_dep_endEffector_pos_vel(Robot *robot, int m=1){
    // todo: need to add time-dependence of endpos, e.g. endpos = endpos(simTime)
    // todo: for different mode, etc: running, stance, trot, we need a key to control

    // triplet walking gait
    if (int(2 * simTime) % f_inv == 0 && int(2 * simTime) % (2 * f_inv) != 0) 
        gait_switch = 1;
    if (int(2 * simTime) % (2 * f_inv) == 0) 
        gait_switch = 0;
    switch (m)
    {
        case 1:
        {
            double s1 = abs(0.03 * sin(2 * M_PI / 6 * simTime));
            double s2 = v_ref*dt;
            double d1 = 0.03*cos(2*M_PI/6*simTime)*2*M_PI/6;
            if (sin(2 * M_PI / 6 * simTime)<0) d1 *= -1;
            double d2 = v_ref;
            
            switch (gait_switch)
            {
                case 0:
                {
                    for(int i=0; i<6; i=i+2){
                        endpos[i].x += s2 * goal_dirct[0];
                        endpos[i].z += s2 * goal_dirct[2];
                        endVel[i] = d2 * goal_dirct;
                        compute_y_given_xz(endpos[i], endVel[i]);
                        endVel[i+1].setZero();
                    }
                    break;
                }
                case 1:
                {
                    for(int i=1; i<6; i=i+2){
                        endpos[i].x += s2 * goal_dirct[0];
                        endpos[i].z += s2 * goal_dirct[2];
                        endVel[i] = d2 * goal_dirct;
                        compute_y_given_xz(endpos[i], endVel[i]);
                        endVel[i-1].setZero();
                    }

                    break;
                }
            }
            break;
        }
        case 2:
        {
            P3D root_pos = robot->root->state.pos;
//            const V3D rotate_axis = V3D(root_pos.x, 1, root_pos.z).normalized();
            const V3D rotate_axis = V3D(0,1,0);
            V3D old_dirct;
            switch (gait_switch)
            {
                case 0:
                {
                    for(int i=0; i<6; i=i+2){
                        old_dirct = V3D(root_pos, endpos[i]);
                        V3D new_dirct = rotateVec(old_dirct, w_ref * dt, rotate_axis);
                        endpos[i] = root_pos + new_dirct;
                        endVel[i] = (new_dirct - old_dirct)/dt;
                        compute_y_given_xz(endpos[i], endVel[i]);
                        endVel[i+1].setZero();
                    }
                    break; 
                }
                case 1:
                {
                    for(int i=1; i<6; i=i+2){
                        old_dirct = V3D(root_pos, endpos[i]);
                        V3D new_dirct = rotateVec(old_dirct, w_ref * dt, rotate_axis);
                        endpos[i] = root_pos + new_dirct;
                        endVel[i] = (new_dirct - old_dirct)/dt;
                        compute_y_given_xz(endpos[i], endVel[i]);
                        endVel[i-1].setZero();
                    }
                    break; 
                }
            }
            break;
        }
        /** case 3: from the current position to normal position via in-situ adjustment, for every endeffector, we need
         *  to provide its own current-to-desired vector, the current desired position of all endeffectolrs can be computed
         *  by robot->root->globalpos() + default pos = endeffector global position */
        case 3:
        {
            double s1 = abs(0.03 * sin(2 * M_PI / f_inv * time_switch));
            double s2 = v_ref*dt;
            /** move the foot in the air to go back to the desired position first*/
            if (endpos[0].y>1e-2 && endpos[1].y==0){
                for (int i=0;i<6;i=i+2) {
                    P3D cur_des_pos =
                        robot->root->state.getWorldCoordinates( default_end_offset[i]);
                    cur_des_pos[1] = 0;
                    P3D cur_to_des = cur_des_pos - endpos[i];
                    endpos[i] += cur_to_des / 180.0;   // so after 6 dt, the foot in the air will go to the desired position
                }
//                cout << "0 is in the air"<< V3D(endpos[0]) << endl;
                break;
            }
            if (endpos[1].y>1e-2 && endpos[0].y==0){
                for (int i=1;i<6;i=i+2) {
                    P3D cur_des_pos =
                        robot->root->state.getWorldCoordinates(default_end_offset[i]);
                    cur_des_pos[1] = 0;
                    P3D cur_to_des = cur_des_pos - endpos[i];
                    endpos[i] += cur_to_des / 180.0;
                }
//                cout << "1 is in the air"<< V3D(endpos[1]) << endl;
                break;
            }
            /** move the foot in the groud to the desired position, which means currently all feet are on the ground*/
            if (abs(endpos[0].x - (robot->root->state.getWorldCoordinates( default_end_offset[0])).x) > 1e-2 ){
                for (int i=0;i<6;i=i+2){
                    P3D cur_des_pos =
                        robot->root->state.getWorldCoordinates(default_end_offset[i]);
                    P3D cur_to_des = cur_des_pos - endpos[i];
                    endpos[i] += cur_to_des / 180.0;
                    endpos[i].y = s1;
                }
//                cout << "both is in the ground, 0 not accurate"<< V3D(endpos[0]) << endl;
                break;
            }
            if (abs(endpos[1].x - (robot->root->state.getWorldCoordinates( default_end_offset[1])).x)> 1e-2 ){
                for (int i=1;i<6;i=i+2){
                    P3D cur_des_pos =
                        robot->root->state.getWorldCoordinates(default_end_offset[i]);
                    P3D cur_to_des = cur_des_pos - endpos[i];
                    endpos[i] += cur_to_des / 180.0;
                    endpos[i].y = s1;
                }
//                cout << "both is in the ground, 1 not accurate"<< V3D(endpos[1]) << endl;
                break;
            }
            /** after these four iterations (at most), then all feet are on the desired positions*/
            swith_gait = false;
//            simTime = 0;
        }
        case 4: {
            for (int i = 0; i < 6; i++) {
                if (pre_your_endeffector == 1) {
                    endpos[i].x -= 0.05 * shr_ext_direct[i][0] * dt;
                    endpos[i].z -= 0.05 * shr_ext_direct[i][2] * dt;
                }
                if (pre_your_endeffector == 2) {
                    endpos[i].x += 0.05 * shr_ext_direct[i][0] * dt;
                    endpos[i].z += 0.05 * shr_ext_direct[i][2] * dt;

                }
                if (pre_your_endeffector == 0) {
                    endpos[i].y += init_vel_end * dt;
                }
                if (pre_your_endeffector == 3) {
                    endpos[i].y = 0;
                    endpos[i].x += 0.05 * shr_ext_direct[i][0] * dt;
                    endpos[i].z += 0.05 * shr_ext_direct[i][2] * dt;
                }
                if (pre_your_endeffector == 4) {
                    endpos[i].x -= 0.005 * shr_ext_direct[i][0] * dt;
                    endpos[i].z -= 0.005 * shr_ext_direct[i][2] * dt;
                }
            }
        }
    }
}

/** stack the jacobian, J: dimension: 18*24 */
void GaitControl::get_stacked_jacobian(Robot *robot, Eigen::MatrixXd &J){
    J.fill(0);
    Matrix dpdq;
    GeneralizedCoordinatesRobotRepresentation geRepre(robot);
//    dVector q_copy;geRepre.getQDot(q_copy);
    for (const auto joint: robot->root->cJoints) {
        RobotRB *rb = joint->child->cJoints[0]->child->cJoints[0]->child;
        geRepre.compute_dpdq(rb->rbProps.endEffectorPoints[0], rb, dpdq);
        J.middleRows<3>(ID_Name[joint->name]*3) = dpdq;
    }
}

void GaitControl::get_pos_vel_by_jacob(Robot *robot){
    jaco_obj.x_dot = get_stacked_end_vel(endVel,robot);
    jaco_obj.v_ref << 0,0,v_ref/2;
    get_project_matrix(jaco_obj.Proj);
    get_stacked_jacobian(robot, jaco_obj.Jac);
    // todo: use greadient descent to cpmpute the velocity
    dVector x(vel);
    minimizer_.minimize(&jaco_obj, x);
    vel = x;
}

void GaitControl::get_Time_dep_body_pos(Robot *robot, int m, bool auto_navi){
    switch (m)
    {
        // IK test mode: update robot position and orientation
        case -1:{
            robot->root->state.pos = P3D(data.dev_x, data.dev_y, data.dev_z) + P3D(0,0.115,0);
            robot->root->state.orientation = iKsolver.euler_to_quaternion(data.yaw,data.pitch,data.roll,robot);
            break;
        }
        // rest mode: after reaching the target, rest where you already are
        case 3:
        case 0:{
            // stance position, for IK test.  don't use +=, otherwise keep increasing/decreasing
            // dev_x is actually the current position relative to the origin, therefore it is fairly big already after
            // arriving the target.
            robot->root->state.pos = P3D(dev_x, dev_y, dev_z) + P3D(0,0.115,0);
            robot->root->state.orientation = iKsolver.euler_to_quaternion(yaw,pitch,roll,robot);
            break;
        }
        case 1:{
            dev_x += v_ref /2 * goal_dirct[0]* dt;
            dev_z += v_ref /2 * goal_dirct[2]* dt;

            if(compute_body_pos){
                vel.setConstant(24,0);
                get_pos_vel_by_jacob(robot);
                dev_y += vel[1] * dt;
                boundToRange(dev_y, -0.02,0.02);
            }else{
                P3D dev_root = P3D(dev_x,0,dev_z);
                V3D trival_root = V3D(0,0,0);
                compute_y_given_xz(dev_root,trival_root,false);
                dev_y = dev_root.y;
            }
            robot->root->state.pos = P3D(dev_x, dev_y, dev_z) + P3D(0,0.115,0);
            robot->root->state.orientation = iKsolver.euler_to_quaternion(yaw,pitch,roll,robot);
            break;
        }
        case 2:{
            if(compute_body_pos){
                vel.setConstant(24,0);
                get_pos_vel_by_jacob(robot);
                dev_y += vel[1] * dt;
                boundToRange(dev_y, -0.02,0.02);
            }
            yaw += w_ref /2 * dt;
            if (yaw>2*M_PI)
                yaw -= 2*M_PI;
            if (yaw<0)
                yaw += 2*M_PI;
            robot->forward = V3D(sin(yaw),0,cos(yaw));
            robot->right = V3D(cos(yaw),0,-sin(yaw));
            robot->root->state.pos = P3D(dev_x, dev_y, dev_z) + P3D(0,0.115,0);
            robot->root->state.orientation = iKsolver.euler_to_quaternion(yaw,pitch,roll,robot);
            break;
        }
        case 4: {
            if (compute_body_pos) {
                vel.setConstant(24, 0);
                get_pos_vel_by_jacob(robot);
                dev_y += vel[1] * dt;
                boundToRange(dev_y, -0.02, 0.02);
            }
        }
        default:
            break;
    }
}

/** stack the end effeffector velocities */
dVector GaitControl::get_stacked_end_vel(vector<V3D> vel, Robot *robot){
    dVector x_dot(18);
    for (const auto joint: robot->root->cJoints){
        x_dot.segment<3>(ID_Name[joint->name]*3) = vel[ID_Name[joint->name]];
    }
    return x_dot;
}

/** add projection matrix for regularization purpose, (not used in most cases)*/
void GaitControl::get_project_matrix(Eigen::MatrixXd & proj){
    proj(0,0) = 1;
    proj(1,1) = 1;
    proj(2,2) = 1;
}

/** initialization: return the mode and collision control*/
void GaitControl::get_robot_initialization(Robot *robot) {
    f_inv = data.f_inv;
    dt = f_inv / 180.;
    gradient = data.grad;
    st_jump = data.st_jump;
    auto_navi = data.auto_navi;
    auto_navi_with_obstacles = data.auto_navi_with_obstacles;
    with_sphere_terrain = data.with_sphere_terrain;
    obstacles = data.obstacles; sphere_pos = data.sphere_pos; sphere_r = data.sphere_r;
    compute_body_pos = data.compute_body_pos;
    // adaptive velocity and angular velocity control
    P3D root_pos = robot->root->state.pos;
    bool find_sphere = false;
    // umconmment following to climb uphills horizontally
    // if (with_sphere_terrain == true && 
    //     sqrt(pow(root_pos.x-sphere_pos.x,2)+pow(root_pos.z-sphere_pos.z,2)) - sqrt(pow(sphere_r,2)-pow(sphere_pos.y,2))<0.35)
    //     {find_sphere = true;}
    if (auto_navi_with_obstacles) {
        initialize_auto_navi_with_obstacles(robot, find_sphere);
    } else if (auto_navi) {
        initialize_auto_navi(robot, find_sphere);
    }
      else if (st_jump){

        mode = 4;
    }
      else {
        mode = -1;
    }

}

/** check if the robot can detect the obstacles at current position
 *  should return the position of the obstacle if detected, otherwise return (-1,-1)*/
P3D GaitControl::check_collision(Robot *robot){
    double min_distance = 100;
    P3D obstable_pos(-1,-1,-1);
    for (vector<P3D>::iterator p=obstacles.begin(); p!=obstacles.end(); ++p){
        double temp_distance = V3D(*p - robot->root->state.pos).norm();
        if (temp_distance<min_distance && temp_distance < 0.35){
            min_distance = temp_distance;
            obstable_pos = *p;
        }
    }
    return obstable_pos;
}


/**initialize in the setting: auto navigation without obstacles, you can guide the trajectory by changing the goal position*/
void GaitControl::initialize_auto_navi(Robot *robot, bool find_sphere) {

    goal_pos = data.goal_pos;
    current_to_goal = V3D(robot->root->state.pos,goal_pos);
    current_to_goal[1] = 0; // we dont consider the y direction for now
    goal_dirct = current_to_goal.normalized();

    /** Rel_angle: range (-Pi/2,PI/2) */
    if (find_sphere){
        Rel_angle = angleBetween(robot->right,current_to_goal,V3D(0,1,0));
    }else{
        Rel_angle = angleBetween(robot->forward,current_to_goal,V3D(0,1,0));
    }
    // when the goal is at the back, walk backwards
    if (Rel_angle>M_PI/2 && Rel_angle<M_PI*3/2){
        Rel_angle -= M_PI;
    }
    // when the goal is on the right, turn right
    if (Rel_angle>M_PI*3/2 && Rel_angle<M_PI*2){
        Rel_angle -= M_PI*2;
    }
    v_ref = data.v_ref * current_to_goal.norm();
    boundToRange(v_ref,0.01,0.2);
    w_ref = data.w_ref * Rel_angle;
    w_ref>0?boundToRange(w_ref,0.015,0.1):boundToRange(w_ref,-0.1,-0.015);
    // if (abs(Rel_angle)>abs(w_ref * dt)){
    if (current_to_goal.norm()<0.05){
        mode = 0;
    }
    else if (abs(Rel_angle)>0.02){
        mode = 2; // turning mode
    }
        // else if (current_to_goal.norm()>(v_ref*dt) ) {  // the very next step the angle passes the desired value and the distance is far, so please walk
    else {  // the very next step the angle passes the desired value and the distance is far, so please walk
        mode = 1; //walking mode
    }
}

/**initialize in the setting: auto navigation with obstacles*/
void GaitControl::initialize_auto_navi_with_obstacles(Robot *robot, bool find_sphere) {
    /** feed the final goal position*/
    if (first_feed) {
        sub_goals.push_back(data.goal_pos);
        first_feed = false;
    }
    /** check collision: if we encounter an obstacle, then we should change our plan. i.e. pop out current sub goal if
     *  the current sub goal is not the final goal.*/
    P3D obstacle_pos = check_collision(robot);
    if (obstacle_pos[0]!=-1 && V3D(obstacle_pos-current_obstacle).norm()>1e-5){
        current_obstacle = obstacle_pos;
        // need to avoid the obstacle by setting a temporarily sub-goal point
        V3D current_to_obstacle = V3D(robot->root->state.pos,obstacle_pos);
        current_to_obstacle[1] = 0;
        double angle_between_x_axis_and_current_to_obstacle = abs(angleBetween(V3D(1,0,0),current_to_obstacle) - M_PI_2);
        if (angle_between_x_axis_and_current_to_obstacle>M_PI_2){
            //heading for axis z
            // todo: we need multiple goal_pos, therefore we need a vector to store it, and use a stack structure
            /** Here we head for a new direction with distance 0.3, which is fixed and safe, we can also assign with
             *  an obstacle-depedenet distance*/
            P3D sub_goal = P3D(-0.1,0,1)*0.6*(goal_dirct.z())/abs(goal_dirct.z()) + robot->root->state.pos;
            if (V3D(sub_goals.back()-data.goal_pos).norm()>1e-2) sub_goals.pop_back();
            sub_goals.push_back(sub_goal);
        }
        else{
            P3D sub_goal = P3D(1,0,-0.1)*0.6*(goal_dirct.x())/abs(goal_dirct.x()) + robot->root->state.pos;
            if (V3D(sub_goals.back()-data.goal_pos).norm()>1e-2) sub_goals.pop_back();
            sub_goals.push_back(sub_goal);
        }
    }

    /** take the current sub_goal*/
    if (sub_goals.empty()){
        mode = 0;
        return;
    }
    P3D sub_goal = sub_goals.back();
    cout << "current sub goal position"<< V3D(sub_goal).transpose() << endl;
    current_to_goal = V3D(robot->root->state.pos,sub_goal);
    current_to_goal[1] = 0; // we dont consider the y direction for now
    goal_dirct = current_to_goal.normalized();

    /** Rel_angle: range (-Pi/2,PI/2) */
    if (find_sphere){
        Rel_angle = angleBetween(robot->right,current_to_goal,V3D(0,1,0));
    }else{
        Rel_angle = angleBetween(robot->forward,current_to_goal,V3D(0,1,0));
    }
    // when the goal is at the back, walk backwards
    if (Rel_angle>M_PI/2 && Rel_angle<M_PI*3/2){
        Rel_angle -= M_PI;
    }
    // when the goal is on the right, turn right
    if (Rel_angle>M_PI*3/2 && Rel_angle<M_PI*2){
        Rel_angle -= M_PI*2;
    }
    v_ref = data.v_ref * current_to_goal.norm();
    boundToRange(v_ref,0.01,0.2);
    w_ref = data.w_ref * Rel_angle;
    w_ref>0?boundToRange(w_ref,0.015,0.1):boundToRange(w_ref,-0.1,-0.015);

    if (current_to_goal.norm()<0.05){
        mode = 0;
        sub_goals.pop_back(); // pop out the current sub goal and turn to the next.
        // swith_gait = true;
    }
    else if (abs(Rel_angle)>0.02){
        mode = 2; // turning mode
    }
        // the very next step the angle passes the desired value and the distance is far, so please walk
    else {
        mode = 1; //walking mode
    }

}

void GaitControl::Go_for_Jumping_end(double end_pos_y, double body_pos_y) {
    int pre_status_e = pre_your_endeffector;
    if (pre_status_e == 5 && begin_jump)
    {
        pre_your_endeffector = 1;
        begin_jump = false;
    }
    if (shrink_count >= 18 && pre_status_e == 1)
    {
        pre_your_endeffector = 0;
    }
    if (end_pos_y >= 0.1 && pre_status_e == 0)
    {
        pre_your_endeffector = 2;
    }
    if (extend_count >= 18 && pre_status_e == 2)
    {
        pre_your_endeffector = 3;
    }
    if (body_pos_y <= 0.115 && pre_status_e == 3)
    {
        pre_your_endeffector = 4;
    }
    if (shrink_count >= 10 * extend_count && pre_your_endeffector == 4)
    {
        pre_your_endeffector = 5;
    }
    if (pre_your_endeffector == 1 || pre_your_endeffector == 4) {
        shrink_count += 1;
    }
    if (pre_your_endeffector == 2 || pre_your_endeffector == 3) {
        extend_count += 1;
    }
    cout << "the endeffector status is:" << pre_status_e << endl;
//    cout << "shrink_count is:" << shrink_count << endl;
//   cout << "extend_count is:" << extend_count << endl;
}