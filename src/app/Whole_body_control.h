//
// Created by chao on 2020-06-10.
//

#ifndef FINAL_PROJECT_WHOLE_BODY_CONTROL_H
#define FINAL_PROJECT_WHOLE_BODY_CONTROL_H

#endif //FINAL_PROJECT_WHOLE_BODY_CONTROL_H

#include <robot/Robot.h>
#include <robot/GeneralizedCoordinatesRobotRepresentation.h>
#include <eiquadprog/eiquadprog.hpp>
#include <optLib/NewtonMinimizer.h>
#include <optLib/SQPMinimizer.h>
#include <optLib/ConstrainedObjectiveFunction.h>
#include <utils/mathUtils.h>
#include "app.h"

//map<string, int> ID_Name = {
//    {"rb0_rb1", 0},
//    {"rb0_rb4", 1},
//    {"rb0_rb7", 2},
//    {"rb0_rb10", 3},
//    {"rb0_rb13", 4},
//    {"rb0_rb16", 5}
//};

map<int, const char*> Id_to_name = {
    {0, "rb0_rb1"},
    {1, "rb0_rb4"},
    {2, "rb0_rb7"},
    {3, "rb0_rb10"},
    {4, "rb0_rb13"},
    {5, "rb0_rb16"}
};

/** here we use torqur control. i.e. directly compute the torque and feed into the desired/default signal
 *
 *  we are aimed at the problem as follows:
 *
 *
 *      min ||Jb_dot * q_dot + Jb * q_ddot - x_ref_ddot||^2
 *
 *      s.t.  M(q)q_ddot + C(q,q_dot) + J^T \lambda = [0;\tau]
 *
 *      note: The first 6 rows of the equation above are constraints that should be enforced.
 *
 *            for foot in contact:
 *                  Je * q_dot = 0 \in R^3
 *                  Je_dot* q_dot + Je * q_ddpt = 0 \in R^3
 *                  f*n>0
 *                  -\mu f*n <= f*t <= \mu f*n
 *                  -\mu f*n <= f*b <= \mu f*n......where b and n are basis of the tangential contact plane
 *            for foot in air:
 *                  \lambda = 0 \in R^3
 *
 *
 *     Notice that we can compute \tau by feedforward generating instead of putting it in the optimizatiohn
 *     proboem:
 *
 *       [0;\tau] = M(q) * q_ddot + C(q,q_dot) + J^T \lambda
 *
 *     The optimization variables here are then q_ddot \in R^24 and lambda \in R^18
 *
 *     Some notations:
 *     1. We define the phase time, therefore we will know which legs are in the air and which legs are in contact
 *     2. x_ref_ddot = x_des_ddot + kp * x + kd * x\dot   we could start from standing therefore the desired x and x_dot are 0
 *     3. we should also add the contact constraints as forces should be in the friction cone if in contact
 *        this could be done by using a pymrid approximation
 *
 *     4. We use ConstrainedObjectiveFunction class  */

class wholeBodyControl {
public:

    /** main part: Go the spider!!!*/
    void spider_Go(Robot *robot) {

        simTime += dt;
        if (int(2 * simTime) % f_inv == 0 && int(2 * simTime) % (2 * f_inv) != 0)
            gait_switch = 1;
        if (int(2 * simTime) % (2 * f_inv) == 0)
            gait_switch = 0;

//        get_generalized_representation(robot);
        GeneralizedCoordinatesRobotRepresentation geRepre(robot);
//        geRepre.syncRobotStateWithGeneralizedCoordinates();
        geRepre.syncGeneralizedCoordinatesWithRobotState();
        /** test geRepre*/
        {
            dVector q_now;
            geRepre.getQ(q_now);
            cout << "current q\n" << q_now << endl;
            geRepre.getQDot(q_now);
            cout << "current q_dot\n" << q_now << endl;
        }
        {
            geRepre.computeMassMatrix(M);
            geRepre.computeCoriolisAndCentrifugalForcesTerm(fb);
            geRepre.computeGravityForcesTerm(gravity);
            get_stacked_jacobian(geRepre,robot,J_all,gait_switch);
        }
        cout << " test1" << endl;

        get_projection_matrix();
        geRepre.getQDot(q_dot);

        // use gait_switch to control which foot is in the air, which foot is in contact
//        /** get objective function ready*/
//        {
//            fun.x_des_ddot = x_des_ddot;
//            fun.q_dot = q_dot;
//            get_proj_for_objective_class(fun.proj);
//            get_base_pos_jacobian(robot,fun.J_b);
//            get_base_pos_jacobian_dot(robot,fun.J_b_dot);
//            get_equality_constraint_matrix(robot, fun.A,gait_switch);
//            get_inequality_constraint_matrix( fun.C, gait_switch);
//            get_b(robot, fun.b,gait_switch);
//        }

//        /** get sqpminimizer ready:*/
//        dVector vel;vel.setConstant(state_dim,0);dVector x(vel);
//        sqpMinimizer.minimize(&fun,x);

        /** get all matrix ready: we use the eiquadprog directly*/
        {
            cout << " test1.1" << endl;
            x_des_ddot.setConstant(6,0);
            get_proj_for_objective_class(proj);
            get_base_pos_jacobian(geRepre, robot,J_b);
            get_base_pos_jacobian_dot(robot,J_b_dot);
            get_equality_constraint_matrix(geRepre, robot, A,gait_switch);
            get_inequality_constraint_matrix( C, gait_switch);
            get_b(geRepre, robot, b,gait_switch);
            get_ci(robot, ci0, gait_switch);
        }
        dVector x;x.setConstant(state_dim,0); cout << " test2" << endl;
        double feval;
        // solve QP problem
        {
            // CE x + ce0 = 0   CI x + ci0 >= 0
            G = (J_b * proj).transpose() * (J_b * proj) + 1e-5 * Eigen::MatrixXd::Identity(state_dim, state_dim);
            g0 = (J_b * proj).transpose() * (J_b_dot * q_dot - x_des_ddot);
            CE = A.transpose();
            ce0 = b;
            CI = C.transpose();
            ci0.setConstant(num_inequ_constraint,0);
            feval = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
//        cout << "fevaliation: " << feval << endl;
        }


//        bool success = !(solve_quadprog(G, g0, CE, ce0, CI, ci0, x) == std::numeric_limits<double>::infinity());


//        todo: after SQP optimization, compute torque, pay attention: we might need to compute the gravity term ourselves.

        dVector q_ddot = x.segment(0,joint_dim);
        dVector lambda = x.segment(joint_dim,state_dim-joint_dim); //contact force R 9
//        Matrix M;// 24*24

        auto tau = M * q_ddot + fb + gravity + J_all.transpose()*lambda;  // R 24
//        cout << tau << endl;
        for(const auto joint : robot->jointList) {
            joint->controlMode = FORCE_MODE;
            joint->desiredControlSignal = tau[geRepre.getQIndexForJoint(joint)];
        }
//        cout << "TEST" << J_b_dot * q_dot + J_b * proj * x <<endl;
//        cout << "TEST Lambda" << lambda << endl;

        /** TEST OUTPUTS*/
        {
            cout <<'\n' << endl;
            cout << "f_evaluation: \n" << feval << endl;
            cout << "q_dot: \n" << q_dot << endl;
            cout << "J_b: " << J_b << endl;
            cout << "J_b_dot: " <<J_b_dot << endl;
//            cout << "A: " << A << endl;
            cout << "b: " << b << endl;
            cout << "Solution joint q_ddot: " << q_ddot << endl;
            cout << "Solution lambda: " << lambda << endl;
            cout << "J_all: " << J_all<< endl;
            cout << "J_all_b: " << J_all.transpose().topRows(6) << endl;
            cout << "gravity force" << gravity << endl;
            cout << "h(q,q_dot): " << fb << endl;
            cout << "CE transpose top 6: \n" << CE.transpose().topRows(6) << endl;
            cout << "ce0: \n" << ce0 << endl;
            cout << "CE.transpose()*x\n" << (CE.transpose()*x) << endl;

        }


    }

    void get_proj_for_objective_class(Eigen::MatrixXd &J){
        J.fill(0);
        J = proj;
    }
    /** equality constraints matrix, including force constraints (or we don't need to explicitly enforce them) and zero velocity contact constraints
     *  and EoM constraints, total constraint numbers are 9+9+6=24, ( if we don't enforce zero contact forces constraints, it would be 15 equality constraints*/
    void get_equality_constraint_matrix(GeneralizedCoordinatesRobotRepresentation geRepre, Robot* robot, Eigen::MatrixXd &J, int idx){
//        GeneralizedCoordinatesRobotRepresentation geRepre(robot);
        J.fill(0);
        int start = 0;
        // idx=0 or 1
        Eigen::MatrixXd temp = Eigen::MatrixXd::Zero(6,state_dim);
        temp << M.topRows(6),J_all.transpose().topRows(6);
        J.topRows(6) = temp;
        start = 6;
        for (int i=idx;i<6;i=i+2){
            RBJoint* joint = robot->getJointByName(Id_to_name[i]);
            Matrix dpdq; // (R: 3*24)
            RobotRB * rb = joint->child->cJoints[0]->child->cJoints[0]->child;
            geRepre.compute_dpdq(rb->rbProps.endEffectorPoints[0], rb, dpdq);
            Eigen::MatrixXd temp = Eigen::MatrixXd::Zero(3,state_dim);
            temp.leftCols(joint_dim) = dpdq;
            J.middleRows<3>(start) = temp;
            start += 3;
        }
//        for (int i = 1-idx; i<6;i+=2){
//            Eigen::MatrixXd J_lambda = Eigen::MatrixXd::Zero(3,state_dim);
//            J_lambda.middleCols<3>(joint_dim+i*3) = Eigen::MatrixXd::Identity(3,3);
//            J.middleRows<3>(start) = J_lambda;
//            start += 3;
//        }
    }

    void get_inequality_constraint_matrix(Eigen::MatrixXd &J, int idx){
        J.fill(0);
        int start = 0;
        // first 15 non sliding inequality constraints
        for (int i=idx; i<6;i+=2){
            Eigen::MatrixXd temp = Eigen::MatrixXd::Zero(5,state_dim);
            temp.middleCols<3>(joint_dim+i*3) <<
                0,1,0,
                1, mu,0,
                -1,mu,0,
                0, mu,1,
                0, mu,-1;

            J.middleRows<5>(start+0) = temp;
            start += 5;
        }

        // range constraints
        J.block<24,24>(15,0)<< Eigen::MatrixXd::Identity(joint_dim,joint_dim);
        J.block<24,24>(39,0)<< -Eigen::MatrixXd::Identity(joint_dim,joint_dim);
        J.block<9,9>(63,24)<< Eigen::MatrixXd::Identity(state_dim-joint_dim,state_dim-joint_dim);
        J.block<9,9>(72,24)<< -Eigen::MatrixXd::Identity(state_dim-joint_dim,state_dim-joint_dim);

    }

    void get_ci(Robot *robot, dVector &c, int idx){
        c.setConstant(num_inequ_constraint,0);
        dVector temp;
        temp.setConstant(24,-q_ddot_min);
        c.segment<24>(15).setConstant(-q_ddot_min);
        c.segment<24>(39).setConstant(q_ddot_max);
        c.segment<9>(63).setConstant(-lambda_min);
        c.segment<9>(72).setConstant(lambda_max);
    }

    void get_generalized_representation(Robot *robot){
//        geRepre = GeneralizedCoordinatesRobotRepresentation(robot);
        GeneralizedCoordinatesRobotRepresentation geRepre(robot);
    }

    void get_projection_matrix(){
        proj.middleCols<24>(0) = Eigen::MatrixXd::Identity(joint_dim,joint_dim);
    }

    void get_b(GeneralizedCoordinatesRobotRepresentation geRepre, Robot *robot, dVector & b, int idx){
        int start = 6;
//        GeneralizedCoordinatesRobotRepresentation geRepre(robot);
        b.setConstant(num_equ_constraint,0);
        b.head(6) = fb.head(6) + gravity.head(6);
        for (int i=idx;i<6;i=i+2){
            RBJoint* joint = robot->getJointByName(Id_to_name[i]);
            Matrix dpdq_dot; // (R: 3*24)
            RobotRB * rb = joint->child->cJoints[0]->child->cJoints[0]->child;
            geRepre.compute_dpdq_dot(rb->rbProps.endEffectorPoints[0], rb, dpdq_dot);
            b.segment<3>(start) = dpdq_dot * q_dot;
            start += 3;
        }
    }

    void get_base_pos_jacobian(GeneralizedCoordinatesRobotRepresentation geRepre, Robot *robot, Eigen::MatrixXd &J_b){
//        GeneralizedCoordinatesRobotRepresentation geRepre(robot);
        geRepre.compute_dpdq(robot->root->state.getLocalCoordinates(robot->root->state.pos),robot->root,J_b);
    }

    void get_base_pos_jacobian_dot(Robot *robot, Eigen::MatrixXd &J_b_dot){
        GeneralizedCoordinatesRobotRepresentation geRepre(robot);
        geRepre.compute_dpdq_dot(robot->root->state.getLocalCoordinates(robot->root->state.pos),robot->root,J_b_dot);
    }

    /** stack the contact jacobian, J: dimension: 9*24 */
    void get_stacked_jacobian(GeneralizedCoordinatesRobotRepresentation geRepre, Robot *robot, Eigen::MatrixXd &J, int idx){
        J.resize(9,joint_dim);
        J.fill(0);
        Matrix dpdq;
        int start = 0;
        for (int i=idx;i<6;i+=2){
            RBJoint* joint = robot->getJointByName(Id_to_name[i]);
            Matrix dpdq; // (R: 3*24)
            RobotRB * rb = joint->child->cJoints[0]->child->cJoints[0]->child;
            geRepre.compute_dpdq(rb->rbProps.endEffectorPoints[0], rb, dpdq);
            J.middleRows<3>(start) = dpdq;
            start += 3;
        }
    }
public:
    whole_body_objective fun;
    SQPMinimizer sqpMinimizer = SQPMinimizer();
//    GeneralizedCoordinatesRobotRepresentation geRepre(robot);
    dVector x_des_ddot;
    int gait_switch;
    int simTime =0;
    double dt = 1.0/30;
    int f_inv = 6;
    double mu = 0.8;
//    Eigen::MatrixXd proj = Eigen::MatrixXd::Constant(24,42,0);
    int state_dim = 33;  // 24+9=33 constraints, we just add three forces explicitly
    int joint_dim = 24;
    int num_equ_constraint = 15;
    int num_inequ_constraint = 81; // wew need to add ranges, therefore another 33*2=66 inequ constraints. totally 15 nonsliding constraints+ 66 = 81 inequ constraints
    int num_endeffector = 9; // three feet in contact, we need this to compute the generalized contact forces
//    dVector q_dot;
    Matrix M;// 24*24
    dVector fb; // 24*1
    Eigen::MatrixXd J_all = Eigen::MatrixXd::Constant(num_endeffector, joint_dim, 0); // stack all contact endeffector jacobian  dim 9*24
    dVector gravity;

    /** I try to use eiguqdprog directly without using SQP minimizer*/
    Eigen::MatrixXd J_b = Eigen::MatrixXd::Constant(3, joint_dim, 0); //Base position jacobian
    Eigen::MatrixXd J_b_dot = Eigen::MatrixXd::Constant(3, joint_dim, 0); //Base position jacobian derivatrive
    Eigen::MatrixXd proj = Eigen::MatrixXd::Constant(joint_dim, state_dim, 0);  //If we just need generalized joints
    dVector q_dot; // computed by generalization function
    Eigen::MatrixXd A = Eigen::MatrixXd::Constant(num_equ_constraint, state_dim, 0);// Equality constraint matrix wrt q
    Eigen::MatrixXd C = Eigen::MatrixXd::Constant(num_inequ_constraint, state_dim, 0);// Inequality constraint matrix
    dVector b; //Ax=b the right side  dimension 24
    dVector f;
    dVector d;  // lower bound of inequality constraints, could be infinity
    dVector l;  // boxbound of parameters. dimension 42
    dVector u;
    dVector current_A_p;
    dVector current_C_p;

    Eigen::MatrixXd G;
    dVector g0;
    Eigen::MatrixXd CE;
    dVector ce0;
    Eigen::MatrixXd CI;
    dVector ci0;
    double q_ddot_min = - 10000;
    double q_ddot_max = 10000;
    double lambda_min = -50000;
    double lambda_max = 50000;

};