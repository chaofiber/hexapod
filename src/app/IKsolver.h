// Created by chao on 2020-05-17.
//

#ifndef FINAL_PROJECT_IKSOLVER_H
#define FINAL_PROJECT_IKSOLVER_H
#endif // FINAL_PROJECT_IKSOLVER_H

#include <robot/Robot.h>
#include <optLib/ObjectiveFunction.h>
#include <optLib/ConstrainedObjectiveFunction.h>
#include <optLib/GradientDescentMinimizer.h>
#include <optLib/NewtonMinimizer.h>
#include <utils/mathUtils.h>
#include <array>

using Eigen::Vector3d;
using Eigen::Matrix3d;
typedef std::array<Matrix3d, 3> Tensor3x3x3;
typedef map<string,int> MSI;

class IKsolver {
public:
    Vector3d anaLytical(const V3D &p, RBJoint* kidney) {

        Vector3d angle(0,0,0);
        double length[3] = {0.05, 0.0885, 0.155};
        double Lh = std::sqrt(pow(p[0],2)+pow(p[2],2)) - length[0];
        double L = std::sqrt(pow(Lh,2) + pow(p[1],2));
        angle(0) = std::atan2(p[2],fabs(p[0]));
        angle(1) = safeACOS(Lh/L) - safeACOS((pow(L,2) + pow(length[1],2)- pow(length[2],2))/(2 * L * length[1]));
        angle(2) = PI - safeACOS((pow(length[1],2) + pow(length[2],2)- pow(L,2))/(2 * length[1] * length[2]));
        if (kidney->name == "rb0_rb1" || kidney->name == "rb0_rb10" || kidney->name == "rb0_rb13"){
            angle = -angle;
        }
        return angle;
    }


    Quaternion euler_to_quaternion(double yaw, double pitch, double roll, const Robot *robot){
        Quaternion q = Quaternion::Identity();
         q *= getRotationQuaternion(roll, robot->forward);
         q *= getRotationQuaternion(pitch, robot->right);
//        q *= getRotationQuaternion(roll, V3D(0,0,1));
//        q *= getRotationQuaternion(pitch, V3D(1,0,0));
        q *= getRotationQuaternion(yaw, RBGlobals::worldUp);
        return q;
    }

};

struct Leg
{
    Vector3d p0 = {0, 0, 0};
    const double length[3] = {0.05, 0.0885, 0.155};  //symmetric: just need compute once
    
    // forward kinematics
    Vector3d fk(const Vector3d &angles) const {
        auto p = p0;
        p[0] = cos(angles[0]) * (length[0] + length[1]*cos(angles[1])
                                 +length[2]*cos(angles[1]+angles[2]));
        p[1] = -length[1]*sin(angles[1]) - length[2]*sin(angles[1]+angles[2]);
        p[2] = sin(angles[0]) * (length[0] + length[1]*cos(angles[1])
                                 +length[2]*cos(angles[1]+angles[2]));
    return p;
    }

    // Jacobian of fk(angles)[2]
    Matrix3d dfk_dangles(const Vector3d &angles) const {
        Matrix3d dp_dangles = Matrix3d::Zero();

        dp_dangles(0,0) = -sin(angles[0]) * (length[0] + length[1]*cos(angles[1])
                                             + length[2]*cos(angles[1]+angles[2]));
        dp_dangles(0,1) = -cos(angles[0]) * (length[1]*sin(angles[1])
                                             + length[2]*sin(angles[1]+angles[2]));
        dp_dangles(0,2) = -length[2] * cos(angles[0]) * sin(angles[1]+angles[2]);
        dp_dangles(1,0) = 0;
        dp_dangles(1,1) = -length[1] * cos(angles[1]) - length[2] * cos(angles[1]+length[2]);
        dp_dangles(1,2) = -length[2] * cos(angles[1]+angles[2]);
        dp_dangles(2,0) = cos(angles[0]) * (length[0] + length[1]*cos(angles[1])
                                            + length[2]*cos(angles[1]+angles[2]));
        dp_dangles(2,1) = -sin(angles[0]) * (length[1]*sin(angles[1])
                                             + length[2]*sin(angles[1]+angles[2]));
        dp_dangles(2,2) = -length[2] * sin(angles[0]) * sin(angles[1]+angles[2]);        
        return dp_dangles;
    }

};

class hexobjective: public ObjectiveFunction
{
public:
    const Leg *leg;
    V3D desire_kid_to_end;
    RBJoint* kidney;
public:
    void prepareForOptimizationStep(const dVector& x) const override {
        // todo: add some thing here; for example, initialization, or can just ignore it for now;
    }
    double evaluate(const dVector& x) const override{
        // todo: compute e=0.5*(x(q)-x_des)^2, where x_des is the desired kid_to_end in the tempbody frame. therefore, we need a feedforward function. ..
        // todo: we might also need to pass the variable of x_des, or just define it in the public area
        double e = 0;
        Vector3d kid_to_end = leg->fk(x);
        if (kidney->name == "rb0_rb1" || kidney->name == "rb0_rb10" || kidney->name == "rb0_rb13"){
            kid_to_end = -kid_to_end;
        }
        e = 0.5 * (kid_to_end - desire_kid_to_end).transpose() * (kid_to_end - desire_kid_to_end);
        
        return e;
    }

    void getGradient(const dVector& x, dVector& grad) const override{
        // todo
        Vector3d kid_to_end = leg->fk(x);
        Matrix3d dfk_dangles = leg->dfk_dangles(x);
        if (kidney->name == "rb0_rb1" || kidney->name == "rb0_rb10" || kidney->name == "rb0_rb13"){
            kid_to_end = -kid_to_end;
            dfk_dangles = -dfk_dangles;
        }
         grad = dfk_dangles.transpose() * (kid_to_end - desire_kid_to_end);
    }

};

class vel_objective: public ObjectiveFunction{
public:
    Eigen::MatrixXd Jac = Eigen::MatrixXd::Constant(18,24,0);
    Eigen::MatrixXd Proj = Eigen::MatrixXd::Constant(3,24,0);
    dVector x_dot;
    Vector3d v_ref{0,0,0};
public:
    void prepareForOptimizationStep(const dVector& x) const override {
        // todo: add some thing here; for example, initialization, or can just ignore it for now;
    }
    double evaluate(const dVector& x) const override{
        // todo: compute e=0.5*(x(q)-x_des)^2, where x_des is the desired kid_to_end in the tempbody frame. therefore, we need a feedforward function. ..
        // todo: we might also need to pass the variable of x_des, or just define it in the public area
        double e = 0;
        e = 0.5 * (Jac * x - x_dot).transpose() * (Jac *x -x_dot);//
//        e += 1e-1 * (Proj * x - v_ref).transpose() * (Proj*x - v_ref);
        return e;
    }

    void getGradient(const dVector& x, dVector& grad) const override{
        // todo
        grad = Jac.transpose() * Jac * x - Jac.transpose()*x_dot;
//        grad += 1e-1* 2 * Proj.transpose() * (Proj*x - v_ref);
    }

    void getHessian(const dVector &x, SparseMatrix &hessian) const override{

        hessian.resize(x.size(),x.size());
        hessian.setZero();
        std::vector<MTriplet> triplets;
        triplets.clear();
        addHessianEntriesTo(x, triplets);
        hessian.setFromTriplets(triplets.begin(), triplets.end());
    }

    void addHessianEntriesTo(const dVector &x, std::vector<MTriplet>& hessianEntries) const{

        auto hess = Jac.transpose()*Jac;
//        hess += 1e-2*2*Proj.transpose()*Proj;
        for (int i = 0; i<x.size();++i){
            for (int j=0;j<x.size();++j){
                hessianEntries.push_back(MTriplet(i,j,hess(i,j)));
            }
        }
    }
};

class whole_body_objective: public ConstrainedObjectiveFunction {
public:
    Vector3d x_des_ddot;
    int state_dim = 42;
    int joint_dim = 24;
    int num_equ_constraint = 24;
    int num_inequ_constraint = 15;
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

//    Eigen::MatrixXd Ja = Eigen::MatrixXd::Constant(18,42,0);// Equality constraint jacobian
//    Eigen::MatrixXd Jc = Eigen::MatrixXd::Constant(15,42,0);// Inequality constraint jacobian
public:
    void prepareForOptimizationStep(const dVector &x) const override {
        // todo: add some thing here; for example, initialization, or can just ignore it for now;
    }

    /** x = [q_b_ddot:6, q_j_ddot:18, lambda:18] \in R^42*/
    double evaluate(const dVector &x) const override {
        double e = 0;
        dVector q_ddot = proj * x;
        e = 0.5 * (J_b_dot * q_dot + J_b * q_ddot - x_des_ddot).transpose() *
            (J_b_dot * q_dot + J_b * q_ddot - x_des_ddot);
        return e;
    }

    void getGradient(const dVector &x, dVector &grad) const override {
        grad = (J_b * proj).transpose() * (J_b_dot * q_dot + J_b * proj * x - x_des_ddot);
    }

    void getHessian(const dVector &x, SparseMatrix &hessian) const override {
        hessian.resize(x.size(), x.size());
        hessian.setZero();
        std::vector<MTriplet> triplets;
        triplets.clear();
        addHessianEntriesTo(x, triplets);
        hessian.setFromTriplets(triplets.begin(), triplets.end());
    }

    void addHessianEntriesTo(const dVector &x, std::vector<MTriplet> &hessianEntries) const {

        auto hess = (J_b * proj).transpose() * (J_b * proj) + 1 * Eigen::MatrixXd::Identity(state_dim, state_dim);
        for (int i = 0; i < x.size(); ++i) {
            for (int j = 0; j < x.size(); ++j) {
                hessianEntries.push_back(MTriplet(i, j, hess(i, j)));
            }
        }
    }

    /*!
	 * Get the current value of the equality constraints.
	 * @returns A(p) of A(p) = b
	 */
    const dVector &getEqualityConstraintValues(const dVector &p) override {
        current_A_p = A * p;
        return current_A_p;
    }

    /*!
	 * get the current value of the equality constraints...
	 * @returns C(p)  of d <= C(p) <= f
	 */
    const dVector &getInequalityConstraintValues(const dVector &p) override {
        current_C_p = C * p;
        return current_C_p;
    }

    /** provide analytical jacobian: A */
    void getEqualityConstraintsJacobian(const dVector &p, SparseMatrix &J) override {
        // todo: use the same trick as Hessian matrix
        J.resize(num_equ_constraint, state_dim);
        J.setZero();
        std::vector<MTriplet> triplets;
        triplets.clear();
        addEqualityConstraintsJacobianEntriesTo(triplets);
        J.setFromTriplets(triplets.begin(), triplets.end());

    }

    void addEqualityConstraintsJacobianEntriesTo(std::vector<MTriplet> &JacEntries) {
        auto EqualityJac = A;
        for (int i = 0; i < EqualityJac.rows(); ++i) {
            for (int j = 0; j < EqualityJac.cols(); ++j) {
                JacEntries.push_back(MTriplet(i, j, EqualityJac(i, j)));
            }
        }

    }

    void getInequalityConstraintsJacobian(const dVector &p, SparseMatrix &J) override {
        // todo:
        J.resize(num_inequ_constraint, state_dim);
        J.setZero();
        std::vector<MTriplet> triplets;
        triplets.clear();
        addInequalityConstraintsJacobianEntriesTo(triplets);
        J.setFromTriplets(triplets.begin(), triplets.end());
    }

    void addInequalityConstraintsJacobianEntriesTo(std::vector<MTriplet> &JacEntries) {
        auto InequalityJac = C;
        for (int i = 0; i < InequalityJac.rows(); ++i) {
            for (int j = 0; j < InequalityJac.cols(); ++j) {
                JacEntries.push_back(MTriplet(i, j, InequalityJac(i, j)));
            }
        }
    }

    /*! @returns b of A(p) = b
	 */
    const dVector &getEqualityConstraintsTargetValues() override { return b; }


    /*! @returns d of d <= C(p) <= f
    */
    const dVector &getInequalityConstraintsMinValues() override {
        d.setConstant(num_inequ_constraint, -1000000);
        return d;
    }

    /*! @returns f of d <= C(p) <= f
    */
    const dVector &getInequalityConstraintsMaxValues() override {
        f.setConstant(num_inequ_constraint, 0);
        return f;
    }


    /*! @returns min of constraint min <= p <= max
*/
    const dVector &getBoundConstraintsMinValues() override {
        l.setConstant(state_dim, 0);
        l.segment(0, joint_dim).setConstant(-200);
        l.segment(joint_dim, state_dim).setConstant(-1000);
        return l;
    };

    /*! @returns max of constraint min <= p <= max
    */
    const dVector &getBoundConstraintsMaxValues() override {
        u.setConstant(state_dim, 0);
        u.segment(0, joint_dim).setConstant(200);
        u.segment(joint_dim, state_dim).setConstant(1000);
        return u;
    };
};

struct Data
{
    double dev_x;
    double dev_y;
    double dev_z;
    double yaw;
    double pitch;
    double roll;
    double f_inv;
    double v_ref;
    double w_ref;
    bool grad;
    bool auto_navi;
    bool auto_navi_with_obstacles;
    bool with_sphere_terrain;
    bool st_jump;
    bool compute_body_pos;
    P3D goal_pos{0,0,0};
    vector<P3D> obstacles;
    P3D sphere_pos;
    double sphere_r;
};