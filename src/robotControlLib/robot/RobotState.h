#pragma once

#include <robot/Robot.h>

#include <utils/mathUtils.h>
#include <utils/utils.h>

class JointState {
public:
	Quaternion qRel = Quaternion::Identity();
	V3D angVelRel = V3D(0,0,0);
};

class RobotState {
private:
	Quaternion rootQ = Quaternion::Identity();
	P3D rootPos = P3D(0, 0, 0);
	V3D rootVel = V3D(0, 0, 0);
	V3D rootAngVel = V3D(0, 0, 0);

	//to compute headings, we need to know which axis defines it (the yaw)
	V3D headingAxis = RBGlobals::worldUp;

	std::vector<JointState> joints;

public:
	~RobotState() {}

	inline RobotState(const RobotState& other) {
		rootQ = other.rootQ;
		rootPos = other.rootPos;
		rootVel = other.rootVel;
		rootAngVel = other.rootAngVel;
		headingAxis = other.headingAxis;
		joints = other.joints;
	}

	inline RobotState(int jCount = 0, int aJCount = 0){
		joints.resize(jCount);
	}

	inline RobotState(Robot* robot, bool useDefaultAngles = false){
		robot->populateState(this, useDefaultAngles);
	}

	inline void setJointCount(int jCount) {
		joints.resize(jCount);
	}

	inline void setHeadingAxis(V3D v){
		headingAxis = v;
	}

	inline V3D getHeadingAxis() {
		return headingAxis;
	}

	inline int getJointCount() const{
		return (int)joints.size();
	}

	inline P3D getPosition() const{
		return rootPos;
	}

	inline void setPosition(P3D p){
		rootPos = p;
	}

	inline Quaternion getOrientation() const{
		return rootQ;
	}

	inline void setOrientation(Quaternion q){
		rootQ = q;
	}

	inline V3D getVelocity() const{
		return rootVel;
	}

	inline void setVelocity(V3D v){
		rootVel = v;
	}

	inline V3D getAngularVelocity() const{
		return rootAngVel;
	}

	inline void setAngularVelocity(V3D v){
		rootAngVel = v;
	}
	
	inline Quaternion getJointRelativeOrientation(int jIndex) const{
		if ((uint)jIndex < joints.size())
			return joints[jIndex].qRel;
	//	exit(0);
		return Quaternion::Identity();
	}

	inline V3D getJointRelativeAngVelocity(int jIndex) const {
		if ((uint)jIndex < joints.size())
			return joints[jIndex].angVelRel;
	//	exit(0);
		return V3D(0,0,0);
	}

	inline void setJointRelativeOrientation(const Quaternion& q, int jIndex){
		if ((uint)jIndex < joints.size())
			joints[jIndex].qRel = q;
	//	else
	//		exit(0);

	}

	inline void setJointRelativeAngVelocity(V3D w, int jIndex) {
		if ((uint)jIndex < joints.size())
			joints[jIndex].angVelRel = w;
	//	else
	//		exit(0);
	}

	inline double getHeading() {
		//first we need to get the current heading of the robot. 
		return getRotationAngle(computeHeading(getOrientation(), headingAxis), headingAxis);
	}

	bool operator == (const RobotState& other) {
		if (getJointCount() != other.getJointCount()) {
			//			Logger::consolePrint("jCount: %d vs %d\n", getJointCount(), other.getJointCount());
			return false;
		}

		if (V3D(getPosition(), other.getPosition()).norm() > 1e-10) {
			//			Logger::consolePrint("pos: %lf %lf %lf vs %lf %lf %lf\n", 
			//				getPosition().x(), getPosition().y(), getPosition().z(), 
			//				other.getPosition().x(), other.getPosition().y(), other.getPosition().z());
			return false;
		}

		Quaternion q1 = getOrientation();
		Quaternion q2 = other.getOrientation();

		if (!sameRotation(q1, q2)) {
			//	Logger::consolePrint("orientation: %lf %lf %lf %lf vs %lf %lf %lf %lf\n", q1.s, q1.v.x(), q1.v.y(), q1.v.z(), q2.s, q2.v.x(), q2.v.y(), q2.v.z());
			return false;
		}

		if ((getVelocity() - other.getVelocity()).norm() > 1e-10) {
			//			Logger::consolePrint("vel: %lf %lf %lf vs %lf %lf %lf\n",
			//				getVelocity().x(), getVelocity().y(), getVelocity().z(),
			//				other.getVelocity().x(), other.getVelocity().y(), other.getVelocity().z());
			return false;
		}

		if ((getAngularVelocity() - other.getAngularVelocity()).norm() > 1e-10) {
			//			Logger::consolePrint("ang vel: %lf %lf %lf vs %lf %lf %lf\n",
			//				getAngularVelocity().x(), getAngularVelocity().y(), getAngularVelocity().z(),
			//				other.getAngularVelocity().x(), other.getAngularVelocity().y(), other.getAngularVelocity().z());
			return false;
		}

		for (int i = 0; i < getJointCount(); i++) {
			if ((getJointRelativeAngVelocity(i) - other.getJointRelativeAngVelocity(i)).norm() > 1e-10) {
				//				Logger::consolePrint("joint %d ang vel: %lf %lf %lf vs %lf %lf %lf\n", i,
				//					getJointRelativeAngVelocity(i).x(), getJointRelativeAngVelocity(i).y(), getJointRelativeAngVelocity(i).z(),
				//					other.getJointRelativeAngVelocity(i).x(), other.getJointRelativeAngVelocity(i).y(), other.getJointRelativeAngVelocity(i).z());
				return false;
			}

			Quaternion q1 = getJointRelativeOrientation(i);
			Quaternion q2 = other.getJointRelativeOrientation(i);

			if (!sameRotation(q1, q2)) {
				//				Logger::consolePrint("joint %d orientation: %lf %lf %lf %lf vs %lf %lf %lf %lf\n", i, q1.s, q1.v.x(), q1.v.y(), q1.v.z(), q2.s, q2.v.x(), q2.v.y(), q2.v.z());
				return false;
			}
		}

		return true;
	}

	void writeToFile(const char* fName, Robot* robot = nullptr);
	void readFromFile(const char* fName);
	void setHeading(double heading);
};
