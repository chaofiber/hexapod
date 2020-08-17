#pragma once

#include <utils/mathUtils.h>
#include <utils/utils.h>
#include <robot/RobotRB.h>
#include <robot/RBJoint.h>
#include <robot/RBLoader.h>

#ifdef CRL_USE_URDFDOM
#include <urdf_parser/urdf_parser.h>
#endif

/*--------------------------------------------------------------------------------------------------------------------------------------------*
 * This class implements a container for robot rigid bodies. It reads a .rbs file and interprets it.             *
 *--------------------------------------------------------------------------------------------------------------------------------------------*/
class RBEngine{
	
public:
	//this is a list of all the objects in the world
	std::vector<RobotRB*> rbs;
	//and we'll keep a list of all the joints in the world as well - they impose constraints on the relative movement between the rigid bodies they connet
	std::vector<RBJoint*> joints;

public:
	//the constructor
	RBEngine(void) {}
	//the destructor
	virtual ~RBEngine(void);

	virtual void destroy();

	virtual void addRigidBodyToEngine(RobotRB* rb);

	virtual void addJointToEngine(RBJoint* j);
	
	/**
		This method returns the reference to the rigid body with the given name, or nullptr if it is not found
	*/
	RobotRB* getRBByName(const char* name);

	/**
		This method returns the reference to the joint whose name matches, or nullptr if it is not found
	*/
	RBJoint* getJointByName(char* name);

	/**
		This method returns true if rb is in rbs or false if it is not found 
	*/
	bool hasRB(const RobotRB* rb) {
		for (auto it = rbs.begin(); it != rbs.end(); ++it) 
			if(rb == *it)
				return true;
		return false;
	}
};

