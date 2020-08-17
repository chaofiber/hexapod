#include <robot/Robot.h>

#include <robot/RobotState.h>
#include <robot/RBEngine.h>
#include <robot/RBLoader.h>
#include <robot/GeneralizedCoordinatesRobotRepresentation.h>
#include <robot/RBJoint.h>

/**
	the constructor
*/
Robot::Robot(const char *filePath, const char *statePath)
	: Robot(nullptr, filePath, statePath)
{
}

Robot::Robot(RBEngine *rbEngine, const char *filePath, const char *statePath)
{
    //load robot from rbLoader
    crl::robot::RBLoader rbLoader(filePath);
    rbLoader.populateRobot(this);

	//this needs to be investigated more soon
    //TODO: is this correct? Are there RBs (temp/fused ones) that do not make it into the robot, and so should not make it into the rb engine either?
    // if (rbEngine)
    //     rbLoader.populateRBEngine(rbEngine);

    //TODO: otherwise, revert to this bit of code
	if (rbEngine) {
        //add to rbs and joints to rbengine
        for (auto rb : this->rbList)
            rbEngine->addRigidBodyToEngine(rb);

        for (auto j : this->jointList)
            rbEngine->addJointToEngine(j);
    }
	
	//load robot state
	if (statePath)
		loadReducedStateFromFile(statePath);

}

Robot::~Robot()
{
}

/**
	uses the state of the robot to populate the input
*/
void Robot::populateState(RobotState *state, bool useDefaultAngles)
{
	//we'll push the root's state information - ugly code....
	state->setPosition(root->state.pos);
	state->setOrientation(root->state.orientation);
	state->setVelocity(root->state.velocity);
	state->setAngularVelocity(root->state.angularVelocity);
	state->setHeadingAxis(RBGlobals::worldUp);

	state->setJointCount((int)jointList.size());

	//now each joint introduces one more rigid body, so we'll only record its state relative to its parent.
	//we are assuming here that each joint is revolute!!!

	for (uint i = 0; i < jointList.size(); i++)
	{
		if (!useDefaultAngles)
		{
			state->setJointRelativeOrientation(getRelativeOrientationForJoint(jointList[i]), i);
			state->setJointRelativeAngVelocity(getRelativeLocalCoordsAngularVelocityForJoint(jointList[i]), i);
		}
		else
		{
			state->setJointRelativeAngVelocity(V3D(0, 0, 0), i);
			state->setJointRelativeOrientation(getRotationQuaternion(jointList[i]->defaultJointAngle, jointList[i]->rotationAxis), i);
		}
	}
}

void Robot::setDefaultState()
{
	RobotState rs;
	populateState(&rs, true);
	setState(&rs);
}

/**
	This method populates the state of the current robot with the values that are passed
	in the dynamic array. The same conventions as for the getState() method are assumed.
*/
void Robot::setState(RobotState *state)
{
	//kinda ugly code....
	root->state.pos = state->getPosition();
	root->state.orientation = state->getOrientation();
	root->state.orientation.normalize();
	root->state.velocity = state->getVelocity();
	root->state.angularVelocity = state->getAngularVelocity();

	//now each joint introduces one more rigid body, so we'll only record its state relative to its parent.
	//we are assuming here that each joint is revolute!!!
	for (uint j = 0; j < jointList.size(); j++)
	{
		setRelativeOrientationForJoint(jointList[j], state->getJointRelativeOrientation((int)j).normalized());
		setRelativeLocalCoordsAngularVelocityForJoint(jointList[j], state->getJointRelativeAngVelocity((int)j));
		//and now set the linear position and velocity
		jointList[j]->fixJointConstraints(true, true, true, true);
	}
}

/**
	makes sure the state of the robot is consistent with all the joint types...
*/
void Robot::fixJointConstraints()
{
	for (size_t j = 0; j < jointList.size(); j++)
		jointList[j]->fixJointConstraints(true, true, true, true);
}

/**
	This method is used to compute the center of mass of the robot.
*/
P3D Robot::computeCOM()
{
	P3D COM = root->state.pos * root->rbProps.mass;
	double totalMass = root->rbProps.mass;

	for (uint i = 0; i < jointList.size(); i++)
	{
		totalMass += jointList[i]->child->rbProps.mass;
		COM += jointList[i]->child->state.pos * jointList[i]->child->rbProps.mass;
	}

	return COM / totalMass;
}

double Robot::getMass()
{
	//compute the mass of the robot
	double mass = root->rbProps.mass;
	for (uint i = 0; i < jointList.size(); i++)
		mass += jointList[i]->child->rbProps.mass;
	return mass;
}

/**
	This method is used to compute the velocity of the center of mass of the robot.
*/
V3D Robot::computeCOMVelocity()
{
	V3D COMVel = root->state.velocity * root->rbProps.mass;
	double totalMass = root->rbProps.mass;

	for (uint i = 0; i < jointList.size(); i++)
	{
		totalMass += jointList[i]->child->rbProps.mass;
		COMVel += jointList[i]->child->state.velocity * jointList[i]->child->rbProps.mass;
	}

	return COMVel / totalMass;
}

/**
	this method updates the robot's root state and all connected rigid bodies
*/
void Robot::setRootState(const P3D &position, const Quaternion &orientation)
{
	RobotState state(this);
	populateState(&state);
	state.setPosition(position);
	state.setOrientation(orientation);
	setState(&state);
}

/**
	this method is used to read the reduced state of the robot from the file
*/
void Robot::loadReducedStateFromFile(const char *fName)
{
	RobotState state(this);
	state.readFromFile(fName);
	setState(&state);
}

/**
	this method is used to write the reduced state of the robot to a file
*/
void Robot::saveReducedStateToFile(const char *fName)
{
	RobotState state(this);
	state.writeToFile(fName, this);
}

/**
this method is used to return a reference to the robot's rigid body whose name is passed in as a parameter,
or nullptr if it is not found.
*/
RobotRB *Robot::getRBByName(const char *jName)
{
	for (uint i = 0; i < jointList.size(); i++)
	{
		if (strcmp(jointList[i]->parent->name.c_str(), jName) == 0)
			return jointList[i]->parent;
		if (strcmp(jointList[i]->child->name.c_str(), jName) == 0)
			return jointList[i]->child;
	}
	std::cout << "WARNING: Robot:getRBByName -> rigid body could not be found..." << std::endl;
	return nullptr;
}
