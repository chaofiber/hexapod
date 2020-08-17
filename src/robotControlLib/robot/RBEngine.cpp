#include <robot/RBEngine.h>
#include <robot/RBUtils.h>
#include <utils/utils.h>

RBEngine::~RBEngine(void){
	destroy();
}


void RBEngine::destroy() {
	//delete all the rigid bodies in this world
	for (uint i = 0; i < rbs.size();i++)
		delete rbs[i];
	rbs.clear();
	for (uint i = 0; i < joints.size();i++) 
		delete joints[i];
	joints.clear();
}


/**
	This method returns the reference to the rigid body with the given name, or nullptr if it is not found
*/
RobotRB* RBEngine::getRBByName(const char* name){
	if (name == nullptr)
		return nullptr;
	for (int i=(int)rbs.size()-1;i>=0;i--)
		if (strcmp(name, rbs[i]->name.c_str()) == 0)
			return rbs[i];
    throwError("RBEngine::getRBByName -> rigid body with name %s does not exist", name);
	return nullptr;
}


/**
	This method returns the reference to the joint whose name matches, or nullptr if it is not found
*/
RBJoint* RBEngine::getJointByName(char* name) {
	if (name == nullptr)
		return nullptr;
	for (int i = (int)joints.size() - 1;i >= 0;i--)
		if (strcmp(name, joints[i]->name.c_str()) == 0)
			return joints[i];
    throwError("RBEngine::getJointByName -> joint with name %s does not exist", name);
	return nullptr;
}


void RBEngine::addRigidBodyToEngine(RobotRB* rb) {
	rbs.push_back(rb);
}


void RBEngine::addJointToEngine(RBJoint* j) {
	joints.push_back(j);
}
