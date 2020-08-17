#pragma once
#include <ode/ode.h>

#include <robot/RBEngine.h>
#include <robot/RBJoint.h>

#include "ode/ContactForce.h"

#define MAX_CONTACT_FEEDBACK 200
#define MAX_AMOTOR_FEEDBACK 200	

using namespace std;

using RigidBody = RobotRB;				// RigidBody is now renamed into RobotRB
using Joint = RBJoint;					// Joint is now renamed into RBJoint
using HingeJoint = RBJoint;				// HingeJoint is now renamed into RBJoint

using BoxCDP = crl::robot::RRBCollisionBox;
using SphereCDP = crl::robot::RRBCollisionSphere;
using PlaneCDP = crl::robot::RRBCollisionPlane;

namespace crl {
namespace sim {

//this structure is used to map a rigid body to the id of its ODE counterpart
typedef struct ODE_RB_Map_struct {
	dBodyID id;
	RigidBody* rb;
	ODE_RB_Map_struct(dBodyID newId, RigidBody* newRb) { this->id = newId; this->rb = newRb; }
	ODE_RB_Map_struct() {};
} ODE_RB_Map;

//map joints that are controlled with desired position or velocity to an ODE motor
typedef struct ODE_MOTOR_JOINT_Map {
	dJointID motorID;
	Joint* joint;
	ODE_MOTOR_JOINT_Map(dJointID motorID, HingeJoint* joint) { this->motorID = motorID; this->joint = joint; }
} ODE_Motor_Joint_Map;

//this structure is used to map joint to the id of its ODE counterpart
typedef struct ODE_Joint_Map_struct {
	dJointID id;
	Joint* j;
	ODE_Joint_Map_struct(dJointID newId, Joint* newJ) { this->id = newId; this->j = newJ; }
	ODE_Joint_Map_struct() {};
} ODE_Joint_Map;

//this structure is used to map a collision detection primitive to the id of its ODE counterpart
typedef struct ODE_CDP_Map_struct {
	dGeomID id;
	dGeomID t;
	RigidBody* rb;
	int cdpIndex;
	ODE_CDP_Map_struct(dGeomID newId, dGeomID newT, RigidBody* newRB, int _cdpIndex) { this->id = newId; this->t = newT; this->rb = newRB; cdpIndex = _cdpIndex; }
} ODE_CDP_Map;

/*--------------------------------------------------------------------------------------*
 * This class is used as a wrapper for the Open Dynamics Engine.                        *
 *--------------------------------------------------------------------------------------*/
class ODERBEngine : public RBEngine {
	friend void collisionCallBack(void* rbEngine, dGeomID o1, dGeomID o2);
public://TODO: this was private before. need public variables above
	// ODE's id for the simulation world
	dWorldID worldID;
	// id of collision detection space
	dSpaceID spaceID;
	// id of contact group
	dJointGroupID contactGroupID;
	//keep track of the mapping between the rigid bodies and their ODE counterparts with this
	DynamicArray<ODE_RB_Map> odeToRbs;
	DynamicArray<ODE_Joint_Map> odeToJoints;
	DynamicArray<ODE_CDP_Map> odeToCDP;
	DynamicArray<ODE_Motor_Joint_Map> motorToJointmap;

	//contact force 
	DynamicArray<ContactForce> contactForces;

	//joint mode
	

private: //TODO: this was not here before. need public variables above
	//keep an array of contact points that is used for each pair of geom collisions
	dContact cps[10];

	dJointFeedback jointFeedback[MAX_CONTACT_FEEDBACK];
	dJointFeedback amotorFeedback[MAX_AMOTOR_FEEDBACK];
	//this is the current number of contact joints, for the current step of the simulation
	int jointFeedbackCount = 0;


	/**
	this method is used to set up an ODE sphere geom. It is properly placed in body coordinates.
	*/
	dGeomID getSphereGeom(const SphereCDP *s, const RigidBody *rb);

	/**
	this method is used to set up an ODE box geom. It is properly placed in body coordinates.
	*/
// #ifdef DEPRECATED
	dGeomID getBoxGeom(const BoxCDP* b, const RigidBody *rb);
// #endif

	/**
	this method is used to set up an ODE plane geom. It is properly placed in body coordinates.
	*/
	dGeomID getPlaneGeom(PlaneCDP* p);

	/**
	this method is used to set up an ODE capsule geom. It is properly placed in body coordinates.
	*/
#ifdef DEPRECATED
	dGeomID getCapsuleGeom(CapsuleCDP* c);
	void setCapsuleGeomTransformation(CapsuleCDP* c, dGeomID g);
#endif

	/**
	this method is used to process the collision between the two objects passed in as parameters. More specifically,
	it is used to determine if the collision should take place, and if so, it calls the method that generates the
	contact points.
	*/
	void processCollisions(dGeomID o1, dGeomID o2, DynamicArray<ContactForce> &contactForces);

	/**
	this method is used to create ODE geoms for all the collision primitives of the rigid body that is passed in as a paramter
	*/
	void createODECollisionPrimitives(RigidBody* body);

	/**
	this method is used to update ODE geoms for all the collision primitives of the rigid body that is passed in as a paramter
	*/
	void updateODECollisionPrimitives(RigidBody* body);

	/**
	this method is used to copy the state of the ith rigid body to its ode counterpart.
	*/
	void setODEStateFromRB(int i);

	/**
	this method is used to copy the state of the ith rigid body, from the ode object to its rigid body counterpart
	*/
	void setRBStateFromODE(int i);

	/**
	This method is used to set up an ode hinge joint, based on the information in the hinge joint passed in as a parameter
	*/
	void setupODEHingeJoint(HingeJoint* hj);

	/**
	Create an ODE motor (control purposes) for the specified joint
	*/
    int getODEMotorForJoint(Joint* j);

	/**
	This method is used to set up an ode universal joint, based on the information in the universal joint passed in as a parameter
	*/
#ifdef DEPRECATED
	void setupODEUniversalJoint(UniversalJoint* uj);
#endif

	/**
	This method is used to set up an ode ball-and-socket joint, based on the information in the ball in socket joint passed in as a parameter
	*/
#ifdef DEPRECATED
	void setupODEBallAndSocketJoint(BallAndSocketJoint* basj);
#endif

	/**
	this method is used to transfer the state of the rigid bodies, from the simulator to the rigid body wrapper
	*/
	void setRBStateFromEngine(DynamicArray<RigidBody*> &objects);

	/**
	this method is used to transfer the state of the rigid bodies, from the rigid body wrapper to the simulator's rigid bodies
	*/
	void setEngineStateFromRB(DynamicArray<RigidBody*> &objects);

public:
	bool iterativeSolution = false;

	double contactDampingCoefficient = 0.00001;
	double contactStiffnessCoefficient = 0.2;

	/**
		default constructor
	*/
	ODERBEngine();

	/**
		destructor
	*/
	virtual ~ODERBEngine(void);

	void addRigidBodyToEngine(RigidBody* rb) override;

	void addJointToEngine(Joint* j) override;

	/**
		This method is used to integrate the simulation forward in time.
	*/
	void step(double deltaT);

	/**
		This method is used to set the state of all the rigid body in the physical world.
	*/
#ifdef DEPRECATED
	void setState(DynamicArray<double>* state, int start = 0);
#endif

	/**
		this method applies a force to a rigid body, at the specified point. The point is specified in local coordinates,
		and the force is specified in world coordinates.
	*/
	void applyForceTo(RigidBody* b, const V3D& f, const P3D& p);

	/**
		this method applies a torque to a rigid body. The torque is specified in world coordinates.
	*/
	void applyTorqueTo(RigidBody* b, const V3D& t);

	/**
	this method applies a torque to a rigid body. The torque is specified in relative body coordinates.
	*/
	void applyRelativeTorqueTo(RigidBody* b, const V3D& t);
	
	///< ?
	void setGlobalCFM(double CFM);

	void setMotorsCFMAndFMax(double CFM, double FMAX);

	/**
		Update ODERBEngine using current rigidbodies.
	*/
	void updateODERBEngineFromRBs();

	/*
		Get the total contact force applied on a rigid body
	*/
	virtual DynamicArray<ContactForce> getContactForceOnRB(RigidBody* b);

	//David G. added
	/**
		Get the world ID from ODE
	*/

	virtual dWorldID getWorldID();
	virtual dSpaceID getSpaceID();

	void loadRBsFromFile(const char* fName) {
		crl::robot::RBLoader rbLoader(fName);
		std::vector<RobotRB *> rbs =rbLoader.rbs;
		for (auto i:rbs){
			cout<<i->name;
		}
		rbLoader.populateRBEngine(this);
	}

	/**
		Draw function for debugging purposes
	*/
#ifdef CRL_USE_GUI
	void drawODERB(const Shader &shader);
	void drawODEJoint(const Shader &shader);
	void drawODEContact(const Shader &shader);
#endif
};

} // namespace sim
} // namespace crl
