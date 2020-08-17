#include <memory>

#ifdef CRL_USE_GUI
#include <gui/renderer.h>
#endif

#include "ode/ODERBEngine.h"

#include <utils/logger.h>
#include <robot/RBUtils.h>

namespace crl 
{
namespace sim 
{
//TODO: once joints are set to position mode, the motors are persistent. We should perhaps have a way to turn them off, if we want, for instance, to transition between passive and active, or between position and control mode...

/**
Default constructor
*/
ODERBEngine::ODERBEngine() : RBEngine() {
	//Initialize the world, simulation space and joint groups
	dInitODE();
	worldID = dWorldCreate();
	spaceID = dHashSpaceCreate(0);
	contactGroupID = dJointGroupCreate(0);

	//make sure that when we destroy the space group, we destroy all the geoms inside it
	dSpaceSetCleanup(spaceID, 1);

	//set a few of the constants that ODE needs to be aware of
	dWorldSetContactSurfaceLayer(worldID, 0.001);							// the ammount of interpenetration allowed between rbs
	dWorldSetContactMaxCorrectingVel(worldID, 1.0);							// maximum velocity that contacts are allowed to generate  

	V3D gravity = RBGlobals::worldUp * RBGlobals::g;
	dWorldSetGravity(worldID, gravity[0], gravity[1], gravity[2]);
}

/**
destructor
*/
ODERBEngine::~ODERBEngine(void) {
	//destroy the ODE physical world, simulation space and joint group
	dJointGroupDestroy(contactGroupID);
	dSpaceDestroy(spaceID);
	dWorldDestroy(worldID);
	dCloseODE();
}


/**
	this method is used to set up an ODE sphere geom. It is properly placed in body coordinates.
*/
dGeomID ODERBEngine::getSphereGeom(const SphereCDP *s, const RigidBody *rb) {
	dGeomID g = dCreateSphere(0, s->radius);
	P3D c = s->localCoordinates;
	dGeomSetPosition(g, c[0], c[1], c[2]);
	return g;
}

/**
	this method is used to set up an ODE sphere geom. NOTE: ODE only allows planes to
	be specified in world coordinates, not attached to a body, so we need to fix it once and
	for all.
*/
dGeomID ODERBEngine::getPlaneGeom(PlaneCDP* p) {
	//and create the ground plane
	V3D n = p->n;
	V3D o = V3D(p->p);
	dGeomID g = dCreatePlane(spaceID, n[0], n[1], n[2], o.dot(n));
	return g;
}

dGeomID ODERBEngine::getBoxGeom(const BoxCDP* b, const RigidBody *rb) {
	P3D dim = b->dim;
	dGeomID g = dCreateBox(0, dim.x, dim.y, dim.z);
	P3D c = b->localCoordinates;
	dGeomSetPosition(g, c[0], c[1], c[2]);
	return g;
}
/**
	this method is used to set up an ODE box geom. It is properly placed in body coordinates.
*/
// #ifdef DEPRECATED
// dGeomID ODERBEngine::getBoxGeom(BoxCDP* b) {
// 	dGeomID g = dCreateBox(0, b->getXLen(), b->getYLen(), b->getZLen());
// 	P3D c = b->getCenter();
// 	dGeomSetPosition(g, c[0], c[1], c[2]);
// 	return g;
// }
// #endif


#ifdef DEPRECATED
void ODERBEngine::setCapsuleGeomTransformation(CapsuleCDP* c, dGeomID g) {
	V3D ab = V3D(c->p1, c->p2);

	if(c->hasFlatCaps)
		dGeomCylinderSetParams(g, c->r, ab.length());
	else
		dGeomCapsuleSetParams(g, c->r, ab.length());

	P3D cen = c->p1*0.5 + c->p2*0.5;
	dGeomSetPosition(g, cen[0], cen[1], cen[2]);

	V3D defA(0, 0, 1);

	V3D axis = defA.cross(ab);

	if (!IS_ZERO(axis.length())){

		axis.toUnit();
		double rotAngle = defA.angleWith(ab);

		Quaternion relOrientation = getRotationQuaternion(rotAngle, axis);

		dQuaternion q;
		q[0] = relOrientation.s;
		q[1] = relOrientation.v[0];
		q[2] = relOrientation.v[1];
		q[3] = relOrientation.v[2];

		dGeomSetQuaternion(g, q);
	}
}
#endif

/**
	this method is used to set up an ODE sphere geom. It is properly placed in body coordinates.
*/
#ifdef DEPRECATED
dGeomID ODERBEngine::getCapsuleGeom(CapsuleCDP* c) {
	P3D a = c->p1;
	P3D b = c->p2;
	V3D ab(a, b);
	dGeomID g;

	if (c->hasFlatCaps)
		g = dCreateCylinder(0, c->r, ab.length());
	else
		g = dCreateCapsule(0, c->r, ab.length());

	setCapsuleGeomTransformation(c, g);

	return g;
}
#endif

/**
this method is used to process the collision between the two rbs passed in as parameters. More generally,
it is used to determine if the collision should take place, and if so, it calls the method that generates the
contact points.
*/
void ODERBEngine::processCollisions(dGeomID o1, dGeomID o2, DynamicArray<ContactForce> &contactForces) {
	dBodyID b1, b2;
	RigidBody *rb1, *rb2;
	b1 = dGeomGetBody(o1);
	b2 = dGeomGetBody(o2);
	rb1 = (RigidBody*)dGeomGetData(o1);
	rb2 = (RigidBody*)dGeomGetData(o2);

	bool joined = b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact);

	if (joined)
		return;
	if (!rb1->rbProps.fixed && !rb2->rbProps.fixed)
		return;
//	if (rb1->rbProps.collisionGroupID == rb2->rbProps.collisionGroupID && rb1->rbProps.collisionGroupID != -1)
//		return;

	//we'll use the minimum of the two coefficients of friction of the two bodies.
	double mu1 = rb1->rbProps.frictionCoeff;
	double mu2 = rb2->rbProps.frictionCoeff;
	double mu_to_use = std::min(mu1, mu2);

	//	mu_to_use = 0;

	double eps1 = rb1->rbProps.restitutionCoeff;
	double eps2 = rb2->rbProps.restitutionCoeff;
	double eps_to_use = std::min(eps1, eps2);

	int maxContactCount = sizeof(cps) / sizeof(cps[0]);
	int num_contacts = dCollide(o1, o2, maxContactCount, &(cps[0].geom), sizeof(dContact));

	// and now add them contact points to the simulation
	for (int i = 0;i<num_contacts;i++) {
		//fill in the missing properties for the contact points
		cps[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactBounce;
		cps[i].surface.mu = mu_to_use;

		cps[i].surface.bounce = eps_to_use;
		cps[i].surface.bounce_vel = 0.00001;

		cps[i].surface.soft_cfm = contactDampingCoefficient;
		cps[i].surface.soft_erp = contactStiffnessCoefficient;

		//create a joint, and link the two geometries.
		dJointID c = dJointCreateContact(worldID, contactGroupID, &cps[i]);
		dJointAttach(c, b1, b2);

		if (jointFeedbackCount >= MAX_CONTACT_FEEDBACK)
			Logger::consolePrint("Warning: too many contacts are established. Some of them will not be reported.\n");
		else {
			if (contactForces.size() != jointFeedbackCount) {
				Logger::consolePrint("Warning: Contact forces need to be cleared after each simulation, otherwise the results are not predictable.\n");
			}
			contactForces.push_back(ContactForce());
			//now we'll set up the feedback for this contact joint
			contactForces[jointFeedbackCount].rb1 = rb1;
			contactForces[jointFeedbackCount].rb2 = rb2;
			contactForces[jointFeedbackCount].cp = P3D(cps[i].geom.pos[0], cps[i].geom.pos[1], cps[i].geom.pos[2]);
			contactForces[jointFeedbackCount].n = V3D(cps[i].geom.normal[0], cps[i].geom.normal[1], cps[i].geom.normal[2]);
			dJointSetFeedback(c, &(jointFeedback[jointFeedbackCount]));
			jointFeedbackCount++;
		}
	}
}

/**
this method is used to create ODE geoms for all the collision primitives of the rigid body that is passed in as a paramter
*/
void ODERBEngine::createODECollisionPrimitives(RigidBody* body) {
	//now we'll set up the body's collision detection primitives
	for (uint j = 0;j<body->rbProps.collisionShapes.size();j++) {
		dGeomID g;
		if (auto cdp = std::dynamic_pointer_cast<SphereCDP>(body->rbProps.collisionShapes[j])) {
			g = getSphereGeom(cdp.get(), body);
		}
		else if (auto cdp = std::dynamic_pointer_cast<PlaneCDP>(body->rbProps.collisionShapes[j])) {
			g = getPlaneGeom(cdp.get());
			dGeomSetData(g, body);
			continue;
		}
		else if (auto cdp = std::dynamic_pointer_cast<BoxCDP>(body->rbProps.collisionShapes[j])) {
			g = getBoxGeom(cdp.get(),body);
			cout<<"1"<<endl;
			continue;
		}
		else {
			throwError("Unknown type of collision encountered...\n");
		}
#ifdef DEPRECATED
		else if (CapsuleCDP* cdp = dynamic_cast<CapsuleCDP*>(body->cdps[j])) {
			g = getCapsuleGeom(cdp);
		}
		else if (BoxCDP* cdp = dynamic_cast<BoxCDP*>(body->cdps[j])) {
			g = getBoxGeom(cdp);
		}
		else if (PlaneCDP* cdp = dynamic_cast<PlaneCDP*>(body->cdps[j])) {
			g = getPlaneGeom(cdp);
			dGeomSetData(g, body);
			continue;
		}
		else {
			throwError("Unknown type of collision encountered...\n");
		}
#endif		
		//now associate the geom to the rigid body that it belongs to, so that we can look up the properties we need later...
		dGeomSetData(g, body);

		//now we've created a geom for the current body. Note: g will be rotated relative to t, so that it is positioned
		//well in body coordinates, and then t will be attached to the body.
		dGeomID t = dCreateGeomTransform(spaceID);
		//make sure that when we destroy the transfromation, we destroy the encapsulated rbs as well.
		dGeomTransformSetCleanup(t, 1);

		//associate the transform geom with the body as well
		dGeomSetData(t, body);

		//if the object is fixed, then we want the geometry to take into account the initial position and orientation of the rigid body
		if (body->rbProps.fixed) {
			dGeomSetPosition(t, body->state.pos.x, body->state.pos.y, body->state.pos.z);
			dQuaternion q;
			q[0] = body->state.orientation.w();
			q[1] = body->state.orientation.x();
			q[2] = body->state.orientation.y();
			q[3] = body->state.orientation.z();
			dGeomSetQuaternion(t, q);
		}

		dGeomTransformSetGeom(t, g);
		//now add t (which contains the correctly positioned geom) to the body, but only if the body isn't static...
		dGeomSetBody(t, odeToRbs[body->rbProps.id].id);

		odeToCDP.push_back(ODE_CDP_Map_struct(g, t, body, j));
	}
}

/**
	this method is used to update ODE geoms for all the collision primitives of the rigid body that is passed in as a paramter
*/
void ODERBEngine::updateODECollisionPrimitives(RigidBody* body){
	int nCdps = (int)body->rbProps.collisionShapes.size();
	for (int j = 0;j<nCdps;j++) {
		//find the geom that belongs to this CDP
		dGeomID g;
		dGeomID t;

		bool found = false;
		for (int k = 0; k < (int)odeToCDP.size(); k++){
			if (odeToCDP[k].rb == body && odeToCDP[k].cdpIndex == j) {
				g = odeToCDP[k].id;
				t = odeToCDP[k].t;
				found = true;
				break;
			}
		}
		if (!found) continue;

#ifdef DEPRECATED
		if (CapsuleCDP* cdp = dynamic_cast<CapsuleCDP*>(body->cdps[j])) {
			setCapsuleGeomTransformation(cdp, g);
		}
		else if (BoxCDP* cdp = dynamic_cast<BoxCDP*>(body->cdps[j])) {
			dGeomBoxSetLengths(g, cdp->getXLen(), cdp->getYLen(), cdp->getZLen());
			P3D c = cdp->getCenter();
			dGeomSetPosition(g, c[0], c[1], c[2]);
		}
		else {
			throwError("Unknown type of collision encountered...\n");
		}
#endif
		if (auto cdp = std::dynamic_pointer_cast<SphereCDP>(body->rbProps.collisionShapes[j])) {
			P3D c = body->state.getWorldCoordinates(cdp->localCoordinates);
			dGeomSphereSetRadius(g, cdp->radius);
			dGeomSetPosition(g, c.x, c.y, c.z);
		}
		else if (auto cdp = std::dynamic_pointer_cast<PlaneCDP>(body->rbProps.collisionShapes[j])) {
			V3D n = cdp->n;
			V3D o = V3D(cdp->p);
			dGeomPlaneSetParams(g, n[0], n[1], n[2], o.dot(n));
		}
		else if (auto cdp = std::dynamic_pointer_cast<BoxCDP>(body->rbProps.collisionShapes[j])) {
			P3D dim = cdp->dim;
			dGeomBoxSetLengths(g, dim.x, dim.y, dim.z);
			P3D c = cdp->localCoordinates;
			dGeomSetPosition(g, c[0], c[1], c[2]);
		}
		else {
			throwError("Unknown type of collision encountered...\n");
		}

		//if the object is fixed, then we want the geometry to take into account the initial position and orientation of the rigid body, otherwise, the
		//transform should already be linked to the right ODE body, which is linked to the right rigid body...
		if (body->rbProps.fixed) {
			dGeomSetPosition(t, body->state.pos.x, body->state.pos.y, body->state.pos.z);
			dQuaternion q;
			q[0] = body->state.orientation.w();
			q[1] = body->state.orientation.x();
			q[2] = body->state.orientation.y();
			q[3] = body->state.orientation.z();
			dGeomSetQuaternion(t, q);
		}

	}
}

/**
	this method is used to copy the state of the ith rigid body to its ode counterpart.
*/
void ODERBEngine::setODEStateFromRB(int i) {
	if (i<0 || (uint)i >= odeToRbs.size())
		return;

	dQuaternion tempQ;
	tempQ[0] = odeToRbs[i].rb->state.orientation.w();
	tempQ[1] = odeToRbs[i].rb->state.orientation.x();
	tempQ[2] = odeToRbs[i].rb->state.orientation.y();
	tempQ[3] = odeToRbs[i].rb->state.orientation.z();

	dBodySetPosition(odeToRbs[i].id, odeToRbs[i].rb->state.pos.x, odeToRbs[i].rb->state.pos.y, odeToRbs[i].rb->state.pos.z);
	dBodySetQuaternion(odeToRbs[i].id, tempQ);
	dBodySetLinearVel(odeToRbs[i].id, odeToRbs[i].rb->state.velocity[0], odeToRbs[i].rb->state.velocity[1], odeToRbs[i].rb->state.velocity[2]);
	dBodySetAngularVel(odeToRbs[i].id, odeToRbs[i].rb->state.angularVelocity[0], odeToRbs[i].rb->state.angularVelocity[1], odeToRbs[i].rb->state.angularVelocity[2]);
}

/**
this method is used to copy the state of the ith rigid body, from the ode object to its rigid body counterpart
*/
void ODERBEngine::setRBStateFromODE(int i) {
	const dReal *tempData;

	tempData = dBodyGetPosition(odeToRbs[i].id);
	odeToRbs[i].rb->state.pos.x = tempData[0];
	odeToRbs[i].rb->state.pos.y = tempData[1];
	odeToRbs[i].rb->state.pos.z = tempData[2];

	tempData = dBodyGetQuaternion(odeToRbs[i].id);
	odeToRbs[i].rb->state.orientation.w() = tempData[0];
	odeToRbs[i].rb->state.orientation.x() = tempData[1];
	odeToRbs[i].rb->state.orientation.y() = tempData[2];
	odeToRbs[i].rb->state.orientation.z() = tempData[3];

	tempData = dBodyGetLinearVel(odeToRbs[i].id);
	odeToRbs[i].rb->state.velocity[0] = tempData[0];
	odeToRbs[i].rb->state.velocity[1] = tempData[1];
	odeToRbs[i].rb->state.velocity[2] = tempData[2];

	tempData = dBodyGetAngularVel(odeToRbs[i].id);
	odeToRbs[i].rb->state.angularVelocity[0] = tempData[0];
	odeToRbs[i].rb->state.angularVelocity[1] = tempData[1];
	odeToRbs[i].rb->state.angularVelocity[2] = tempData[2];
}

int ODERBEngine::getODEMotorForJoint(Joint* j) {
	HingeJoint* hj = dynamic_cast<HingeJoint*> (j);
	int motorID = -1;
	//create a motor if desired...
	if (hj) {
		dJointID aMotor = NULL;
		// Retrieve motor from map
		for (uint m = 0; m < motorToJointmap.size(); m++) {
			if (motorToJointmap[m].joint == hj) {
				aMotor = motorToJointmap[m].motorID;
				motorID = (int)m;
				break;
			}
		}
		if (aMotor == NULL) {
			aMotor = dJointCreateAMotor(worldID, 0);
			motorToJointmap.push_back(ODE_Motor_Joint_Map(aMotor, hj));
			motorID = (int)motorToJointmap.size()-1;
			dJointAttach(aMotor, odeToRbs[(int)(hj->parent->rbProps.id)].id, odeToRbs[(int)(hj->child->rbProps.id)].id);
			dJointSetAMotorMode(aMotor, dAMotorUser);
			dJointSetAMotorNumAxes(aMotor, 1);
			dJointSetAMotorParam(aMotor, dParamFMax, hj->maxTorque);
			//just in case the configuration of the joint changes (i.e. rotation axis), update the motor axis
			V3D a = hj->parent->state.getWorldCoordinates(hj->rotationAxis);
			dJointSetAMotorAxis(aMotor, 0, 1, a[0], a[1], a[2]);
			//we set ERP and CFM in step 
			// dJointSetAMotorParam(aMotor, dParamCFM, hj->motorConstraintForceRegularizer);
		}
	}else
		Logger::consolePrint("Joint motors are only implemented for Hinge Joints for now!\n");

	return motorID;

}

/**
	This method is used to set up an ode hinge joint, based on the information in the hinge joint passed in as a parameter
*/
void ODERBEngine::setupODEHingeJoint(HingeJoint* hj) {
	dJointID j;
	uint k = 0;
	for (k = 0; k < odeToJoints.size(); k++){
		if (odeToJoints[k].j == hj){
			j = odeToJoints[k].id;
			break;
		}
	}

	if (k == odeToJoints.size()){
		j = dJointCreateHinge(worldID, 0);
		odeToJoints.push_back(ODE_Joint_Map_struct(j, hj));
		dJointAttach(j, odeToRbs[(int)(hj->child->rbProps.id)].id, odeToRbs[(int)(hj->parent->rbProps.id)].id);
	}

	P3D p = hj->child->state.getWorldCoordinates(hj->cJPos);
	dJointSetHingeAnchor(j, p[0], p[1], p[2]);
	V3D a = hj->parent->state.getWorldCoordinates(hj->rotationAxis);
	dJointSetHingeAxis(j, a[0], a[1], a[2]);

	//now set the joint limits
	if (hj->jointLimitsActive == false)
		return;

	dJointSetHingeParam(j, dParamLoStop, hj->minAngle);
	dJointSetHingeParam(j, dParamHiStop, hj->maxAngle);
}

/**
This method is used to set up an ode universal joint, based on the information in the universal joint passed in as a parameter
*/
#ifdef DEPRECATED
void ODERBEngine::setupODEUniversalJoint(UniversalJoint* uj) {
	dJointID j;
	uint k = 0;
	for (; k < odeToJoints.size(); k++){
		if (odeToJoints[k].j == uj){
			j = odeToJoints[k].id;
			break;
		}
	}

	if (k == odeToJoints.size()){
		j = dJointCreateUniversal(worldID, 0);
		odeToJoints.push_back(ODE_Joint_Map_struct(j, uj));
		dJointAttach(j, odeToRbs[(int)(uj->child->id)].id, odeToRbs[(int)(uj->parent->id)].id);
	}

	P3D p = uj->child->getWorldCoordinates(uj->cJPos);
	dJointSetUniversalAnchor(j, p[0], p[1], p[2]);

	V3D a = uj->parent->getWorldCoordinates(uj->rotAxisParent);
	V3D b = uj->child->getWorldCoordinates(uj->rotAxisChild);

	dJointSetUniversalAxis1(j, b[0], b[1], b[2]);
	dJointSetUniversalAxis2(j, a[0], a[1], a[2]);

	//now set the joint limits
	if (uj->shouldUseJointLimits() == false)
		return;

	dJointSetUniversalParam(j, dParamLoStop2, uj->minAnglePRA);
	dJointSetUniversalParam(j, dParamHiStop2, uj->maxAnglePRA);
	dJointSetUniversalParam(j, dParamLoStop, uj->minAngleCRA);
	dJointSetUniversalParam(j, dParamHiStop, uj->maxAngleCRA);
}
#endif 

/**
This method is used to set up an ode ball-and-socket joint, based on the information in the ball in socket joint passed in as a parameter
*/
#ifdef DEPRECATED
void ODERBEngine::setupODEBallAndSocketJoint(BallAndSocketJoint* basj) {
	dJointID j;
	uint k = 0;
	for (; k < odeToJoints.size(); k++) {
		if (odeToJoints[k].j == basj) {
			j = odeToJoints[k].id;
			break;
		}
	}
	if (k == odeToJoints.size()) {
		j = dJointCreateBall(worldID, 0);
		odeToJoints.push_back(ODE_Joint_Map_struct(j, basj));
		dJointAttach(j, odeToRbs[(int)(basj->child->id)].id, odeToRbs[(int)(basj->parent->id)].id);
	}

	P3D p = basj->child->getWorldCoordinates(basj->cJPos);
	//now we'll set the world position of the ball-and-socket joint. It is important that the bodies are placed in the world
	//properly at this point
	dJointSetBallAnchor(j, p[0], p[1], p[2]);

	//now deal with the joint limits
	if (basj->shouldUseJointLimits() == true) {
		Logger::consolePrint("Joint limits for ball and socket joints are not yet implemented!\n");
	}
}
#endif

/**
	this method is used to transfer the state of the rigid bodies, from ODE to the rigid body wrapper
*/
void ODERBEngine::setRBStateFromEngine(DynamicArray<RigidBody*> &rbs) {
	//now update all the rigid bodies...
	for (uint i = 0;i<rbs.size();i++) {
		setRBStateFromODE(i);
	}
}

/**
	this method is used to transfer the state of the rigid bodies, from the rigid body wrapper to ODE's rigid bodies
*/
void ODERBEngine::setEngineStateFromRB(DynamicArray<RigidBody*> &rbs) {
	//now update all the rigid bodies...
	for (uint i = 0;i<rbs.size();i++) {
		setODEStateFromRB(i);
	}
}

void ODERBEngine::addRigidBodyToEngine(RigidBody* rb) {
	RBEngine::addRigidBodyToEngine(rb);

	//create and link rigid body to ODE corresponding body
	dBodyID newBody = dBodyCreate(worldID);
	odeToRbs.push_back(ODE_RB_Map(newBody, rb));
	//the ID of this rigid body will be its index in the
	rb->rbProps.id = (int)rbs.size() - 1;
	//we will use the user data of the object to store the index in this mapping as well, for easy retrieval
	dBodySetData(odeToRbs[rb->rbProps.id].id, (void*)rb->rbProps.id);
	//PROCESS THE COLLISION PRIMITIVES OF THE BODY
	createODECollisionPrimitives(rb);

	//if the body is fixed, we'll create constraints to keep it in place...
	if (rb->rbProps.fixed) {
		dBodySetKinematic(newBody);
	}
	else {
		//SET THE INERTIAL PARAMETERS
		dMass m;

		//set the mass and principal moments of inertia for this object
		m.setZero();

		m.setParameters(rb->rbProps.mass, 0, 0, 0,
			rb->rbProps.MOI_local(0, 0),
			rb->rbProps.MOI_local(1, 1),
			rb->rbProps.MOI_local(2, 2),
			rb->rbProps.MOI_local(0, 1),
			rb->rbProps.MOI_local(0, 2),
			rb->rbProps.MOI_local(1, 2));

		dBodySetMass(odeToRbs[rb->rbProps.id].id, &m);
	}

	setODEStateFromRB(rb->rbProps.id);
}

void ODERBEngine::addJointToEngine(Joint* j) {
	RBEngine::addJointToEngine(j);

	//update the ODE state for the parent and child rigid bodies, since the joint axis/position will be set up in world coordinates
	setODEStateFromRB(j->parent->rbProps.id);
	setODEStateFromRB(j->child->rbProps.id);

	//connect the joint to the two bodies
#ifdef DEPRECATED	
	if (BallAndSocketJoint* bJoint = dynamic_cast<BallAndSocketJoint*>(j)) {
		setupODEBallAndSocketJoint(bJoint);
	}
	else if (HingeJoint* hJoint = dynamic_cast<HingeJoint*>(j)) {
		setupODEHingeJoint(hJoint);
	}
	else if (UniversalJoint* uJoint = dynamic_cast<UniversalJoint*>(j)) {
		setupODEUniversalJoint(uJoint);
	}
	else
		throwError("Ooops.... Only BallAndSocket, Hinge and Universal joints are currently supported.\n");
#endif
	//at the moment we only support hinge joint
	setupODEHingeJoint(j);
}

/**
	Update ODERBEngine using current rigidbodies.
*/
void ODERBEngine::updateODERBEngineFromRBs(){
	//now we'll make sure that the joint constraints are satisfied
	for (uint i = 0;i<rbs.size();i++) {

		dBodyID odeBody = NULL;
		for (uint k = 0; k < odeToRbs.size(); k++){
			if (odeToRbs[k].rb == rbs[i]){
				odeBody = odeToRbs[k].id;
				break;
			}
		}
		if (odeBody == NULL)
			throwError("RigidBody not gound...\n");

		//PROCESS THE COLLISION PRIMITIVES OF THE BODY
		updateODECollisionPrimitives(rbs[i]);

		//reset the inertial parameters, in case they've changed...
		if (rbs[i]->rbProps.fixed == false) {
			dMass m;

			//set the mass and principal moments of inertia for this object
			m.setZero();
			m.setParameters(odeToRbs[i].rb->rbProps.mass, 0, 0, 0,
				odeToRbs[i].rb->rbProps.MOI_local(0, 0),
				odeToRbs[i].rb->rbProps.MOI_local(1, 1),
				odeToRbs[i].rb->rbProps.MOI_local(2, 2),
				odeToRbs[i].rb->rbProps.MOI_local(0, 1),
				odeToRbs[i].rb->rbProps.MOI_local(0, 2),
				odeToRbs[i].rb->rbProps.MOI_local(1, 2));

			dBodySetMass(odeToRbs[i].id, &m);
			setODEStateFromRB(i);
		}
	}

	//now we will go through all the joints, and update them (setup...() does update them if they exist already)
	for (uint i = 0;i<joints.size();i++) {
#ifdef DEPRECATED
		//connect the joint to the two bodies
		if (BallAndSocketJoint* bJoint = dynamic_cast<BallAndSocketJoint*>(joints[i])) {
			setupODEBallAndSocketJoint(bJoint);
		}
		else if (HingeJoint* hJoint = dynamic_cast<HingeJoint*>(joints[i])) {
			setupODEHingeJoint(hJoint);
		}
		else if (UniversalJoint* uJoint = dynamic_cast<UniversalJoint*>(joints[i])) {
			setupODEUniversalJoint(uJoint);
		}
		else
			throwError("Ooops.... Only BallAndSocket, Hinge and Universal joints are currently supported.\n");
#endif
		HingeJoint* hJoint = joints[i];
		setupODEHingeJoint(hJoint);	
	}
}

/**
This method is used to set the state of all the rigid body in this collection.
*/
#ifdef DEPRECATED
void ODERBEngine::setState(DynamicArray<double>* state, int start) {
	AbstractRBEngine::setState(state, start);
}
#endif

/**
	call back function that passes the message to the world whose rbs are being acted upon.
*/
void collisionCallBack(void* rbEngine, dGeomID o1, dGeomID o2) {
	ODERBEngine *pWorld = (ODERBEngine*)rbEngine;
	pWorld->processCollisions(o1, o2, pWorld->contactForces);
}

void ODERBEngine::setGlobalCFM(double CFM){
	dWorldSetCFM(worldID, CFM);
}

void ODERBEngine::setMotorsCFMAndFMax(double CFM, double FMAX){
	for (uint j = 0; j < joints.size(); j++){
		Joint *hj = joints[j];

		dJointID aMotor;
		bool found = false;
		// Retrieve motor from map
		for (uint m = 0; m < motorToJointmap.size(); m++){
			if (motorToJointmap[m].joint == hj){
				aMotor = motorToJointmap[m].motorID;
				found = true;
			}
		}
		if (!found)
			continue;

		dJointSetAMotorParam(aMotor, dParamCFM, CFM);
		dJointSetAMotorParam(aMotor, dParamFMax, FMAX);
	}
}

/**
This method is used to integrate the forward simulation in time.
*/
void ODERBEngine::step(double deltaT) {
	//make sure that the state of the RB's is synchronized with the engine...
	setEngineStateFromRB(this->rbs);

	//restart the counter for the joint feedback terms
	jointFeedbackCount = 0;

	//apply control inputs as needed...
	for (uint j = 0;j<joints.size();j++) {
		if (joints[j]->controlMode == RBJointControlMode::POSITION_MODE || joints[j]->controlMode == RBJointControlMode::VELOCITY_MODE) {
			int motorID = getODEMotorForJoint(joints[j]);
			if (motorID >= 0) {
				//the motor may have been placed in limbo, so reattach it to its rigid bodies...
				dJointAttach(motorToJointmap[motorID].motorID, odeToRbs[(int)(joints[j]->parent->rbProps.id)].id, odeToRbs[(int)(joints[j]->child->rbProps.id)].id);

				if (joints[j]->controlMode == RBJointControlMode::POSITION_MODE) {
					V3D rotAxis = joints[j]->parent->state.getWorldCoordinates(joints[j]->rotationAxis);
					rotAxis.normalize(); if (rotAxis.norm() < 0.9) rotAxis = V3D(1,0,0);

					double currentAngle = joints[j]->getCurrentJointAngle();	
					double desiredAngle = motorToJointmap[motorID].joint->desiredControlSignal;
					double rotAngle = desiredAngle - currentAngle;
					
					// ERP = h*kp / (h*kp + kd)
					double erp = deltaT * joints[j]->motorKp / (deltaT * joints[j]->motorKp + joints[j]->motorKd);
					
					if(rotAngle >= 2*PI) rotAngle -= 2*PI;
					if(rotAngle <= -2*PI) rotAngle += 2*PI;

					double desSpeed = - rotAngle / deltaT * erp;
					boundToRange(&desSpeed, -joints[j]->maxSpeed, joints[j]->maxSpeed);		
//					Logger::consolePrint("joint: %d - desired speed: %lf (%lf %lf %lf %lf)\n", j, desSpeed, qErr.s, qErr.v[0], qErr.v[1], qErr.v[2]);
//					Logger::consolePrint("joint: %d\t%lf\n", j, desSpeed);
					
					// CFM = 1 / (h*kp + kd)
					dJointSetAMotorParam(motorToJointmap[motorID].motorID, dParamCFM, 1.0 / (deltaT * joints[j]->motorKp + joints[j]->motorKd));
					dJointSetAMotorAxis(motorToJointmap[motorID].motorID, 0, 1, rotAxis[0], rotAxis[1], rotAxis[2]);
					dJointSetAMotorParam(motorToJointmap[motorID].motorID, dParamVel, desSpeed);
				} else {
					//the desired relative angular velocity needs to be expressed in world coordinates...
					V3D rotAxis = joints[j]->parent->state.getWorldCoordinates(joints[j]->rotationAxis);
					rotAxis.normalize(); if (rotAxis.norm() < 0.9) rotAxis = V3D(1,0,0);
					double angVelocity = joints[j]->desiredControlSignal;
					dJointSetAMotorNumAxes(motorToJointmap[motorID].motorID, 1);
					dJointSetAMotorAxis(motorToJointmap[motorID].motorID, 0, 1, rotAxis[0], rotAxis[1], rotAxis[2]);
					dJointSetAMotorParam(motorToJointmap[motorID].motorID, dParamVel, -angVelocity);
				}
				//setup the feedback structure so that we can read off the torques that were applied
				if (j < MAX_AMOTOR_FEEDBACK)
					dJointSetFeedback(motorToJointmap[motorID].motorID, &(amotorFeedback[j]));
			}
			else
				Logger::consolePrint("Warning: control mode for joint %d requires a motor, but none has been created...\n");
		}

		if (joints[j]->controlMode == RBJointControlMode::PASSIVE_MODE) {
			int motorID = getODEMotorForJoint(joints[j]);
			//we need to place the motor in limbo, such that it does not interfere with torque controller...
			if (motorID >= 0)
				dJointAttach(motorToJointmap[motorID].motorID, 0, 0);
		}

		if (joints[j]->controlMode == RBJointControlMode::FORCE_MODE) {
			int motorID = getODEMotorForJoint(joints[j]);
			//we need to place the motor in limbo, such that it does not interfere with torque controller...
			if (motorID >= 0)
				dJointAttach(motorToJointmap[motorID].motorID, 0, 0);

			V3D rotAxis = joints[j]->parent->state.getWorldCoordinates(joints[j]->rotationAxis.normalized());
			V3D t = rotAxis * joints[j]->desiredControlSignal;
			//we will apply to the parent a positive torque, and to the child a negative torque
			dBodyAddTorque(odeToRbs[joints[j]->parent->rbProps.id].id, -t[0], -t[1], -t[2]);
			dBodyAddTorque(odeToRbs[joints[j]->child->rbProps.id].id, t[0], t[1], t[2]);
		}
	}

	//get an up-to-date list of contact forces...
	contactForces.clear();
	dJointGroupEmpty(contactGroupID);
	//initiate the collision detection
	dSpaceCollide(spaceID, this, &collisionCallBack);

	//advance the simulation
	if (iterativeSolution == false)
		dWorldStep(worldID, deltaT);
	else
		dWorldQuickStep(worldID, deltaT);

	//copy over the state of the ODE bodies to the rigid bodies...
	setRBStateFromEngine(this->rbs);

	//copy over the torque information...
	for (uint j = 0;j<joints.size();j++) {
		if (joints[j]->controlMode != RBJointControlMode::POSITION_MODE && joints[j]->controlMode != RBJointControlMode::VELOCITY_MODE)
			continue;
		int motorID = getODEMotorForJoint(joints[j]);

		if (motorID >= 0) {
			// If this is a hinge joint with a motor attached, then we've used the motors, so see what torque they ended up applying...
			V3D rotAxis = joints[j]->parent->state.getWorldCoordinates(joints[j]->rotationAxis.normalized());
			if (j < MAX_AMOTOR_FEEDBACK)
				joints[j]->motorFeedback = V3D(amotorFeedback[j].t1[0], amotorFeedback[j].t1[1], amotorFeedback[j].t1[2]).dot(rotAxis);
		}
	}

	//copy over the force information for the contact forces
	for (int i = 0;i<jointFeedbackCount;i++) {
		contactForces[i].f = V3D(jointFeedback[i].f1[0], jointFeedback[i].f1[1], jointFeedback[i].f1[2]);
		//make sure that the force always points away from the static rbs
		if (contactForces[i].rb1->rbProps.fixed && !contactForces[i].rb2->rbProps.fixed) {
			contactForces[i].n = contactForces[i].n * (-1);
			contactForces[i].f = contactForces[i].f * (-1);
			RigidBody* tmpBdy = contactForces[i].rb1;
			contactForces[i].rb1 = contactForces[i].rb2;
			contactForces[i].rb2 = tmpBdy;
		}
	}
#ifdef DEPRECATED
	markRBContacts(0.1);
#endif
}


/**
this method applies a force to a rigid body, at the specified point. The point is specified in local coordinates,
and the force is specified in world coordinates.
*/
void ODERBEngine::applyForceTo(RigidBody* b, const V3D& f, const P3D& p) {
	if (!b)
		return;
	dBodyAddForceAtRelPos(odeToRbs[b->rbProps.id].id, f.x(), f.y(), f.z(), p.x, p.y, p.z);
}


/**
this method applies a torque to a rigid body. The torque is specified in world coordinates.
*/
void ODERBEngine::applyTorqueTo(RigidBody* b, const V3D& t) {
	if (!b)
		return;
	dBodyAddTorque(odeToRbs[b->rbProps.id].id, t.x(), t.y(), t.z());
}

/**
this method applies a torque to a rigid body. The torque is specified in relative body coordinates.
*/
void ODERBEngine::applyRelativeTorqueTo(RigidBody* b, const V3D& t) {
	if (!b)
		return;
	dBodyAddRelTorque(odeToRbs[b->rbProps.id].id, t.x(), t.y(), t.z());
}


/*
this method finds the total contact force applied on the given rigid body if it is in contact with another body.
(e.g, finding ground reaction forces applied on the foot of the character)
*/
DynamicArray<ContactForce> ODERBEngine::getContactForceOnRB(RigidBody* bs) {
	
	DynamicArray<ContactForce> grfs;

	for (std::vector<ContactForce>::iterator it = contactForces.begin(); it != contactForces.end(); ++it) 
		if (((*it).rb1->rbProps.id == bs->rbProps.id) || ((*it).rb2->rbProps.id == bs->rbProps.id)) {
			grfs.push_back(*it);
		}

	return grfs;
}

dWorldID ODERBEngine::getWorldID() {
	return worldID;
};

dSpaceID ODERBEngine::getSpaceID() {
	return spaceID;
};

#ifdef CRL_USE_GUI
void ODERBEngine::drawODERB(const Shader &shader) {
	for (uint i = 0; i < this->odeToRbs.size(); i++)
	{
		if(this->odeToRbs[i].rb->rbProps.fixed) continue;

		const dReal *tempData;
		tempData = dBodyGetPosition(odeToRbs[i].id);
		P3D pos(tempData[0], tempData[1], tempData[2]);
		tempData = dBodyGetQuaternion(odeToRbs[i].id);
		Quaternion rbQ(tempData[0], tempData[1], tempData[2], tempData[3]);

		dMass dmass; 
		dBodyGetMass(odeToRbs[i].id, &dmass);

		double mass = dmass.mass;
		
		Matrix3x3 inertia;
		inertia << 
			dmass.I[0], dmass.I[1], dmass.I[2],
			dmass.I[4], dmass.I[5], dmass.I[6], 
			dmass.I[8], dmass.I[9], dmass.I[10];

		Eigen::EigenSolver<Matrix3x3> eigenvalueSolver(inertia);

		Eigen::Vector3cd principleMomentsOfInertia = eigenvalueSolver.eigenvalues();

		assert(IS_ZERO(principleMomentsOfInertia[0].imag()) && IS_ZERO(principleMomentsOfInertia[1].imag()) && IS_ZERO(principleMomentsOfInertia[1].imag()));

		Eigen::Matrix3cd V = eigenvalueSolver.eigenvectors();

		double Ixx = principleMomentsOfInertia[0].real(); // = m(y2 + z2)/12
		double Iyy = principleMomentsOfInertia[1].real(); // = m(z2 + x2)/12
		double Izz = principleMomentsOfInertia[2].real(); // = m(y2 + x2)/12

		double x = sqrt((Iyy + Izz - Ixx) * 6 / mass);
		double y = sqrt((Izz + Ixx - Iyy) * 6 / mass);
		double z = sqrt((Ixx + Iyy - Izz) * 6 / mass);

		P3D pmin(-x / 2, -y / 2, -z / 2), pmax(x / 2, y / 2, z / 2);

		if (V.determinant().real() < 0.0)
		{
			V(0, 2) *= -1;
			V(1, 2) *= -1;
			V(2, 2) *= -1;
		}
		assert(IS_ZERO(abs(V.determinant().real() - 1.0)) && "Rotation matrices have a determinant which is equal to 1.0!");

		Quaternion q(V.real());
		drawCuboid(pos, rbQ * q, V3D(x, y, z), shader, V3D(1.0, 1.0, 0.0));
	}
}

void ODERBEngine::drawODEJoint(const Shader &shader) {
	// Hinge joints
    for(uint i = 0; i < joints.size(); i++){
		dVector3 tempData;
		dJointGetHingeAnchor(odeToJoints[i].id, tempData);
		P3D p(tempData[0], tempData[1], tempData[2]);
		dJointGetHingeAxis(odeToJoints[i].id, tempData);
		V3D v(tempData[0], tempData[1], tempData[2]);
		drawArrow3d(p, v*0.1, 0.01, shader, V3D(0.0, 0.0, 1.0));
    }
}

void ODERBEngine::drawODEContact(const Shader &shader) {
	for (auto c : contactForces) {
		drawArrow3d(c.cp, c.f * 0.1, 0.01, shader, V3D(1.0, 0.0, 0.0));
	}
}
#endif

} // namespace sim
} // namespace crl
