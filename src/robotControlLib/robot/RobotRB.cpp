#include <robot/RobotRB.h>
#include <robot/RBJoint.h>

#include <robot/RBUtils.h>
#include <utils/utils.h>

Matrix3x3 RobotRB::getMOI(const Matrix3x3& RToWorld) {
	return RToWorld * rbProps.MOI_local * RToWorld.transpose();
}


Matrix3x3 RobotRB::getWorldMOI() {
	Matrix3x3 R = state.orientation.toRotationMatrix();
	return R * rbProps.MOI_local * R.transpose();
}

#define UPDATE_RAY_INTERSECTION(P1, P2)														\
if (ray.getDistanceToSegment(P1, P2, &tmpIntersectionPoint) < cylRadius) {					\
	double t = ray.getRayParameterFor(tmpIntersectionPoint);								\
	if (t < tMin) {																			\
		intersectionPoint = ray.origin + ray.dir * t;										\
		tMin = t;																			\
	}																						\
}

bool RobotRB::getRayIntersectionPoint(const Ray& ray, P3D& intersectionPoint, bool checkMeshes, bool checkSkeleton) {
	P3D tmpIntersectionPoint;
	double tMin = DBL_MAX;
	double t = tMin;
	//we will check all meshes and all cylinders that are used to show the abstract view...

	//meshes first...
#ifdef CRL_USE_GUI	
	if (checkMeshes)
		for (uint i = 0; i < rbProps.meshes.size(); i++) {
			RigidTransformation meshTransform(state.orientation, state.pos);
			meshTransform *= rbProps.meshes[i].transform;

			rbProps.meshes[i].model->position = meshTransform.T;
			rbProps.meshes[i].model->orientation = meshTransform.R;

			if (rbProps.meshes[i].model->hitByRay(ray.origin, ray.dir, t)) {
				if (t < tMin) {
					intersectionPoint = ray.origin + ray.dir * t;
					tMin = t;
				}
			}
		}
#endif

	if (checkSkeleton){
		//and now the cylinders...
		double cylRadius = rbProps.abstractViewCylRadius;

		if (pJoint != NULL)
			UPDATE_RAY_INTERSECTION(pJoint->getWorldPosition(), state.pos);

		for (uint i = 0; i < cJoints.size(); i++)
			UPDATE_RAY_INTERSECTION(state.pos, cJoints[i]->getWorldPosition());

	    if (cJoints.size() == 0) {
			if (rbProps.endEffectorPoints.size() > 0) {
				P3D startPos = state.getWorldCoordinates(P3D(0, 0, 0));
				P3D endPos = state.getWorldCoordinates(rbProps.endEffectorPoints[0]);
				UPDATE_RAY_INTERSECTION(startPos, endPos);
			}
		}


	}

	return tMin < DBL_MAX / 2.0;
}
