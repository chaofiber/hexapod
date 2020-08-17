#include <memory>

#include <gui/renderer.h>

#include "robot/RBRenderer.h"

namespace crl {
namespace robot {

/**
	This method draws the local coordinate frame of the rigid body
*/
void RBRenderer::drawCoordFrame(const RobotRB *rb, const Shader &shader)
{
    drawArrow3d(rb->state.pos, V3D(rb->state.getWorldCoordinates(V3D(1, 0, 0)) * 0.1), 0.01, shader, V3D(1.0, 0.0, 0.0));
    drawArrow3d(rb->state.pos, V3D(rb->state.getWorldCoordinates(V3D(0, 1, 0)) * 0.1), 0.01, shader, V3D(0.0, 1.0, 0.0));
    drawArrow3d(rb->state.pos, V3D(rb->state.getWorldCoordinates(V3D(0, 0, 1)) * 0.1), 0.01, shader, V3D(0.0, 0.0, 1.0));
}

/**
	This method draws the current rigid body.
*/
void RBRenderer::drawSkeletonView(const RobotRB *rb, const Shader &shader, bool showJointAxes, bool showJointLimits)
{
    //draw capsules that define the "skeleton" of this body: parent joints TO features TO child joints and end effectors

    V3D drawColor = rb->rbProps.selected ? rb->rbProps.highlightColor : rb->rbProps.color;

    if (rb->pJoint != NULL) {
        P3D startPos = rb->state.getWorldCoordinates(rb->pJoint->cJPos);
        P3D endPos = rb->state.getWorldCoordinates(P3D(0, 0, 0));
        drawCapsule(startPos, endPos, rb->rbProps.abstractViewCylRadius, shader, drawColor);
    }

    for (uint i = 0; i < rb->cJoints.size(); i++) {
        P3D startPos = rb->state.getWorldCoordinates(P3D(0, 0, 0));
        P3D endPos = rb->state.getWorldCoordinates(rb->cJoints[i]->pJPos);
        drawCapsule(startPos, endPos, rb->rbProps.abstractViewCylRadius, shader, drawColor);
    }

    if (rb->cJoints.size() == 0) {
        if (rb->rbProps.endEffectorPoints.size() > 0) {
            P3D startPos = rb->state.getWorldCoordinates(P3D(0, 0, 0));
            P3D endPos = rb->state.getWorldCoordinates(rb->rbProps.endEffectorPoints[0]);
            drawCapsule(startPos, endPos, rb->rbProps.abstractViewCylRadius, shader, drawColor);
        }
    }

    for (uint i = 0; i < rb->cJoints.size(); i++) {
        V3D drawColor = rb->cJoints[i]->selected ? rb->rbProps.highlightColor : rb->rbProps.jointDrawColor;

        P3D globalJointPos = rb->cJoints[i]->getWorldPosition();
        drawSphere(globalJointPos, rb->rbProps.abstractViewCylRadius * 1.3, shader, drawColor);
    }

    //draw joint limits
    if (showJointLimits) {
        for (uint i = 0; i < rb->cJoints.size(); i++) {
            drawJointLimits(rb->cJoints[i], shader);
        }
    }

    //drawing joints happens in world coordinates directly...
    if (showJointAxes) {
        for (uint i = 0; i < rb->cJoints.size(); i++)
            drawAxis(rb->cJoints[i], shader);
    }
}

void RBRenderer::drawMeshes(const RobotRB *rb, const Shader &shader)
{
    for (uint i = 0; i < rb->rbProps.meshes.size(); i++) {
        V3D drawColor = rb->rbProps.selected ? rb->rbProps.highlightColor : rb->rbProps.meshes[i].color;

        RigidTransformation meshTransform(rb->state.orientation, rb->state.pos);
        meshTransform *= rb->rbProps.meshes[i].transform;

        rb->rbProps.meshes[i].model->position = meshTransform.T;
        rb->rbProps.meshes[i].model->orientation = meshTransform.R;
        rb->rbProps.meshes[i].model->color = drawColor;

        rb->rbProps.meshes[i].model->draw(shader);
    }
}

void RBRenderer::drawCollisionSpheres(const RobotRB *rb, const Shader &shader)
{
    for (uint i = 0; i < rb->rbProps.collisionShapes.size(); i++) {
        if (auto cs = std::dynamic_pointer_cast<RRBCollisionSphere>(rb->rbProps.collisionShapes[i])) {
            P3D pos = rb->state.getWorldCoordinates(cs->localCoordinates);
            drawSphere(pos, cs->radius, shader, rb->rbProps.colSphereDrawColor);
        }
    }
}

void RBRenderer::drawMOI(const RobotRB *rb, const Shader &shader)
{
    Eigen::EigenSolver<Matrix3x3> eigenvalueSolver(rb->rbProps.MOI_local);

    Eigen::Vector3cd principleMomentsOfInertia = eigenvalueSolver.eigenvalues();

    assert(IS_ZERO(principleMomentsOfInertia[0].imag()) && IS_ZERO(principleMomentsOfInertia[1].imag()) && IS_ZERO(principleMomentsOfInertia[1].imag()));

    Eigen::Matrix3cd V = eigenvalueSolver.eigenvectors();

    double Ixx = principleMomentsOfInertia[0].real(); // = m(y2 + z2)/12
    double Iyy = principleMomentsOfInertia[1].real(); // = m(z2 + x2)/12
    double Izz = principleMomentsOfInertia[2].real(); // = m(y2 + x2)/12

    double x = sqrt((Iyy + Izz - Ixx) * 6 / rb->rbProps.mass);
    double y = sqrt((Izz + Ixx - Iyy) * 6 / rb->rbProps.mass);
    double z = sqrt((Ixx + Iyy - Izz) * 6 / rb->rbProps.mass);

    P3D pmin(-x / 2, -y / 2, -z / 2), pmax(x / 2, y / 2, z / 2);

    if (V.determinant().real() < 0.0) {
        V(0, 2) *= -1;
        V(1, 2) *= -1;
        V(2, 2) *= -1;
    }
    assert(IS_ZERO(abs(V.determinant().real() - 1.0)) && "Rotation matrices have a determinant which is equal to 1.0!");

    Quaternion q(V.real());

    drawCuboid(rb->state.pos, rb->state.orientation * q, V3D(x, y, z), shader, V3D(0.7, 0.7, 0.7));
}

void RBRenderer::drawEndEffectors(const RobotRB *rb, const Shader &shader)
{
    for (uint i = 0; i < rb->rbProps.endEffectorPoints.size(); i++) {
        P3D pos = rb->state.getWorldCoordinates(rb->rbProps.endEffectorPoints[i]);
        drawSphere(pos, 0.01, shader, rb->rbProps.endEffectorDrawColor);
    }
}

/**
	draws the axes of rotation
*/
void RBRenderer::drawAxis(RBJoint *j, const Shader &shader)
{
    drawArrow3d(j->getWorldPosition(), V3D(j->parent->state.getWorldCoordinates(j->rotationAxis) * 0.1), 0.01, shader, V3D(1.0, 0.0, 0.0));
}

/**
	draw joint limits
*/
void RBRenderer::drawJointLimits(RBJoint *j, const Shader &shader)
{
    if (!j->jointLimitsActive)
        return;

    P3D p = j->getWorldPosition();
    V3D n = j->parent->state.getWorldCoordinates(j->rotationAxis);

    //this is the feature that we will be tracking as the joint rotates through its range of motion... expressed in the coordinate frame of the child, as it is the thing that rotates with the joint...
    V3D vFeature = V3D(j->cJPos, P3D());
    if (j->child->cJoints.size() == 1)
        vFeature = V3D(j->cJPos, j->child->cJoints[0]->pJPos);

    vFeature -= j->rotationAxis.dot(vFeature) * j->rotationAxis;

    if (vFeature.norm() < 0.000001)
        return;

    //now, we need to see what this feature vector would look like when rotated according to the range of motion of the joint...
    V3D from = j->parent->state.getWorldCoordinates(getRotationQuaternion(j->minAngle, j->rotationAxis) * vFeature);
    V3D to = j->parent->state.getWorldCoordinates(getRotationQuaternion(j->maxAngle, j->rotationAxis) * vFeature);
    V3D currentV = j->child->state.getWorldCoordinates(vFeature);

    drawSector(p, from, to, n, shader, V3D(1, 0, 0));
    drawArrow3d(p, currentV, 0.01, shader, V3D(0, 1, 0));
}

} // namespace robot
} // namespace crl