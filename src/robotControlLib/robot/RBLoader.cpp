#include "robot/RBLoader.h"
#include "robot/RBEngine.h"

namespace crl {
namespace robot {

RBLoader::RBLoader(const char *filePath)
{   
    loadRBsFromFile(filePath);

    // Set root and merge RBs that are fused together
    mergeFixedChildren(rbs[0]);
}

RBLoader::~RBLoader()
{
    // Delete leftovers.
    for (uint i = 0; i < rbs.size(); i++)
        delete rbs[i];
    rbs.clear();
    for (uint i = 0; i < joints.size(); i++)
        delete joints[i];
    joints.clear();
}

void RBLoader::populateRBEngine(RBEngine *rbEngine)
{
    //add to rbs and joints to rbengine
    while (this->rbs.size()) {
        rbEngine->addRigidBodyToEngine(this->rbs.back());
        this->rbs.pop_back();
    }

    while (this->joints.size()) {
        rbEngine->addJointToEngine(this->joints.back());
        this->joints.pop_back();
    }
}

void RBLoader::populateRobot(Robot *robot)
{
    // Set root of robot
    robot->root = rbs[0];

    // Gather the list of jointList for the robot
    std::vector<RobotRB *> tempRBs;
    tempRBs.push_back(rbs[0]);

    while (tempRBs.size() > 0) {
        auto rb = tempRBs[0];

        // Add and update all the children jointList to the list
        for (uint i = 0; i < rb->cJoints.size(); i++) {
            auto cJoint = rb->cJoints[i];
            if (cJoint->type == RBJoint::REVOLUTE) {
                robot->jointList.push_back(cJoint);

                // Erase cJoint from this->joints
                for (auto it = this->joints.begin(); it != this->joints.end(); it++) {
                    if (*it == cJoint) {
                        this->joints.erase(it);
                        break;
                    }
                }

                tempRBs.push_back(cJoint->child);
            }
            else {
                throwError("Not supported joint type \'%s\' for joint \'%s\'!", cJoint->type, rb->cJoints[i]->name.c_str());
            }
        }

        // Add rb to robot and rbEngine
        robot->rbList.push_back(rb);

        // Erase rb from this->rbs and tempRBs
        tempRBs.erase(tempRBs.begin());
        for (auto it = this->rbs.begin(); it != this->rbs.end(); it++) {
            if (*it == rb) {
                this->rbs.erase(it);
                break;
            }
        }
    }

    //index the jointList properly...
    for (uint i = 0; i < robot->jointList.size(); i++)
        robot->jointList[i]->jIndex = i;

    //fix link states
    //this is necessary for the case that joints are created from random orders
    //this is not the case for rbs but for urdf the joints are created from leaf node
    robot->fixJointConstraints();
}

void RBLoader::mergeFixedChildren(RobotRB *rb)
{
    double mass = rb->rbProps.mass;
    P3D com(0, 0, 0);

    for (auto cJoint : rb->cJoints) {
        auto ch = cJoint->child;
        mergeFixedChildren(ch); // here we can assume ch is merged with its children

        if (cJoint->type == RBJoint::FIXED) {
            mass += ch->rbProps.mass;
            com += (cJoint->pJPos - cJoint->cJPos) * ch->rbProps.mass;
        }
    }

    //update rb->rbProps due to com change
    if (mass != 0) {
        //except for the case where the rb has zero mass
        //this rb would be merged with its parents
        com /= mass;
        rb->rbProps.mass = mass;
        rb->rbProps.offsetMOI(com.x, com.y, com.z);

        if (rb->pJoint)
            rb->pJoint->cJPos -= com;
    }

    //here we merge rb and its children
    for (uint i = 0; i < rb->cJoints.size(); i++) {
        auto cJoint = rb->cJoints[i];
        auto ch = cJoint->child;

        //update cJoint's position due to new com
        cJoint->pJPos -= com;

        if (cJoint->type == RBJoint::FIXED) {
            P3D comFromCh = cJoint->cJPos - cJoint->pJPos;

            //update MOI
            ch->rbProps.offsetMOI(comFromCh.x, comFromCh.y, comFromCh.z);
            rb->rbProps.MOI_local += ch->rbProps.MOI_local;

            //update collision sphere
            for (auto &c : rb->rbProps.collisionShapes) {
                if (auto cs = std::dynamic_pointer_cast<crl::robot::RRBCollisionSphere>(c))
                    cs->localCoordinates -= com;
            }
            for (auto &c : ch->rbProps.collisionShapes) {
                if (auto cs = std::dynamic_pointer_cast<crl::robot::RRBCollisionSphere>(c)) {
                    cs->localCoordinates -= comFromCh;
                    rb->rbProps.collisionShapes.push_back(cs);
                }
            }

            //update mesh transformation
            for (auto &m : rb->rbProps.meshes) {
                m.transform.T -= com;
            }
            for (auto &m : ch->rbProps.meshes) {
                m.transform.T -= comFromCh;
                rb->rbProps.meshes.push_back(m);
            }

            //update end effectors
            for (auto &ee : rb->rbProps.endEffectorPoints) {
                ee -= com;
            }
            for (auto &ee : ch->rbProps.endEffectorPoints) {
                ee -= comFromCh;
                rb->rbProps.endEffectorPoints.push_back(ee);
            }

            for (auto ccJoint : ch->cJoints) {
                //change parent
                ccJoint->parent = rb;
                ccJoint->pJPos += (cJoint->pJPos - cJoint->cJPos);
                rb->cJoints.push_back(ccJoint);
            }

            //erase current fixed joints from rb->cjoints
            rb->cJoints.erase(rb->cJoints.begin() + i--);
        }
    }
}

/**
	This method returns the reference to the rigid body with the given name, or nullptr if it is not found
*/
RobotRB *RBLoader::getRBByName(const char *name)
{
    if (name == nullptr)
        return nullptr;
    for (int i = (int)rbs.size() - 1; i >= 0; i--)
        if (strcmp(name, rbs[i]->name.c_str()) == 0)
            return rbs[i];
    throwError("RBLoader::getRBByName -> rigid body with name %s does not exist", name);
    return nullptr;
}

/**
	This method returns the reference to the joint whose name matches, or nullptr if it is not found
*/
RBJoint *RBLoader::getJointByName(char *name)
{
    if (name == nullptr)
        return nullptr;
    for (int i = (int)joints.size() - 1; i >= 0; i--)
        if (strcmp(name, joints[i]->name.c_str()) == 0)
            return joints[i];
    throwError("RBLoader::getJointByName -> joint with name %s does not exist", name);
    return nullptr;
}

/**
	This method reads a list of rigid bodies from the specified file.
*/
void RBLoader::loadRBsFromFile(const char *fName)
{
    if (fName == nullptr)
        throwError("nullptr file name provided.");

    std::string fn = std::string(fName);
    std::string fmt = fn.substr(fn.find_last_of(".") + 1);

    // Check file extension
    if (fmt == "urdf") {
#ifdef CRL_USE_URDFDOM
        // Read urdf file
        auto urdf = urdf::parseURDFFile(fName);

        // Recursively load RB
        auto urdfRootLink = urdf->getRoot();
        Quaternion rootQuat = Quaternion(-0.5, 0.5, 0.5, 0.5);
        loadRBsFromURDFLink(urdfRootLink, fn.substr(0, fn.find_last_of("/\\")).c_str(), rootQuat);
#else
        throwError("URDF is not supported without URDFDOM. Give cmake -DCRL_BUILD_URDFDOM=ON option while configuration");
#endif
    }

    else if (fmt == "rbs") {
        // Read rbs file
        FILE *f = fopen(fName, "r");
        if (f == nullptr)
            throwError("Could not open file: %s", fName);

        //have a temporary buffer used to read the file line by line...
        char buffer[200];
        RobotRB *newBody = nullptr;
        RBJoint *j = nullptr;

        //this is where it happens.
        while (!feof(f)) {
            //get a line from the file...
            readValidLine(buffer, 200, f);
            char *line = lTrim(buffer);
            if (strlen(line) == 0)
                continue;
            int lineType = getRRBLineType(line);
            switch (lineType) {
            case RB_RB:
                //create a new rigid body and have it load its own info...
                newBody = new RobotRB();
                loadFromFile(newBody, f);
                rbs.push_back(newBody);
                break;
            case RB_JOINT:
                j = new RBJoint();
                loadFromFile(j, f);
                joints.push_back(j);
                break;
            case RB_NOT_IMPORTANT:
                if (strlen(line) != 0 && line[0] != '#')
                    std::cout << "Ignoring input line: " << line << std::endl;
                break;
            default:
                throwError("Incorrect rigid body input file: \'%s\' - unexpected line.", buffer);
            }
        }

        fclose(f);
    }
    else {
        throwError("unsupported robot file.");
    }
}

/**
	This method loads all the pertinent information regarding the rigid body from a file.
*/
void RBLoader::loadFromFile(RobotRB *rb, FILE *fp)
{
    if (fp == nullptr)
        throwError("Invalid file pointer.");

    //have a temporary buffer used to read the file line by line...
    char buffer[200];

    //this is where it happens.
    while (!feof(fp)) {
        //get a line from the file...
        readValidLine(buffer, 200, fp);
        char *line = lTrim(buffer);
        int lineType = getRRBLineType(line);
        switch (lineType) {
        case RB_NAME: {
            rb->name = std::string() + trim(line);
        } break;
        case RB_MESH_NAME: {
            char tmpStr[200];
            sscanf(line, "%s", tmpStr);
            std::string str(tmpStr);
            rb->rbProps.meshes.push_back(RB3DModel());
#ifdef CRL_USE_GUI
            if (str != "None") { //ToDo
                rb->rbProps.meshes.back().model = new Model(CRL_DATA_FOLDER "/" + str);
            }
#endif
        } break;
        case RB_MESH_COLOR: {
            double r, g, b;
            if (sscanf(line, "%lf %lf %lf", &r, &g, &b) != 3)
                throwError("Incorrect rigid body input file - mesh color parameter expects 3 arguments (mesh color %s)\n", line);
            rb->rbProps.meshes.back().color = V3D(r, g, b);
        } break;
        case RB_MATCAP_TEXTURE: {
            //ToDo...
        } break;
        case RB_MASS: {
            double t = 1;
            if (sscanf(line, "%lf", &t) != 1)
                std::cout << "Incorrect rigid body input file - a mass needs to be specified if the 'mass' keyword is used." << std::endl;
            rb->rbProps.mass = t;
        } break;
        case RB_MOI: {
            double t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0, t6 = 0;
            sscanf(line, "%lf %lf %lf %lf %lf %lf", &t1, &t2, &t3, &t4, &t5, &t6);
            if (t1 <= 0 || t2 <= 0 || t3 <= 0)
                std::cout << "Incorrect values for the principal moments of inertia." << std::endl;
            rb->rbProps.setMOI(t1, t2, t3, t4, t5, t6);
        } break;
        case RB_NOT_IMPORTANT: {
            if (strlen(line) != 0 && line[0] != '#') {
                std::cout << "Ignoring input line: " << line << std::endl;
            }
        } break;
        case RB_COLLISION_SPHERE: {
            double t0 = 0.0, t1 = 0.0, t2 = 0.0, t3 = 0.0;
            if (sscanf(line, "%lf %lf %lf %lf", &t0, &t1, &t2, &t3) != 4) //First three entries: local coodinates, forth: radius
                throwError("Incorrect rigid body input file - 4 arguments are required to specify the local coordinates and the radius of a collision sphere\n", line);
            rb->rbProps.collisionShapes.push_back(std::make_shared<RRBCollisionSphere>(P3D(t0, t1, t2), t3));
        } break;
        case RB_COLLISION_PLANE: {
            double t0 = 0.0, t1 = 0.0, t2 = 0.0, t3 = 0.0, t4 = 0.0, t5 = 0.0;
            if (sscanf(line, "%lf %lf %lf %lf %lf %lf", &t0, &t1, &t2, &t3, &t4, &t5) != 6) //Normal vector and position of the origin
                throwError("Incorrect rigid body input file - 6 arguments are required to specify the normal and the origin of a collision plane\n", line);
            rb->rbProps.collisionShapes.push_back(std::make_shared<RRBCollisionPlane>(P3D(t3, t4, t5), V3D(t0, t1, t2)));
        } break;
        case RB_COLLISION_BOX: {
            double t0 = 0.0, t1 = 0.0, t2 = 0.0, t3 = 0.0, t4 = 0.0, t5 = 0.0;
            if (sscanf(line, "%lf %lf %lf %lf %lf %lf", &t0, &t1, &t2, &t3, &t4, &t5) != 6) // local coordinates and dims of the box
                throwError("Incorrect rigid body input file - 6 arguments are required to specify the local coordinates and the dims of a collision box\n", line);
            rb->rbProps.collisionShapes.push_back(std::make_shared<RRBCollisionBox>(P3D(t3, t4, t5), P3D(t0, t1, t2)));
        } break;

        case RB_MESH_DESCRIPTION: {
            char tmpStr[200];
            sscanf(line, "%s", tmpStr);
            std::string str(tmpStr);
            rb->rbProps.meshes.back().description = str;
        } break;
        case RB_MESH_TRANSFORMATION: {
            double qw, qx, qy, qz;
            P3D T = P3D(0, 0, 0);
            sscanf(line, "%lf %lf %lf %lf %lf %lf %lf",
                   &qw, &qx, &qy, &qz, &T.x, &T.y, &T.z);
            rb->rbProps.meshes.back().transform.R = Quaternion(qw, qx, qy, qz);
            rb->rbProps.meshes.back().transform.T = T;
        } break;
        case RB_END_EFFECTOR: {
            P3D EE = P3D(0, 0, 0);
            // Only parse EE position. Ignore others.
            sscanf(line, "%lf %lf %lf %*lf %*d %*lf %*lf %*lf %*d", &EE.x, &EE.y, &EE.z);
            rb->rbProps.endEffectorPoints.push_back(EE);
        } break;
        case RB_FRICTION_COEFF: {
            sscanf(line, "%lf", &rb->rbProps.frictionCoeff);
        }break;
        case RB_REST_COEFF: {
            sscanf(line, "%lf", &rb->rbProps.restitutionCoeff);
        }break;
        case RB_IS_FROZEN:
            rb->rbProps.fixed = true;
            break;
        case RB_END_RB: {
            return; //and... done
        } break;
        default:
            throwError("Incorrect rigid body input file: \'%s\' - unexpected line.", buffer);
        }
    }
    throwError("Incorrect articulated body input file! No /End found");
}

/**
	This method is used to load the details of a joint from file. The PhysicalWorld parameter points to the rbEngine in which the objects
	that need to be linked live in.
*/
void RBLoader::loadFromFile(RBJoint *j, FILE *f)
{
    if (f == nullptr)
        throwError("Invalid file pointer.");
    //have a temporary buffer used to read the file line by line...
    char buffer[200];
    char tempName[100];

    //this is where it happens.
    while (!feof(f)) {
        //get a line from the file...
        if (!fgets(buffer, 200, f))
            throwError("The input file reading failed.");
        if (strlen(buffer) > 195)
            throwError("The input file contains a line that is longer than ~200 characters - not allowed");
        char *line = lTrim(buffer);

        if (processInputLine(j, line))
            continue;

        int lineType = getRRBLineType(line);
        switch (lineType) {
        case RB_NAME:
            j->name = std::string() + trim(line);
            break;
        case RB_PARENT:
            sscanf(line, "%s", tempName);
            if (j->parent != nullptr)
                throwError("This joint already has a parent");
            j->parent = getRBByName(tempName);
            if (j->parent == nullptr)
                throwError("The articulated rigid body \'%s\' cannot be found!", tempName);
            break;
        case RB_CHILD:
            sscanf(line, "%s", tempName);
            if (j->child != nullptr)
                throwError("This joint already has a parent");
            j->child = getRBByName(tempName);
            if (j->child == nullptr)
                throwError("The articulated rigid body \'%s\' cannot be found!", tempName);
            break;
        case RB_CPOS:
            sscanf(line, "%lf %lf %lf", &j->cJPos.x, &j->cJPos.y, &j->cJPos.z);
            break;
        case RB_PPOS:
            sscanf(line, "%lf %lf %lf", &j->pJPos.x, &j->pJPos.y, &j->pJPos.z);
            break;
        case RB_JOINT_END:
            //we now have to link together the child and parent bodies
            if (j->child == nullptr)
                throwError("A joint has been found that does not have a child rigid body");
            if (j->parent == nullptr)
                throwError("A joint has been found that does not have a parent rigid body");
            if (j->child->pJoint != nullptr)
                throwError("A joint has been found that does not have a parent rigid body");

            j->child->pJoint = j;
            j->parent->cJoints.push_back(j);
            j->fixJointConstraints(true, false, false, false);

            return; //and... done
            break;
        case RB_NOT_IMPORTANT:
            if (strlen(line) != 0 && line[0] != '#') {
                Logger::print("Ignoring input line: \'%s\'\n", line);
            }
            break;
        default:
            throwError("Incorrect articulated body input file: \'%s\' - unexpected line.", buffer);
        }
    }
    throwError("Incorrect articulated body input file! No /ArticulatedFigure found");
}

/**
	Processes a line of input, if it is specific to this type of joint. Returns true if processed, false otherwise.
*/
bool RBLoader::processInputLine(RBJoint *j, char *line)
{
    int lineType = getRRBLineType(line);
    switch (lineType) {
    case RB_JOINT_AXIS:
        sscanf(line, "%lf %lf %lf", &j->rotationAxis[0], &j->rotationAxis[1], &j->rotationAxis[2]);
        j->rotationAxis.normalize();
        return true;
        break;
    case RB_JOINT_LIMITS:
        sscanf(line, "%lf %lf", &j->minAngle, &j->maxAngle);
        j->jointLimitsActive = true;
        return true;
        break;
    case RB_DEFAULT_ANGLE:
        sscanf(line, "%lf", &j->defaultJointAngle);
        return true;
        break;
    case RB_JOINT_MOTOR_KP:
        sscanf(line, "%lf", &j->motorKp);
        return true;
        break;
    case RB_JOINT_MOTOR_KD:
        sscanf(line, "%lf", &j->motorKd);
        return true;
        break;
    default:
        return false;
    }
}

#ifdef CRL_USE_URDFDOM
void RBLoader::loadRBsFromURDFLink(urdf::LinkConstSharedPtr urdfLinkPtr, const char *urdfDirectoryPath, const Quaternion &linkOrientation)
{

    auto *newBody = new RobotRB();
    loadFromURDFLink(newBody, urdfLinkPtr, urdfDirectoryPath, linkOrientation);
    rbs.push_back(newBody);

    // Add links recursively
    for (auto urdfChildLink : urdfLinkPtr->child_links) {

        Quaternion chLinkOrientation(
            urdfChildLink->parent_joint->parent_to_joint_origin_transform.rotation.w,
            urdfChildLink->parent_joint->parent_to_joint_origin_transform.rotation.x,
            urdfChildLink->parent_joint->parent_to_joint_origin_transform.rotation.y,
            urdfChildLink->parent_joint->parent_to_joint_origin_transform.rotation.z);
        loadRBsFromURDFLink(urdfChildLink, urdfDirectoryPath, linkOrientation * chLinkOrientation);

        auto *newJoint = new RBJoint();
        loadFromURDFJoint(newJoint, urdfChildLink->parent_joint, urdfLinkPtr, urdfChildLink, linkOrientation);
        joints.push_back(newJoint);
    }
}

void RBLoader::loadFromURDFLink(RobotRB *rb, urdf::LinkConstSharedPtr urdfLinkPtr, const char *urdfPath, const Quaternion &linkOrientation)
{

    // Offset: from URDF link frame to COM frame
    P3D offsetPos(0, 0, 0);
    Quaternion offsetRot = Quaternion::Identity();

    // Name
    rb->name = urdfLinkPtr->name;

    // Inertial node
    if (urdfLinkPtr->inertial) {

        auto inertial = urdfLinkPtr->inertial;

        offsetPos = P3D(
            inertial->origin.position.x,
            inertial->origin.position.y,
            inertial->origin.position.z);

        offsetRot = Quaternion(
            inertial->origin.rotation.w,
            inertial->origin.rotation.x,
            inertial->origin.rotation.y,
            inertial->origin.rotation.z);

        Quaternion RinW = linkOrientation * offsetRot;

        rb->rbProps.mass = inertial->mass;

        rb->rbProps.setMOI(
            inertial->ixx,
            inertial->iyy,
            inertial->izz,
            inertial->ixy,
            inertial->ixz,
            inertial->iyz);

        // Rotate frame to align with world frame
        rb->rbProps.rotateMOI(
            RinW.w(),
            RinW.x(),
            RinW.y(),
            RinW.z());
    }
    else {
        // Empty link: this will be merged with parent or child later
        rb->rbProps.mass = 0;
        rb->rbProps.MOI_local = Matrix3x3::Zero();
    }

    // Visual nodes
    if (urdfLinkPtr->visual_array.size() > 0) {
        for (auto v : urdfLinkPtr->visual_array) {

            V3D scale(1, 1, 1);
            std::string meshPath;

            // Geometry type
            switch (v->geometry->type) {
            case urdf::Geometry::MESH: {
                // Mesh type
                auto m = std::dynamic_pointer_cast<urdf::Mesh>(v->geometry);
                meshPath = std::string(urdfPath) + "/" + m->filename;
                scale = V3D(m->scale.x, m->scale.y, m->scale.z);
                break;
            }
            case urdf::Geometry::SPHERE: {
                // Sphere type
                auto s = std::dynamic_pointer_cast<urdf::Sphere>(v->geometry);
                meshPath = CRL_DATA_FOLDER "/meshes/sphere.obj";
                scale = V3D(s->radius, s->radius, s->radius);
                break;
            }
            case urdf::Geometry::BOX: {
                // Box type
                auto b = std::dynamic_pointer_cast<urdf::Box>(v->geometry);
                meshPath = CRL_DATA_FOLDER "/meshes/cube.obj";
                scale = V3D(b->dim.x, b->dim.y, b->dim.z);
                break;
            }
            case urdf::Geometry::CYLINDER: {
                // Cylinder type
                auto c = std::dynamic_pointer_cast<urdf::Cylinder>(v->geometry);
                meshPath = CRL_DATA_FOLDER "/meshes/cylinder.obj";
                scale = V3D(c->radius, c->radius, c->length);
                break;
            }
            default: {
                throwError("Not supported visual geometry: \'%s\'", v->geometry->type);
            }
            }

            // Transformation
            Quaternion R = linkOrientation * Quaternion(
                                                 v->origin.rotation.w,
                                                 v->origin.rotation.x,
                                                 v->origin.rotation.y,
                                                 v->origin.rotation.z); // again we need to convert into Yup axis
            Vector3d tempP(
                v->origin.position.x - offsetPos.x,
                v->origin.position.y - offsetPos.y,
                v->origin.position.z - offsetPos.z);
            tempP = linkOrientation * tempP;
            P3D T(tempP.x(), tempP.y(), tempP.z());

            loadVisualFromURDFLink(rb, meshPath, urdfLinkPtr->name, scale, R, T);
        }
    }

    // Collision node
    // At the moment we convert collision primitive into vertices
    if (urdfLinkPtr->collision_array.size() > 0) {
        for (auto c : urdfLinkPtr->collision_array) {

            Vector3d origin(
                c->origin.position.x - offsetPos.x,
                c->origin.position.y - offsetPos.y,
                c->origin.position.z - offsetPos.z);
            origin = linkOrientation * origin;

            switch (c->geometry->type) {
            case urdf::Geometry::BOX: {
                // Vertices of the box
                auto b = std::dynamic_pointer_cast<urdf::Box>(c->geometry);

                Vector3d vertice(b->dim.x * 0.5, b->dim.y * 0.5, b->dim.z * 0.5);
                vertice = linkOrientation * Quaternion(c->origin.rotation.w, c->origin.rotation.x, c->origin.rotation.y, c->origin.rotation.z) * vertice;

                rb->rbProps.collisionShapes.push_back(std::make_shared<RRBCollisionSphere>(P3D(
                                                                                               origin.x() + vertice.x(),
                                                                                               origin.y() + vertice.y(),
                                                                                               origin.z() + vertice.z()),
                                                                                           0.01));
                rb->rbProps.collisionShapes.push_back(std::make_shared<RRBCollisionSphere>(P3D(
                                                                                               origin.x() - vertice.x(),
                                                                                               origin.y() + vertice.y(),
                                                                                               origin.z() + vertice.z()),
                                                                                           0.01));
                rb->rbProps.collisionShapes.push_back(std::make_shared<RRBCollisionSphere>(P3D(
                                                                                               origin.x() + vertice.x(),
                                                                                               origin.y() - vertice.y(),
                                                                                               origin.z() + vertice.z()),
                                                                                           0.01));
                rb->rbProps.collisionShapes.push_back(std::make_shared<RRBCollisionSphere>(P3D(
                                                                                               origin.x() - vertice.x(),
                                                                                               origin.y() - vertice.y(),
                                                                                               origin.z() + vertice.z()),
                                                                                           0.01));
                rb->rbProps.collisionShapes.push_back(std::make_shared<RRBCollisionSphere>(P3D(
                                                                                               origin.x() + vertice.x(),
                                                                                               origin.y() + vertice.y(),
                                                                                               origin.z() - vertice.z()),
                                                                                           0.01));
                rb->rbProps.collisionShapes.push_back(std::make_shared<RRBCollisionSphere>(P3D(
                                                                                               origin.x() - vertice.x(),
                                                                                               origin.y() + vertice.y(),
                                                                                               origin.z() - vertice.z()),
                                                                                           0.01));
                rb->rbProps.collisionShapes.push_back(std::make_shared<RRBCollisionSphere>(P3D(
                                                                                               origin.x() + vertice.x(),
                                                                                               origin.y() - vertice.y(),
                                                                                               origin.z() - vertice.z()),
                                                                                           0.01));
                rb->rbProps.collisionShapes.push_back(std::make_shared<RRBCollisionSphere>(P3D(
                                                                                               origin.x() - vertice.x(),
                                                                                               origin.y() - vertice.y(),
                                                                                               origin.z() - vertice.z()),
                                                                                           0.01));
                break;
            }
            case urdf::Geometry::SPHERE: {
                // Sphere on the center
                // TODO maybe better to make hull?
                auto s = std::dynamic_pointer_cast<urdf::Sphere>(c->geometry);
                rb->rbProps.collisionShapes.push_back(std::make_shared<RRBCollisionSphere>(P3D(
                                                                                               origin.x(),
                                                                                               origin.y(),
                                                                                               origin.z()),
                                                                                           s->radius));
                break;
            }
            case urdf::Geometry::CYLINDER: {
                // End points of the cylinder
                auto cyl = std::dynamic_pointer_cast<urdf::Cylinder>(c->geometry);

                Vector3d vertice(0, 0, cyl->length * 0.5);
                vertice = linkOrientation * Quaternion(c->origin.rotation.w, c->origin.rotation.x, c->origin.rotation.y, c->origin.rotation.z) * vertice;

                rb->rbProps.collisionShapes.push_back(std::make_shared<RRBCollisionSphere>(P3D(
                                                                                               origin.x() + vertice.x(),
                                                                                               origin.y() + vertice.y(),
                                                                                               origin.z() + vertice.z()),
                                                                                           0.01));
                rb->rbProps.collisionShapes.push_back(std::make_shared<RRBCollisionSphere>(P3D(
                                                                                               origin.x() - vertice.x(),
                                                                                               origin.y() - vertice.y(),
                                                                                               origin.z() - vertice.z()),
                                                                                           0.01));
                break;
            }
            default: {
                throwError("Not supported collision geometry: \'%s\'", c->geometry->type);
            }
            }
        }
    }
}

void RBLoader::loadVisualFromURDFLink(RobotRB *rb, const std::string &path, const std::string &description, const V3D &scale, const Quaternion &R, const P3D &T)
{
#ifdef CRL_USE_ASSIMP
    const aiScene *scene = importer.ReadFile(path,
                                             aiProcess_Triangulate |
                                                 aiProcess_JoinIdenticalVertices |
                                                 aiProcess_GlobalScale);
    if (!scene) {
        throw std::runtime_error("unable to load mesh: " + std::string(importer.GetErrorString()));
    }

    // Visual node transform
    Quaternion visR = R;

    // Check up-axis for collada (.dae)
    if (path.substr(path.find_last_of(".") + 1) == "dae") {
        TiXmlDocument doc(path.c_str());
        if (doc.LoadFile()) {
            TiXmlElement *root = doc.FirstChildElement("COLLADA");
            if (root) {
                TiXmlElement *asset = root->FirstChildElement("asset");
                if (asset) {
                    TiXmlElement *upAxis = asset->FirstChildElement("up_axis");
                    if (upAxis) {
                        if (strncmp(upAxis->GetText(), "Z_UP", 4) == 0) {
                            visR *= Quaternion(1, 1, 0, 0);
                        }
                        else if (strncmp(upAxis->GetText(), "X_UP", 4) == 0) {
                            visR *= Quaternion(1, 0, 0, -1);
                        }
                    }
                }
            }
        }
    }

    // Load materials
    for (uint i = 0; i < scene->mNumMaterials; i++)
    {
        aiString name;
        if (AI_SUCCESS != scene->mMaterials[i]->Get(AI_MATKEY_NAME, name))
        {
            continue;
        }

        materials.push_back(RBMaterial());
        materials.back().name = name.C_Str();

        aiColor3D color(0.9f, 0.9f, 0.9f);
        if (AI_SUCCESS == scene->mMaterials[i]->Get(AI_MATKEY_COLOR_DIFFUSE, color))
        {
            materials.back().color.x() = color.r;
            materials.back().color.y() = color.g;
            materials.back().color.z() = color.b;
        }
    }

    // Load mesh recursively
    if (scene->mRootNode) {
        loadMeshesRecursively(rb, scene->mRootNode, scene, scale, visR, T);
    }
    else {
        throw std::runtime_error("unable to load mesh: no root node");
    }
#else
#ifdef CRL_USE_GUI
    rb->rbProps.meshes.back().model = new Model(path);
    rb->rbProps.meshes.back().model->scale = scale;
#endif
    rb->rbProps.meshes.back().description = description;
    rb->rbProps.meshes.back().transform.R = R;
    rb->rbProps.meshes.back().transform.T = T;
#endif
}

void RBLoader::loadFromURDFJoint(RBJoint *j, urdf::JointConstSharedPtr urdfJointPtr, urdf::LinkConstSharedPtr urdfParentLinkPtr, urdf::LinkConstSharedPtr urdfChildLinkPtr, const Quaternion &parentOrientation)
{
    auto parentRB = getRBByName(urdfJointPtr->parent_link_name.c_str());
    if (parentRB == nullptr)
        throwError("The articulated rigid body \'%s\' cannot be found!", urdfJointPtr->parent_link_name.c_str());

    auto childRB = getRBByName(urdfJointPtr->child_link_name.c_str());
    if (childRB == nullptr)
        throwError("The articulated rigid body \'%s\' cannot be found!", urdfJointPtr->child_link_name.c_str());

    j->name = urdfJointPtr->name;
    j->parent = parentRB;
    j->child = childRB;

    // Joint type
    switch (urdfJointPtr->type) {
    case urdf::Joint::REVOLUTE: {
        j->type = RBJoint::REVOLUTE;
        break;
    }
    case urdf::Joint::FIXED: {
        j->type = RBJoint::FIXED;
        break;
    }
    default: {
        throwError("Not supported joint type \'%s\' for joint \'%s\'!", urdfJointPtr->type, urdfJointPtr->name.c_str());
        break;
    }
    }

    // Position in parent's coordinate
    if (urdfParentLinkPtr->inertial) {
        Vector3d pJPinW(
            urdfJointPtr->parent_to_joint_origin_transform.position.x - urdfParentLinkPtr->inertial->origin.position.x,
            urdfJointPtr->parent_to_joint_origin_transform.position.y - urdfParentLinkPtr->inertial->origin.position.y,
            urdfJointPtr->parent_to_joint_origin_transform.position.z - urdfParentLinkPtr->inertial->origin.position.z);
        pJPinW = parentOrientation * pJPinW;
        j->pJPos = P3D(
            pJPinW.x(),
            pJPinW.y(),
            pJPinW.z());
    }
    else {
        Vector3d pJPinW(
            urdfJointPtr->parent_to_joint_origin_transform.position.x,
            urdfJointPtr->parent_to_joint_origin_transform.position.y,
            urdfJointPtr->parent_to_joint_origin_transform.position.z);
        pJPinW = parentOrientation * pJPinW;
        j->pJPos = P3D(
            pJPinW.x(),
            pJPinW.y(),
            pJPinW.z());
    }

    // Position in child's coordinate
    Quaternion R(
        urdfJointPtr->parent_to_joint_origin_transform.rotation.w,
        urdfJointPtr->parent_to_joint_origin_transform.rotation.x,
        urdfJointPtr->parent_to_joint_origin_transform.rotation.y,
        urdfJointPtr->parent_to_joint_origin_transform.rotation.z);

    if (urdfChildLinkPtr->inertial) {
        Vector3d cJPinW(
            -urdfChildLinkPtr->inertial->origin.position.x,
            -urdfChildLinkPtr->inertial->origin.position.y,
            -urdfChildLinkPtr->inertial->origin.position.z);
        cJPinW = parentOrientation * R * cJPinW;
        j->cJPos = P3D(
            cJPinW.x(),
            cJPinW.y(),
            cJPinW.z());
    }
    else {
        j->cJPos = P3D(0, 0, 0);
    }

    // Rotation axis
    Vector3d axis(
        urdfJointPtr->axis.x,
        urdfJointPtr->axis.y,
        urdfJointPtr->axis.z);
    axis = parentOrientation * axis;

    j->rotationAxis = V3D(
        axis.x(),
        axis.y(),
        axis.z());

    // Joint limits
    if (urdfJointPtr->limits) {
        j->jointLimitsActive = true;
        j->minAngle = urdfJointPtr->limits->lower;
        j->maxAngle = urdfJointPtr->limits->upper;
    }

    j->child->pJoint = j;
    j->parent->cJoints.push_back(j);
    j->fixJointConstraints(true, false, false, false);
}
#endif

#ifdef CRL_USE_ASSIMP
void RBLoader::loadMeshesRecursively(RobotRB *rb, const aiNode *node, const aiScene *scene, const V3D &scale, const Quaternion &R, const P3D &T)
{
    // Transformation from parent
    aiVector3D aiScale;
    aiQuaternion aiQuat;
    aiVector3D aiPos;

    node->mTransformation.Decompose(aiScale, aiQuat, aiPos);
    Quaternion nodeQuat = R * Quaternion(aiQuat.w, aiQuat.x, aiQuat.y, aiQuat.z);
    P3D nodeT = P3D(
        aiPos.x,
        aiPos.y,
        aiPos.z);
    V3D temp(nodeT.x, nodeT.y, nodeT.z);
    temp = R * temp;
    nodeT = P3D(
        T.x + temp.x(),
        T.y + temp.y(),
        T.z + temp.z());
    V3D nodeScale(scale.x() * aiScale.x, scale.y() * aiScale.y, scale.z() * aiScale.z);

    // Add children meshes
    for (uint i = 0; i < node->mNumMeshes; ++i) {
        auto mesh = scene->mMeshes[node->mMeshes[i]];

        if(mesh == NULL)
            continue;

        // Create 3DModel
        rb->rbProps.meshes.push_back(RB3DModel());
        rb->rbProps.meshes.back().model = new Model();
        rb->rbProps.meshes.back().model->scale = nodeScale;
        rb->rbProps.meshes.back().transform.R = nodeQuat;
        rb->rbProps.meshes.back().transform.T = nodeT;

        // Add mesh to 3DModel
        std::vector<Vertex> vertices;
        std::vector<unsigned int> indices;
        Mesh::TextureMap textures;

        if (mesh->HasPositions()) {

            for (uint j = 0; j < mesh->mNumVertices; j++) {
                Vertex vertex;

                vertex.position[0] = mesh->mVertices[j].x;
                vertex.position[1] = mesh->mVertices[j].y;
                vertex.position[2] = mesh->mVertices[j].z;

                vertex.normal[0] = mesh->mNormals[j].x;
                vertex.normal[1] = mesh->mNormals[j].y;
                vertex.normal[2] = mesh->mNormals[j].z;

                // TODO
                // vertex.texCoords[0] = mesh->mTextureCoords[0].x;
                // vertex.texCoords[1] = mesh->mTextureCoords[0].y;

                vertices.push_back(vertex);
            }
        }

        if (mesh->HasFaces()) {
            for (uint j = 0; j < mesh->mNumFaces; j++) {
                if (mesh->mFaces[j].mNumIndices == 3) {
                    indices.push_back(mesh->mFaces[j].mIndices[0]);
                    indices.push_back(mesh->mFaces[j].mIndices[1]);
                    indices.push_back(mesh->mFaces[j].mIndices[2]);
                }
            }
        }

        rb->rbProps.meshes.back().model->meshes.push_back(Mesh(vertices, indices, textures));

        // Set Material 
        if(mesh->mMaterialIndex && mesh->mMaterialIndex < materials.size()) {
            rb->rbProps.meshes.back().color = materials[mesh->mMaterialIndex].color;
        }
    }

    // Do same thing for childrend
    for (uint i = 0; i < node->mNumChildren; i++) {
        loadMeshesRecursively(rb, node->mChildren[i], scene, nodeScale, nodeQuat, nodeT);
    }
}
#endif

} // namespace robot
} // namespace crl
