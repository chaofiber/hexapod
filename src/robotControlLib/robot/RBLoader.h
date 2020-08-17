#pragma once

#include "robot/Robot.h"
#include "robot/RBUtils.h"

#include <utils/utils.h>
#include <utils/logger.h>

#ifdef CRL_USE_URDFDOM
#include <urdf_parser/urdf_parser.h>
#endif

#ifdef CRL_USE_ASSIMP
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#endif

#if defined CRL_USE_URDFDOM && defined CRL_USE_ASSIMP
// tinyxml is needed for checking up-axis of .dae file
#include <tinyxml.h>
#endif

class Robot;

namespace crl
{
namespace robot
{

class RBMaterial 
{
public:
	std::string name;
	V3D color = V3D(0.9, 0.9, 0.9);
};

/**
    This class is for loading RobotRB, RBJoint and their properties from a various file formats.
	RBLoader object is constructed when the new robot is created from file and destructed when the loading is finished
 */
class RBLoader
{
public:
	// The following lists are only temporal.
	// They contains loaded rbs and joints and once the loading is finished, those rbs and joints are pop from list and moved to rbEngine.
	// Note that some of rbs may not be moved to rbEngine as a result of rb merging. These rb is deleted when RBLoader is destructed

	// List of links loaded from file.
	std::vector<RobotRB *> rbs;
	// List of joints loaded from file.
	std::vector<RBJoint *> joints;

private:
	std::vector<RBMaterial> materials;

#ifdef CRL_USE_ASSIMP
	Assimp::Importer importer;
#endif

public:
	RBLoader(const char* filePath);
	~RBLoader();

	void populateRobot(Robot *robot);	

	void populateRBEngine(RBEngine *rbEngine);
	
private:
	/**
	 	This method merge every chidren of rb fixed to rb by fixed joints in recursive fashion.
	*/
	void mergeFixedChildren(RobotRB *rb);

	/**
		This method reads a list of rigid bodies from the specified file
	*/
	void loadRBsFromFile(const char *fName);

	/**
		This method returns the reference to the rigid body with the given name, or nullptr if it is not found
	*/
	RobotRB *getRBByName(const char *name);

	/**
		This method returns the reference to the joint whose name matches, or nullptr if it is not found
	*/
	RBJoint *getJointByName(char *name);

	/**
		This method loads all the pertinent information regarding the rigid body from a RBS file.
	*/
	void loadFromFile(RobotRB *rb, FILE *fp);

	/**
		This method is used to load the details of a joint from file.
	*/
	void loadFromFile(RBJoint *j, FILE *fp);

	/**
		Processes a line of input, if it is specific to this type of joint. Returns true if processed, false otherwise.
	*/
	bool processInputLine(RBJoint *j, char *line);


#ifdef CRL_USE_URDFDOM
	/**
	   This method reads a list of rigid bodies and joints from LinkConstSharedPtr and JointConstSharedPtr in recursive manner
	*/
	void loadRBsFromURDFLink(urdf::LinkConstSharedPtr urdfLinkPtr, const char *urdfDirectoryPath, const Quaternion &linkOrientation);

	/**
		This method load all the pertinent information from URDF LinkConstSharedPtr.  
		Note that we use rather RBS coordinate system than URDF.
		every link frame shares same orientation aligned with world frame and its origin is on COM.
	*/
	void loadFromURDFLink(RobotRB *rb, urdf::LinkConstSharedPtr urdfLinkPtr, const char *urdfPath, const Quaternion &linkOrientation);

	/**
        This method load all visuals (meshes) that are belong to URDF Link. 
    */
	void loadVisualFromURDFLink(RobotRB *rb, const std::string &path, const std::string &description, const V3D &scale, const Quaternion &R, const P3D &T);

	/**
		This method load all the pertinent information from URDF JointConstSharedPtr 
	*/
	void loadFromURDFJoint(RBJoint *j, urdf::JointConstSharedPtr urdfJoint, urdf::LinkConstSharedPtr urdfParentLink, urdf::LinkConstSharedPtr urdfChildLink, const Quaternion &parentOrientation);
#endif

#ifdef CRL_USE_ASSIMP
	/**
	  	Assimp parse mesh files as tree structure. At the moment we only use assimp for URDF.
	*/
	void loadMeshesRecursively(RobotRB *rb, const aiNode *node, const aiScene *scene, const V3D &scale, const Quaternion &R, const P3D &T);
#endif
};

} // namespace robot
} // namespace crl
