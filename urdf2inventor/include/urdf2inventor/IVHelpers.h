#ifndef URDF2INVENTOR_IVHELPERS_H
#define URDF2INVENTOR_IVHELPERS_H

#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoMaterial.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <set>

namespace urdf2inventor
{

typedef Eigen::Transform<double, 3, Eigen::Affine> EigenTransform;

extern SoTransform * getTransform(const EigenTransform& trans);

/**
 * Adds a SoNode (addAsChild) as a child (to parent), transformed by the given transform (eTrans)
 * and returns the SoSeparator containing parent with the child.
 */
extern SoSeparator * addSubNode(SoNode * addAsChild, SoNode* parent,
    const urdf2inventor::EigenTransform& eTrans);

/**
 * Adds a SoNode (addAsChild) as a child (to parent), transformed by the given transform (eTrans) and
 * returns the SoSeparator containing parent with the child.
 */
extern SoSeparator * addSubNode(SoNode * addAsChild, SoNode* parent, SoTransform * trans);

extern void addSubNode(SoNode * addAsChild, SoSeparator * parent,
    const EigenTransform& transform, SoMaterial * mat);

extern void addSphere(SoSeparator * addToNode, const Eigen::Vector3d& pos, float radius,
    float r, float g, float b, float a=0);

extern void addBox(SoSeparator * addToNode, const EigenTransform& trans,
    float width, float height, float depth,
    float r, float g, float b, float a=0);

/**
 * Add a cylinder oriented around z axis, pointed along +z, originating at \e pos
 */
extern void addCylinder(SoSeparator * addToNode, const Eigen::Vector3d& pos,
    const Eigen::Quaterniond& rot,
    float radius, float height,
    float r, float g, float b, float a=0);

extern void addLocalAxes(SoSeparator * addToNode, float axesRadius, float axesLength);

/**
 * Searches through all nodes and returns a set of absolute paths to textures
 * which are in use.
 */
extern std::set<std::string> getAllTexturePaths(SoNode * root);

/**
 * writes the contents of SoNode into the inventor (*.iv) format and returns the file
 * content as a string.
 */
extern bool writeInventorFileString(SoNode * node, std::string& result);



}
#endif  // URDF2INVENTOR_IVHELPERS_H
