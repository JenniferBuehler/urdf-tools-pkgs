/**
    Copyright (C) 2016 Jennifer Buehler

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software Foundation,
    Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
**/

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


/*! Returns axis-aligned bounding box min and max points of the node 
 */
extern void getBoundingBox(SoNode* node, Eigen::Vector3d& minPoint, Eigen::Vector3d& maxPoint);


extern EigenTransform getEigenTransform(const SbMatrix& m);
extern SbMatrix getSbMatrix(const urdf2inventor::EigenTransform& m);
extern std::string printMatrix(const urdf2inventor::EigenTransform& t);

/**
 * Adds the node \e addAsChild as a child to \e parent, transformed by the given transform \e eTrans,
 * which requires the insertion of an in-between transform node. The name of this in-between transform
 * node can be tiven in \e name.
 * \return false if either \e addAsChild or \e parent are SoSepartor nodes and can have children.
 */
extern bool addSubNode(SoNode * addAsChild, SoNode* parent,
                                const urdf2inventor::EigenTransform& eTrans,
                                const char * name = NULL);

/**
 * Adds the node \e addAsChild as a child to \e parent but inserts a transform node in-between,
 * defined by \e trans.
 * \param transName optional: name to give the transform node inserted in-between.
 * \return false if either \e addAsChild or \e parent are SoSepartor nodes and can have children.
 */
extern bool addSubNode(SoNode * addAsChild, SoNode* parent, SoTransform * trans,
                       const char * transName = NULL);

/**
 * Adds the node \e addAsChild as a child to \e parent, transformed by the given \e transform.
 * A transform node will have to be inserted in-between \e parent and \e addAsChild.
 * The \e name can be used to give the actual child added to \e parent a specific name.
 */
extern void addSubNode(SoNode * addAsChild, SoSeparator * parent,
                       const EigenTransform& transform, SoMaterial * mat,
                       const char * name=NULL);

extern void addSphere(SoSeparator * addToNode, const Eigen::Vector3d& pos, float radius,
                      float r, float g, float b, float a = 0);

extern void addBox(SoSeparator * addToNode, const EigenTransform& trans,
                   float width, float height, float depth,
                   float r, float g, float b, float a = 0);

/**
 * Add a cylinder oriented around z axis, pointed along +z, originating at \e pos.
 * The rotation \e rot is going to be post-multiplied on the \e pos translation.
 * This function is deprecated, use other addCylinder (with EigenTransform parameter) instead.
 * \param name name to give the child node of \e addToNode which is inserted (containing
 * the cylinder)
 */
extern void addCylinder(SoSeparator * addToNode, const Eigen::Vector3d& pos,
                        const Eigen::Quaterniond& rot,
                        float radius, float height,
                        float r, float g, float b, float a = 0,
                        const char * name = NULL);

/**
 * Add a cylinder oriented around z axis, pointed along +z to \e addToNode, and transform
 * the cylinder given \e trans from \e addToNode.
 * \param name name to give the child node of \e addToNode which is inserted (containing
 * the cylinder)
 */
extern void addCylinder(SoSeparator * addToNode, 
                        const urdf2inventor::EigenTransform& trans,
                        float radius, float height,
                        float r, float g, float b, float a = 0,
                        const char * name = NULL);

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
