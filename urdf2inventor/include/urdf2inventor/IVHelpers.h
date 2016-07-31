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
                      float r, float g, float b, float a = 0);

extern void addBox(SoSeparator * addToNode, const EigenTransform& trans,
                   float width, float height, float depth,
                   float r, float g, float b, float a = 0);

/**
 * Add a cylinder oriented around z axis, pointed along +z, originating at \e pos.
 * The rotation \e rot is going to be post-multiplied on the \e pos translation.
 * This function is deprecated, use other addCylinder (with EigenTransform parameter) instead.
 */
extern void addCylinder(SoSeparator * addToNode, const Eigen::Vector3d& pos,
                        const Eigen::Quaterniond& rot,
                        float radius, float height,
                        float r, float g, float b, float a = 0);

/**
 * Add a cylinder oriented around z axis, pointed along +z to \e addToNode, and transform
 * the cylinder given \e trans from \e addToNode.
 */
extern void addCylinder(SoSeparator * addToNode, 
                        const urdf2inventor::EigenTransform& trans,
                        float radius, float height,
                        float r, float g, float b, float a = 0);

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
