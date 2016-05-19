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

#ifndef URDF_TRAVERSER_FUNCTIONS_H
#define URDF_TRAVERSER_FUNCTIONS_H
// Copyright Jennifer Buehler

/**
 * Helper functions for operations on the urdf elements.
 * \author Jennifer Buehler
 * \date last edited October 2015
 */

#include <iostream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <urdf_traverser/Types.h>

namespace urdf_traverser
{
    // Returns the joint's rotation axis as Eigen Vector
    extern Eigen::Vector3d getRotationAxis(const JointPtr& j); 

    /**
     *  Scales up the **translation part** of the transform \e t by the given factor
     */
    void scaleTranslation(EigenTransform& t, double scale_factor);

    /**
     * scales the translation part of the joint transform by the given factor
     */
    bool scaleTranslation(JointPtr& joint, double scale_factor);

    /**
     * scales the translation part of the origins of visuals/collisions/inertial by the given factor
     */
    void scaleTranslation(LinkPtr& link, double scale_factor);


    void setTransform(const EigenTransform& t, urdf::Pose& p);
    void setTransform(const EigenTransform& t, JointPtr& joint);

    // Get joint transform to parent
    EigenTransform getTransform(const urdf::Pose& p);

    // Get joint transform to parent
    EigenTransform getTransform(const JointConstPtr& joint); 

    // Get transform to parent link (transform of link's parent joint)
    EigenTransform getTransform(const LinkConstPtr& link); 

    Eigen::Matrix4d getTransformMatrix(const LinkConstPtr& from_link,  const LinkConstPtr& to_link);

    EigenTransform getTransform(const LinkConstPtr& from_link,  const LinkConstPtr& to_link); 


    /**
     * Applies the transformation on the joint transform
     * \param scaleTransform set to true if the urdf's transforms are to be scaled (using scaleFactor) before applying the transform
     */
    bool applyTransform(JointPtr& joint, const EigenTransform& trans, bool preMult);

    /**
     * Applies the transformation on the link's visuals, collisions and intertial.
     * \param scaleTransform set to true if the urdf's transforms are to be scaled (using scaleFactor) before applying the transform
     */
    void applyTransform(LinkPtr& link, const EigenTransform& trans, bool preMult);

    void applyTransform(const EigenTransform& t, urdf::Vector3& v);

    /**
     * Returns all joints between \e from_link and \e to_link which are along the path between the
     * links - this will only work if there is only one path between both links.
     */
    std::vector<JointPtr> getChain(const LinkConstPtr& from_link, const LinkConstPtr& to_link);

}  // namespace

#endif  // URDF_TRAVERSER_FUNCTIONS_H
