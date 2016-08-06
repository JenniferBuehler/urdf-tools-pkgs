/**
 * <ORGANIZATION> = Jennifer Buehler 
 * <COPYRIGHT HOLDER> = Jennifer Buehler 
 * 
 * Copyright (c) 2016 Jennifer Buehler 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ------------------------------------------------------------------------------
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
extern Eigen::Vector3d getRotationAxis(const JointConstPtr& j);

// Returns if this is an active joint in the URDF description
extern bool isActive(const JointPtr& joint);

extern bool isChildOf(const LinkConstPtr& parent, const LinkConstPtr& child);
extern bool isChildJointOf(const LinkConstPtr& parent, const JointConstPtr& joint);

/**
 * \return true if this joint needs a transformation to align its rotation axis with the given axis.
 * In this case the rotation parameter contains the necessary rotation.
 */
bool jointTransformForAxis(const JointConstPtr& joint, const Eigen::Vector3d& axis, Eigen::Quaterniond& rotation);

/**
 *  Scales up the **translation part** of the transform \e t by the given factor
 */
extern void scaleTranslation(EigenTransform& t, double scale_factor);

/**
 * scales the translation part of the joint transform by the given factor
 */
extern bool scaleTranslation(JointPtr& joint, double scale_factor);

/**
 * scales the translation part of the origins of visuals/collisions/inertial by the given factor
 */
extern void scaleTranslation(LinkPtr& link, double scale_factor);


extern void setTransform(const EigenTransform& t, urdf::Pose& p);
extern void setTransform(const EigenTransform& t, JointPtr& joint);

// Get joint transform to parent
extern EigenTransform getTransform(const urdf::Pose& p);

// Get joint transform to parent
extern EigenTransform getTransform(const JointConstPtr& joint);

// Get transform to parent link (transform of link's parent joint)
extern EigenTransform getTransform(const LinkConstPtr& link);

extern Eigen::Matrix4d getTransformMatrix(const LinkConstPtr& from_link,  const LinkConstPtr& to_link);

extern EigenTransform getTransform(const LinkConstPtr& from_link,  const LinkConstPtr& to_link);


/**
 * Applies the transformation on the joint transform
 * \param scaleTransform set to true if the urdf's transforms are to be scaled (using scaleFactor) before applying the transform
 */
extern bool applyTransform(JointPtr& joint, const EigenTransform& trans, bool preMult);

/**
 * Applies the transformation on the link's visuals, collisions and intertial.
 * \param scaleTransform set to true if the urdf's transforms are to be scaled (using scaleFactor) before applying the transform
 */
extern void applyTransform(LinkPtr& link, const EigenTransform& trans, bool preMult);

extern void applyTransform(const EigenTransform& t, urdf::Vector3& v);

/**
 * Returns all joints between \e from_link and \e to_link which are along the path between the
 * links - this will only work if there is only one path between both links.
 */
extern std::vector<JointPtr> getChain(const LinkConstPtr& from_link, const LinkConstPtr& to_link);



}  // namespace

#endif  // URDF_TRAVERSER_FUNCTIONS_H
