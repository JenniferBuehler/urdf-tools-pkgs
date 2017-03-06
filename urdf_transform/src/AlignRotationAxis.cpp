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
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <urdf_traverser/Functions.h>
#include <urdf_traverser/Helpers.h>
#include <urdf_traverser/UrdfTraverser.h>
#include <urdf_transform/AlignRotationAxis.h>

using urdf_traverser::UrdfTraverser;
using urdf_traverser::RecursionParams;


/**
 * \brief Recursion parameters with one 3D vector
 * \author Jennifer Buehler
 */
class Vector3RecursionParams: public RecursionParams
{
public:
    typedef baselib_binding::shared_ptr<Vector3RecursionParams>::type Ptr;
    Vector3RecursionParams(): RecursionParams() {}
    Vector3RecursionParams(const Eigen::Vector3d& _vec):
        RecursionParams(),
        vec(_vec) {}
    Vector3RecursionParams(const Vector3RecursionParams& o):
        RecursionParams(o),
        vec(o.vec) {}
    virtual ~Vector3RecursionParams() {}

    // Result set
    Eigen::Vector3d vec;
};

/**
 * Recursion method to be used with traverseTreeTopDown() and recursion parameters
 * of type *Vector3RecursionParams*.
 *
 * Re-arranges the joint-transform of the recursion link's *parent joint*, along with
 * the link's visual/collision/intertial rotations, such that all joints rotate around the axis
 * given in the recursion parameters vector.
 */
int allRotationsToAxisCB(urdf_traverser::RecursionParamsPtr& p)
{
    urdf_traverser::LinkPtr link = p->getLink();
    if (!link)
    {
        ROS_ERROR("allRotationsToAxis: NULL link passed");
        return -1;
    }

    Vector3RecursionParams::Ptr param = baselib_binding_ns::dynamic_pointer_cast<Vector3RecursionParams>(p);
    if (!param)
    {
        ROS_ERROR("Wrong recursion parameter type");
        return -1;
    }

    urdf_traverser::JointPtr joint = link->parent_joint;
    if (!joint)
    {
        ROS_INFO_STREAM("allRotationsToAxis: Joint for link " << link->name << " is NULL, so this must be the root joint");
        return 1;
    }

    Eigen::Vector3d axis = param->vec;

    Eigen::Quaterniond alignAxis;
    if (urdf_traverser::jointTransformForAxis(joint, axis, alignAxis))
    {
        Eigen::Vector3d rotAxis(joint->axis.x, joint->axis.y, joint->axis.z);
        // ROS_INFO_STREAM("Transforming axis "<<rotAxis<<" for joint "<<joint->name<<" with transform "<<urdf_traverser::EigenTransform(alignAxis));

        urdf_traverser::applyTransform(joint, urdf_traverser::EigenTransform(alignAxis), false);

        // the link has to receive the inverse transform, so it stays at the original position
        Eigen::Quaterniond alignAxisInv = alignAxis.inverse();
        urdf_traverser::applyTransform(link, urdf_traverser::EigenTransform(alignAxisInv), true);

        // we also have to fix the child joint's (1st order child joints) transform
        // to correct for this transformation.
        for (std::vector<urdf_traverser::JointPtr>::iterator pj = link->child_joints.begin();
                pj != link->child_joints.end(); pj++)
        {
            urdf_traverser::applyTransform(*pj, urdf_traverser::EigenTransform(alignAxisInv), true);
        }

        // finally, set the rotation axis to the target
        joint->axis.x = axis.x();
        joint->axis.y = axis.y();
        joint->axis.z = axis.z();
    }

    // all good, indicate that recursion can continue
    return 1;
}


bool urdf_transform::allRotationsToAxis(UrdfTraverser& traverser, const std::string& fromLink, const Eigen::Vector3d& axis)
{
    // ROS_INFO_STREAM("### Transforming all rotations starting from "<<fromLinkName<<" to axis "<<axis);
    std::string startLink = fromLink;
    if (startLink.empty())
    {
        startLink = traverser.getRootLinkName();
    }
    urdf_traverser::LinkPtr startLink_ = traverser.getLink(startLink);
    if (!startLink_)
    {
        ROS_ERROR("Link %s does not exist", startLink.c_str());
        return false;
    }

    Vector3RecursionParams * vp = new Vector3RecursionParams(axis);
    urdf_traverser::RecursionParamsPtr p(vp);

    // traverse top-down, but don't include the link itself, as the method allRotationsToAxis()
    // operates on the links parent joints.
    int travRet = traverser.traverseTreeTopDown(startLink,
                  boost::bind(&allRotationsToAxisCB, _1), p, false);
    if (travRet <= 0)
    {
        ROS_ERROR("Recursion to align all rotation axes failed");
        return false;
    }
    return true;
}



