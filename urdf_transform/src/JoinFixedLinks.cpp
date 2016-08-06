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
#include <ros/ros.h>
#include <urdf_traverser/UrdfTraverser.h>
#include <urdf_traverser/Functions.h>
#include <urdf_traverser/ActiveJoints.h>
#include <urdf_transform/JoinFixedLinks.h>

using urdf_traverser::UrdfTraverser;
using urdf_traverser::RecursionParams;
using urdf_traverser::LinkRecursionParams;

/**
 * Callback: If the parent joint of this link is fixed, it will be removed, and this link's visual will be
 * connected to the parent link.
 * If the joint was active, the function returns the same link as in the parameter.
 * Otherwise, it returns the pointer to the parent link which now contains
 * this link's visual/collision.
 */
int joinFixedLinksOnThis(urdf_traverser::RecursionParamsPtr& params)
{
    LinkRecursionParams::Ptr lparam = baselib_binding_ns::dynamic_pointer_cast<LinkRecursionParams>(params);
    if (!lparam)
    {
        ROS_ERROR("Wrong recursion parameter type");
        return -1;
    }

    urdf_traverser::LinkPtr link = lparam->getLink();

    if (!link)
    {
        lparam->resultLink = link;
        return 1;
    }

    // ROS_INFO("--joinFixedLinksOnThis: Traverse %s",link->name.c_str());
    urdf_traverser::JointPtr jointToParent = link->parent_joint;
    if (!jointToParent)
    {
        // ROS_WARN("End of chain at %s, because of no parent joint", link->name.c_str());
        lparam->resultLink = link;
        return 1;
    }

    urdf_traverser::LinkPtr parentLink;
    lparam->model->getLink(jointToParent->parent_link_name, parentLink);
    if (!parentLink)
    {
        ROS_WARN("End of chain at %s, because of no parent link", link->name.c_str());
        lparam->resultLink = link;
        return 1;
    }

    /*if (link->child_joints.empty()) {
        ROS_WARN("INFO: end effector %s",link->name.c_str());
        return link;
    }*/


    if (urdf_traverser::isActive(jointToParent))
    {
        // ROS_INFO("Parent of %s (%s) is active so won't delete",link->name.c_str(), jointToParent->name.c_str());
        // We won't delete this joint, as it is active.
        // ROS_INFO("Joining chain finished between %s and %s",parentLink->name.c_str(),link->name.c_str());
        lparam->resultLink = link;
        return 1;
    }

    // this joint is fixed, so we will delete it
    // ROS_INFO("Joining fixed joint (%s) between %s and %s",jointToParent->name.c_str(), parentLink->name.c_str(),link->name.c_str());

    // remove this link from the parent
    for (std::vector<urdf_traverser::LinkPtr>::iterator pc = parentLink->child_links.begin();
            pc != parentLink->child_links.end(); pc++)
    {
        urdf_traverser::LinkPtr child = (*pc);
        if (child->name == link->name)
        {
            // ROS_WARN("Remove link %s",link->name.c_str());
            parentLink->child_links.erase(pc);
            break;
        }
    }
    for (std::vector<urdf_traverser::JointPtr>::iterator pj = parentLink->child_joints.begin();
            pj != parentLink->child_joints.end(); pj++)
    {
        urdf_traverser::JointPtr child = (*pj);
        // this child joint is the current one that we just now removed
        if (child->name == jointToParent->name)
        {
            // ROS_WARN("Remove joint %s",child->name.c_str());
            parentLink->child_joints.erase(pj);
            break;
        }
    }

    // the local transfrom of the parent joint
    urdf_traverser::EigenTransform localTrans = urdf_traverser::getTransform(link);
    // ROS_INFO_STREAM("Transform between "<<parentLink->name<<" and "<<link->name<<": "<<localTrans);
    // all this link's child joints now must receive the extra transform from this joint which we removed.
    // then, the joints should be added to the parent link
    for (std::vector<urdf_traverser::JointPtr>::iterator j = link->child_joints.begin(); j != link->child_joints.end(); j++)
    {
        urdf_traverser::JointPtr child = (*j);
        if (!urdf_traverser::isActive(child))
        {
            ROS_ERROR("consistency: At this stage, we should only have active joints, found joint %s!",
                      child->name.c_str());
        }
        urdf_traverser::EigenTransform jTrans = urdf_traverser::getTransform(child);

        jTrans = localTrans * jTrans;
        urdf_traverser::setTransform(jTrans, child);
#if 0
        EigenTransform rotAxTrans = jTrans.inverse();
        // ROS_INFO_STREAM("Transforming rotation axis by "<<rotAxTrans);
        // ROS_INFO_STREAM("Old child axis of "<<child->name<<": "<<child->axis);
        urdf_traverser::applyTransform(rotAxTrans, child->axis);
        /*Eigen::Vector3d childAxis(child->axis.x, child->axis.y, child->axis.z);
        childAxis.normalize();
        child->axis.x=childAxis.x();
        child->axis.y=childAxis.y();
        child->axis.z=childAxis.z();*/
        // ROS_INFO_STREAM("New child axis of "<<child->name<<": "<<child->axis);
#endif
        // add this child joint to the parent
        child->parent_link_name = parentLink->name;
        parentLink->child_joints.push_back(child);

        // this link's child link has to be added to parents as well
        urdf_traverser::LinkPtr childChildLink;
        lparam->model->getLink(child->child_link_name, childChildLink);
        if (!childChildLink)
        {
            ROS_ERROR("consistency: found null child link for joint %s", child->name.c_str());
        }
        parentLink->child_links.push_back(childChildLink);
        childChildLink->setParent(parentLink);
    }

    for (std::vector<urdf_traverser::VisualPtr>::iterator vit = link->visual_array.begin();
            vit != link->visual_array.end(); ++vit)
    {
        urdf_traverser::VisualPtr visual = *vit;
        // apply the transform to the visual before adding it to the parent
        urdf_traverser::EigenTransform jTrans = urdf_traverser::getTransform(visual->origin);
        jTrans = localTrans * jTrans;
        urdf_traverser::setTransform(jTrans, visual->origin);
        parentLink->visual_array.push_back(visual);
    }


    for (std::vector<urdf_traverser::CollisionPtr>::iterator cit = link->collision_array.begin();
            cit != link->collision_array.end(); ++cit)
    {
        urdf_traverser::CollisionPtr coll = *cit;
        // apply the transform to the visual before adding it to the parent
        urdf_traverser::EigenTransform jTrans = urdf_traverser::getTransform(coll->origin);
        jTrans = localTrans * jTrans;
        urdf_traverser::setTransform(jTrans, coll->origin);
        parentLink->collision_array.push_back(coll);
    }


    if (parentLink->visual) parentLink->visual.reset();
    if (parentLink->collision) parentLink->collision.reset();


    // combine inertials
    // parent->inertial=XXX TODO;

    lparam->resultLink = parentLink;
    return 1;
}


bool urdf_transform::joinFixedLinks(UrdfTraverser& traverser, const std::string& fromLink)
{
    std::string rootLinkName = traverser.getRootLinkName();
    std::string startLink = fromLink;
    if (startLink.empty())
    {
        startLink = rootLinkName;
    }
    urdf_traverser::LinkPtr link = traverser.getLink(startLink);
    if (!link)
    {
        ROS_ERROR_STREAM("No link named '" << startLink << "'");
        return false;
    }

    // ROS_INFO_STREAM("### Joining fixed links starting from "<<startLink);

    // join fixed joints *not* incl. parent joint
    bool includeParent = false;
    LinkRecursionParams * lp = new LinkRecursionParams(traverser.getModel());
    urdf_traverser::RecursionParamsPtr p(lp);
    int travResult = traverser.traverseTreeBottomUp(startLink, boost::bind(&joinFixedLinksOnThis, _1), p, includeParent);
    if (travResult < 0)
    {
        ROS_ERROR("Could not join fixed links");
        return false;
    }

    urdf_traverser::LinkPtr newLink = lp->resultLink;
    if (newLink->name != startLink)
    {
        ROS_INFO_STREAM("Starting link " << startLink << " re-assigned to be " << newLink->name << ".");
        if (startLink == rootLinkName)
        {
            ROS_INFO("Re-assigning root link of the model");
            traverser.getModel()->root_link_ = newLink;
        }
    }

    // consistency check: All joints in the tree must be active now!
    if (urdf_traverser::hasFixedJoints(traverser, startLink))
    {
        ROS_ERROR("consistency: We should now only have active joints in the tree!");
        return false;
    }
    return true;
}


