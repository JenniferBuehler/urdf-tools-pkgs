/**
    Copyright (C) 2015 Jennifer Buehler

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

#include <urdf_traverser/Helpers.h>
#include <urdf_traverser/UrdfTraverser.h>

#include <string>
#include <ros/ros.h>
#include <ros/package.h>

#include <map>
#include <vector>
#include <set>
#include <fstream>
#include <algorithm>

#define RAD_TO_DEG 180/M_PI

using urdf_traverser::UrdfTraverser;

// Returns if this is an active joint in the URDF description
bool UrdfTraverser::isActive(const JointPtr& joint) const
{
    if (!joint) return false;
    return (joint->type == urdf::Joint::REVOLUTE) ||
           (joint->type == urdf::Joint::CONTINUOUS) ||
           (joint->type == urdf::Joint::PRISMATIC);
}

std::string UrdfTraverser::getRootLinkName() const
{
    LinkConstPtr root = this->robot.getRoot();
    if (!root)
    {
        ROS_ERROR("Loaded model has no root");
        return "";
    }
    return root->name;
}

bool UrdfTraverser::scaleTranslation(JointPtr& joint, double scale_factor)
{
    EigenTransform vTrans = getTransform(joint);
    scaleTranslation(vTrans, scale_factor);
    setTransform(vTrans, joint);
}

void UrdfTraverser::scaleTranslation(LinkPtr& link, double scale_factor)
{
    for (std::vector<VisualPtr >::iterator vit = link->visual_array.begin();
            vit != link->visual_array.end(); ++vit)
    {
        VisualPtr visual = *vit;
        EigenTransform vTrans = getTransform(visual->origin);
        scaleTranslation(vTrans, scale_factor);
        setTransform(vTrans, visual->origin);
    }


    for (std::vector<CollisionPtr >::iterator cit = link->collision_array.begin();
            cit != link->collision_array.end(); ++cit)
    {
        CollisionPtr coll = *cit;
        EigenTransform vTrans = getTransform(coll->origin);
        scaleTranslation(vTrans, scale_factor);
        setTransform(vTrans, coll->origin);
    }
    if (!link->inertial)
    {
        // ROS_WARN("Link %s  has no inertial",link->name.c_str());
        return;
    }

    EigenTransform vTrans = getTransform(link->inertial->origin);
    scaleTranslation(vTrans, scale_factor);
    setTransform(vTrans, link->inertial->origin);
}


bool UrdfTraverser::scaleModelRecursive(double scale_factor)
{
    LinkPtr root_link = this->robot.root_link_;
    if (!root_link)
    {
        ROS_ERROR("No root link");
        return false;
    }

    // do one call of scaleModel(RecursionParams) for the root link
    RecursionParamsPtr p(new FactorRecursionParams(scale_factor));
    int cvt = scaleModel(p);

    if (cvt < 0)
    {
        ROS_ERROR("Could not convert root mesh");
        return false;
    }

    // go through entire tree
    return (cvt == 0) ||
           (this->traverseTreeTopDown(root_link, boost::bind(&UrdfTraverser::scaleModel, this, _1), p, true, 0) >= 0);
}

int UrdfTraverser::scaleModel(RecursionParamsPtr& p)
{
    FactorRecursionParams::Ptr param = architecture_binding_ns::dynamic_pointer_cast<FactorRecursionParams>(p);
    if (!param)
    {
        ROS_ERROR("Wrong recursion parameter type");
        return -1;
    }

    LinkPtr link = param->link;
    if (!link)
    {
        ROS_ERROR("Recursion parameter must have initialised link!");
        return -1;
    }
    scaleTranslation(link, param->factor);
    JointPtr pjoint = link->parent_joint;
    if (pjoint)
    {
        scaleTranslation(pjoint, param->factor);
    }
    return 1;
}



    


bool UrdfTraverser::getDependencyOrderedJoints(std::vector<JointPtr>& result,
        const JointPtr& from_joint, bool allowSplits, bool onlyActive)
{
    LinkPtr childLink;
    getRobot().getLink(from_joint->child_link_name, childLink);
    if (!childLink)
    {
        ROS_ERROR("Child link %s not found", from_joint->child_link_name.c_str());
        return false;
    }
    if (!getDependencyOrderedJoints(result, childLink, allowSplits, onlyActive))
    {
        ROS_ERROR("Could not get ordered joints for %s", from_joint->child_link_name.c_str());
        return false;
    }
    if (!onlyActive || isActive(from_joint))
    {
        result.insert(result.begin(), from_joint);
    }
    return true;
}

bool UrdfTraverser::getDependencyOrderedJoints(std::vector<JointPtr>& result, const LinkPtr& from_link,
        bool allowSplits, bool onlyActive)
{
    if (!allowSplits && (from_link->child_joints.size() > 1))
    {
        ROS_ERROR("Splitting point at %s!", from_link->name.c_str());
        return false;
    }
    OrderedJointsRecursionParams * p = new OrderedJointsRecursionParams(allowSplits, onlyActive);
    RecursionParamsPtr rp(p);
    int travRet=this->traverseTreeTopDown(from_link, boost::bind(&UrdfTraverser::addJointLink, this, _1), rp, false, 0);
    if (travRet < 0)
    {
        ROS_ERROR("Could not add depenency order");
        p->dependencyOrderedJoints.clear();
        return false;
    }

    result = p->dependencyOrderedJoints;
    return true;
}

int UrdfTraverser::addJointLink(RecursionParamsPtr& p)
{
    OrderedJointsRecursionParams::Ptr param = architecture_binding_ns::dynamic_pointer_cast<OrderedJointsRecursionParams>(p);
    if (!param || !param->link)
    {
        ROS_ERROR("Wrong recursion parameter type, or NULL link");
        return -1;
    }

    // ROS_INFO("At link %s", parent->name.c_str());
    LinkPtr parent = param->link->getParent();
    if (parent->child_joints.empty())
    {
        ROS_ERROR("If links are connected, there must be at least one joint");
        return -1;
    }
    if (!param->link->parent_joint)
    {
        ROS_ERROR("NULL parent joint");
        return -1;
    }
    if (parent->child_joints.size() > 1)
    {
        if (!param->allowSplits)
        {
            ROS_ERROR("Splitting point at %s!", parent->name.c_str());
            return -1;
        }
        // this is a splitting point, we have to add support for this
    }

    if (param->onlyActive && !isActive(param->link->parent_joint))
    {
        // ROS_INFO("No type");
        return 1;
    }

    // ROS_INFO("Adding %s",link->parent_joint->name.c_str());
    param->dependencyOrderedJoints.push_back(param->link->parent_joint);
    return 1;
}




int UrdfTraverser::getChildJoint(const JointPtr& joint, JointPtr& child)
{
    LinkPtr childLink = getChildLink(joint);
    if (!childLink)
    {
        ROS_ERROR("Consistency: all joints must have child links");
        return -2;
    }
    if (childLink->child_joints.size() > 1)
    {
        return -1;
    }
    if (childLink->child_joints.empty())
    {   // this is the end link, and we've defined the end
        // frame to be at the same location as the last joint,
        // so no rotation should be needed?
        return 0;
    }
    // there must be only one joint
    child = childLink->child_joints.front();
    return 1;
}

UrdfTraverser::LinkPtr UrdfTraverser::getChildLink(const JointPtr& joint)
{
    LinkPtr childLink;
    getRobot().getLink(joint->child_link_name, childLink);
    return childLink;
}

UrdfTraverser::LinkConstPtr UrdfTraverser::readChildLink(const JointPtr& joint) const
{
    LinkPtr childLink;
    getRobot().getLink(joint->child_link_name, childLink);
    return childLink;
}



UrdfTraverser::JointPtr UrdfTraverser::getParentJoint(const JointPtr& joint)
{
    LinkConstPtr parentLink = getRobot().getLink(joint->parent_link_name);
    if (!parentLink) return JointPtr();
    return parentLink->parent_joint;
}

UrdfTraverser::JointConstPtr UrdfTraverser::readParentJoint(const JointPtr& joint) const
{
    LinkConstPtr parentLink = getRobot().getLink(joint->parent_link_name);
    if (!parentLink) return JointPtr();
    return parentLink->parent_joint;
}





bool UrdfTraverser::isChildOf(const LinkConstPtr& parent, const LinkConstPtr& child) const
{
    for (unsigned int i = 0; i < parent->child_links.size(); ++i)
    {
        LinkPtr childLink = parent->child_links[i];
        if (childLink->name == child->name) return true;
    }
    return false;
}


bool UrdfTraverser::isChildJointOf(const LinkConstPtr& parent, const JointConstPtr& joint) const
{
    for (unsigned int i = 0; i < parent->child_joints.size(); ++i)
    {
        JointPtr childJnt = parent->child_joints[i];
        if (childJnt->name == joint->name) return true;
    }
    return false;
}


bool UrdfTraverser::printModel(const std::string& fromLink, bool verbose)
{
    // get root link
    LinkPtr root_link=getLink(fromLink);
    if (!root_link)
    {
        ROS_ERROR("no root link %s", this->robot.getName().c_str());
        return false;
    }

    ROS_INFO("Root link: %s", root_link->name.c_str());

    // go through entire tree
    RecursionParamsPtr p(new FlagRecursionParams(verbose));
    return this->traverseTreeTopDown(root_link, boost::bind(&UrdfTraverser::printLink, this, _1), p, true, 0) >= 0;
}

bool UrdfTraverser::printModel(bool verbose)
{
    // get root link
    LinkPtr root_link = this->robot.root_link_;
    if (!root_link)
    {
        ROS_ERROR("no root link %s", this->robot.getName().c_str());
        return false;
    }

    ROS_INFO("Root link: %s", root_link->name.c_str());

    // go through entire tree
    RecursionParamsPtr p(new FlagRecursionParams(verbose));
    return this->traverseTreeTopDown(root_link, boost::bind(&UrdfTraverser::printLink, this, _1), p, true, 0) >= 0;
}


int UrdfTraverser::printLink(RecursionParamsPtr& p)
{
    if (!p->link)
    {
        ROS_ERROR("printLink: NULL link in parameters!");
        return -1;
    }
    LinkPtr link = p->link;
    LinkPtr parent = link->getParent();
    unsigned int level = p->level;

    bool verbose=true;
    // if the flag in the paramter is true, print verbose.
    FlagRecursionParams::Ptr flagParam = architecture_binding_ns::dynamic_pointer_cast<FlagRecursionParams>(p);
    if (flagParam) verbose=flagParam->flag;

    std::stringstream _indent;
    for (unsigned int i = 0; i < level; ++i) _indent << "   ";
    std::string indent = _indent.str();

    std::string pjoint("NULL");
    if (link->parent_joint) pjoint = link->parent_joint->name;
    ROS_INFO("%s**%s: parent joint %s", indent.c_str(), link->name.c_str(), pjoint.c_str());
    
    if (!verbose) return 1;

    // can only print more information if the parent joint is not NULL
    if (!link->parent_joint) return 1;

    Eigen::Vector3d rotAx=getRotationAxis(link->parent_joint);
    ROS_INFO("%s  - Parent joint axis: %f %f %f", indent.c_str(), rotAx.x(), rotAx.y(), rotAx.z());

    // get translation
    double x = link->parent_joint->parent_to_joint_origin_transform.position.x;
    double y = link->parent_joint->parent_to_joint_origin_transform.position.y;
    double z = link->parent_joint->parent_to_joint_origin_transform.position.z;
    ROS_INFO("%s  - Translation: %f %f %f", indent.c_str(), x, y, z);

    double qx = link->parent_joint->parent_to_joint_origin_transform.rotation.x;
    double qy = link->parent_joint->parent_to_joint_origin_transform.rotation.y;
    double qz = link->parent_joint->parent_to_joint_origin_transform.rotation.z;
    double qw = link->parent_joint->parent_to_joint_origin_transform.rotation.w;
    ROS_INFO("%s  - Quaternion: %f %f %f %f", indent.c_str(), qx, qy, qz, qw);

    // get rpy
    double roll, pitch, yaw;
    link->parent_joint->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw);

    if (isnan(roll) || isnan(pitch) || isnan(yaw))
    {
        ROS_ERROR("getRPY() returned nan!");
        return -1;
    }

    ROS_INFO("%s     (=RPY: %f %f %f)", indent.c_str(), roll, pitch, yaw);
    return 1;
}


bool UrdfTraverser::getJointNames(const std::string& fromLink, const bool skipFixed, std::vector<std::string>& result)
{
    std::string rootLink=fromLink;
    if (rootLink.empty()){
        rootLink = getRootLinkName();
    }

    // get root link
    LinkPtr root_link;
    this->robot.getLink(rootLink, root_link);
    if (!root_link)
    {
        ROS_ERROR("no root link %s", fromLink.c_str());
        return false;
    }

    ROS_INFO("Get joint names starting from link: %s", root_link->name.c_str());

    // go through entire tree
    StringVectorRecursionParams * stringParams=new StringVectorRecursionParams(skipFixed);
    RecursionParamsPtr p(stringParams);
    bool success = (this->traverseTreeTopDown(root_link, boost::bind(&UrdfTraverser::getJointNames, this, _1), p, true, 0) >= 0);
    if (success)
    {
        result=stringParams->names;
    }
    return success;
}


int UrdfTraverser::getJointNames(RecursionParamsPtr& p)
{
    StringVectorRecursionParams::Ptr param = architecture_binding_ns::dynamic_pointer_cast<StringVectorRecursionParams>(p);
    if (!param)
    {
        ROS_ERROR("Wrong recursion parameter type");
        return -1;
    }
    if (!p->link)
    {
        ROS_ERROR("printLink: NULL link in parameters!");
        return -1;
    }
    
    LinkPtr link = p->link;
    LinkPtr parent = link->getParent();
    //unsigned int level = p->level;

    // only return parent joints starting from first recursion level,
    // because we want only the joints in-between the starting linke
    // and the end of the chain.
    //if (level <=1 ) return 1;

    std::string pjoint;
    if (link->parent_joint)
    {
        pjoint = link->parent_joint->name;
        if (!param->skipFixed || isActive(link->parent_joint))
            param->names.push_back(pjoint);
    }
    //ROS_INFO("Information about %s: parent joint %s (recursion level %i)", link->name.c_str(), pjoint.c_str(), level);
    return 1;
}

bool UrdfTraverser::hasChildLink(const LinkConstPtr& link, const std::string& childName) const
{
    for (unsigned int i = 0; i < link->child_links.size(); ++i)
    {
        LinkPtr childLink = link->child_links[i];
        if (childLink->name == childName) return true;
    }
    return false;
}




int UrdfTraverser::traverseTreeTopDown(const std::string& linkName, boost::function< int(RecursionParamsPtr&)> link_cb,
                                      RecursionParamsPtr& params, bool includeLink)
{
    LinkPtr link=getLink(linkName);
    if (!link)
    {
        ROS_ERROR_STREAM("Could not get Link "<<linkName);
        return -1;
    }
    return traverseTreeTopDown(link, link_cb, params, includeLink, 0);
}

int UrdfTraverser::traverseTreeTopDown(const LinkPtr& link, boost::function< int(RecursionParamsPtr&)> link_cb,
                                      RecursionParamsPtr& params, bool includeLink, unsigned int level)
{
    if (includeLink)
    {
        params->setParams(link, level);
        int link_ret = link_cb(params);
        if (link_ret <= 0)
        {
            // stopping traversal
            return link_ret;
        }
    }

    level += 1;
    for (std::vector<LinkPtr>::const_iterator child = link->child_links.begin();
            child != link->child_links.end(); child++)
    {
        LinkPtr childLink = *child;
        if (childLink)
        {
            params->setParams(childLink, level);
            int link_ret = link_cb(params);
            if (link_ret <= 0)
            {
                // stopping traversal
                return link_ret;
            }

            // recurse down the tree
            int ret = traverseTreeTopDown(childLink, link_cb, params, false, level);
            if (ret < 0)
            {
                ROS_ERROR("Error parsing branch of %s", childLink->name.c_str());
                return -1;
            }
        }
        else
        {
            ROS_ERROR("root link: %s has a null child!", link->name.c_str());
            return false;
        }
    }
    return 1;
};





int UrdfTraverser::traverseTreeBottomUp(const std::string& linkName, boost::function< int(RecursionParamsPtr&)> link_cb,
                                      RecursionParamsPtr& params, bool includeLink)
{
    LinkPtr link=getLink(linkName);
    if (!link)
    {
        ROS_ERROR_STREAM("Could not get Link "<<linkName);
        return -1;
    }
    return traverseTreeBottomUp(link, link_cb, params, includeLink, 0);
}



int UrdfTraverser::traverseTreeBottomUp(const LinkPtr& link, boost::function<int(RecursionParamsPtr&)> link_cb,
                         RecursionParamsPtr& params, bool includeLink, unsigned int level)
{
    std::set<std::string> toTraverse;
    for (unsigned int i = 0; i < link->child_links.size(); ++i)
    {
        LinkPtr childLink = link->child_links[i];
        toTraverse.insert(childLink->name);
    }

    for (std::set<std::string>::iterator it = toTraverse.begin(); it != toTraverse.end(); ++it)
    {
        if (!hasChildLink(link,*it))
        {
            ROS_ERROR_STREAM("Consistency: Link "<<link->name<<" does not have child "<<*it<<" any more.");
            return -1;
        }
        
        LinkPtr childLink;
        this->robot.getLink(*it, childLink);

        if (childLink)
        {
            // ROS_INFO("Traversal into child %s",childLink->name.c_str());
            // recurse down the tree
            int travRes=traverseTreeBottomUp(childLink, link_cb, params, true, level+1);
            if (travRes==0)
            {
                ROS_INFO("Stopping traversal at %s", childLink->name.c_str());
                return 0;
            }
            else if (travRes<0)
            {
                ROS_ERROR("Error parsing branch of %s", childLink->name.c_str());
                return -1;
            }
            // childLink = params->result;
        }
        else
        {
            ROS_ERROR("root link: %s has a null child!", link->name.c_str());
            return -1;
        }
    }

    if (!includeLink)
    {
        return 1;
    }
    // ROS_INFO("Callback for link %s",link->name.c_str());
    params->setParams(link, level);
    int cbRet=link_cb(params);
    if (cbRet < 0)
    {
        ROS_ERROR("Error parsing branch of %s", link->name.c_str());
        return -1;
    }
    return cbRet;
}
/*
bool UrdfTraverser::traverseTreeBottomUp(LinkPtr& link, boost::function< LinkPtr(LinkPtr&)> link_cb)
{
    std::set<std::string> toTraverse;
    for (unsigned int i = 0; i < link->child_links.size(); ++i)
    {
        LinkPtr childLink = link->child_links[i];
        toTraverse.insert(childLink->name);
    }

    for (std::set<std::string>::iterator it = toTraverse.begin(); it != toTraverse.end(); ++it)
    {
        if (!hasChildLink(link,*it))
        {
            ROS_ERROR_STREAM("Consistency: Link "<<link->name<<" does not have child "<<*it<<" any more.");
            return false;
        }
        
        LinkPtr childLink;
        this->robot.getLink(*it, childLink);

        if (childLink)
        {
            // ROS_INFO("Traversal into child %s",childLink->name.c_str());
            // recurse down the tree
            if (!traverseTreeBottomUp(childLink, link_cb))
            {
                ROS_ERROR("Error parsing branch of %s", childLink->name.c_str());
                return false;
            }
        }
        else
        {
            ROS_ERROR("root link: %s has a null child!", link->name.c_str());
            return false;
        }
    }

    // ROS_INFO("Relink child %s",link->name.c_str());
    link = link_cb(link);
    if (!link)
    {
        ROS_ERROR("Error parsing branch of %s", link->name.c_str());
        return false;
    }
    return true;
}
*/

#if 0  // OLD
bool UrdfTraverser::allRotationsToAxis(const std::string& fromLinkName, const Eigen::Vector3d& axis)
{
    // ROS_INFO_STREAM("### Transforming all rotations starting from "<<fromLinkName<<" to axis "<<axis);
    std::string rootLink=fromLinkName;
    if (rootLink.empty()){
        rootLink = getRootLinkName();
    }
    LinkPtr from_link=getLink(rootLink);
    if (!from_link)
    {
        ROS_ERROR("Link %s does not exist", fromLinkName.c_str());
        return false;
    }

    if (from_link->parent_joint && !allRotationsToAxis(from_link->parent_joint, axis))
    {
        ROS_ERROR("Aborting recursion.");
        return false;
    }

    for (std::vector<JointPtr>::iterator pj = from_link->child_joints.begin();
            pj != from_link->child_joints.end(); pj++)
    {
        if (!allRotationsToAxis(*pj, axis))
        {
            ROS_ERROR("Aborting recursion.");
            return false;
        }
    }
    return true;
}


bool UrdfTraverser::allRotationsToAxis(JointPtr& joint, const Eigen::Vector3d& axis)
{
    if (!joint)
    {
        ROS_ERROR_STREAM("allRotationsToAxis: Joint is NULL");
        return false;
    }

    LinkPtr childLink;
    getRobot().getLink(joint->child_link_name, childLink);

    if (!childLink)
    {
        ROS_ERROR("All joints must have a child link!");
        return false;
    }
    Eigen::Quaterniond alignAxis;
    if (jointTransformForAxis(*joint, axis, alignAxis))
    {
        // ROS_INFO_STREAM("Transforming axis for joint "<<joint->name<<" with transform "<<alignAxis);
        applyTransform(joint, EigenTransform(alignAxis), false);
        // the link has to receive the inverse transorm, so it stays at the original position
        Eigen::Quaterniond alignAxisInv = alignAxis.inverse();
        applyTransform(childLink, EigenTransform(alignAxisInv), true);

        // now, we have to fix the child joint's (1st order child joints) transform
        // to correct for this transformation.
        for (std::vector<JointPtr>::iterator pj = childLink->child_joints.begin();
                pj != childLink->child_joints.end(); pj++)
        {
            applyTransform(*pj, EigenTransform(alignAxisInv), true);
        }

        // finally, set the rotation axis to the target
        joint->axis.x = axis.x();
        joint->axis.y = axis.y();
        joint->axis.z = axis.z();
    }
    // recurse
    for (std::vector<JointPtr>::iterator pj = childLink->child_joints.begin();
            pj != childLink->child_joints.end(); pj++)
    {
        if (!allRotationsToAxis(*pj, axis))
        {
            ROS_ERROR("Aborting recursion.");
            return false;
        }
    }
    return true;
}
#endif

bool UrdfTraverser::allRotationsToAxis(const std::string& fromLinkName, const Eigen::Vector3d& axis)
{
    // ROS_INFO_STREAM("### Transforming all rotations starting from "<<fromLinkName<<" to axis "<<axis);
    std::string rootLink=fromLinkName;
    if (rootLink.empty()){
        rootLink = getRootLinkName();
    }
    LinkPtr from_link=getLink(rootLink);
    if (!from_link)
    {
        ROS_ERROR("Link %s does not exist", fromLinkName.c_str());
        return false;
    }

    Vector3RecursionParams * vp = new Vector3RecursionParams(axis);
    RecursionParamsPtr p(vp);

    // traverse top-down, but don't include the link itself, as the method allRotationsToAxis()
    // operates on the links parent joints.
    int travRet = this->traverseTreeTopDown(from_link,
            boost::bind(&UrdfTraverser::allRotationsToAxis, this, _1), p, false, 0);
    if (travRet <= 0)
    {
        ROS_ERROR("Recursion to align all rotation axes failed");
        return false;
    }
    return true;
}


int UrdfTraverser::allRotationsToAxis(RecursionParamsPtr& p)
{
    LinkPtr link = p->link;
    if (!link)
    {
        ROS_ERROR("allRotationsToAxis: NULL link passed");
        return -1;
    }
    
    Vector3RecursionParams::Ptr param = architecture_binding_ns::dynamic_pointer_cast<Vector3RecursionParams>(p);
    if (!param)
    {
        ROS_ERROR("Wrong recursion parameter type");
        return -1;
    }
    
    JointPtr joint = link->parent_joint;
    if (!joint)
    {
        ROS_INFO_STREAM("allRotationsToAxis: Joint for link "<<link->name<<" is NULL, so this must be the root joint");
        return 1;
    }
    
    Eigen::Vector3d axis=param->vec;

    Eigen::Quaterniond alignAxis;
    if (jointTransformForAxis(*joint, axis, alignAxis))
    {
        // ROS_INFO_STREAM("Transforming axis for joint "<<joint->name<<" with transform "<<alignAxis);
        applyTransform(joint, EigenTransform(alignAxis), false);
        // the link has to receive the inverse transorm, so it stays at the original position
        Eigen::Quaterniond alignAxisInv = alignAxis.inverse();
        applyTransform(link, EigenTransform(alignAxisInv), true);

        // now, we have to fix the child joint's (1st order child joints) transform
        // to correct for this transformation.
        for (std::vector<JointPtr>::iterator pj = link->child_joints.begin();
                pj != link->child_joints.end(); pj++)
        {
            applyTransform(*pj, EigenTransform(alignAxisInv), true);
        }

        // finally, set the rotation axis to the target
        joint->axis.x = axis.x();
        joint->axis.y = axis.y();
        joint->axis.z = axis.z();
    }

    // all good, indicate that recursion can continue
    return 1;
}









int UrdfTraverser::checkActiveJoints(RecursionParamsPtr& p)
{
    LinkPtr link = p->link;
    LinkPtr parent = link->getParent();
    unsigned int level = p->level;
    if (level==0) return 1;
    if (link->parent_joint && !isActive(link->parent_joint))
    {
        ROS_INFO("UrdfTraverser: Found fixed joint %s", link->parent_joint->name.c_str());
        return -1;
    }
    return 1; 
}

bool UrdfTraverser::hasFixedJoints(LinkPtr& from_link)
{
    LinkPtr l = from_link;
    RecursionParamsPtr p(new RecursionParams(l, 0));
    if ((checkActiveJoints(p) < 0) ||
            (this->traverseTreeTopDown(from_link,
                boost::bind(&UrdfTraverser::checkActiveJoints, this, _1), p, true, 0) < 0))
    {
        return true;
    }
    return false;
}

bool UrdfTraverser::joinFixedLinks(const std::string& from_link)
{
    // ROS_INFO_STREAM("### Joining fixed links starting from "<<fromLinkName);
    std::string rootLink=from_link;
    if (rootLink.empty()){
        rootLink = getRootLinkName();
    }
    LinkPtr link=getLink(rootLink);
    if (!link)
    {
        ROS_ERROR_STREAM("No link named '"<<from_link<<"'");
        return false;
    }
    return joinFixedLinks(link);
}

bool UrdfTraverser::joinFixedLinks(LinkPtr& from_link)
{
//    ROS_INFO_STREAM("### Joining fixed links starting from "<<from_link->name
//        <<" with "<<from_link->child_links.size()<<" children");

 
#if 0
    // call joinFixedLinksOnThis only from child
    // links of from_link, because the joining happens with the
    // parent joint.
    std::set<std::string> toTraverse;
    for (unsigned int i = 0; i < from_link->child_links.size(); ++i)
    {
        LinkPtr childLink = from_link->child_links[i];
        toTraverse.insert(childLink->name);
    }

    for (std::set<std::string>::iterator it = toTraverse.begin(); it != toTraverse.end(); ++it)
    {
        if (!hasChildLink(from_link,*it))
        {
            ROS_WARN_STREAM("UrdfTraverser, test case: Link "<<from_link->name
                <<" does not have child "<<*it<<" any more, this can happen if it was joined.");
            continue;
        }
        
        LinkPtr childLink;
        this->robot.getLink(*it, childLink);
 
        LinkRecursionParams * lp =new LinkRecursionParams();
        RecursionParamsPtr p(lp);
        int travResult=this->traverseTreeBottomUp(childLink, boost::bind(&UrdfTraverser::joinFixedLinksOnThis, this, _1), p, true, 1);
        if (travResult < 0)
        {
            ROS_ERROR_STREAM("Could not join fixed links from link "<<childLink->name);
            return false;
        }
        // childLink=lp->result;
    }
#else
    // ALTERNATIVE: join fixed joints incl. parent joint
    bool includeParent = false;
    LinkRecursionParams * lp =new LinkRecursionParams();
    RecursionParamsPtr p(lp);
    int travResult=this->traverseTreeBottomUp(from_link, boost::bind(&UrdfTraverser::joinFixedLinksOnThis, this, _1), p, includeParent, 0);
    if (travResult < 0)
    {
        ROS_ERROR("Could not join fixed links");
        return false;
    }
    if (includeParent) from_link=lp->resultLink;
#endif

    // consistency check: All joints in the tree must be active now!
    if (hasFixedJoints(from_link))
    {
        ROS_ERROR("consistency: We should now only have active joints in the tree!");
        return false;
    }
    return true;
}

//UrdfTraverser::LinkPtr UrdfTraverser::joinFixedLinksOnThis(LinkPtr& link)
int UrdfTraverser::joinFixedLinksOnThis(RecursionParamsPtr& params)
{
    LinkRecursionParams::Ptr lparam = architecture_binding_ns::dynamic_pointer_cast<LinkRecursionParams>(params);
    if (!lparam)
    {
        ROS_ERROR("Wrong recursion parameter type");
        return -1;
    }

    LinkPtr link=lparam->link;

    if (!link)
    {
        lparam->resultLink=link;
        return 1;
    }

    // ROS_INFO("--joinFixedLinksOnThis: Traverse %s",link->name.c_str());
    JointPtr jointToParent = link->parent_joint;
    if (!jointToParent)
    {
        // ROS_WARN("End of chain at %s, because of no parent joint", link->name.c_str());
        lparam->resultLink=link;
        return 1;
    }


    LinkPtr parentLink;
    this->robot.getLink(jointToParent->parent_link_name, parentLink);
    if (!parentLink)
    {
        ROS_WARN("End of chain at %s, because of no parent link", link->name.c_str());
        lparam->resultLink=link;
        return 1;
    }

    /*if (link->child_joints.empty()) {
        ROS_WARN("INFO: end effector %s",link->name.c_str());
        return link;
    }*/


    if (isActive(jointToParent))
    {
        // ROS_INFO("Parent of %s (%s) is active so won't delete",link->name.c_str(), jointToParent->name.c_str());
        // We won't delete this joint, as it is active.
        // ROS_INFO("Joining chain finished between %s and %s",parentLink->name.c_str(),link->name.c_str());
        lparam->resultLink=link;
        return 1;
    }

    // this joint is fixed, so we will delete it
    // ROS_INFO("Joining fixed joint (%s) between %s and %s",jointToParent->name.c_str(), parentLink->name.c_str(),link->name.c_str());
    
    // remove this link from the parent
    for (std::vector<LinkPtr >::iterator pc = parentLink->child_links.begin();
            pc != parentLink->child_links.end(); pc++)
    {
        LinkPtr child = (*pc);
        if (child->name == link->name)
        {
            // ROS_WARN("Remove link %s",link->name.c_str());
            parentLink->child_links.erase(pc);
            break;
        }
    }
    for (std::vector<JointPtr >::iterator pj = parentLink->child_joints.begin();
            pj != parentLink->child_joints.end(); pj++)
    {
        JointPtr child = (*pj);
        // this child joint is the current one that we just now removed
        if (child->name == jointToParent->name)
        {
            // ROS_WARN("Remove joint %s",child->name.c_str());
            parentLink->child_joints.erase(pj);
            break;
        }
    }

    // the local transfrom of the parent joint
    EigenTransform localTrans = getTransform(link);
    // ROS_INFO_STREAM("Transform between "<<parentLink->name<<" and "<<link->name<<": "<<localTrans);
    // all this link's child joints now must receive the extra transform from this joint which we removed.
    // then, the joints should be added to the parent link
    for (std::vector<JointPtr >::iterator j = link->child_joints.begin(); j != link->child_joints.end(); j++)
    {
        JointPtr child = (*j);
        if (!isActive(child))
        {
            ROS_ERROR("consistency: At this stage, we should only have active joints, found joint %s!",
                      child->name.c_str());
        }
        EigenTransform jTrans = getTransform(child);

        jTrans = localTrans * jTrans;
        setTransform(jTrans, child);
#if 0
        EigenTransform rotAxTrans=jTrans.inverse();
        // ROS_INFO_STREAM("Transforming rotation axis by "<<rotAxTrans);
        // ROS_INFO_STREAM("Old child axis of "<<child->name<<": "<<child->axis);
        applyTransform(rotAxTrans,child->axis); 
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
        LinkPtr childChildLink;
        this->robot.getLink(child->child_link_name, childChildLink);
        if (!childChildLink)
        {
            ROS_ERROR("consistency: found null child link for joint %s", child->name.c_str());
        }
        parentLink->child_links.push_back(childChildLink);
        childChildLink->setParent(parentLink);
    }

    for (std::vector<VisualPtr >::iterator vit = link->visual_array.begin();
            vit != link->visual_array.end(); ++vit)
    {
        VisualPtr visual = *vit;
        // apply the transform to the visual before adding it to the parent
        EigenTransform jTrans = getTransform(visual->origin);
        jTrans = localTrans * jTrans;
        setTransform(jTrans, visual->origin);
        parentLink->visual_array.push_back(visual);
    }


    for (std::vector<CollisionPtr >::iterator cit = link->collision_array.begin();
            cit != link->collision_array.end(); ++cit)
    {
        CollisionPtr coll = *cit;
        // apply the transform to the visual before adding it to the parent
        EigenTransform jTrans = getTransform(coll->origin);
        jTrans = localTrans * jTrans;
        setTransform(jTrans, coll->origin);
        parentLink->collision_array.push_back(coll);
    }


    if (parentLink->visual) parentLink->visual.reset();
    if (parentLink->collision) parentLink->collision.reset();


    // combine inertials
    // parent->inertial=XXX TODO;

    lparam->resultLink=parentLink;
    return 1;
}


std::vector<UrdfTraverser::JointPtr> UrdfTraverser::getChain(const LinkConstPtr& from_link, const LinkConstPtr& to_link) const
{
    std::vector<JointPtr> chain;

    if (to_link->name == from_link->name) return chain;

    LinkConstPtr curr = to_link;
    LinkConstPtr pl = to_link->getParent();

    while (curr && (curr->name != from_link->name))
    {
        // ROS_INFO_STREAM("Chain: "<<curr->name);
        JointPtr pj = curr->parent_joint;
        if (!pj)
        {
            ROS_ERROR("UrdfTraverser: End of chain at link '%s'", curr->name.c_str());
            return std::vector<JointPtr>();
        }
        chain.push_back(pj);
        curr = pl;
        pl = curr->getParent();
        // if (pl) ROS_INFO_STREAM("Parent: "<<pl->name);
    }
    if (curr->name != from_link->name)
    {
        ROS_ERROR_STREAM("UrdfTraverser: could not find link "<<
            from_link->name<<" while traversing up the chain starting from "<<
            to_link->name<<". Failed to find parent chain!");
        return std::vector<JointPtr>();
    }

    std::reverse(chain.begin(), chain.end());

    return chain;
}

UrdfTraverser::LinkPtr UrdfTraverser::getLink(const std::string& name)
{
    LinkPtr ptr;
    this->robot.getLink(name, ptr);
    return ptr;
}

UrdfTraverser::LinkConstPtr UrdfTraverser::readLink(const std::string& name) const
{
    LinkPtr ptr;
    this->robot.getLink(name, ptr);
    return ptr;
}

UrdfTraverser::JointPtr UrdfTraverser::getJoint(const std::string& name)
{
    JointPtr ptr;
    if (this->robot.joints_.find(name) == this->robot.joints_.end()) ptr.reset();
    else ptr = this->robot.joints_.find(name)->second;
    return ptr;
}

UrdfTraverser::JointConstPtr UrdfTraverser::readJoint(const std::string& name) const
{
    JointConstPtr ptr;
    if (this->robot.joints_.find(name) == this->robot.joints_.end()) ptr.reset();
    else ptr = this->robot.joints_.find(name)->second;
    return ptr;
}

void UrdfTraverser::setTransform(const EigenTransform& t, urdf::Pose& p)
{
    Eigen::Vector3d trans(t.translation());
    Eigen::Quaterniond rot(t.rotation());

    p.position.x = trans.x();
    p.position.y = trans.y();
    p.position.z = trans.z();
    p.rotation.x = rot.x();
    p.rotation.y = rot.y();
    p.rotation.z = rot.z();
    p.rotation.w = rot.w();
}

void UrdfTraverser::applyTransform(const EigenTransform& t, urdf::Vector3& v)
{
    Eigen::Vector3d _v(v.x,v.y,v.z);
    Eigen::Quaterniond rot(t.rotation());
    //ROS_INFO_STREAM("Rotation: "<<rot);
    _v=rot*_v;
    v.x=_v.x();
    v.y=_v.y();
    v.z=_v.z();
}

void UrdfTraverser::setTransform(const EigenTransform& t, JointPtr& joint)
{
    setTransform(t, joint->parent_to_joint_origin_transform);
}

void UrdfTraverser::scaleTranslation(EigenTransform& t, double scale_factor)
{
    Eigen::Vector3d trans = t.translation();
    trans *= scale_factor;
    Eigen::Matrix3d rot = t.rotation();
    t.setIdentity();
    t.translate(trans);
    t.rotate(rot);
}


UrdfTraverser::EigenTransform UrdfTraverser::getTransform(const JointConstPtr& joint) const
{
    return getTransform(joint->parent_to_joint_origin_transform);
}

UrdfTraverser::EigenTransform UrdfTraverser::getTransform(const LinkConstPtr& link) const
{
    return getTransform(link->parent_joint);
}

UrdfTraverser::EigenTransform UrdfTraverser::getTransform(const LinkConstPtr& from_link,  const LinkConstPtr& to_link) const
{
    return EigenTransform(getTransformMatrix(from_link, to_link));
}

UrdfTraverser::EigenTransform UrdfTraverser::getTransform(const urdf::Pose& p) const
{
    urdf::Vector3 _jtr = p.position;
    Eigen::Vector3d jtr(_jtr.x, _jtr.y, _jtr.z);
    urdf::Rotation _jrot = p.rotation;
    Eigen::Quaterniond jrot(_jrot.w, _jrot.x, _jrot.y, _jrot.z);
    jrot.normalize();
    EigenTransform tr;
    tr.setIdentity();
    tr = tr.translate(jtr);
    tr = tr.rotate(jrot);
    return tr;
}

Eigen::Matrix4d UrdfTraverser::getTransformMatrix(const LinkConstPtr& from_link,  const LinkConstPtr& to_link) const
{
    if (from_link->name == to_link->name) return Eigen::Matrix4d::Identity();

    std::vector<JointPtr> pjoints = getChain(from_link, to_link);

    if (pjoints.empty())
    {
        ROS_ERROR("could not get chain from %s to %s", from_link->name.c_str(), to_link->name.c_str());
        return Eigen::Matrix4d::Identity();
    }

    // ROS_INFO("Chain from %s to %s",from_link->name.c_str(),to_link->name.c_str());

    Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();

    for (std::vector<JointPtr>::iterator it = pjoints.begin(); it != pjoints.end(); ++it)
    {
        // ROS_INFO("Chain joint %s",(*it)->name.c_str());
        Eigen::Matrix4d mat = getTransform(*it).matrix();
        ret *= mat;
    }
    return ret;
}

UrdfTraverser::EigenTransform UrdfTraverser::getTransform(const LinkPtr& from_link,  const JointPtr& to_joint) const
{
    LinkPtr link1 = from_link;
    LinkPtr link2;
    this->robot.getLink(to_joint->child_link_name, link2);
    if (!link1 || !link2)
    {
        ROS_ERROR("Invalid joint specifications (%s, %s), first needs parent and second child",
                  link1->name.c_str(), link2->name.c_str());
    }
    return getTransform(link1, link2);
}


bool equalAxes(const Eigen::Vector3d& z1, const Eigen::Vector3d& z2, double tolerance)
{
    Eigen::Vector3d _z1=z1;
    Eigen::Vector3d _z2=z2;
    _z1.normalize();
    _z2.normalize();
    double dot = _z1.dot(_z2);
    return (std::fabs(dot - 1.0)) < tolerance;
}


bool UrdfTraverser::jointTransformForAxis(const urdf::Joint& joint,
        const Eigen::Vector3d& axis, Eigen::Quaterniond& rotation)
{
    Eigen::Vector3d rotAxis(joint.axis.x, joint.axis.y, joint.axis.z);
    rotAxis.normalize();
    // ROS_INFO_STREAM("Rotation axis for joint "<<joint.name<<": "<<rotAxis);
    if (equalAxes(rotAxis, axis, 1e-06)) return false;

    rotation = Eigen::Quaterniond::FromTwoVectors(rotAxis, axis);
    // ROS_WARN_STREAM("z alignment: "<<rotation);
    return true;
}


bool UrdfTraverser::applyTransform(JointPtr& joint, const EigenTransform& trans, bool preMult)
{
    EigenTransform vTrans = getTransform(joint);
    if (preMult) vTrans = trans * vTrans;
    else vTrans = vTrans * trans;
    setTransform(vTrans, joint);
}

void UrdfTraverser::applyTransform(LinkPtr& link, const EigenTransform& trans, bool preMult)
{
    // ROS_INFO("applying transform to link %s",link->name.c_str());

    for (std::vector<VisualPtr >::iterator vit = link->visual_array.begin();
            vit != link->visual_array.end(); ++vit)
    {
        VisualPtr visual = *vit;
        EigenTransform vTrans = getTransform(visual->origin);
        // ROS_INFO_STREAM("a visual for link"<<link->name<<" with transform "<<vTrans);
        if (preMult) vTrans = trans * vTrans;
        else vTrans = vTrans * trans;
        setTransform(vTrans, visual->origin);
    }


    for (std::vector<CollisionPtr >::iterator cit = link->collision_array.begin();
            cit != link->collision_array.end(); ++cit)
    {
        CollisionPtr coll = *cit;
        EigenTransform vTrans = getTransform(coll->origin);
        if (preMult) vTrans = trans * vTrans;
        else vTrans = vTrans * trans;
        setTransform(vTrans, coll->origin);
    }

    EigenTransform vTrans = getTransform(link->inertial->origin);
    if (preMult) vTrans = trans * vTrans;
    else vTrans = vTrans * trans;
    setTransform(vTrans, link->inertial->origin);
}


void UrdfTraverser::printJointNames(const std::string& fromLink) 
{
    std::vector<std::string> jointNames;
    if (!getJointNames(fromLink, false, jointNames))
    {
        ROS_WARN("Could not retrieve joint names to print on screen");
    }
    else
    {
        ROS_INFO_STREAM("Joint names starting from "<<fromLink<<":");
        for (int i=0; i<jointNames.size(); ++i) ROS_INFO_STREAM(jointNames[i]);
        ROS_INFO("---");
    }
}


bool UrdfTraverser::loadModelFromFile(const std::string& urdfFilename)
{
    std::string xml_file;
    if (!getModelFromFile(urdfFilename, xml_file))
    {
        ROS_ERROR("Could not load file");
        return false;
    }

    if (!loadModelFromXMLString(xml_file))
    {
        ROS_ERROR("Could not load file");
        return false;
    }
    return true;
}



bool UrdfTraverser::loadModelFromXMLString(const std::string& xmlString)
{
    bool success = robot.initString(xmlString);
    if (!success)
    {
        ROS_ERROR("Could not load model from XML string");
        return false;
    }
    robot_urdf=xmlString;
    return true;
}

/*bool UrdfTraverser::loadModelFromParameterServer()
{
    if (!robot.initParam("robot_description")) return false;
    isScaled = false;
    return true;
}*/


bool UrdfTraverser::getModelFromFile(const std::string& filename, std::string& xml_string) const
{
    std::fstream xml_file(filename.c_str(), std::fstream::in);
    if (xml_file.is_open())
    {
        while (xml_file.good())
        {
            std::string line;
            std::getline(xml_file, line);
            xml_string += (line + "\n");
        }
        xml_file.close();
        return true;
    }
    else
    {
        ROS_ERROR("Could not open file [%s] for parsing.", filename.c_str());
    }
    return false;
}
