#include <ros/ros.h>
#include <urdf_traverser/Functions.h>
#include <urdf_traverser/UrdfTraverser.h>
#include <urdf_traverser/JointNames.h>

using urdf_traverser::UrdfTraverser;
using urdf_traverser::RecursionParams;
using urdf_traverser::StringVectorRecursionParams;

/**
 * Helper function for getJointNames(const std::string&, std::vector<std::string>&).
 */
int getJointNamesCB(urdf_traverser::RecursionParamsPtr& p)
{
    StringVectorRecursionParams::Ptr param = baselib_binding_ns::dynamic_pointer_cast<StringVectorRecursionParams>(p);
    if (!param)
    {
        ROS_ERROR("Wrong recursion parameter type");
        return -1;
    }
    
    urdf_traverser::LinkPtr link = p->getLink();
    if (!link)
    {
        ROS_ERROR("printLink: NULL link in parameters!");
        return -1;
    }

    urdf_traverser::LinkPtr parent = link->getParent();
    //unsigned int level = p->level;

    // only return parent joints starting from first recursion level,
    // because we want only the joints in-between the starting linke
    // and the end of the chain.
    //if (level <=1 ) return 1;

    std::string pjoint;
    if (link->parent_joint)
    {
        pjoint = link->parent_joint->name;
        if (!param->skipFixed || urdf_traverser::isActive(link->parent_joint))
            param->names.push_back(pjoint);
    }
    //ROS_INFO("Information about %s: parent joint %s (recursion level %i)", link->name.c_str(), pjoint.c_str(), level);
    return 1;
}

bool urdf_traverser::getJointNames(UrdfTraverser& traverser, const std::string& fromLink,
        const bool skipFixed, std::vector<std::string>& result)
{
    ROS_INFO("Get joint names starting from link: %s", fromLink.c_str());

    // go through entire tree
    StringVectorRecursionParams * stringParams=new StringVectorRecursionParams(skipFixed);
    urdf_traverser::RecursionParamsPtr p(stringParams);
    bool success = (traverser.traverseTreeTopDown(fromLink, boost::bind(&getJointNamesCB, _1), p, false) >= 0);
    if (success)
    {
        result=stringParams->names;
    }
    return success;
}



