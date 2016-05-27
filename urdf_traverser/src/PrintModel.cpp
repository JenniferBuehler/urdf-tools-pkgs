#include <ros/ros.h>
#include <urdf_traverser/UrdfTraverser.h>
#include <urdf_traverser/PrintModel.h>
#include <urdf_traverser/Functions.h>

using urdf_traverser::UrdfTraverser;
using urdf_traverser::RecursionParams;
using urdf_traverser::FlagRecursionParams;

int printLink(urdf_traverser::RecursionParamsPtr& p)
{
    if (!p->getLink())
    {
        ROS_ERROR("printLink: NULL link in parameters!");
        return -1;
    }
    urdf_traverser::LinkPtr link = p->getLink();
    urdf_traverser::LinkPtr parent = link->getParent();
    unsigned int level = p->getLevel();

    bool verbose = true;
    // if the flag in the paramter is true, print verbose.
    urdf_traverser::FlagRecursionParamsPtr flagParam = baselib_binding_ns::dynamic_pointer_cast<FlagRecursionParams>(p);
    if (flagParam) verbose = flagParam->flag;

    std::stringstream _indent;
    for (unsigned int i = 0; i < level; ++i) _indent << "   ";
    std::string indent = _indent.str();

    std::string pjoint("NULL");
    if (link->parent_joint) pjoint = link->parent_joint->name;
    ROS_INFO("%s**%s: parent joint %s", indent.c_str(), link->name.c_str(), pjoint.c_str());

    if (!verbose) return 1;

    // can only print more information if the parent joint is not NULL
    if (!link->parent_joint) return 1;

    Eigen::Vector3d rotAx = urdf_traverser::getRotationAxis(link->parent_joint);
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

bool urdf_traverser::printModel(UrdfTraverser& traverser, const std::string& fromLink, bool verbose)
{
    ROS_INFO("Printing model down from link: %s", fromLink.c_str());

    // go through entire tree
    urdf_traverser::RecursionParamsPtr p(new FlagRecursionParams(verbose));
    return traverser.traverseTreeTopDown(fromLink, boost::bind(&printLink, _1), p, true) >= 0;
}

bool urdf_traverser::printModel(UrdfTraverser& traverser, bool verbose)
{
    std::string root_link = traverser.getRootLinkName();
    return printModel(traverser, root_link, verbose);
}
