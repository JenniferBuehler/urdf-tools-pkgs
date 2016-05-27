#include <ros/ros.h>
#include <urdf_traverser/UrdfTraverser.h>
#include <urdf_traverser/DependencyOrderedJoints.h>
#include <urdf_traverser/Functions.h>

using urdf_traverser::UrdfTraverser;
using urdf_traverser::RecursionParams;

/**
 * \brief Recursion data for getting a list of joints, ordered by dependency (no joint depending on others
 * will come before them in the result vector)
 * \author Jennifer Buehler
 */
class OrderedJointsRecursionParams: public urdf_traverser::RecursionParams
{
public:
    typedef baselib_binding::shared_ptr<OrderedJointsRecursionParams>::type Ptr;
    OrderedJointsRecursionParams(): RecursionParams() {}
    OrderedJointsRecursionParams(bool _allowSplits, bool _onlyActive):
        allowSplits(_allowSplits),
        onlyActive(_onlyActive) {}
    OrderedJointsRecursionParams(const OrderedJointsRecursionParams& o):
        RecursionParams(o),
        allowSplits(o.allowSplits),
        onlyActive(o.onlyActive),
        dependencyOrderedJoints(o.dependencyOrderedJoints) {}
    virtual ~OrderedJointsRecursionParams() {}

    // Result set
    std::vector<urdf_traverser::JointPtr> dependencyOrderedJoints;

    // Allow splits, i.e. one link has several child joints. If this is set to false,
    // the recursive operation will fail at splitting points.
    bool allowSplits;

    // Only add joints to the result which are active.
    bool onlyActive;
};

// callback for getDependencyOrderedJoints()
int addJointLink(urdf_traverser::RecursionParamsPtr& p)
{
    OrderedJointsRecursionParams::Ptr param = baselib_binding_ns::dynamic_pointer_cast<OrderedJointsRecursionParams>(p);
    if (!param || !param->getLink())
    {
        ROS_ERROR("Wrong recursion parameter type, or NULL link");
        return -1;
    }

    // ROS_INFO("At link %s", parent->name.c_str());
    urdf_traverser::LinkPtr link = param->getLink();
    assert(link);

    urdf_traverser::LinkPtr parent = link->getParent();
    if (parent->child_joints.empty())
    {
        ROS_ERROR("If links are connected, there must be at least one joint");
        return -1;
    }
    if (!link->parent_joint)
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

    if (param->onlyActive && !urdf_traverser::isActive(link->parent_joint))
    {
        // ROS_INFO("No type");
        return 1;
    }

    // ROS_INFO("Adding %s",link->parent_joint->name.c_str());
    param->dependencyOrderedJoints.push_back(link->parent_joint);
    return 1;
}



bool urdf_traverser::getDependencyOrderedJoints(UrdfTraverser& traverser,
        std::vector<urdf_traverser::JointPtr>& result,
        const urdf_traverser::JointPtr& fromJoint,
        bool allowSplits, bool onlyActive)
{
    urdf_traverser::LinkPtr childLink = traverser.getChildLink(fromJoint);
    if (!childLink)
    {
        ROS_ERROR("Child link %s not found", fromJoint->child_link_name.c_str());
        return false;
    }
    if (!urdf_traverser::getDependencyOrderedJoints(traverser, result, childLink->name, allowSplits, onlyActive))
    {
        ROS_ERROR("Could not get ordered joints for %s", fromJoint->child_link_name.c_str());
        return false;
    }
    if (!onlyActive || urdf_traverser::isActive(fromJoint))
    {
        result.insert(result.begin(), fromJoint);
    }
    return true;
}


bool urdf_traverser::getDependencyOrderedJoints(UrdfTraverser& traverser,
        std::vector<JointPtr>& result, const std::string& fromLink,
        bool allowSplits, bool onlyActive)
{
    urdf_traverser::LinkPtr _fromLink = traverser.getLink(fromLink);
    if (!_fromLink)
    {
        ROS_ERROR_STREAM("No link named " << fromLink << " in URDF.");
        return false;
    }
    if (!allowSplits && (_fromLink->child_joints.size() > 1))
    {
        ROS_ERROR("Splitting point at %s!", fromLink.c_str());
        return false;
    }
    OrderedJointsRecursionParams * p = new OrderedJointsRecursionParams(allowSplits, onlyActive);
    RecursionParamsPtr rp(p);
    int travRet = traverser.traverseTreeTopDown(fromLink, boost::bind(&addJointLink, _1), rp, false);
    if (travRet < 0)
    {
        ROS_ERROR("Could not add depenency order");
        p->dependencyOrderedJoints.clear();
        return false;
    }

    result = p->dependencyOrderedJoints;
    return true;
}
