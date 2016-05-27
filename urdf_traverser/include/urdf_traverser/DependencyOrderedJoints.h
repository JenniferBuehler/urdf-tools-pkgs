#ifndef URDF_TRAVERSER_DEPENDENCYORDEREDJOINTS_H
#define URDF_TRAVERSER_DEPENDENCYORDEREDJOINTS_H

#include <string>
#include <urdf_traverser/Types.h>

namespace urdf_traverser
{
class UrdfTraverser;

/**
 * Returns all joints starting from fromJoint (including \e fromJoint) within the tree.
 * This is obtained by depth-first traversal,
 * so all joints in the result won't depend on any joints further back in the result set.
 */
extern bool getDependencyOrderedJoints(urdf_traverser::UrdfTraverser& traverser,
                                       std::vector<JointPtr>& result, const JointPtr& fromJoint,
                                       bool allowSplits = true, bool onlyActive = true);

/**
 * Returns all joints down from \e fromLink within the tree. This is obtained by depth-first traversal,
 * so all joints in the result won't depend on any joints further back in the result set.
 */
extern bool getDependencyOrderedJoints(urdf_traverser::UrdfTraverser& traverser,
                                       std::vector<JointPtr>& result, const std::string& fromLink,
                                       bool allowSplits = true, bool onlyActive = true);

}

#endif  // URDF_TRAVERSER_DEPENDENCYORDEREDJOINTS_H
