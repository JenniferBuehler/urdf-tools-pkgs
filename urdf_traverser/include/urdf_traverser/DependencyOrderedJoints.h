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
