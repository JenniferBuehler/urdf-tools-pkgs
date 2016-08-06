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
