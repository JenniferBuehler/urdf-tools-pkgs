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
#include <urdf_transform/ScaleModel.h>
#include <urdf_traverser/UrdfTraverser.h>
#include <urdf_traverser/Functions.h>
#include <ros/ros.h>

using urdf_traverser::UrdfTraverser;

/**
 * Function used for recursion by scaleModel().
 */
int scaleModelFunc(urdf_traverser::RecursionParamsPtr& p)
{
    urdf_traverser::FactorRecursionParamsPtr param =
        baselib_binding_ns::dynamic_pointer_cast<urdf_traverser::FactorRecursionParams>(p);
    if (!param)
    {
        ROS_ERROR("Wrong recursion parameter type");
        return -1;
    }

    urdf_traverser::LinkPtr link = param->getLink();
    if (!link)
    {
        ROS_ERROR("Recursion parameter must have initialised link!");
        return -1;
    }
    urdf_traverser::scaleTranslation(link, param->factor);
    urdf_traverser::JointPtr pjoint = link->parent_joint;
    if (pjoint)
    {
        urdf_traverser::scaleTranslation(pjoint, param->factor);
    }
    return 1;
}

bool urdf_transform::scaleModel(UrdfTraverser& traverser, const std::string& fromLink, double scale_factor)
{
    // do one call of scaleModel(RecursionParams) for the root link
    urdf_traverser::RecursionParamsPtr p(new urdf_traverser::FactorRecursionParams(scale_factor));
    return traverser.traverseTreeTopDown(fromLink, boost::bind(&scaleModelFunc, _1), p, true) == 1;
}

bool urdf_transform::scaleModel(UrdfTraverser& traverser, double scale_factor)
{
    std::string root_link = traverser.getRootLinkName();
    return scaleModel(traverser, root_link, scale_factor);
}


