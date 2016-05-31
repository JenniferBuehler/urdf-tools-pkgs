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


