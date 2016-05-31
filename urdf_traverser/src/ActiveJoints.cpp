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


#include <ros/ros.h>
#include <urdf_traverser/Functions.h>
#include <urdf_traverser/UrdfTraverser.h>
#include <urdf_traverser/ActiveJoints.h>

using urdf_traverser::UrdfTraverser;
using urdf_traverser::RecursionParams;

/**
 * Function which can be used for recursion which returns -1 if there are any inactive joints in the urdf.
 */
int checkActiveJoints(urdf_traverser::RecursionParamsPtr& p)
{
    urdf_traverser::LinkPtr link = p->getLink();
    urdf_traverser::LinkPtr parent = link->getParent();
    unsigned int level = p->getLevel();
    if (level == 0) return 1;
    if (link->parent_joint && !urdf_traverser::isActive(link->parent_joint))
    {
        ROS_INFO("UrdfTraverser: Found fixed joint %s", link->parent_joint->name.c_str());
        return -1;
    }
    return 1;
}

bool urdf_traverser::hasFixedJoints(UrdfTraverser& traverser, const std::string& fromLink)
{
    urdf_traverser::RecursionParamsPtr p(new RecursionParams());
    if (traverser.traverseTreeTopDown(fromLink,
                                      boost::bind(&checkActiveJoints, _1), p, false) < 0)
    {
        return true;
    }
    return false;
}





