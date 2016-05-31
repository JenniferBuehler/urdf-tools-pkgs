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


#ifndef URDF_TRANSFORM_JOINFIXEDLINKS_H
#define URDF_TRANSFORM_JOINFIXEDLINKS_H

#include <string>

namespace urdf_traverser
{
class UrdfTraverser;
}

namespace urdf_transform
{
/**
 * Recursively removes all fixed links down the chain in the model by adding
 * visuals and collision geometry to the first parent link which is
 * attached to a non-fixed link.
 */
bool joinFixedLinks(urdf_traverser::UrdfTraverser& traverser, const std::string& fromLink);
}

#endif  // URDF_TRANSFORM_JOINFIXEDLINKS_H
