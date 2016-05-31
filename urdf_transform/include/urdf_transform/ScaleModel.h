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


#ifndef SCALE_MODEL_H
#define SCALE_MODEL_H

#include <string>

namespace urdf_traverser
{
class UrdfTraverser;
}

namespace urdf_transform
{
/**
 * Calls other scaleModel() function from root node.
 */
extern bool scaleModel(urdf_traverser::UrdfTraverser& traverser, double scale_factor);

/**
 * Scales the URDF model by this factor, starting from \e fromLink.
 * This means all translation parts of the joint transforms are multiplied by this.
 * The mesh files are not touched, but the visual/collision/intertial translations are scaled as well.
 * Meshes can be scaled using convertMeshes().
 */
extern bool scaleModel(urdf_traverser::UrdfTraverser& traverser, const std::string& fromLink, double scale_factor);

}

#endif  // SCALE_MODEL_H
