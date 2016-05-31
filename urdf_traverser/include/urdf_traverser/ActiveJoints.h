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

#ifndef URDF_TRAVERSER_ACTIVEJOINTS_H
#define URDF_TRAVERSER_ACTIVEJOINTS_H

#include <string>

namespace urdf_traverser
{
class UrdfTraverser;

/**
 * returns true if there are any fixed joints down from from_link
 * Only joints *after* the given link are returned.
 */
bool hasFixedJoints(UrdfTraverser& traverser, const std::string& fromLink);
}

#endif  // URDF_TRAVERSER_ACTIVEJOINTS_H
