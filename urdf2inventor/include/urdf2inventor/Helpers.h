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

#ifndef URDF2INVENTOR_HELPERS_H
#define URDF2INVENTOR_HELPERS_H
// Copyright Jennifer Buehler

/**
 * Helper functions for i/o operations.
 * \author Jennifer Buehler
 * \date last edited October 2015
 */

#include <iostream>
#include <string>

#include <urdf_traverser/Helpers.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <urdf/model.h>

namespace urdf2inventor
{

namespace helpers
{

/**
 * ################################################################################################################
 * Minor helpers (e.g. file writing)
 * \author Jennifer Buehler
 * \date last edited October 2015
 * ################################################################################################################
 */

extern void resetStdOut();

// see http://homepage.ntlworld.com/jonathan.deboynepollard/FGA/redirecting-standard-io.html
extern void redirectStdOut(const char * toFile);

}  //  namespace helpers
}  //  namespace urdf2inventor

#endif  // URDF2INVENTOR_HELPERS_H
