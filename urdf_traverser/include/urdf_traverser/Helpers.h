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

#ifndef URDF_TRAVERSER_HELPERS_H
#define URDF_TRAVERSER_HELPERS_H
// Copyright Jennifer Buehler

/**
 * Helper functions for i/o operations.
 * \author Jennifer Buehler
 * \date last edited October 2015
 */

#include <iostream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <urdf/model.h>

namespace urdf_traverser
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

// returns filename extension without the dot. Make sure first that this is a file.
extern std::string fileExtension(const char* file);

extern bool directoryExists(const char* dPath);

extern std::string getPath(const char * file);

extern bool fileExists(const char* file);

std::string getFilename(const char* file);

std::string getFilenameWithoutExtension(const char* file);

extern bool makeDirectoryIfNeeded(const char * dPath);

extern bool writeToFile(const std::string& content, const std::string& filename);
extern void deleteFile(const char* file);

extern void findAndReplace(const std::string& newStr, const std::string& oldStr, const std::string& in, std::string& out);

// transforms a path specification in the form package://<package-name>/<path> to an absolute path on the computer
extern std::string packagePathToAbsolute(std::string& packagePath);

}  //  namespace helpers
}  //  namespace urdf_traverser

extern std::ostream& operator<<(std::ostream& o, const Eigen::Vector3d& v);

extern std::ostream& operator<<(std::ostream& o, const Eigen::Vector3f& v);

extern std::ostream& operator<<(std::ostream& o, const Eigen::Quaterniond& v);


extern std::ostream& operator<<(std::ostream& o, const Eigen::Transform<double, 3, Eigen::Affine>& t);

extern std::ostream& operator<<(std::ostream& o, const Eigen::Matrix4d& m);

extern std::ostream& operator<<(std::ostream& o, const urdf::Pose& p);


extern std::ostream& operator<<(std::ostream& o, const urdf::Vector3& p);

#endif  // URDF_TRAVERSER_HELPERS_H
