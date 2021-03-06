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
#include <set>

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

/**
 * Ensures this path is in the form such that it refers to a directory.
 * For example (on unix):
 * (1) this/is/a/file while (2) this/is/a/directory/.
 * This method enforces the form (2) for \e path.
 * \param printWarn print a warning if the directory needs to be altered
 */
extern void enforceDirectory(std::string& path, bool printWarn);

/**
 * When \e to is a subdirectory of \e from, the relative path from \e from to \e to
 * is returned. If \e to is not a subdirectory of \e from, false is returned and
 * the result is undefined.
 * If both \e from and \e to are directories, and they are the same,
 * "." is returned for current directory)
 * If \e from is a file and not a directory, an exception is thrown.
 */
extern bool getSubdirPath(const std::string& from, const std::string& to, std::string& result);

/**
 * Determines the common parent path of \e p1 and \e p2.
 * If paths are relative, they are converted to absolute relative to the current directory
 * Returns false if there is no commom path, which means the paths don't have a common root.
 * An empty path is returned only if both paths are relative, and the common parent is the
 * directory they are both relative to.
 */
extern bool getCommonParentPath(const std::string& p1, const std::string& p2, std::string& result);

/**
 * Returns the common parent path of all paths in the set by calling other getCommonParentPath().
 */
extern bool getCommonParentPath(const std::set<std::string>& allFiles, std::string& result);

/**
 * Returns file or directory \e path relative to directory \e relTo.
 * Example: \e path is my/special/file.txt, and \e relTo is target/directory/.
 * Then the result would be ../../my/special/file.txt
 * Both paths either (1) should exist, or (2) be symbolic (not exist),
 * in which case the endings will determine whether they are a directory or not
 * (for example on unix:
 * this/is/a/file while this/is/a/directory/. To enforce a directory, you may use enforceDirectory()).
 *
 * If either \e path or \e relTo are relative paths, they will temporarily be converted to absolute paths
 * within the current directory.
 * \return false if \e path and \e relTo have no common parent path (there has to be at least
 * a common root).
 */
extern bool getRelativeDirectory(const std::string& path, const std::string& relTo, std::string& result);

/**
 * Returns boost::filesystem::path(path).parent_path() if the \e path is a file,
 * and if \e path is a directory, returns the \e path itself
 */
extern std::string getDirectory(const std::string& path);

/**
 * Checks if the path references a directory, ie. it must
 * end with a separator. Does not actually check if the directory
 * or file exists, so this can be used to check paths which
 * are to be created.
 */
extern bool isDirectoryPath(const std::string& path);

/**
 * Returns the name of the directory at the lowest
 * level (eg. name of the directory a file is in, not
 * the whole path - only the directory name).
 */
extern std::string getDirectoryName(const std::string& path);


/**
 * Replace all occurrences of \e from in the string \e text and replace them with \e to
 */
extern std::string replaceAll(const std::string& text, const std::string& from, const std::string& to);

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
