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

#include <urdf_traverser/Helpers.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <fcntl.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <string>


/**
 * ################################################################################################################
 * Minor helpers (e.g. file writing)
 * \author Jennifer Buehler
 * \date last edited October 2015
 * ################################################################################################################
 */

void urdf_traverser::helpers::findAndReplace(const std::string& newStr, const std::string& oldStr, const std::string& in, std::string& out)
{
    size_t pos = 0;
    out = in;
    while ((pos = out.find(oldStr, pos)) != std::string::npos)
    {
        out = out.replace(pos, oldStr.length(), newStr);
        pos += newStr.length();
    }
}

void urdf_traverser::helpers::deleteFile(const char* file)
{
    std::remove(file);
}

bool urdf_traverser::helpers::fileExists(const char* file)
{
    return boost::filesystem::exists(file);
}

std::string urdf_traverser::helpers::getFilename(const char* file)
{
    boost::filesystem::path dPath(file);
    return dPath.filename().string();
}

std::string urdf_traverser::helpers::getFilenameWithoutExtension(const char* file)
{
    boost::filesystem::path dPath(file);
    return dPath.stem().string();
}


std::string urdf_traverser::helpers::fileExtension(const char* file)
{
    boost::filesystem::path dPath(file);
    std::string extension = dPath.extension().string();
    extension.erase(0, 1);
    return extension;
}

bool urdf_traverser::helpers::directoryExists(const char* dPath)
{
    // using boost
    return boost::filesystem::exists(dPath);
}

std::string urdf_traverser::helpers::getPath(const char * file)
{
    boost::filesystem::path dPath(file);
    return dPath.parent_path().string();
}

bool urdf_traverser::helpers::makeDirectoryIfNeeded(const char * dPath)
{
    if (directoryExists(dPath)) return true;
    try
    {
        boost::filesystem::path dir(dPath);
        boost::filesystem::path buildPath;

        for (boost::filesystem::path::iterator it(dir.begin()), it_end(dir.end()); it != it_end; ++it)
        {
            buildPath /= *it;
            //std::cout << buildPath << std::endl;

            if (!boost::filesystem::exists(buildPath) &&
                    !boost::filesystem::create_directory(buildPath))
            {
                ROS_ERROR_STREAM("Could not create directory " << buildPath);
                return false;
            }
        }
    }
    catch (const boost::filesystem::filesystem_error& ex)
    {
        ROS_ERROR_STREAM(ex.what());
        return false;
    }
    return true;
}

bool urdf_traverser::helpers::isDirectoryPath(const std::string& path)
{
    if (path.empty()) return false;
    if ((path == ".") || (path == "..")) return true;
    return path[path.length() - 1] == boost::filesystem::path::preferred_separator;
}

bool urdf_traverser::helpers::getSubdirPath(const std::string& from, const std::string& to, std::string& result)
{
    if (!isDirectoryPath(from))
    {
        ROS_ERROR_STREAM("Base path (" << from << ") must be a directory");
        throw std::exception();
    }

    if (from == to)
    {
        result = ".";
        return true;
    }

    boost::filesystem::path _from(from);
    boost::filesystem::path _to(to);

    boost::filesystem::path _absFrom = boost::filesystem::absolute(_from);
    boost::filesystem::path _absTo = boost::filesystem::absolute(_to);

    boost::filesystem::path::iterator it(_absFrom.begin());
    boost::filesystem::path::iterator it_end(_absFrom.end());
    boost::filesystem::path::iterator it2(_absTo.begin());
    boost::filesystem::path::iterator it2_end(_absTo.end());
    --it_end;  // don't go to last entry, which will be either a '.' or a file
    for (; it != it_end; ++it, ++it2)
    {
        if (it2 == it2_end) return false; // cannot be proper sub-directory
        if (it->string() != it2->string())
        {
            // ROS_INFO_STREAM("Comp: "<<it->string()<<", "<<it2->string());
            return false;
        }
    }

    boost::filesystem::path buildPath;
    for (; it2 != it2_end; ++it2)
    {
        buildPath /= *it2;
    }
    result = buildPath.string();
    // if the path was a directory, remove the last '.' which was inserted
    if (!result.empty() && (result[result.length() - 1] == '.'))
        result.erase(result.length() - 1, 1);

    // ROS_INFO_STREAM("PATH RESULT: "<<result);

    boost::filesystem::path _res(result);
    if (!_res.is_relative())
    {
        ROS_ERROR_STREAM("Could not correctly construct a relative path, got "
                         << result << " (input: " << from << " and " << to << ")");
        return false;
    }
    return true;
}

void urdf_traverser::helpers::enforceDirectory(std::string& path, bool printWarn)
{
    if (path.empty()) return;
    if (!isDirectoryPath(path))
    {
        if (printWarn)
            ROS_WARN_STREAM("Path " << path << " supposed to be a directory but does not end with separator. Enforcing.");
        path.append(1, boost::filesystem::path::preferred_separator);
    }
    assert(isDirectoryPath(path));
}


std::string urdf_traverser::helpers::replaceAll(const std::string& text, const std::string& from, const std::string& to)
{
    // ROS_INFO_STREAM("Replace "<<from<<" with "<<to);
    std::string ret = text;
    for (size_t start_pos = ret.find(from); start_pos != std::string::npos; start_pos = ret.find(from, start_pos))
    {
        // ROS_INFO_STREAM("DO replace "<<from<<" with "<<to);
        ret.replace(start_pos, from.length(), to);
    }
    return ret;
}

int numDirectories(const std::string& path)
{
    // ROS_INFO_STREAM("cnt of path "<<path);
    int cnt = 0;
    boost::filesystem::path _path(path);
    boost::filesystem::path::iterator it(_path.begin());
    boost::filesystem::path::iterator it_end(_path.end());
    for (; it != it_end; ++it)
    {
        // ROS_INFO_STREAM(it->string());
        // if (it->string()==".") continue;  // because "./" leads to a count of 1 too but should be 0
        ++cnt;
    }
    if (cnt > 0)
    {
        // count is always one more, as last directory
        // is either '.', or it is a file which doesn't count
        --cnt;
    }
    // ROS_INFO_STREAM("Res: "<<cnt);
    return cnt;
}

std::string urdf_traverser::helpers::getDirectory(const std::string& path)
{
    if (urdf_traverser::helpers::isDirectoryPath(path)) return path; // already ends with '/'
    // does not end with '/' so it must be a file: get the parent
    boost::filesystem::path _path(path);
    std::string ret = _path.parent_path().string();
    // enforce notation to be a directory as we now know it is one
    urdf_traverser::helpers::enforceDirectory(ret, false);
    return ret;
}

std::string urdf_traverser::helpers::getDirectoryName(const std::string& path)
{
    std::string dir = getDirectory(path);
    // dir will now be a directory (enforced to end with / by getDirectory()).
    // So parent_path will return path to the parent directory where it looks
    // like it was a file.
    boost::filesystem::path ret(dir);
    return ret.parent_path().filename().string();
}



bool urdf_traverser::helpers::getCommonParentPath(const std::string& p1, const std::string& p2, std::string& result)
{
    if (p1.empty() || p2.empty())
    {
        ROS_ERROR("Both p1 and p2 have to be set in getCommonParentPath()");
        return false;
    }
    boost::filesystem::path __p1(p1);
    boost::filesystem::path __p2(p2);
    bool correctAbsolute = __p1.is_relative() || __p2.is_relative();

    boost::filesystem::path _p1(boost::filesystem::absolute(__p1));
    boost::filesystem::path _p2(boost::filesystem::absolute(__p2));
    boost::filesystem::path buildPath;

    boost::filesystem::path::iterator it1(_p1.begin());
    boost::filesystem::path::iterator it1_end(_p1.end());
    boost::filesystem::path::iterator it2(_p2.begin());
    boost::filesystem::path::iterator it2_end(_p2.end());

    int p1Len = numDirectories(p1);
    int p2Len = numDirectories(p2);
    // ROS_INFO_STREAM("p1: "<<p1<<", p2: "<<p2);
    // ROS_INFO_STREAM("p1: "<<p1Len<<", p2: "<<p2Len);
    if (p2Len > p1Len)
    {
        //swap around paths
        boost::filesystem::path::iterator tmp(it2);
        it2 = it1;
        it1 = tmp;
        tmp = it2_end;
        it2_end = it1_end;
        it1_end = tmp;
    }

    --it1_end;  // make sure the iteration goes only until the previous-last
    // entry, because the last entry is either '.' or a filename
    for (; it1 != it1_end; ++it1, ++it2)
    {
        // ROS_INFO_STREAM("Comp "<<it1->string()<<", 2 "<<it2->string());
        if ((it2 == it2_end)
                || (*it1 != *it2))
        {
            break;
        }

        // append the directory
        buildPath /= *it1;

        // std::cout << buildPath.string() << std::endl;
    }
    if (!buildPath.empty())
    {
        result = buildPath.string();
        // make sure notation ends with separator.
        // Last element *will* be directory as no files
        // have been added in the loop.
        enforceDirectory(result, false);
        // If the current directory was used to build an absolute path, remove it again.
        if (correctAbsolute)
        {
            std::string currDir = boost::filesystem::current_path().string();
            urdf_traverser::helpers::enforceDirectory(currDir, false);
            if (!urdf_traverser::helpers::getSubdirPath(currDir, result, result))
            {
                ROS_ERROR_STREAM("Could not remove temporarily created current directory for absolute path");
                return false;
            }
        }
        return true;
    }
    return false;
}



bool urdf_traverser::helpers::getCommonParentPath(const std::set<std::string>& allFiles, std::string& result)
{
    if (allFiles.empty())
    {
        ROS_ERROR("Cannot get common path of empty set");
        return false;
    }
    std::set<std::string>::const_iterator it = allFiles.begin();
    std::string commonPath = urdf_traverser::helpers::getDirectory(*it);
    ++it;
    // ROS_INFO_STREAM("Init cPath: "<<commonPath);
    while (it != allFiles.end())
    {
        std::string dir = urdf_traverser::helpers::getDirectory(*it);
        if (!getCommonParentPath(commonPath, dir, commonPath))
        {

            ROS_ERROR_STREAM("There is no root between " <<
                             dir << " and " << commonPath << ", cannot determine common parent!");
        }
        // ROS_INFO_STREAM("CPath: "<<commonPath);
        ++it;
    }
    result = commonPath;
    return true;
}

bool urdf_traverser::helpers::getRelativeDirectory(const std::string& path, const std::string& relTo, std::string& result)
{
    // ROS_INFO_STREAM("Get relative dir of "<<path<<" to "<<relTo);

    boost::filesystem::path _path(boost::filesystem::absolute(path));
    boost::filesystem::path _relTo(boost::filesystem::absolute(relTo));

    std::string commonParent;
    if (!getCommonParentPath(path, relTo, commonParent))
    {
        ROS_ERROR_STREAM("Directories " << path << " and " << relTo << " have no common parent directory.");
        return false;
    }
    // ROS_INFO_STREAM("Common parent: "<<commonParent);

    // get both directories relative to the common parent directory:

    std::string relPath;
    if (!urdf_traverser::helpers::getSubdirPath(commonParent, path, relPath))
    {
        ROS_ERROR_STREAM("The file " << path << " is not in a subdirectory of " << commonParent);
        return false;
    }
    // ROS_INFO_STREAM("Path relative to common: "<<relPath);

    std::string relToTarget;
    if (!urdf_traverser::helpers::getSubdirPath(commonParent, relTo, relToTarget))
    {
        ROS_ERROR_STREAM("Relative path " << relTo << " is not a subdirectory of " << commonParent);
        return false;
    }
    // ROS_INFO_STREAM("relTo relative to common: "<<relToTarget);

    // make sure this is a directory, not a file (remove filename if that's the case)
    std::string relToDir = getDirectory(relToTarget);
    // ROS_INFO_STREAM("relTo relative to common: "<<relToDir);

    // Go up in the hierarchie the number of directories in \e relTo which it takes
    // to get to the common parent directory
    int relToLen = numDirectories(relToDir);
    // ROS_INFO_STREAM("Num dirs in "<<relToTarget<<"("<<relToDir<<"):"<<relToLen);
    std::stringstream upDirs;
    for (int i = 0; i < relToLen; ++i)
        upDirs << ".." << boost::filesystem::path::preferred_separator;

    // append the relative path from common parent dir to \e path
    upDirs << relPath;

    result = upDirs.str();

    return true;
}

bool urdf_traverser::helpers::writeToFile(const std::string& content, const std::string& filename)
{
    std::string dir = getDirectory(filename);
    if (!urdf_traverser::helpers::makeDirectoryIfNeeded(dir.c_str()))
    {
        return false;
    }

    std::ofstream outf(filename.c_str());
    if (!outf)
    {
        ROS_ERROR("%s could not be opened for writing!", filename.c_str());
        return false;
    }
    outf << content;
    outf.close();
    return true;
}


// transforms a path specification in the form package://<package-name>/<path> to an absolute path on the computer
std::string urdf_traverser::helpers::packagePathToAbsolute(std::string& packagePath)
{
    // ROS_INFO("We have a mesh %s",packagePath.c_str());
    char pack[1000];
    char rest[1000];
    int numScanned = sscanf(packagePath.c_str(), "package://%[^/]/%s", pack, rest);
    // ROS_INFO("Pack: %s Rest: %s",pack,rest);
    if (numScanned != 2)
    {
        ROS_ERROR("Only package:// style mesh file specifications supported!");
        return std::string();
    }

    std::string packPath = ros::package::getPath(pack);

    if (packPath.empty())  // path, if found, should if it's returned by ros::package
    {
        ROS_ERROR("No package for file specified");
        return std::string();
    }

    std::stringstream absolute;
    absolute << packPath << "/" << rest;

    return absolute.str();
}


std::ostream& operator<<(std::ostream& o, const Eigen::Vector3f& v)
{
    o << "[" << v[0] << ", " << v[1] << ", " << v[2] << "]";
    return o;
}


std::ostream& operator<<(std::ostream& o, const Eigen::Vector3d& v)
{
    o << "[" << v[0] << ", " << v[1] << ", " << v[2] << "]";
    return o;
}
std::ostream& operator<<(std::ostream& o, const Eigen::Quaterniond& v)
{
    o << "[" << v.x() << ", " << v.y() << ", " << v.z() << ", " << v.w() << "]";
    return o;
}

std::ostream& operator<<(std::ostream& o, const Eigen::Transform<double, 3, Eigen::Affine>& t)
{
    //  o<<"T: trans="<<t.translation()<<" rot="<<Eigen::Quaterniond(t.rotation());
    Eigen::AngleAxisd ax(t.rotation());
    o << "T: trans=" << Eigen::Vector3d(t.translation()) << " rot=" << ax.angle() << " / " << ax.axis();
    return o;
}

std::ostream& operator<<(std::ostream& o, const Eigen::Matrix4d& m)
{
    o << m(0, 0) << "," << m(0, 1) << "," << m(0, 2) << "," << m(0, 3) << "," <<
      m(1, 0) << "," << m(1, 1) << "," << m(1, 2) << "," << m(1, 3) << "," <<
      m(2, 0) << "," << m(2, 1) << "," << m(2, 2) << "," << m(2, 3) << "," <<
      m(3, 0) << "," << m(3, 1) << "," << m(3, 2) << "," << m(3, 3);
    return o;
}


std::ostream& operator<<(std::ostream& o, const urdf::Pose& p)
{
    o << "trans=[" << p.position.x << ", " << p.position.y << ", " << p.position.z << "], ";
    o << "rot=[" << p.rotation.x << ", " << p.rotation.y << ", " << p.rotation.z << ", " << p.rotation.w << "]";
    return o;
}

std::ostream& operator<<(std::ostream& o, const urdf::Vector3& p)
{
    o << "v=[" << p.x << ", " << p.y << ", " << p.z << "], ";
    return o;
}
