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
#include <urdf2inventor/Helpers.h>

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

// handle for stdout redirect
int stdout_fd;

void urdf2inventor::helpers::resetStdOut()
{
    if (stdout_fd < 0)
    {
        return;
    }
    fflush(stdout);
    if (dup2(stdout_fd, STDOUT_FILENO) < 0)
    {
        ROS_ERROR("Could not restore stdout");
        return;
    }
    close(stdout_fd);

    // setbuf(stdout,NULL);//reset to unnamed buffer
}

// See http://homepage.ntlworld.com/jonathan.deboynepollard/FGA/redirecting-standard-io.html
void urdf2inventor::helpers::redirectStdOut(const char * toFile)
{
    fflush(stdout);

    mode_t mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;
    int file = open(toFile, O_CREAT | O_APPEND | O_WRONLY, mode);
    if (file < 0)
    {
        ROS_ERROR("could not create new output stream %s: %s", toFile, strerror(errno));
        return;
    }
    stdout_fd = dup(STDOUT_FILENO);                // Clone stdout to a new descriptor
    if (dup2(file, STDOUT_FILENO) < 0)
    {
        ROS_ERROR("could not redirect output stream");
        return;      // Change stdout to file
    }
    // close(file);  // stdout is still valid

    // setvbuf(stdout,toString,_IOFBF,stringSize);
}



bool urdf2inventor::helpers::writeFiles(const std::map<std::string, std::set<std::string> >& files, const std::string& outputDir)
{
    bool ret = true;
    boost::filesystem::path _outputDir(boost::filesystem::absolute(outputDir));

    for (std::map<std::string, std::set<std::string> >::const_iterator mit = files.begin(); mit != files.end(); ++mit)
    {
        for (std::set<std::string>::const_iterator tit = mit->second.begin(); tit != mit->second.end(); ++tit)
        {
            boost::filesystem::path fullPath;
            boost::filesystem::path tFile(mit->first);
            if (tFile.is_relative())
            {
                fullPath = _outputDir;
            }
            else
            {
                // outputDir should be a superdirectory of tFile, or print a warning
                std::string testRelParent;
                if (!urdf_traverser::helpers::getSubdirPath(outputDir, tFile.string(), testRelParent))
                {
                    ROS_WARN_STREAM("File " << tFile.string() << " given as absolute path, but it is not in a subdirectory of " << outputDir);
                }
            }
            fullPath /= tFile;
            // ROS_INFO_STREAM("cp "<<*tit<<" "<<fullPath.string());

            // first, create directory if needed
            std::string targetTexDir = urdf_traverser::helpers::getDirectory(fullPath.string());
            if (!urdf_traverser::helpers::makeDirectoryIfNeeded(targetTexDir.c_str()))
            {
                ROS_ERROR_STREAM("Could not create directory " << targetTexDir);
                ret = false;
                continue;
            }

            try
            {
                // copy the file
                // copy_option::fail_if_exists would be proper but annoying to debug
                boost::filesystem::copy_file(*tit, fullPath, boost::filesystem::copy_option::overwrite_if_exists);
            }
            catch (const boost::filesystem::filesystem_error& ex)
            {
                ROS_ERROR_STREAM("Could not copy file: " << ex.what());
                ret = false;
                continue;
            }
        }
    }

    return ret;
}


bool urdf2inventor::helpers::fixFileReferences(
    const std::string& modelDir,
    const std::string& fileDir,
    const std::string& fileRootDir,
    const std::set<std::string>& filesInUse,
    std::string& modelString,
    std::map<std::string, std::set<std::string> >& filesToCopy)
{
    if (filesInUse.empty()) return true;

    std::string _fileDir(fileDir);
    urdf_traverser::helpers::enforceDirectory(_fileDir, false);

    // do error checking first
    std::string testCommonParent;
    if (!urdf_traverser::helpers::getCommonParentPath(filesInUse, testCommonParent))
    {
        ROS_ERROR_STREAM("Could not find common parent path of all files");
        return false;
    }
    // test: testCommonParent must be a subdirectory to fileRootDir, or the same directory.
    std::string testRelParent;
    if (!urdf_traverser::helpers::getSubdirPath(fileRootDir, testCommonParent, testRelParent))
    {
        ROS_ERROR_STREAM("File " << testCommonParent << " is not in a subdirectory of " << fileRootDir);
        return false;
    }
    // end error checking

    // now iterate through all file files and change references in \e modelString
    for (std::set<std::string>::iterator itFile = filesInUse.begin(); itFile != filesInUse.end(); ++itFile)
    {
        std::string absFile = *itFile;
        std::string file;
        if (!urdf_traverser::helpers::getSubdirPath(fileRootDir, absFile, file))
        {
            ROS_ERROR_STREAM("File " << absFile << " is not in a subdirectory of " << fileRootDir);
            continue;
        }
        // ROS_INFO_STREAM("Relative file: "<<file);

        std::stringstream filePath;  // the absolute path the the file
        filePath << _fileDir << file;

        std::string newFileReference;
        if (!urdf_traverser::helpers::getRelativeDirectory(filePath.str(), modelDir, newFileReference))
        {
            ROS_ERROR_STREAM("Could not determine relative directory between " << filePath.str()
                             << " and " << modelDir << ".");
            continue;
        }

        // replace all occurrences in mesh string
        // ROS_INFO_STREAM("Replacing new file reference: "<<newFileReference);

        // first, replace all full filenames with paths
        modelString = urdf_traverser::helpers::replaceAll(modelString,
                      absFile, newFileReference);

        // now, replace all remaining occurrences of the path with a version without
        // path separators (this is only in case there is left-over names made up of the path)
        boost::filesystem::path _absFileMod(absFile);
        _absFileMod.replace_extension("");
        boost::filesystem::path _texRefMod(newFileReference);
        _texRefMod.replace_extension("");
        std::string texRefMod = urdf_traverser::helpers::replaceAll(_texRefMod.string(), "/", "_");
        texRefMod = urdf_traverser::helpers::replaceAll(texRefMod, ".", "_");
        modelString = urdf_traverser::helpers::replaceAll(modelString,
                      _absFileMod.string(), texRefMod);

        // ROS_INFO_STREAM("File to copy to "<<filePath.str()<<" : "<<absFile);
        // add this file to the result set
        filesToCopy[filePath.str()].insert(absFile);
    }
    return true;
}

