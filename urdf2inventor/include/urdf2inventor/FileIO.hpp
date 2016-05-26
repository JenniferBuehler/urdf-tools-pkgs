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
#include <urdf2inventor/Helpers.h>
#include <boost/filesystem.hpp>

template<typename MeshFormat>
bool urdf2inventor::FileIO<MeshFormat>::initOutputDir(const std::string& robotName) const
{
    return urdf_traverser::helpers::makeDirectoryIfNeeded(outputDir.c_str())
           && initOutputDirImpl(robotName);
}

template<typename MeshFormat>
bool urdf2inventor::FileIO<MeshFormat>::writeMeshFiles(const std::map<std::string, MeshFormat>& meshes,
        const std::string& meshOutputExtension,
        const std::string& meshOutputDirectoryName) const
{
    std::string outputMeshDir =  outputDir + "/" + meshOutputDirectoryName;

    if (!urdf_traverser::helpers::makeDirectoryIfNeeded(outputMeshDir.c_str()))
    {
        ROS_ERROR("Could not create directory %s", outputMeshDir.c_str());
        return false;
    }

    ROS_INFO_STREAM("urdf2inventor::FileIO::writeMeshFiles into "<<outputDir);

    // write the mesh files
    typename std::map<std::string, MeshFormat>::const_iterator mit;
    for (mit = meshes.begin(); mit != meshes.end(); ++mit)
    {
        std::stringstream outFilename;
        outFilename << outputMeshDir << "/" << mit->first << meshOutputExtension;

        std::string pathToFile=urdf_traverser::helpers::getPath(outFilename.str().c_str()); 
        //ROS_INFO_STREAM("Directory of path "<<outFilename.str()<<": "<<pathToFile);
        if (!pathToFile.empty() &&
            !urdf_traverser::helpers::makeDirectoryIfNeeded(pathToFile.c_str()))
        {
            ROS_ERROR_STREAM("Could not make directory "<<pathToFile);
        }

        if (!urdf_traverser::helpers::writeToFile(mit->second, outFilename.str()))
        {
            ROS_ERROR("Could not write file %s", outFilename.str().c_str());
            return false;
        }
    }

    return true;
}



template<typename MeshFormat>
bool urdf2inventor::FileIO<MeshFormat>::write(const ConversionResultPtr& data) const
{
    // First of all, see if we can create output directory
    if (!initOutputDir(data->robotName))
    {
        ROS_ERROR("Can't initialize output directory %s", outputDir.c_str());
        return false;
    }

    if (!writeMeshFiles(data->meshes, data->meshOutputExtension, data->meshOutputDirectoryName))
    {
        ROS_ERROR("Could not write mesh files");
        return false;
    }
       
    urdf2inventor::helpers::writeTextures(data->textureFiles, outputDir); 
    return writeImpl(data);
}
