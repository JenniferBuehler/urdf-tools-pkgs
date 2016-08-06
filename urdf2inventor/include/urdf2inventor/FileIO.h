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
#ifndef URDF2INVENTOR_FILEIO_H
#define URDF2INVENTOR_FILEIO_H
// Copyright Jennifer Buehler

//-----------------------------------------------------
#include <urdf/model.h>
#include <urdf2inventor/ConversionResult.h>
#include <baselib_binding/SharedPtr.h>

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <vector>
#include <map>



namespace urdf2inventor
{

/**
 * \brief Reads and writes URDF and GraspIt! files to disk.
 *
 *  Parameter MeshFormat: Datatype to use for the meshes (e.g. a string for XML).
 * \author Jennifer Buehler
 * \date March 2016
 */
template<typename MeshFormat = std::string>
class FileIO
{
public:
    typedef ConversionResult<MeshFormat> ConversionResultT;
    typedef typename baselib_binding::shared_ptr<ConversionResultT>::type ConversionResultPtr;

    /**
     * \param _outputDir directory where to save the files.
     */
    explicit FileIO(const std::string& _outputDir):
        outputDir(_outputDir) {}

    virtual ~FileIO()
    {
    }

    /**
     * Initializes the directory, then writes the meshes.
     */
    bool write(const ConversionResultPtr& data) const;

    /**
     * Initializes the output directory and does implementation-specific
     * initialization depending on the robot name \e robotName.
     * This method is called from write(), so it should only be used
     * if write() is not called as well.
     */
    bool initOutputDir(const std::string& robotName) const;


protected:
    bool writeMeshFiles(const std::map<std::string, MeshFormat>& meshes,
                        const std::string& MESH_OUTPUT_EXTENSION,
                        const std::string& MESH_OUTPUT_DIRECTORY_NAME) const;
    /**
     * Called from initOutputDir(), can be used by subclassees
     * Will be called after creating the directory \e outputDir.
     */
    virtual bool initOutputDirImpl(const std::string& robotName) const
    {
        return true;
    };

    /**
     * Called from write(ConversionResultPtr&), after initOutputDir() has been called and
     * the meshes (in base class of \e data) has been written with writeMeshFiles().
     * Can be used by subclasses to write other things belonging to the result.
     */
    virtual bool writeImpl(const ConversionResultPtr& data) const
    {
        return true;
    }

    inline const std::string& getOutputDirectory() const
    {
        return outputDir;
    }
private:
    std::string outputDir;
};
}  //  namespace urdf2inventor

#include <urdf2inventor/FileIO.hpp>

#endif   // URDF2INVENTOR_FILEIO_H
