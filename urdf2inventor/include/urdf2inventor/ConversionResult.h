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

#ifndef URDF2INVENTOR_CONVERSIONRESULT_H
#define URDF2INVENTOR_CONVERSIONRESULT_H
// Copyright Jennifer Buehler

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <map>
#include <set>

namespace urdf2inventor
{

/**
 * Parameters for one conversion.
 *
 * \author Jennifer Buehler
 * \date March 2016
 */
class ConversionParameters
{
public:
    typedef Eigen::Transform<double, 3, Eigen::Affine> EigenTransform;
    explicit ConversionParameters(const std::string& _rootLinkName,
                                  const std::string& _material,
                                  const EigenTransform& _addVisualTransform):
        rootLinkName(_rootLinkName),
        material(_material),
        addVisualTransform(_addVisualTransform) {}
    ConversionParameters(const ConversionParameters& o):
        rootLinkName(o.rootLinkName),
        material(o.material),
        addVisualTransform(o.addVisualTransform) {}

    virtual ~ConversionParameters() {}

    // root link where the conversion starts from
    std::string rootLinkName;

    // material for the meshes
    std::string material;

    /**
     * this transform will be post-multiplied on all links' **visuals** (not links!) local
     * transform (their "origin"). This can be used to correct transformation errors which may have been
     * introduced in converting meshes from one format to the other, losing orientation information
     * (for example, .dae has an "up vector" definition which may have been ignored)
     */
    EigenTransform addVisualTransform;

private:
    ConversionParameters() {}
};



/**
 * \brief Encapsulates all result fields for a conversion
 * \author Jennifer Buehler
 * \date November 2015
 */
template<typename MeshFormat>
class ConversionResult
{
public:
    explicit ConversionResult(const std::string& _meshOutputExtension,
                              const std::string& _meshOutputDirectoryName,
                              const std::string& _texOutputDirectoryName):
        meshOutputExtension(_meshOutputExtension),
        meshOutputDirectoryName(_meshOutputDirectoryName),
        texOutputDirectoryName(_texOutputDirectoryName),
        success(false) {}
    ConversionResult(const ConversionResult& o):
        robotName(o.robotName),
        meshes(o.meshes),
        meshOutputExtension(o.meshOutputExtension),
        meshOutputDirectoryName(o.meshOutputDirectoryName),
        texOutputDirectoryName(o.texOutputDirectoryName),
        success(o.success) {}

    virtual ~ConversionResult() {}

    std::string robotName;

    // the resulting meshes (inventor files), indexed by the link name
    std::map<std::string, MeshFormat> meshes;

    /**
     * The texture file names to copy to target directory:
     * The key is the path to a target texture directory. It is either absolute,
     * or if it's relative, it is relative to a global output directory ``<output-dir>``.
     * Value is a list of *absolute* filenames of textures to copy into this directory.
     * So copying all files i ``<mapIterator->second[i]>`` to
     * ``[<output-dir>/]<mapIterator->first>`` will be required when installing
     * the mesh file which is meant to go in ``<output-dir>``/``<file.extension>``.
     */
    std::map<std::string, std::set<std::string> > textureFiles;

    // the extension to use for mesh files (e.g. ".iv" for inventor)
    std::string meshOutputExtension;

    // output directory for all meshes
    std::string meshOutputDirectoryName;

    // output directory for all textures
    std::string texOutputDirectoryName;

    bool success;

private:
    ConversionResult():
        success(false) {}
};

}  //  namespace urdf2inventor
#endif   // URDF2INVENTOR_CONVERSIONRESULT_H
