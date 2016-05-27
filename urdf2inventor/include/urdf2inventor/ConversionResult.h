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
