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
#ifndef URDF2INVENTOR_CONVERTMESH_H
#define URDF2INVENTOR_CONVERTMESH_H

#include <urdf2inventor/MeshConvertRecursionParams.h>
#include <urdf_traverser/Types.h>

// this is a temporary filename (extension .iv) which is needed for internal usage, but can be deleted after execution.
#define TMP_FILE_IV "/tmp/urdf2inventor_tmp.iv"

namespace urdf_traverser
{
class UrdfTraverser;
}

namespace urdf2inventor
{
/**
 * Convert all meshes starting from fromLinkName into the inventor format, and store them in the given
 * mesh files container.
 * While converting, the mesh files can be scaled by the scale factor set in the constructor.
 *
 * Note that this template function is only instantiated for MeshFormat=std::string at this point,
 * which means returned meshes can be represented in a string (eg. XML format).
 *
 * \param scaleFactor a factor to scale the mesh by while converting
 * \param material the material to use in the converted format
 * \param file_extension when the meshes are written to file, this is the extension they will have.
 *      This information may be required for generating the result meshes.
 * \param addVisualTransform this transform will be post-multiplied on all links' **visuals** (not links!) local
 *      transform (their "origin"). This can be used to correct transformation errors which may have been
 *      introduced in converting meshes from one format to the other, losing orientation information
 *      (for example, .dae has an "up vector" definition which may have been ignored)
 * \param meshes the resulting meshes (inventor files), indexed by the link names
 * \param textureFiles if the \e resultMeshes have textures, this is a list of textures
 *      (absolute file paths) referenced from the meshes. Key is the same as in \e resultMeshes.
 */
/*template<class MeshFormat>
bool convertMeshesSimple(urdf_traverser::UrdfTraverser& traverser,
                   const std::string& fromLinkName,
                   const float scaleFactor,
                   const std::string& material,
                   const std::string& file_extension,
                   const urdf_traverser::EigenTransform& addVisualTransform,
                   std::map<std::string, MeshFormat>& meshes,
                   std::map<std::string, std::set<std::string> >& textureFiles);
*/

/**
 * Convert all meshes starting from fromLinkName into the inventor format, and store them in the given
 * mesh files container.
 * Results will be in \e meshParams.
 *
 * Note that this template function is only instantiated for MeshFormat=std::string at this point,
 * which means returned meshes can be represented in a string (eg. XML format).
 */
template <class MeshFormat>
bool convertMeshes(urdf_traverser::UrdfTraverser& traverser,
                   const std::string& fromLink,
                   const typename urdf2inventor::MeshConvertRecursionParams<MeshFormat>::Ptr& meshParams);


/**
 * Changes the texture references (absolute paths) in the model string descriptions so they reference the
 * file in the output directory with a *relative* path.
 * This is achieved by calling \e urdf2inventor::helpers::fixFileReferences() for each entry of \e textureFiles and \e models
 * respectively. For documentation of parameters, please refer to this method.
 *
 * The common parent directory will be determined to be the one which is parent to *all* textures used
 * for *all* models.
 *
 * Texture output directory is the relative directory \e relTexDir, e.g. "tex/", which is created as ``<install-prefix>/<relTexDir>``.
 * The model output directory \e relModelDir is also relative, e.g. "iv/", which is then installed to ``<install-prefix>/<relModelDir>``.
 * Each model can be stored in its own subdirectory, ``<install-prefix>/<relModelDir>/<model-path-i>``. This subdirectory ``<model-path-i>
 * is detemined by its *key* name in the map \e models.
 *
 * Implementation background:
 * All textures have a common parent directory ``<common-tex>`` (worst case it's the root).
 * So all texture paths i are of the form ``<common-tex>/<tex-path-i>``.
 * All textures are installed in ``<install-prefix>/<relTexDir>/<tex-path-i>``.
 * The relative distance between ``<model-path-i>`` and ``<tex-path-i>`` is used to reference textures
 * from within the model files, e.g. "../tex/texture.png".
 *
 * \param textureFiles the texture files, as returned by convertMehes().
 * \param models the models incl. meshes (inventor files) to fix, as returned by convertMeshes().
 *      This method only works for the string model format (eg. XML representation of model).
 * \param texturesToCopy The texture file names to copy to target directories. See also urdf2inventor::helpers::fixFileReferences().
 * \param texturesToCopy as output \e filesToCopy in urdf2inventor::helpers::fixFileReferences().
 * \return false if no common parent directory could be determined for all of the texture files
 */
bool fixTextureReferences(const std::string& relModelDir,
                          const std::string& relTexDir,
                          const std::map<std::string, std::set<std::string> >& textureFiles,
                          std::map<std::string, std::string>& models,
                          std::map<std::string, std::set<std::string> >& texturesToCopy);



/**
 * Get the mesh from link, scale it up by scale_factor, and pack it into an SoNode which is also respects the
 * scale_factor in its translations
 * \param scaleUrdfTransforms set to true if the transforms coming from this urdf model should be scaled up as well.
 * If this is false, only the meshes are scaled.
 * \param addTransform this transform will be post-multiplied on all links' **visuals** (not links!) local
 *      transform (their "origin"). This can be used to correct transformation errors which may have been
 *      introduced in converting meshes from one format to the other, losing orientation information
 *      (for example, .dae has an "up vector" definition which may have been ignored)
 * \param useVisuals true to use the visuals as base for the geometry, false if the collision geometry should be used instead.
 */
SoNode * getAllGeometry(const urdf_traverser::LinkPtr link, double scale_factor,
                       const urdf_traverser::EigenTransform& addTransform,
                       const bool useVisuals,
                       const bool scaleUrdfTransforms); // default: false

/**
 * Removes all texture copies in the nodes.
 */
void removeTextureCopies(SoNode * root);


}  // namespace
#endif  // URDF2INVENTOR_CONVERTMESH_H
