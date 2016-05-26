#ifndef URDF2INVENTOR_CONVERTMESH_H
#define URDF2INVENTOR_CONVERTMESH_H

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
    template<class MeshFormat>
    bool convertMeshes(urdf_traverser::UrdfTraverser& traverser,
                       const std::string& fromLinkName,
                       const float scaleFactor,
                       const std::string& material,
                       const std::string& file_extension,
                       const urdf_traverser::EigenTransform& addVisualTransform,
                       std::map<std::string, MeshFormat>& meshes,
                       std::map<std::string, std::set<std::string> >& textureFiles);

    /**
     *
     * Changes the texture references (absolute paths) 
     * in the mesh string descriptions so they reference the
     * file in the actual output directory with a relative path.
     *
     * Texture output directory is the relative directory \e relTexDir, e.g. "tex", which is created as {install-prefix}/{relTexDir}.
     * The mesh output directory \e relMeshDir is also relative, e.g. "iv", which is then installed to {install-prefix}/{relMeshDir}.
     * Each mesh can be stored in its own subdirectory, {install-prefix}/{relMeshDir}/{mesh-path-i}. This subdirectory is detemined
     * by the key name in \e meshes.
     *
     * What is done by this method:
     * All textures have a common parent directory {common-tex} (worst case it's the root).
     * So all texture paths i are of the form {common-tex}/{tex-path-i}.
     * All textures are installed in {install-prefix}/{relTexDir}/{tex-path-i}.
     * The relative distance between {mesh-path-i} and {tex-path-i} is used to reference textures
     * from within the mesh files, e.g. "../tex/texture.png".
     *
     * \param relTargetMeshDir when writing the mesh files to disk, and the installation destination is {install-prefix},
     *      all mesh files will be put in {install-prefix}/relTargetMeshDir. This string can be empty.
     * \param relTargetTexDir when writing the mesh files to disk, and the installation destination is {install-prefix},
     *      all texture files will be put in {install-prefix}/relTargetTexDir. This string can be empty.
     * \param textureFiles the texture files, as returned by convertMehes().
     * \param meshes the meshes (inventor files) to fix, as resulted from convertMeshes(). Only works for string mesh format.
     * \param texturesToCopy The texture file names to copy to target directories:
     *      Key is the *relative* path to the texture directory, which will be located in the output directory.
     *      Value is a list of *absolute* filenames of textures into copy in this directory.
     *      So copying all files i {mapIterator->second[i]} to {output-dir}/{mapIterator->first} will be required when installing
     *      the inventor file {output-dir}/{file.iv}.
     * \return false if no common parent directory could be determined for all of the texture files
     */
    bool fixTextureReferences(const std::string& relMeshDir,
                       const std::string& relTexDir,
                       const std::map<std::string, std::set<std::string> >& textureFiles,
                       std::map<std::string, std::string>& meshes,
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
     */
    SoNode * getAllVisuals(const urdf_traverser::LinkPtr link, double scale_factor,
        const urdf_traverser::EigenTransform& addTransform,
        bool scaleUrdfTransforms = false);

    /**
     * Removes all texture copies in the nodes.
     */
    void removeTextureCopies(SoNode * root);


}  // namespace
#endif  // URDF2INVENTOR_CONVERTMESH_H
