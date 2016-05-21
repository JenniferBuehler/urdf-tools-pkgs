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
     * \param meshes the resulting meshes (inventor files), indexed by the link names
     * \param meshDescXML the resulting GraspIt! XML description files for the meshes, indexed by the link names
     * \param addVisualTransform this transform will be post-multiplied on all links' **visuals** (not links!) local
     *      transform (their "origin"). This can be used to correct transformation errors which may have been 
     *      introduced in converting meshes from one format to the other, losing orientation information
     *      (for example, .dae has an "up vector" definition which may have been ignored)
     */
    template<class MeshFormat>
    bool convertMeshes(urdf_traverser::UrdfTraverser& traverser,
                       const std::string& fromLinkName,
                       const float scaleFactor,
                       const std::string& material,
                       const urdf_traverser::EigenTransform& addVisualTransform,
                       std::map<std::string, MeshFormat>& meshes);


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

}  // namespace
#endif  // URDF2INVENTOR_CONVERTMESH_H
