#ifndef URDF2INVENTOR_MESHCONVERTRECURSIONPARAMS
#define URDF2INVENTOR_MESHCONVERTRECURSIONPARAMS

#include <urdf_traverser/RecursionParams.h>
#include <set>
#include <string>
#include <map>

namespace urdf2inventor
{

/**
 * \brief Includes parameters to be passed on in recursion when generating meshes.
 */
template<class MeshFormat>
class MeshConvertRecursionParams: public urdf_traverser::FactorRecursionParams
{
public:
    typedef MeshConvertRecursionParams<MeshFormat> Self;
    typedef typename baselib_binding::shared_ptr<Self>::type Ptr;
    explicit MeshConvertRecursionParams(double _scale_factor, const std::string _material,
                                        const std::string& _extension,
                                        const urdf_traverser::EigenTransform& _addVisualTransform):
        FactorRecursionParams(_scale_factor),
        material(_material),
        extension(_extension),
        addVisualTransform(_addVisualTransform) {}
    MeshConvertRecursionParams(const MeshConvertRecursionParams& o):
        FactorRecursionParams(o),
        material(o.material),
        extension(o.extension),
        resultMeshes(o.resultMeshes),
        addVisualTransform(o.addVisualTransform),
        textureFiles(o.textureFiles) {}
    virtual ~MeshConvertRecursionParams() {}

    // If the material cannot be converted, use this material name instead
    std::string material;

    // When the meshes are written to file, this is the extension they
    // will have.
    std::string extension;

    /**
     * this transform will be post-multiplied on all links' **visuals** (not links!) local
     * transform (their "origin"). This can be used to correct transformation errors which may have been
     * introduced in converting meshes from one format to the other, losing orientation information
     * (for example, .dae has an "up vector" definition which may have been ignored)
     */
    urdf_traverser::EigenTransform addVisualTransform;

    // the resulting meshes (files in output MeshFormat), indexed by their name.
    // The name (map key) may also be a relative path, with the last element
    // being the link name (without file type extension). This will then mean
    // the mesh should be stored in this directory, relative to the output directory.
    // Note: If the resulting meshes reference texture files, they must reference
    // *absolute* file paths (which can be converted in a post-processing step)
    std::map<std::string, MeshFormat> resultMeshes;

    // if the \e resultMeshes have textures, this is a list of textures
    // (absolute file paths) referenced from the meshes.
    // Key is the same as in \e resultMeshes.
    std::map<std::string, std::set<std::string> > textureFiles;
private:
    explicit MeshConvertRecursionParams() {}
};

}  // namespace

#endif   // URDF2INVENTOR_MESHCONVERTRECURSIONPARAMS
