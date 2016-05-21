#ifndef URDF2INVENTOR_MESHCONVERTRECURSIONPARAMS
#define URDF2INVENTOR_MESHCONVERTRECURSIONPARAMS

#include <urdf_traverser/RecursionParams.h>

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
                               const std::string& _out_extension,
                               const urdf_traverser::EigenTransform& _addVisualTransform):
        FactorRecursionParams(_scale_factor),
        material(_material),
        out_extension(_out_extension),
        addVisualTransform(_addVisualTransform) {}
    MeshConvertRecursionParams(const MeshConvertRecursionParams& o):
        FactorRecursionParams(o),
        material(o.material),
        out_extension(o.out_extension),
        resultMeshes(o.resultMeshes),
        addVisualTransform(o.addVisualTransform) {}
    virtual ~MeshConvertRecursionParams() {}

    // If the material cannot be converted, use this material name instead
    std::string material;

    // When the meshes are written to file, this is the extension they
    // will have.
    std::string out_extension;

    /** 
     * this transform will be post-multiplied on all links' **visuals** (not links!) local
     * transform (their "origin"). This can be used to correct transformation errors which may have been 
     * introduced in converting meshes from one format to the other, losing orientation information
     * (for example, .dae has an "up vector" definition which may have been ignored)
     */
    urdf_traverser::EigenTransform addVisualTransform;

    // the resulting meshes (inventor files), indexed by the link name
    std::map<std::string, MeshFormat> resultMeshes;
private:
    explicit MeshConvertRecursionParams(){}
};

}  // namespace

#endif   // URDF2INVENTOR_MESHCONVERTRECURSIONPARAMS
