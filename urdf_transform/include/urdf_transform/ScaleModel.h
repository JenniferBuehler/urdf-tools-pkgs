#ifndef SCALE_MODEL_H
#define SCALE_MODEL_H

#include <string>

namespace urdf_traverser
{
class UrdfTraverser;
}

namespace urdf_transform
{
/**
 * Calls other scaleModel() function from root node.
 */
extern bool scaleModel(urdf_traverser::UrdfTraverser& traverser, double scale_factor);

/**
 * Scales the URDF model by this factor, starting from \e fromLink.
 * This means all translation parts of the joint transforms are multiplied by this.
 * The mesh files are not touched, but the visual/collision/intertial translations are scaled as well.
 * Meshes can be scaled using convertMeshes().
 */
extern bool scaleModel(urdf_traverser::UrdfTraverser& traverser, const std::string& fromLink, double scale_factor);

}

#endif  // SCALE_MODEL_H
