#ifndef URDF_TRANSFORM_JOINFIXEDLINKS_H
#define URDF_TRANSFORM_JOINFIXEDLINKS_H

#include <string>

namespace urdf_traverser
{
    class UrdfTraverser;
}

namespace urdf_transform
{
    /**
     * Recursively removes all fixed links down the chain in the model by adding
     * visuals and collision geometry to the first parent link which is
     * attached to a non-fixed link.
     */
    bool joinFixedLinks(urdf_traverser::UrdfTraverser& traverser, const std::string& fromLink);
}

#endif  // URDF_TRANSFORM_JOINFIXEDLINKS_H
