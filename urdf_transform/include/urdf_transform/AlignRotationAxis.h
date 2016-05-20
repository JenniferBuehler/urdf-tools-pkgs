#ifndef URDF_TRANSFORM_ALIGNROTATIONAXIS_H
#define URDF_TRANSFORM_ALIGNROTATIONAXIS_H

#include <string>

namespace urdf_traverser
{
    class UrdfTraverser;
}

namespace urdf_transform
{

    /**
     * Transform the URDF such that all rotation axises (in the joint's local reference frame) are this axis
     */
    bool allRotationsToAxis(urdf_traverser::UrdfTraverser& traverser, const std::string& fromLinkName, const Eigen::Vector3d& axis);
}

#endif  // URDF_TRANSFORM_ALIGNROTATIONAXIS_H
