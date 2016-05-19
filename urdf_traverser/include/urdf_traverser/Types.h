#ifndef URDF_TRAVERSER_TYPES_H
#define URDF_TRAVERSER_TYPES_H

#include <architecture_binding/SharedPtr.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <urdf_model/link.h>
#include <urdf_model/joint.h>

namespace urdf_traverser
{
   
typedef Eigen::Transform<double, 3, Eigen::Affine> EigenTransform;
    
typedef architecture_binding::shared_ptr<urdf::Link>::type LinkPtr;
typedef architecture_binding::shared_ptr<const urdf::Link>::type LinkConstPtr;

typedef architecture_binding::shared_ptr<urdf::Joint>::type JointPtr;
typedef architecture_binding::shared_ptr<const urdf::Joint>::type JointConstPtr;

typedef architecture_binding::shared_ptr<urdf::Visual>::type VisualPtr;
typedef architecture_binding::shared_ptr<urdf::Geometry>::type GeometryPtr;
typedef architecture_binding::shared_ptr<urdf::Mesh>::type MeshPtr;
typedef architecture_binding::shared_ptr<urdf::Sphere>::type SpherePtr;
typedef architecture_binding::shared_ptr<urdf::Box>::type BoxPtr;
typedef architecture_binding::shared_ptr<urdf::Cylinder>::type CylinderPtr;
typedef architecture_binding::shared_ptr<urdf::Collision>::type CollisionPtr;


};


#endif  // URDF_TRAVERSER_TYPES_H
