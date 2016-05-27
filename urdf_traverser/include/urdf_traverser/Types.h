#ifndef URDF_TRAVERSER_TYPES_H
#define URDF_TRAVERSER_TYPES_H

#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <urdf/model.h>
#include <urdf_model/link.h>
#include <urdf_model/joint.h>

namespace urdf_traverser
{

typedef Eigen::Transform<double, 3, Eigen::Affine> EigenTransform;

typedef boost::shared_ptr<urdf::Link> LinkPtr;
typedef boost::shared_ptr<const urdf::Link> LinkConstPtr;

typedef boost::shared_ptr<urdf::Joint> JointPtr;
typedef boost::shared_ptr<const urdf::Joint> JointConstPtr;

typedef boost::shared_ptr<urdf::Visual> VisualPtr;
typedef boost::shared_ptr<urdf::Geometry> GeometryPtr;
typedef boost::shared_ptr<urdf::Mesh> MeshPtr;
typedef boost::shared_ptr<urdf::Sphere> SpherePtr;
typedef boost::shared_ptr<urdf::Box> BoxPtr;
typedef boost::shared_ptr<urdf::Cylinder> CylinderPtr;
typedef boost::shared_ptr<urdf::Collision> CollisionPtr;
typedef boost::shared_ptr<urdf::Inertial> InertialPtr;
typedef boost::shared_ptr<urdf::Material> MaterialPtr;

typedef boost::shared_ptr<urdf::Model> ModelPtr;
typedef boost::shared_ptr<const urdf::Model> ModelConstPtr;



};


#endif  // URDF_TRAVERSER_TYPES_H
