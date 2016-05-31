/**
    Copyright (C) 2016 Jennifer Buehler

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software Foundation,
    Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
**/

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
