/**
 * <ORGANIZATION> = Jennifer Buehler 
 * <COPYRIGHT HOLDER> = Jennifer Buehler 
 * 
 * Copyright (c) 2016 Jennifer Buehler 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ------------------------------------------------------------------------------
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
