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


#include <urdf2inventor/Helpers.h>
#include <urdf2inventor/IVHelpers.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoNode.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoCylinder.h>
#include <Inventor/nodes/SoTexture2.h>


#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/actions/SoSearchAction.h>

#include <boost/filesystem.hpp>

#include <iostream>

bool urdf2inventor::writeInventorFileString(SoNode * node, std::string& result)
{
    SoOutput out;
    out.setBinary(false);
    size_t initBufSize = 100;
    void * buffer = malloc(initBufSize * sizeof(char));
    out.setBuffer(buffer, initBufSize, std::realloc);
    SoWriteAction write(&out);
    write.apply(node);

    void * resBuf = NULL;
    size_t resBufSize = 0;

    if (!out.getBuffer(resBuf, resBufSize) || (resBufSize == 0))
    {
        return false;
    }

    result = std::string(static_cast<char*>(resBuf), resBufSize);  // buffer will be copied

    free(resBuf);

    return true;
}




/**
 * Searches through all nodes and returns a set of absolute paths to textures
 * which are in use.
 */
std::set<std::string> urdf2inventor::getAllTexturePaths(SoNode * root)
{
    std::set<std::string> allFiles;
    SoSearchAction sa;
    sa.setType(SoTexture2::getClassTypeId());
    sa.setInterest(SoSearchAction::ALL);
    sa.setSearchingAll(TRUE);
    sa.apply(root);
    SoPathList & pl = sa.getPaths();

    for (int i = 0; i < pl.getLength(); i++)
    {
        SoFullPath * p = (SoFullPath*) pl[i];
        if (!p->getTail()->isOfType(SoTexture2::getClassTypeId())) continue;

        SoTexture2 * tex = (SoTexture2*) p->getTail();
        if (tex->filename.getValue().getLength() == 0) continue;

        std::string name(tex->filename.getValue().getString());
        boost::filesystem::path absPath(boost::filesystem::absolute(name));
        allFiles.insert(absPath.string());
    }
    sa.reset();
    return allFiles;
}

urdf2inventor::EigenTransform urdf2inventor::getEigenTransform(const SbMatrix& m)
{
    Eigen::Matrix3d em;
    /*em(0,0)=m[0][0];
    em(0,1)=m[0][1];
    em(0,2)=m[0][2];
    em(0,3)=m[0][3];
    em(1,0)=m[1][0];
    em(1,1)=m[1][1];
    em(1,2)=m[1][2];
    em(1,3)=m[1][3];
    em(2,0)=m[2][0];
    em(2,1)=m[2][1];
    em(2,2)=m[2][2];
    em(2,3)=m[2][3];
    em(3,0)=m[3][0];
    em(3,1)=m[3][1];
    em(3,2)=m[3][2];
    em(3,3)=m[3][3];*/

    em(0,0)=m[0][0];
    em(0,1)=m[1][0];
    em(0,2)=m[2][0];
    em(0,3)=m[3][0];
    em(1,0)=m[0][1];
    em(1,1)=m[1][1];
    em(1,2)=m[2][1];
    em(1,3)=m[3][1];
    em(2,0)=m[0][2];
    em(2,1)=m[1][2];
    em(2,2)=m[2][2];
    em(2,3)=m[3][2];
    em(3,0)=m[0][3];
    em(3,1)=m[1][3];
    em(3,2)=m[2][3];
    em(3,3)=m[3][3];
    
    return urdf2inventor::EigenTransform(em);
}

SbMatrix urdf2inventor::getSbMatrix(const urdf2inventor::EigenTransform& m)
{
    SbMatrix sm;
    /*sm[0][0]=m(0,0);
    sm[0][1]=m(0,1);
    sm[0][2]=m(0,2);
    sm[0][3]=m(0,3);
    sm[1][0]=m(1,0);
    sm[1][1]=m(1,1);
    sm[1][2]=m(1,2);
    sm[1][3]=m(1,3);
    sm[2][0]=m(2,0);
    sm[2][1]=m(2,1);
    sm[2][2]=m(2,2);
    sm[2][3]=m(2,3);
    sm[3][0]=m(3,0);
    sm[3][1]=m(3,1);
    sm[3][2]=m(3,2);
    sm[3][3]=m(3,3);*/

    sm[0][0]=m(0,0);
    sm[0][1]=m(1,0);
    sm[0][2]=m(2,0);
    sm[0][3]=m(3,0);
    sm[1][0]=m(0,1);
    sm[1][1]=m(1,1);
    sm[1][2]=m(2,1);
    sm[1][3]=m(3,1);
    sm[2][0]=m(0,2);
    sm[2][1]=m(1,2);
    sm[2][2]=m(2,2);
    sm[2][3]=m(3,2);
    sm[3][0]=m(0,3);
    sm[3][1]=m(1,3);
    sm[3][2]=m(2,3);
    sm[3][3]=m(3,3);
    
    return sm;
}


SoTransform * getSoTransform(const urdf2inventor::EigenTransform& eTrans)
{
    SoTransform * transform = new SoTransform();

    transform->setMatrix(urdf2inventor::getSbMatrix(eTrans));
/*
    SoSFVec3f translation;
    translation.setValue(eTrans.translation().x(), eTrans.translation().y(), eTrans.translation().z());
    transform->translation = translation;

    SoSFRotation rotation;
    Eigen::Quaterniond vQuat(eTrans.rotation());
    rotation.setValue(vQuat.x(), vQuat.y(), vQuat.z(), vQuat.w());
    transform->rotation = rotation;*/

    return transform;
}

SoSeparator * urdf2inventor::addSubNode(SoNode * addAsChild,
                                        SoNode* parent, const urdf2inventor::EigenTransform& eTrans)
{
    SoTransform * transform = getSoTransform(eTrans);
    return urdf2inventor::addSubNode(addAsChild, parent, transform);
}

SoSeparator * urdf2inventor::addSubNode(SoNode * addAsChild,
                                        SoNode* parent, SoTransform * trans)
{
    SoSeparator * sep = dynamic_cast<SoSeparator*>(parent);
    if (!sep)
    {
        std::cerr << "parent is not a separator" << std::endl;
        return NULL;
    }

    SoSeparator * sepChild = dynamic_cast<SoSeparator*>(addAsChild);
    if (!sepChild)
    {
        std::cerr << "child is not a separator" << std::endl;
        return NULL;
    }

    // ROS_WARN_STREAM("######### Adding transform "<<trans->translation<<", "<<trans->rotation);

    SoSeparator * transNode = new SoSeparator();
    transNode->addChild(trans);
    transNode->addChild(sepChild);

    sep->addChild(transNode);
    return sep;
}


void urdf2inventor::addSubNode(SoNode * addAsChild, SoSeparator * parent,
                               const EigenTransform& transform,
                               SoMaterial * mat)
{
    SoMatrixTransform * trans = new SoMatrixTransform();
    EigenTransform t = transform; //.inverse();
    trans->matrix.setValue(t(0, 0), t(1, 0), t(2, 0), t(3, 0),
                           t(0, 1), t(1, 1), t(2, 1), t(3, 1),
                           t(0, 2), t(1, 2), t(2, 2), t(3, 2),
                           t(0, 3), t(1, 3), t(2, 3), t(3, 3));


    SoSeparator * transSep = new SoSeparator();
    transSep->addChild(trans);
    transSep->addChild(addAsChild);
    if (mat) parent->addChild(mat);
    parent->addChild(transSep);
}


void urdf2inventor::addBox(SoSeparator * addToNode, const EigenTransform& trans,
                           float width, float height, float depth,
                           float r, float g, float b, float a)
{
    SoCube * cube = new SoCube();
    cube->width.setValue(width);
    cube->height.setValue(height);
    cube->depth.setValue(depth);
    SoMaterial * mat = new SoMaterial();
    mat->diffuseColor.setValue(r, g, b);
    mat->ambientColor.setValue(0.2, 0.2, 0.2);
    mat->transparency.setValue(a);
    addSubNode(cube, addToNode, trans, mat);
}


void urdf2inventor::addSphere(SoSeparator * addToNode, const Eigen::Vector3d& pos, float radius,
                              float r, float g, float b, float a)
{
    SoSphere * s = new SoSphere();
    s->radius = radius;
    SoMaterial * mat = new SoMaterial();
    mat->diffuseColor.setValue(r, g, b);
    mat->ambientColor.setValue(0.2, 0.2, 0.2);
    mat->transparency.setValue(a);
    EigenTransform trans;
    trans.setIdentity();
    trans.translate(pos);
    addSubNode(s, addToNode, trans, mat);
}

void urdf2inventor::addCylinder(SoSeparator * addToNode, 
                                const urdf2inventor::EigenTransform& extraTrans,
                                float radius, float height,
                                float r, float g, float b, float a)
{
    SoCylinder * c = new SoCylinder();
    c->radius = radius;
    c->height = height;

    SoMaterial * mat = new SoMaterial();
    mat->diffuseColor.setValue(r, g, b);
    mat->ambientColor.setValue(0.2, 0.2, 0.2);
    mat->transparency.setValue(a);

    // SoCylinder is oriented along y axis, so change this to z axis
    // and also translate such that it extends along +z
    /*    Eigen::Quaterniond toZ = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0,1,0), Eigen::Vector3d(0,0,1));
        typedef Eigen::Transform<double, 3, Eigen::Affine> EigenTransform;
        SoTransform * trans = new SoTransform();
        trans->translation.setValue(0,0,height/2);
        trans->rotation.setValue(toZ.x(), toZ.y(), toZ.z(), toZ.w());
        SoSeparator * cylinder = new SoSeparator();
        cylinder->addChild(trans);
        cylinder->addChild(c);
        EigenTransform trans;
        trans.setIdentity();
        trans.translate(pos);
    */

    // SoCylinder is oriented along y axis, so change this to z axis
    // and also translate such that it extends along +z
    Eigen::Quaterniond toZ = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(0, 0, 1));
    EigenTransform trans = extraTrans;
    trans.translate(Eigen::Vector3d(0, 0, height / 2.0));
    trans.rotate(toZ);

    addSubNode(c, addToNode, trans, mat);
}



void urdf2inventor::addCylinder(SoSeparator * addToNode, const Eigen::Vector3d& pos,
                                const Eigen::Quaterniond& rot,
                                float radius, float height,
                                float r, float g, float b, float a)
{
    EigenTransform trans;
    trans.setIdentity();
    trans.translate(pos);
    trans.rotate(rot);
    addCylinder(addToNode, trans, radius, height, r, g, b, a);
}

void urdf2inventor::addLocalAxes(SoSeparator * addToNode, float axesRadius, float axesLength)
{
    float rx, gx, bx, ry, gy, by, rz, gz, bz;
    rx = ry = rz = gx = gy = gz = bx = by = bz = 0;
    rx = 1;  // x axis red
    gy = 1;  // y axis green
    bz = 1;  // z axis blue

    Eigen::Quaterniond rot;
    rot.setIdentity();
    // z axis
    addCylinder(addToNode, Eigen::Vector3d(0, 0, 0), rot, axesRadius, axesLength , rz, gz, bz);

    Eigen::Vector3d x(1, 0, 0);
    Eigen::Vector3d y(0, 1, 0);
    Eigen::Vector3d z(0, 0, 1);

    // y axis
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(z, y);
    addCylinder(addToNode, Eigen::Vector3d(0, 0, 0), q, axesRadius, axesLength, ry, gy, by);

    // x axis
    q = Eigen::Quaterniond::FromTwoVectors(z, x);
    addCylinder(addToNode, Eigen::Vector3d(0, 0, 0), q, axesRadius, axesLength, rx, gx, bx);
}
