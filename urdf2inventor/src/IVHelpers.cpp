#include <urdf2inventor/Helpers.h>
#include <urdf2inventor/IVHelpers.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoCylinder.h>
#include <iostream>

SoTransform * urdf2inventor::getTransform(const urdf2inventor::EigenTransform& eTrans)
{
    SoTransform * transform = new SoTransform();

    SoSFVec3f translation;
    translation.setValue(eTrans.translation().x(), eTrans.translation().y(), eTrans.translation().z());
    transform->translation = translation;

    SoSFRotation rotation;
    Eigen::Quaterniond vQuat(eTrans.rotation());
    rotation.setValue(vQuat.x(), vQuat.y(), vQuat.z(), vQuat.w());
    transform->rotation = rotation;
    return transform;
}

SoSeparator * urdf2inventor::addSubNode(SoNode * addAsChild, SoNode* parent, const urdf2inventor::EigenTransform& eTrans)
{
    SoTransform * transform = getTransform(eTrans);
    return urdf2inventor::addSubNode(addAsChild, parent, transform);
}

SoSeparator * urdf2inventor::addSubNode(SoNode * addAsChild, SoNode* parent, SoTransform * trans)
{
    SoSeparator * sep = dynamic_cast<SoSeparator*>(parent);
    if (!sep)
    {
        std::cerr<<"parent is not a separator"<<std::endl;
        return NULL;
    }

    SoSeparator * sepChild = dynamic_cast<SoSeparator*>(addAsChild);
    if (!sepChild)
    {
        std::cerr<<"child is not a separator"<<std::endl;
        return NULL;
    }

    // ROS_WARN_STREAM("######### Adding transform "<<trans->translation<<", "<<trans->rotation);

    SoSeparator * transNode = new SoSeparator();
    transNode->addChild(trans);
    transNode->addChild(sepChild);

    sep->addChild(transNode);
    return sep;
}

/*void addSubNode(SoSeparator * addToNode, SoNode * visual, const Eigen::Vector3d& pos,
    const Eigen::Quaterniond& rot, SoMaterial * mat, bool transBeforeRot)
{
    SoSeparator * transSep = new SoSeparator();
    if (transBeforeRot)
    {
        SoTransform * trans = new SoTransform();
        trans->translation.setValue(pos.x(), pos.y(), pos.z());
        SoSeparator * rotTransSep = new SoSeparator();
        SoTransform * rotTrans = new SoTransform();
        trans->rotation.setValue(rot.x(), rot.y(), rot.z(), rot.w());
        rotTransSep->addChild(rotTrans);
        transSep->addChild(rotTrans); 
    }
    else
    {
        SoTransform * trans = new SoTransform();
        trans->translation.setValue(pos.x(), pos.y(), pos.z());
        trans->rotation.setValue(rot.x(), rot.y(), rot.z(), rot.w());
        transSep->addChild(trans);
    }
    transSep->addChild(visual);
    if (mat) addToNode->addChild(mat);
    addToNode->addChild(transSep);
}
*/


void urdf2inventor::addSubNode(SoSeparator * addToNode, SoNode * visual,
    const EigenTransform& transform,
    SoMaterial * mat)
{
#if 0 
    SoTransform * trans = new SoTransform();
    Eigen::Vector3d pos(transform.translation());
    Eigen::Quaterniond rot(transform.rotation());
    // std::cout<<"Translation: "<<pos<<", rotation="<<rot<<std::endl;
    trans->translation.setValue(pos.x(), pos.y(), pos.z());
    trans->rotation.setValue(rot.x(), rot.y(), rot.z(), rot.w());
#else
    SoMatrixTransform * trans = new SoMatrixTransform();
    EigenTransform t=transform;//.inverse();
/*    SbMatrix m( t(0,0), t(0,1), t(0,2), t(0,3),
                t(1,0), t(1,1), t(1,2), t(1,3),
                t(2,0), t(2,1), t(2,2), t(2,3),
                t(3,0), t(3,1), t(3,2), t(3,3) );
*/
    //SbMatrix m( t(0,0), t(1,0), t(2,0), t(3,0), 
    trans->matrix.setValue(t(0,0), t(1,0), t(2,0), t(3,0), 
                t(0,1), t(1,1), t(2,1), t(3,1), 
                t(0,2), t(1,2), t(2,2), t(3,2), 
                t(0,3), t(1,3), t(2,3), t(3,3));

    //trans->setMatrix(m);
#endif

    SoSeparator * transSep = new SoSeparator();
    transSep->addChild(trans);
    transSep->addChild(visual);
    if (mat) addToNode->addChild(mat);
    addToNode->addChild(transSep);
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
    mat->diffuseColor.setValue(r,g,b);
    mat->ambientColor.setValue(0.2, 0.2, 0.2);
    mat->transparency.setValue(a);
    addSubNode(addToNode, cube, trans, mat);
}


void urdf2inventor::addSphere(SoSeparator * addToNode, const Eigen::Vector3d& pos, float radius,
    float r, float g, float b, float a)
{
    SoSphere * s = new SoSphere();
    s->radius = radius;
    SoMaterial * mat = new SoMaterial();
    mat->diffuseColor.setValue(r,g,b);
    mat->ambientColor.setValue(0.2, 0.2, 0.2);
    mat->transparency.setValue(a);
    EigenTransform trans;
    trans.setIdentity();
    trans.translate(pos);
    addSubNode(addToNode, s, trans, mat);
}

void urdf2inventor::addCylinder(SoSeparator * addToNode, const Eigen::Vector3d& pos,
    const Eigen::Quaterniond& rot,
    float radius, float height,
    float r, float g, float b, float a)
{
    SoCylinder * c = new SoCylinder();
    c->radius = radius;
    c->height = height;
    
    SoMaterial * mat = new SoMaterial();
    mat->diffuseColor.setValue(r,g,b);
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
    Eigen::Quaterniond toZ = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0,1,0), Eigen::Vector3d(0,0,1));
    EigenTransform trans;
    trans.setIdentity();
    trans.translate(pos);
    trans.rotate(rot);
    trans.translate(Eigen::Vector3d(0,0,height/2.0));
    trans.rotate(toZ);

    addSubNode(addToNode, c, trans, mat);
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
    addCylinder(addToNode,Eigen::Vector3d(0,0,0), rot, axesRadius, axesLength ,rz,gz,bz);
    
    Eigen::Vector3d x(1,0,0);
    Eigen::Vector3d y(0,1,0);
    Eigen::Vector3d z(0,0,1);

    // y axis
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(z, y);
    addCylinder(addToNode,Eigen::Vector3d(0,0,0), q, axesRadius, axesLength, ry, gy, by);
    
    // x axis
    q = Eigen::Quaterniond::FromTwoVectors(z, x);
    addCylinder(addToNode,Eigen::Vector3d(0,0,0), q, axesRadius, axesLength, rx, gx, bx);
}
