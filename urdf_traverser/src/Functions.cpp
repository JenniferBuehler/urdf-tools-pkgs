#include <ros/ros.h>
#include <urdf_traverser/Functions.h>


bool urdf_traverser::isActive(const JointPtr& joint)
{
    if (!joint) return false;
    return (joint->type == urdf::Joint::REVOLUTE) ||
           (joint->type == urdf::Joint::CONTINUOUS) ||
           (joint->type == urdf::Joint::PRISMATIC);
}

Eigen::Vector3d urdf_traverser::getRotationAxis(const JointConstPtr& j)
{
    return Eigen::Vector3d(j->axis.x, j->axis.y, j->axis.z);
}

bool urdf_traverser::scaleTranslation(JointPtr& joint, double scale_factor)
{
    EigenTransform vTrans = getTransform(joint);
    scaleTranslation(vTrans, scale_factor);
    setTransform(vTrans, joint);
}

void urdf_traverser::scaleTranslation(LinkPtr& link, double scale_factor)
{
    for (std::vector<VisualPtr >::iterator vit = link->visual_array.begin();
            vit != link->visual_array.end(); ++vit)
    {
        VisualPtr visual = *vit;
        EigenTransform vTrans = getTransform(visual->origin);
        scaleTranslation(vTrans, scale_factor);
        setTransform(vTrans, visual->origin);
    }


    for (std::vector<CollisionPtr >::iterator cit = link->collision_array.begin();
            cit != link->collision_array.end(); ++cit)
    {
        CollisionPtr coll = *cit;
        EigenTransform vTrans = getTransform(coll->origin);
        scaleTranslation(vTrans, scale_factor);
        setTransform(vTrans, coll->origin);
    }
    if (!link->inertial)
    {
        // ROS_WARN("Link %s  has no inertial",link->name.c_str());
        return;
    }

    EigenTransform vTrans = getTransform(link->inertial->origin);
    scaleTranslation(vTrans, scale_factor);
    setTransform(vTrans, link->inertial->origin);
}

void urdf_traverser::scaleTranslation(EigenTransform& t, double scale_factor)
{
    Eigen::Vector3d trans = t.translation();
    trans *= scale_factor;
    Eigen::Matrix3d rot = t.rotation();
    t.setIdentity();
    t.translate(trans);
    t.rotate(rot);
}

void urdf_traverser::setTransform(const EigenTransform& t, urdf::Pose& p)
{
    Eigen::Vector3d trans(t.translation());
    Eigen::Quaterniond rot(t.rotation());

    p.position.x = trans.x();
    p.position.y = trans.y();
    p.position.z = trans.z();
    p.rotation.x = rot.x();
    p.rotation.y = rot.y();
    p.rotation.z = rot.z();
    p.rotation.w = rot.w();
}

void urdf_traverser::applyTransform(const EigenTransform& t, urdf::Vector3& v)
{
    Eigen::Vector3d _v(v.x, v.y, v.z);
    Eigen::Quaterniond rot(t.rotation());
    //ROS_INFO_STREAM("Rotation: "<<rot);
    _v = rot * _v;
    v.x = _v.x();
    v.y = _v.y();
    v.z = _v.z();
}

void urdf_traverser::setTransform(const EigenTransform& t, JointPtr& joint)
{
    setTransform(t, joint->parent_to_joint_origin_transform);
}


urdf_traverser::EigenTransform urdf_traverser::getTransform(const JointConstPtr& joint)
{
    return getTransform(joint->parent_to_joint_origin_transform);
}

urdf_traverser::EigenTransform urdf_traverser::getTransform(const LinkConstPtr& link)
{
    return getTransform(link->parent_joint);
}

urdf_traverser::EigenTransform urdf_traverser::getTransform(const LinkConstPtr& from_link,  const LinkConstPtr& to_link)
{
    return EigenTransform(getTransformMatrix(from_link, to_link));
}

urdf_traverser::EigenTransform urdf_traverser::getTransform(const urdf::Pose& p)
{
    urdf::Vector3 _jtr = p.position;
    Eigen::Vector3d jtr(_jtr.x, _jtr.y, _jtr.z);
    urdf::Rotation _jrot = p.rotation;
    Eigen::Quaterniond jrot(_jrot.w, _jrot.x, _jrot.y, _jrot.z);
    jrot.normalize();
    EigenTransform tr;
    tr.setIdentity();
    tr = tr.translate(jtr);
    tr = tr.rotate(jrot);
    return tr;
}



Eigen::Matrix4d urdf_traverser::getTransformMatrix(const LinkConstPtr& from_link,  const LinkConstPtr& to_link)
{
    if (from_link->name == to_link->name) return Eigen::Matrix4d::Identity();

    std::vector<JointPtr> pjoints = getChain(from_link, to_link);

    if (pjoints.empty())
    {
        ROS_ERROR("could not get chain from %s to %s", from_link->name.c_str(), to_link->name.c_str());
        return Eigen::Matrix4d::Identity();
    }

    // ROS_INFO("Chain from %s to %s",from_link->name.c_str(),to_link->name.c_str());

    Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();

    for (std::vector<JointPtr>::iterator it = pjoints.begin(); it != pjoints.end(); ++it)
    {
        // ROS_INFO("Chain joint %s",(*it)->name.c_str());
        Eigen::Matrix4d mat = getTransform(*it).matrix();
        ret *= mat;
    }
    return ret;
}

bool urdf_traverser::applyTransform(JointPtr& joint, const EigenTransform& trans, bool preMult)
{
    EigenTransform vTrans = getTransform(joint);
    if (preMult) vTrans = trans * vTrans;
    else vTrans = vTrans * trans;
    setTransform(vTrans, joint);
}

void urdf_traverser::applyTransform(LinkPtr& link, const EigenTransform& trans, bool preMult)
{
    // ROS_INFO("applying transform to link %s",link->name.c_str());

    for (std::vector<VisualPtr >::iterator vit = link->visual_array.begin();
            vit != link->visual_array.end(); ++vit)
    {
        VisualPtr visual = *vit;
        EigenTransform vTrans = getTransform(visual->origin);
        // ROS_INFO_STREAM("a visual for link"<<link->name<<" with transform "<<vTrans);
        if (preMult) vTrans = trans * vTrans;
        else vTrans = vTrans * trans;
        setTransform(vTrans, visual->origin);
    }


    for (std::vector<CollisionPtr >::iterator cit = link->collision_array.begin();
            cit != link->collision_array.end(); ++cit)
    {
        CollisionPtr coll = *cit;
        EigenTransform vTrans = getTransform(coll->origin);
        if (preMult) vTrans = trans * vTrans;
        else vTrans = vTrans * trans;
        setTransform(vTrans, coll->origin);
    }

    EigenTransform vTrans = getTransform(link->inertial->origin);
    if (preMult) vTrans = trans * vTrans;
    else vTrans = vTrans * trans;
    setTransform(vTrans, link->inertial->origin);
}

std::vector<urdf_traverser::JointPtr> urdf_traverser::getChain(const LinkConstPtr& from_link, const LinkConstPtr& to_link)
{
    std::vector<JointPtr> chain;

    if (to_link->name == from_link->name) return chain;

    LinkConstPtr curr = to_link;
    LinkConstPtr pl = to_link->getParent();

    while (curr && (curr->name != from_link->name))
    {
        // ROS_INFO_STREAM("Chain: "<<curr->name);
        JointPtr pj = curr->parent_joint;
        if (!pj)
        {
            ROS_ERROR("UrdfTraverser: End of chain at link '%s'", curr->name.c_str());
            return std::vector<JointPtr>();
        }
        chain.push_back(pj);
        curr = pl;
        pl = curr->getParent();
        // if (pl) ROS_INFO_STREAM("Parent: "<<pl->name);
    }
    if (curr->name != from_link->name)
    {
        ROS_ERROR_STREAM("UrdfTraverser: could not find link " <<
                         from_link->name << " while traversing up the chain starting from " <<
                         to_link->name << ". Failed to find parent chain!");
        return std::vector<JointPtr>();
    }

    std::reverse(chain.begin(), chain.end());

    return chain;
}


bool urdf_traverser::isChildOf(const LinkConstPtr& parent, const LinkConstPtr& child)
{
    for (unsigned int i = 0; i < parent->child_links.size(); ++i)
    {
        LinkPtr childLink = parent->child_links[i];
        if (childLink->name == child->name) return true;
    }
    return false;
}

bool urdf_traverser::isChildJointOf(const LinkConstPtr& parent, const JointConstPtr& joint)
{
    for (unsigned int i = 0; i < parent->child_joints.size(); ++i)
    {
        JointPtr childJnt = parent->child_joints[i];
        if (childJnt->name == joint->name) return true;
    }
    return false;
}

bool equalAxes(const Eigen::Vector3d& z1, const Eigen::Vector3d& z2, double tolerance)
{
    Eigen::Vector3d _z1 = z1;
    Eigen::Vector3d _z2 = z2;
    _z1.normalize();
    _z2.normalize();
    double dot = _z1.dot(_z2);
    return (std::fabs(dot - 1.0)) < tolerance;
}


bool urdf_traverser::jointTransformForAxis(const JointConstPtr& joint,
        const Eigen::Vector3d& axis, Eigen::Quaterniond& rotation)
{
    Eigen::Vector3d rotAxis(joint->axis.x, joint->axis.y, joint->axis.z);
    if (rotAxis.norm() < 1e-06) return false;
    rotAxis.normalize();
    // ROS_INFO_STREAM("Rotation axis for joint "<<joint.name<<": "<<rotAxis);
    if (equalAxes(rotAxis, axis, 1e-06)) return false;

    rotation = Eigen::Quaterniond::FromTwoVectors(rotAxis, axis);
    // ROS_WARN_STREAM("z alignment: "<<rotation);
    return true;
}



