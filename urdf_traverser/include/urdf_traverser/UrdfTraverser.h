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

#ifndef URDF2INVENTOR_URDF2INVENTOR_H
#define URDF2INVENTOR_URDF2INVENTOR_H
// Copyright Jennifer Buehler

//-----------------------------------------------------

#include <urdf/model.h>

#include <iostream>
#include <string>
#include <map>
#include <vector>


#include <urdf_traverser/Types.h>
#include <urdf_traverser/RecursionParams.h>


namespace urdf_traverser
{

/**
 * \brief This class provides functions to traverse the robot URDF and provides convenience functions to access the URDF model.
 *
 * Traversal through a model is always done on links. This is because the root of a model is always a link.
 * Traversal through joint space is also possible by using the link's parent joint as reference.
 *
 * \author Jennifer Buehler
 * \date May 2016
 */
class UrdfTraverser
{
public:

    /**
     */
    explicit UrdfTraverser()
    {}

    ~UrdfTraverser()
    {
    }

    /**
     * Reads the URDF file from the filename into \e xml_string. Does not change
     * anything in the UrdfTraverser object.
     */
    bool getModelFromFile(const std::string& urdfFilename, std::string& xml_string) const;
   
    /**
     * Loads the URDF model from file
     */ 
    bool loadModelFromFile(const std::string& urdfFilename);

    /**
     * Loads the URDF from a file
     */
    bool loadModelFromXMLString(const std::string& xmlString);

    /**
     * Loads the URDF from parameter server
     * TODO: still need to re-activate this by also reading the
     * robot urdf into field \e robot_urdf
     */
    //bool loadModelFromParameterServer(); 

    /**
     * Removes all fixed links in the model by adding visuals and collision geometry to the first parent link which is
     * attached to a non-fixed link. Model has to be loaded with any of the load() methods first. 
     */
    bool joinFixedLinks(const std::string& from_link);

    /**
     * Transform the URDF such that all rotation axises (in the joint's local reference frame) are this axis
     */
    bool allRotationsToAxis(const std::string& fromLinkName, const Eigen::Vector3d& axis);

    /**
     * Prints the structure of the URDF to standard out
     */
    bool printModel(bool verbose=true);

    /**
     * prints the URDF model to standard out, starting from this link.
     */
    bool printModel(const std::string& fromLink, bool verbose=true);

    /**
     * Prints the joint names. Can't be const in this version because
     * of the use of a recursive method.
     */
    void printJointNames(const std::string& fromLink);

    /**
     * Returns all joint names in depth-frist search order starting from \e fromLink (or from root if
     * \e fromLink is empty)
     */
    bool getJointNames(const std::string& fromLink, const bool skipFixed, std::vector<std::string>& result);

    std::string getRootLinkName() const;
   
    std::string getRobotName() const
    { 
        return this->robot.getName();
    }

    std::string getRobotURDF() const
    {
        return robot_urdf;
    }

    /**
     * Traverses the tree starting from link (depth first) and calls link_cb on each link.
     * \param includeLink include the link itself. If false, will start to traverse starting from the link's children.
     * \param link_cb returns -1 or 0 if traversal is to be stopped, otherwise 1. -1 is for stop because error,
     * 0 is stop because an expected condition found
     * \return -1 or 0 if traversal was stopped, otherwise 1. -1 is for stop because error, 0 is stop because an expected
     * condition found in callback function
     */
    int traverseTreeTopDown(const std::string& linkName, boost::function< int(RecursionParamsPtr&)> link_cb,
                            RecursionParamsPtr& params, bool includeLink=true);
    /**
     * Similar to traverseTreeTopDown(), but traverses bottom-up and is allows to re-link tree (by traversing it safely such
     * that changes in structure won't matter).
     * \param includeLink include the link itself. If false, will start to traverse starting from the link's children.
     */
    int traverseTreeBottomUp(const std::string& linkName, boost::function< int(RecursionParamsPtr&)> link_cb,
                            RecursionParamsPtr& params, bool includeLink=true);
protected:

    /**
     * \brief Recursion data for getting a list of joints, ordered by dependency (no joint depending on others
     * will come before them in the result vector)
     * \author Jennifer Buehler
     */
    class OrderedJointsRecursionParams: public RecursionParams
    {
    public:
        typedef boost::shared_ptr<OrderedJointsRecursionParams> Ptr;
        OrderedJointsRecursionParams(): RecursionParams() {}
        OrderedJointsRecursionParams(bool _allowSplits, bool _onlyActive):
            allowSplits(_allowSplits),
            onlyActive(_onlyActive) {}
        OrderedJointsRecursionParams(const OrderedJointsRecursionParams& o):
            RecursionParams(o),
            allowSplits(o.allowSplits),
            onlyActive(o.onlyActive),
            dependencyOrderedJoints(o.dependencyOrderedJoints) {}
        virtual ~OrderedJointsRecursionParams() {}

        // Result set
        std::vector<JointPtr> dependencyOrderedJoints;

        // Allow splits, i.e. one link has several child joints. If this is set to false,
        // the recursive operation will fail at splitting points.
        bool allowSplits;

        // Only add joints to the result which are active.
        bool onlyActive;
    };

    /**
     * \brief
     * \author Jennifer Buehler
     */
    class Vector3RecursionParams: public RecursionParams
    {
    public:
        typedef boost::shared_ptr<Vector3RecursionParams> Ptr;
        Vector3RecursionParams(): RecursionParams() {}
        Vector3RecursionParams(const Eigen::Vector3d& _vec):
            RecursionParams(),
            vec(_vec){}
        Vector3RecursionParams(const Vector3RecursionParams& o):
            RecursionParams(o),
            vec(o.vec) {}
        virtual ~Vector3RecursionParams() {}

        // Result set
        Eigen::Vector3d vec;
    };

    /**
     * Returns all joints starting from from_joint (including from_joint) within the tree. This is obtained by depth-first traversal,
     * so all joints in the result won't depend on any joints further back in the result set.
     */
    bool getDependencyOrderedJoints(std::vector<JointPtr>& result, const JointPtr& from_joint,
                                    bool allowSplits = true, bool onlyActive = true);

    /**
     * Returns all joints down from from_link within the tree. This is obtained by depth-first traversal,
     * so all joints in the result won't depend on any joints further back in the result set.
     */
    bool getDependencyOrderedJoints(std::vector<JointPtr>& result, const LinkPtr& from_link,
                                    bool allowSplits = true, bool onlyActive = true);

    /**
     * \retval -2 on error
     * \retval -1 if the joint has multiple children
     * \retval 0 if the joint has no child joints (it's an end effector joint),
     * \retval 1 if a child joint is returned in parameter "child"
     */
    int getChildJoint(const JointPtr& joint, JointPtr& child);

    bool isChildOf(const LinkConstPtr& parent, const LinkConstPtr& child) const;
    bool isChildJointOf(const LinkConstPtr& parent, const JointConstPtr& joint) const;
    
    LinkPtr getChildLink(const JointPtr& joint);
    LinkConstPtr readChildLink(const JointPtr& joint) const;
    
    JointPtr getParentJoint(const JointPtr& joint);
    JointConstPtr readParentJoint(const JointPtr& joint) const;
 
    /**
     * Helper function for getJointNames(const std::string&, std::vector<std::string>&).
     */
    int getJointNames(RecursionParamsPtr& p);

    /**
     * Main recursive top-down traversal method, called from other traverseTreeTopDown(). 
     */
    int traverseTreeTopDown(const LinkPtr& link, boost::function< int(RecursionParamsPtr&)> link_cb,
                            RecursionParamsPtr& params, bool includeLink=true, unsigned int level=0);

    /**
     * Main recursive bottom-up traversal method, called from other traverseTreeBottomUp(). 
     */
    int traverseTreeBottomUp(const LinkPtr& link, boost::function<int(RecursionParamsPtr&)> link_cb,
                              RecursionParamsPtr& params, bool includeLink=true, unsigned int level=0);
    //bool traverseTreeBottomUp(LinkPtr& link, boost::function< LinkPtr(LinkPtr&)> link_cb);


    // Returns if this is an active joint in the URDF description
    bool isActive(const JointPtr& joint) const;

    JointPtr getJoint(const std::string& name);
    JointConstPtr readJoint(const std::string& name) const;
    LinkPtr getLink(const std::string& name);
    LinkConstPtr readLink(const std::string& name) const;

    const urdf::Model& getRobot() const
    {
        return robot;
    }
    
    // returns true if there are any fixed joints down from from_link
    bool hasFixedJoints(LinkPtr& from_link);

    /**
     * \return true if this joint needs a transformation to align its rotation axis with the given axis.
     * In this case the rotation parameter contains the necessary rotation.
     */
    bool jointTransformForAxis(const urdf::Joint& joint, const Eigen::Vector3d& axis, Eigen::Quaterniond& rotation);


    /**
     * Recursion method to be used with traverseTreeTopDown() and recursion parameters
     * of type *Vector3RecursionParams*.
     *
     * Re-arranges the joint-transform of the recursion link's *parent joint*, along with
     * the link's visual/collision/intertial rotations, such that all joints rotate around the axis
     * given in the recursion parameters vector.
     */
    int allRotationsToAxis(RecursionParamsPtr& params);

    /**
     * Removes all fixed links down the chain in the model by adding visuals and collision geometry to the first parent link which is
     * attached to a non-fixed link.
     */
    bool joinFixedLinks(LinkPtr& from_link);

    bool hasChildLink(const LinkConstPtr& link, const std::string& childName) const;
    
    EigenTransform getTransform(const LinkPtr& from_link,  const JointPtr& to_joint);

private:

    // Function for recursive getDependencyOrderedJoints
    int addJointLink(RecursionParamsPtr& p);

    /**
     * Function used for recursion by scaleModelRecursive().
     */
    int scaleModel(RecursionParamsPtr& p);

    /**
     * Scales the URDF model by this factor. This means all translation parts of the joint transforms are multiplied by this.
     * The mesh files are not touched, but the visual/collision/intertial translations are scaled as well.
     * Meshes can be scaled using convertMeshes().
     */
    bool scaleModelRecursive(double scale_factor);

    /**
     * printing a link, used by recursive printModel().
     * Supports FlagRecursionParamsPtr, if the flag is true it prints verbose.
     */
    int printLink(RecursionParamsPtr& p);

    /**
     * Function which can be used for recursion which returns -1 if there are any inactive joints in the urdf.
     */
    int checkActiveJoints(RecursionParamsPtr& p);

    /**
     * Helper: If the parent joint of this link is fixed, it will be removed, and this link's visual will be
     * connected to the parent link.
     * If the joint was active, the function returns the same link as in the parameter.
     * Otherwise, it returns the pointer to the parent link which now contains
     * this link's visual/collision.
     */
    int joinFixedLinksOnThis(RecursionParamsPtr& params);
    //LinkPtr joinFixedLinksOnThis(LinkPtr& link);


    urdf::Model robot;
    std::string robot_urdf;
};

}  //  namespace urdf_traverser
#endif   // URDF2INVENTOR_URDF2INVENTOR_H
