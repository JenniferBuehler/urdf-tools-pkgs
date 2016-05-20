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


#include <iostream>
#include <string>
#include <map>
#include <vector>


#include <urdf_traverser/Types.h>
#include <urdf_traverser/RecursionParams.h>


namespace urdf_traverser
{

/**
 * \brief This class provides functions to traverse the URDF and provides convenience functions to access the URDF model.
 *
 * Traversal through a model is always done on links. This is because the root of a model is always a link.
 * Traversal through joint space is also possible by using the link's parent joint as reference.
 *
 *  Methods int traverseTreeTopDown() and traverseTreeBottomUp() can be used and will require a callback function
 *  for the traversal. An example of how to use the functions can for example be found in PrintModel.h or JointNames.h.
 *
 * \author Jennifer Buehler
 * \date May 2016
 */
class UrdfTraverser
{
public:

    /**
     */
    explicit UrdfTraverser():
        model(new urdf::Model())
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
     * urdf into field \e robot_urdf
     */
    //bool loadModelFromParameterServer(); 

    /**
     * Prints the joint names. Can't be const in this version because
     * of the use of a recursive method.
     * Only joints *after* the given link are printed.
     */
    void printJointNames(const std::string& fromLink);

    std::string getRootLinkName() const;
   
    std::string getModelName() const
    { 
        return this->model->getName();
    }

    bool printModel(const std::string& fromLink, bool verbose);

    bool printModel(bool verbose);


    /**
     * Traverses the tree starting from link (depth first) and calls link_cb on each link.
     * \param includeLink include the link itself. If false, will start to traverse starting from the link's children.
     * \param link_cb Callback to be called when traversing a link.
     *      returns -1 or 0 if traversal is to be stopped, otherwise 1. -1 is for stop because of an error,
     *      while 0 is stop because an expected condition found and traversal has been stopped with  no error.
     * \return the last return value of the callback
     */
    int traverseTreeTopDown(const std::string& linkName, boost::function< int(RecursionParamsPtr&)> link_cb,
                            RecursionParamsPtr& params, bool includeLink=true);
    /**
     * Similar to traverseTreeTopDown(), but traverses bottom-up and is allows to re-link tree (by traversing it safely such
     * that changes in structure won't matter).
     * \param includeLink include the link itself. If false, will start to traverse starting from the link's children.
     * \param link_cb Callback to be called when traversing a link.
     *      returns -1 or 0 if traversal is to be stopped, otherwise 1. -1 is for stop because of an error,
     *      while 0 is stop because an expected condition found and traversal has been stopped with  no error.
     * \return the last return value of the callback
     */
    int traverseTreeBottomUp(const std::string& linkName, boost::function< int(RecursionParamsPtr&)> link_cb,
                            RecursionParamsPtr& params, bool includeLink=true);


    JointPtr getJoint(const std::string& name);
    JointConstPtr readJoint(const std::string& name) const;
    LinkPtr getLink(const std::string& name);
    LinkConstPtr readLink(const std::string& name) const;

    LinkPtr getChildLink(const JointPtr& joint);
    LinkConstPtr readChildLink(const JointPtr& joint) const;
    
    JointPtr getParentJoint(const JointPtr& joint);
    JointConstPtr readParentJoint(const JointPtr& joint) const;
 

    ModelPtr getModel()
    {
        return model;
    }
    ModelConstPtr readModel() const
    {
        return model;
    }

    /**
     * Returns all joint names in depth-frist search order starting from \e fromLink (or from root if
     * \e fromLink is empty). Only joints *after* the given link are returned.
     */
    bool getJointNames(const std::string& fromLink,
            const bool skipFixed, std::vector<std::string>& result);


protected:

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

    bool hasChildLink(const LinkConstPtr& link, const std::string& childName) const;
    
    EigenTransform getTransform(const LinkPtr& from_link,  const JointPtr& to_joint);

private:

    /**
     * printing a link, used by recursive printModel().
     * Supports FlagRecursionParamsPtr, if the flag is true it prints verbose.
     */
    int printLink(RecursionParamsPtr& p);


    ModelPtr model;
};

}  //  namespace urdf_traverser
#endif   // URDF2INVENTOR_URDF2INVENTOR_H
