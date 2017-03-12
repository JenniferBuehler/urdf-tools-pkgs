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
#ifndef URDF_TRAVERSER_URDFTRAVERSER_H
#define URDF_TRAVERSER_URDFTRAVERSER_H
// Copyright Jennifer Buehler

//-----------------------------------------------------


#include <iostream>
#include <string>
#include <map>
#include <vector>

#include <boost/filesystem.hpp>

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
     * anything in the UrdfTraverser object. Result can be used with loadModelFromXMLString().
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
     * Set the directory which is considered the base of all
     * mesh and texture files. When loading the model URDF from file,
     * this directory is automatically set to the directory in which
     * the URDF file resides. However if other files referenced from
     * the URDF don't share this same base directory, use this function
     * to override this.
     */
    inline void setModelDirectory(const std::string& dir)
    {
        // make sure this is an absolute canonical path
        boost::filesystem::path _dir(dir);
        boost::filesystem::path _absDir = boost::filesystem::canonical(_dir);
        modelDir = _absDir.string();
    }
    inline std::string getModelDirectory() const
    {
        return modelDir;
    }

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
                            RecursionParamsPtr& params, bool includeLink = true);
    /**
     * Similar to traverseTreeTopDown(), but traverses bottom-up and is allows to re-link tree
     * (by traversing it safely such that changes in structure won't matter).
     * \param includeLink include the link itself. If false, will start to traverse starting
     *    from the link's children.
     * \param link_cb Callback to be called when traversing a link.
     *      returns -1 or 0 if traversal is to be stopped, otherwise 1.
     *      -1 is for stop because of an error, while 0 is stop because an expected
     *      condition found and traversal has been stopped with  no error.
     * \return the last return value of the callback
     */
    int traverseTreeBottomUp(const std::string& linkName,
                             boost::function< int(RecursionParamsPtr&)> link_cb,
                             RecursionParamsPtr& params, bool includeLink = true);


    JointPtr getJoint(const std::string& name);
    JointConstPtr readJoint(const std::string& name) const;
    LinkPtr getLink(const std::string& name);
    LinkConstPtr readLink(const std::string& name) const;

    LinkPtr getChildLink(const JointConstPtr& joint);
    LinkConstPtr readChildLink(const JointConstPtr& joint) const;

    JointPtr getParentJoint(const JointConstPtr& joint);
    JointConstPtr readParentJoint(const JointConstPtr& joint) const;


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
                            RecursionParamsPtr& params, bool includeLink = true, unsigned int level = 0);

    /**
     * Main recursive bottom-up traversal method, called from other traverseTreeBottomUp().
     */
    int traverseTreeBottomUp(const LinkPtr& link, boost::function<int(RecursionParamsPtr&)> link_cb,
                             RecursionParamsPtr& params, bool includeLink = true, unsigned int level = 0);

    bool hasChildLink(const LinkConstPtr& link, const std::string& childName) const;


    EigenTransform getTransform(const LinkPtr& from_link,  const JointPtr& to_joint);

private:

    /**
     * printing a link, used by recursive printModel().
     * Supports FlagRecursionParamsPtr, if the flag is true it prints verbose.
     */
    int printLink(RecursionParamsPtr& p);

    ModelPtr model;

    /**
     * The directory which is considered the base of all
     * mesh and texture files of the model
     */
    std::string modelDir;
};

}  //  namespace urdf_traverser
#endif   // URDF_TRAVERSER_URDFTRAVERSER_H
