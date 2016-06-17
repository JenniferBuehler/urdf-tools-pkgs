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
#include <urdf2inventor/Urdf2Inventor.h>
#include <urdf2inventor/Helpers.h>
#include <urdf2inventor/IVHelpers.h>
#include <urdf2inventor/ConvertMesh.h>

#include <string>
#include <ros/ros.h>
#include <ros/package.h>

#include <Inventor/SoDB.h>      // for file reading
#include <Inventor/SoInput.h>   // for file reading
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoSelection.h>
#include <Inventor/nodekits/SoNodeKit.h>

#include <urdf_traverser/Functions.h>
#include <urdf_transform/JoinFixedLinks.h>
#include <urdf_transform/ScaleModel.h>
#include <urdf_transform/AlignRotationAxis.h>

#include <map>
#include <vector>
#include <set>
#include <fstream>
#include <algorithm>

#define RAD_TO_DEG 180/M_PI

// directory where to dump temporary mesh
// files. Only required to read collada files.
#define TEMP_MESH_DIR "/tmp/tempMeshes"

using urdf2inventor::Urdf2Inventor;

std::string Urdf2Inventor::OUTPUT_EXTENSION = ".iv";
std::string Urdf2Inventor::MESH_OUTPUT_DIRECTORY_NAME = "iv/";
std::string Urdf2Inventor::TEX_OUTPUT_DIRECTORY_NAME = "textures/";

bool Urdf2Inventor::joinFixedLinks(const std::string& fromLink)
{
    std::string startLinkName = fromLink;
    if (startLinkName.empty())
    {
        startLinkName = urdf_traverser->getRootLinkName();
    }
        
    ROS_INFO("############### Joining fixed links");
    ROS_INFO_STREAM("Start from root link: "<<startLinkName);
//    ROS_INFO_STREAM("Getting as inventor starting from '"<<startLinkName<<"'");
    LinkPtr startLink = urdf_traverser->getLink(startLinkName);
    if (!startLink.get())
    {
        ROS_ERROR_STREAM("No link named '" << startLinkName << "'");
        return false;
    }

    if (!urdf_transform::joinFixedLinks(*urdf_traverser, startLinkName))
    {
        ROS_ERROR_STREAM("Could not join fixed links");
        return false;
    }
    return true;
}

bool Urdf2Inventor::allRotationsToAxis(const std::string& fromLinkName, const Eigen::Vector3d& axis)
{
    std::string startLinkName = fromLinkName;
    if (startLinkName.empty())
    {
        startLinkName = urdf_traverser->getRootLinkName();
    }
//    ROS_INFO_STREAM("Getting as inventor starting from '"<<startLinkName<<"'");
    LinkPtr startLink = urdf_traverser->getLink(startLinkName);
    if (!startLink.get())
    {
        ROS_ERROR_STREAM("No link named '" << startLink << "'");
        return false;
    }

    ROS_INFO_STREAM("############### Aligning rotation axis to " << axis);
    if (!urdf_transform::allRotationsToAxis(*urdf_traverser, fromLinkName, axis))
    {
        ROS_ERROR_STREAM("Could not align to axis");
        return false;
    }
    return true;
}

bool Urdf2Inventor::scale()
{
    if (fabs(scaleFactor - 1.0) < 1e-04)
    {
        //ROS_INFO("Scale factor 1, so no need to scale model");
        return true;
    }
    ROS_INFO("############### Scaling model");

    if (!urdf_transform::scaleModel(*urdf_traverser, scaleFactor))
    {
        ROS_ERROR("Could not scale model");
        return false;
    }

    isScaled = true;
    return true;
}


bool Urdf2Inventor::loadModelFromFile(const std::string& urdfFilename)
{
    return urdf_traverser->loadModelFromFile(urdfFilename);
}


urdf_traverser::EigenTransform Urdf2Inventor::getTransform(const LinkPtr& from_link,  const JointPtr& to_joint)
{
    LinkPtr link1 = from_link;
    LinkPtr link2 = urdf_traverser->getLink(to_joint->child_link_name);
    if (!link1 || !link2)
    {
        ROS_ERROR("Invalid joint specifications (%s, %s), first needs parent and second child",
                  link1->name.c_str(), link2->name.c_str());
    }
    return urdf_traverser::getTransform(link1, link2);
}

Urdf2Inventor::ConversionResultPtr Urdf2Inventor::preConvert(const ConversionParametersPtr& params)
{
    ConversionResultPtr res(new ConversionResultT(OUTPUT_EXTENSION, MESH_OUTPUT_DIRECTORY_NAME, TEX_OUTPUT_DIRECTORY_NAME));
    res->success = false;
    return res;
}

Urdf2Inventor::ConversionResultPtr Urdf2Inventor::postConvert(const ConversionParametersPtr& params, ConversionResultPtr& result)
{
    result->success = true;
    return result;
}


Urdf2Inventor::ConversionResultPtr Urdf2Inventor::convert(const ConversionParametersPtr& params)
{
    ConversionResultPtr res = preConvert(params);
    if (!res.get())
    {
        ROS_ERROR("Failed to pre-convert");
        return res;
    }
    res->success = false;

    if (!isScaled && !scale())
    {
        ROS_ERROR("Failed to scale model");
    }

    ROS_INFO_STREAM("############### Converting meshes"); //, base dir " << params->baseDir);

    std::map<std::string, std::set<std::string> > textureFiles;
    if (!urdf2inventor::convertMeshes(*urdf_traverser, params->rootLinkName,
                                      scaleFactor,
                                      params->material,
                                      OUTPUT_EXTENSION,
                                      params->addVisualTransform,
                                      res->meshes, textureFiles))
    {
        ROS_ERROR("Could not convert meshes");
        return res;
    }

    if (!urdf2inventor::fixTextureReferences(
                res->meshOutputDirectoryName,
                res->texOutputDirectoryName,
                textureFiles,
                res->meshes, res->textureFiles))
    {
        ROS_ERROR("Could not fix texture references");
        return res;
    }

    /*    for (std::map<std::string, std::set<std::string> >::iterator mit=res->textureFiles.begin(); mit!=res->textureFiles.end(); ++mit)
        {
            for (std::set<std::string>::iterator tit=mit->second.begin(); tit!=mit->second.end(); ++tit)
                ROS_INFO_STREAM("Required: cp "<<*tit<<" "<<mit->first);
        }*/

    return postConvert(params, res);
}


bool Urdf2Inventor::printModel(const std::string& fromLink)
{
    return urdf_traverser->printModel(fromLink, false);
}

bool Urdf2Inventor::printModel()
{
    return urdf_traverser->printModel(false);
}


bool Urdf2Inventor::getJointNames(const std::string& fromLink, const bool skipFixed, std::vector<std::string>& result)
{
    return urdf_traverser->getJointNames(fromLink, skipFixed, result);
}



void Urdf2Inventor::cleanup()
{
    if (urdf_traverser::helpers::fileExists(TMP_FILE_IV))
    {
        urdf_traverser::helpers::deleteFile(TMP_FILE_IV);
    }
}

bool Urdf2Inventor::writeInventorFile(SoNode * node, const std::string& filename)
{
    SoOutput out;
    if (!out.openFile(filename.c_str())) return false;
    out.setBinary(false);
    SoWriteAction write(&out);
    write.apply(node);
    write.getOutput()->closeFile();
    return true;
}


/*SoTransform * Urdf2Inventor::getTransform(const EigenTransform& eTrans) const
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
}*/

void Urdf2Inventor::printJointNames(const std::string& fromLink)
{
    std::vector<std::string> jointNames;
    if (!getJointNames(fromLink, false, jointNames))
    {
        ROS_WARN("Could not retrieve joint names to print on screen");
    }
    else
    {
        ROS_INFO_STREAM("Joint names starting from " << fromLink << ":");
        for (int i = 0; i < jointNames.size(); ++i) ROS_INFO_STREAM(jointNames[i]);
        ROS_INFO("---");
    }
}

/*
SoNode * Urdf2Inventor::loadAndGetAsInventor(const std::string& urdfFilename,
    const std::string from_link, bool useScaleFactor,   )
{
    if (!loadModelFromFile(urdfFilename))
    {
        ROS_ERROR("Could not load file");
        return NULL;
    }


    ROS_INFO("Converting robot %s URDF to inventor", getRobotName().c_str());
    std::string rootLink=from_link;
    if (rootLink.empty()){
        rootLink = getRootLinkName();
    }
    printJointNames(rootLink);
    return getAsInventor(rootLink, useScaleFactor, addAxes, axesRadius, axesLength);
}*/


void Urdf2Inventor::addLocalAxes(const LinkConstPtr& link, SoSeparator * addToNode, bool useScaleFactor,
                                 float _axesRadius, float _axesLength) const
{
    bool active = urdf_traverser::isActive(link->parent_joint);
    float rad = _axesRadius;
    float rotRad = _axesRadius / 3;
    float h = _axesLength;
    float rotH = h * 1.8;
    float rtr, rtg, rtb;
    // rotation axis pink
    rtr = rtb = 1;
    rtg = 0;
    if (useScaleFactor)
    {
        rad *= scaleFactor;
        h *= scaleFactor;
        rotRad *= scaleFactor;
        rotH *= scaleFactor;
    }
    if (!active)
    {
        rad /= 2;
        h *= 1.5;
    }
    urdf2inventor::addLocalAxes(addToNode, rad, h);

    Eigen::Vector3d rotAxis(0, 0, 1);
    if (link->parent_joint) rotAxis = urdf_traverser::getRotationAxis(link->parent_joint);
    Eigen::Vector3d z(0, 0, 1);
    Eigen::Quaterniond q;
    q.setIdentity();
    if (active) q = Eigen::Quaterniond::FromTwoVectors(z, rotAxis);
    urdf2inventor::addCylinder(addToNode, Eigen::Vector3d(0, 0, 0), q, rotRad, rotH, rtr, rtg, rtb);
}


SoNode * Urdf2Inventor::getAsInventor(const LinkPtr& from_link, bool useScaleFactor,
                                      bool _addAxes, float _axesRadius, float _axesLength,
                                      const EigenTransform& addVisualTransform,
                                      std::set<std::string> * textureFiles
                                     )
{
    if (!from_link.get())
    {
        ROS_ERROR("getAsInventor: from_link is NULL");
        return NULL;
    }
    // ROS_INFO_STREAM("Get all visuals of "<<from_link->name);//<<" (parent joint "<<from_link->parent_joint->name<<")");
    SoNode * allVisuals = getAllVisuals(from_link,
                                        useScaleFactor ? scaleFactor : 1.0,
                                        addVisualTransform,
                                        useScaleFactor);
    if (!allVisuals)
    {
        ROS_ERROR("Could not get visuals");
        return NULL;
    }

    if (textureFiles)
    {
        // collect all relative texture filenames from the absolute texture paths.
        std::set<std::string> allFiles =  urdf2inventor::getAllTexturePaths(allVisuals);
        textureFiles->insert(allFiles.begin(), allFiles.end());
    }

    if (_addAxes)
    {
        SoSeparator * _allVisuals = dynamic_cast<SoSeparator*>(allVisuals);
        if (!_allVisuals)
        {
            ROS_WARN_STREAM("The node for link " << from_link->name << " is not a separator, so cannot add axes");
        }
        else
        {
            addLocalAxes(from_link, _allVisuals, useScaleFactor, _axesRadius, _axesLength);
        }
    }

    for (std::vector<JointPtr>::const_iterator pj = from_link->child_joints.begin();
            pj != from_link->child_joints.end(); pj++)
    {
        JointPtr joint = *pj;
        // ROS_INFO_STREAM("Get inventor files down from joint "<<joint->name);
        LinkPtr childLink = urdf_traverser->getLink(joint->child_link_name);
        if (!childLink.get())
        {
            ROS_ERROR_STREAM("Consistency: Link " << joint->child_link_name << " does not exist.");
            return NULL;
        }
        SoNode * childNode = getAsInventor(childLink, useScaleFactor,
                                           _addAxes, _axesRadius, _axesLength, addVisualTransform, textureFiles);
        if (!childNode)
        {
            ROS_ERROR_STREAM("Could not get child node for " << childLink->name);
            return NULL;
        }
        EigenTransform jointTransform = urdf_traverser::getTransform(joint);
        if (useScaleFactor) urdf_traverser::scaleTranslation(jointTransform, scaleFactor);

        // ROS_WARN_STREAM("Transform joint "<<joint->name<<": "<<jointTransform);
        // ROS_INFO_STREAM("Adding sub node for "<<childLink->name);

        allVisuals = urdf2inventor::addSubNode(childNode, allVisuals, jointTransform);
    }

    return allVisuals;
}


SoNode * Urdf2Inventor::getAsInventor(const std::string& fromLink, bool useScaleFactor,
                                      bool _addAxes, float _axesRadius, float _axesLength, const EigenTransform& addVisualTransform,
                                      std::set<std::string> * textureFiles
                                     )
{
    std::string startLinkName = fromLink;
    if (startLinkName.empty())
    {
        startLinkName = urdf_traverser->getRootLinkName();
    }
//    ROS_INFO_STREAM("Getting as inventor starting from '"<<startLinkName<<"'");
    LinkPtr startLink = urdf_traverser->getLink(startLinkName);
    if (!startLink.get())
    {
        ROS_ERROR_STREAM("No link named '" << startLink << "'");
        return NULL;
    }
    SoNode * root = getAsInventor(startLink, useScaleFactor, _addAxes, _axesRadius, _axesLength,
                                  addVisualTransform, textureFiles);
    urdf2inventor::removeTextureCopies(root);
    return root;
}


bool Urdf2Inventor::writeAsInventor(const std::string& ivFilename,
                                    const std::string& fromLink,
                                    bool useScaleFactor, const EigenTransform& addVisualTransform)
{
    std::string startLinkName = fromLink;
    if (startLinkName.empty())
    {
        startLinkName = urdf_traverser->getRootLinkName();
    }
//    ROS_INFO_STREAM("Getting as inventor starting from '"<<startLinkName<<"'");
    LinkPtr startLink = urdf_traverser->getLink(startLinkName);
    if (!startLink.get())
    {
        ROS_ERROR_STREAM("No link named '" << startLinkName << "'");
        return false;
    }

    ROS_INFO_STREAM("Writing from link '" << startLinkName << "' to file " << ivFilename);
    return writeAsInventor(ivFilename, startLink, useScaleFactor, addVisualTransform);
}

bool Urdf2Inventor::writeAsInventor(const std::string& ivFilename, const LinkPtr& from_link,
                                    bool useScaleFactor, const EigenTransform& addVisualTransform)
{
    ROS_INFO("Converting model...");

    std::set<std::string> textureFiles;
    SoNode * inv = getAsInventor(from_link, useScaleFactor, addAxes, axesRadius, axesLength,
                                 addVisualTransform, &textureFiles);
    if (!inv)
    {
        ROS_ERROR("could not generate overall inventor file");
        return false;
    }

    // get the node to IV XML format
    std::string resultFileContent;
    if (!urdf2inventor::writeInventorFileString(inv, resultFileContent))
    {
        ROS_ERROR("Could not get the mesh file content");
        return false;
    }

    // handle textures: adjust file references, if required.
    if (!textureFiles.empty())
    {

        // get common parent path of all textures
        std::string commonParent;
        if (!urdf_traverser::helpers::getCommonParentPath(textureFiles, commonParent))
        {
            ROS_ERROR_STREAM("Could not find common parent path of all files");
            return false;
        }

        // ROS_INFO_STREAM("Common parent path for all textures: "<<commonParent);

        std::string fileDir = urdf_traverser::helpers::getDirectory(ivFilename);
        //std::string fileDir = urdf_traverser::helpers::getDirectoryName(ivFilename);

        // ROS_INFO_STREAM("Directory name: "<<fileDir);

        std::map<std::string, std::set<std::string> > texToCopy;

        ROS_INFO("Fixing texture file references...");
        if (!urdf2inventor::helpers::fixFileReferences(
                    fileDir,
                    fileDir + TEX_OUTPUT_DIRECTORY_NAME,
                    commonParent,
                    textureFiles,
                    resultFileContent, texToCopy))
        {
            ROS_ERROR("Could not fix texture references");
            return false;
        }

        ROS_INFO("Copying texture files...");
        if (!urdf2inventor::helpers::writeFiles(texToCopy, fileDir))
        {
            ROS_ERROR("Could not write textures");
            return false;
        }
    }

    ROS_INFO("Writing model...");

    // write content to file
    if (!urdf_traverser::helpers::writeToFile(resultFileContent, ivFilename))
    {
        ROS_ERROR_STREAM("Could not write file " << ivFilename);
        return false;
    }

    ROS_INFO_STREAM("Whole robot model written to " << ivFilename);


    return true;
}

Urdf2Inventor::ConversionResultPtr Urdf2Inventor::loadAndConvert(const std::string& urdfFilename,
        bool joinFixed,
        const ConversionParametersPtr& params)
{
    ConversionResultPtr failResult(new ConversionResultT(OUTPUT_EXTENSION, MESH_OUTPUT_DIRECTORY_NAME, TEX_OUTPUT_DIRECTORY_NAME));
    failResult->success = false;

    ROS_INFO_STREAM("Loading model from file " << urdfFilename);

    if (!urdf_traverser->loadModelFromFile(urdfFilename))
    {
        ROS_ERROR("Could not load file");
        return failResult;
    }

    if (joinFixed) ROS_INFO("Joining fixed links..");
    if (joinFixed && !joinFixedLinks(params->rootLinkName))
    {
        ROS_ERROR("Could not join fixed links");
        return failResult;
    }

    // ROS_INFO("00000000000000000000000");
    // p.printModel(rootLink);

    ConversionResultPtr result = convert(params);
    if (!result.get() || !result->success)
    {
        ROS_ERROR("Could not do the conversion");
        return result;
    }

    // ROS_INFO_STREAM("Contacts generated: "<<result.contacts);
    return result;
}
