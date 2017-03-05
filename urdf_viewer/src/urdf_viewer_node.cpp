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
#include <ros/ros.h>
#include <urdf2inventor/Urdf2Inventor.h>
#include <urdf_viewer/InventorViewer.h>
#include <string>

using urdf_viewer::InventorViewer;
using urdf2inventor::Urdf2Inventor;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "urdf_viewer", ros::init_options::AnonymousName);
    ros::NodeHandle priv("~");

    if (argc < 2)
    {
        ROS_INFO_STREAM("Usage: " << argv[0] << " <input-file> [--iv | <from-link>]");
        ROS_INFO_STREAM(" If --iv is specified, the file is assumed to be an inventor file, otherwise a URDF file.");
        ROS_INFO_STREAM(" if <from-link> is specified (only supported if not using --iv), the URDF is displayed from this link down.");
        return 0;
    }

    bool isURDF = true;

    std::string inputFile = argv[1];
    std::string fromLink;

    if (argc > 2)
    {
        std::string arg(argv[2]);
        if (arg == "--iv") isURDF = false;
        else if (arg != "root") fromLink = arg;
    }


    // join fixed links before displaying
    bool joinFixedLinks = false;
    priv.param<bool>("join_fixed_links", joinFixedLinks, joinFixedLinks);

    // rotate all axes to z
    bool rotateAxesZ = false;
    priv.param<bool>("rotate_axes_z", rotateAxesZ, rotateAxesZ);

    // true to display axes of local joint coordinate systems
    bool displayAxes = true;
    priv.param<bool>("display_axes", displayAxes, displayAxes);

    float axRad = 0.001;
    priv.param<float>("axes_radius", axRad, axRad);

    float axLen = 0.015;
    priv.param<float>("axes_length", axLen, axLen);

    // An axis and angle (degrees) can be specified which will transform *all*
    // visuals (not links, but their visuals!) within their local coordinate system.
    // This can be used to correct transformation errors which may have been
    // introduced in converting meshes from one format to the other, losing orientation information
    // For example, .dae has an "up vector" definition which may have been ignored.
    float visCorrAxX = 0;
    priv.param<float>("visual_corr_axis_x", visCorrAxX, visCorrAxX);
    float visCorrAxY = 0;
    priv.param<float>("visual_corr_axis_y", visCorrAxY, visCorrAxY);
    float visCorrAxZ = 0;
    priv.param<float>("visual_corr_axis_z", visCorrAxZ, visCorrAxZ);
    float visCorrAxAngle = 0;
    priv.param<float>("visual_corr_axis_angle", visCorrAxAngle, visCorrAxAngle);
    Urdf2Inventor::EigenTransform addVisualTrans(Eigen::AngleAxisd(visCorrAxAngle * M_PI / 180, Eigen::Vector3d(visCorrAxX, visCorrAxY, visCorrAxZ)));

    bool success = true;
    urdf2inventor::Urdf2Inventor::UrdfTraverserPtr traverser(new urdf_traverser::UrdfTraverser());
    Urdf2Inventor converter(traverser, 1);
    InventorViewer view;
    view.init("WindowName");
    if (isURDF)
    {
        ROS_INFO_STREAM("Converting model from file " << inputFile << "...");
        if (!fromLink.empty()) ROS_INFO_STREAM("Starting from link " << fromLink);

        ROS_INFO("Loading file...");
        if (!converter.loadModelFromFile(inputFile))
        {
            ROS_ERROR_STREAM("Could not load file " << inputFile);
            return 0;
        }
        converter.printJointNames(fromLink);

        ROS_INFO("Check to join fixed links...");
        if (joinFixedLinks && !converter.joinFixedLinks(fromLink))
        {
            ROS_ERROR_STREAM("Could not join fixed links");
            return 0;
        }

        ROS_INFO("Check to align rotation axis...");
        Eigen::Vector3d axis(0, 0, 1);
        if (rotateAxesZ && !converter.allRotationsToAxis(fromLink, axis))
        {
            ROS_ERROR_STREAM("Could not rotate axes");
            return 0;
        }

        ROS_INFO("Getting inventor node...");
        SoNode * node = converter.getAsInventor(fromLink, false, displayAxes, axRad, axLen, addVisualTrans, NULL);
        if (!node)
        {
            ROS_INFO_STREAM("ERROR: Could not get inventor node");
            success = false;
        }
        else
        {
            ROS_INFO_STREAM("Model converted, now loading into viewer...");
            view.loadModel(node);
        }
    }
    else
    {
        view.loadModel(inputFile);
    }
    if (success)  view.runViewer();

    converter.cleanup();
    return 0;
}


