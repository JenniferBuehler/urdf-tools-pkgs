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

#include <urdf_traverser/UrdfTraverser.h>
#include <urdf2inventor/Helpers.h>
#include <urdf2inventor/Urdf2Inventor.h>
#include <urdf2inventor/FileIO.h>
#include <string>
#include <sstream>
#include <vector>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "urdf2inventor", ros::init_options::AnonymousName);
    ros::NodeHandle priv("~");
    ros::NodeHandle pub("");

    if (argc < 3)
    {
        ROS_ERROR("Not enough arguments!");
        ROS_INFO_STREAM("Usage: " << argv[0] <<
                        " <urdf-file> <output-directory> [<root link name>]");
        return 0;
    }

    // set parameters

    std::string urdf_filename = std::string(argv[1]);
    ROS_INFO("URDF file: %s", urdf_filename.c_str());

    std::string outputDir = std::string(argv[2]);
    ROS_INFO("Output dir: %s", outputDir.c_str());

    std::string rootLinkName;
    if (argc > 3)
    {
        rootLinkName = std::string(argv[3]);
        ROS_INFO("Root %s", argv[3]);
    }

    double scaleFactor = 1;

    priv.param<double>("scale_factor", scaleFactor, scaleFactor);
    ROS_INFO("scale_factor: <%f>", scaleFactor);

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
    urdf2inventor::Urdf2Inventor::EigenTransform addTrans(Eigen::AngleAxisd(visCorrAxAngle * M_PI / 180, Eigen::Vector3d(visCorrAxX, visCorrAxY, visCorrAxZ)));

    urdf2inventor::Urdf2Inventor::UrdfTraverserPtr traverser(new urdf_traverser::UrdfTraverser());

    urdf2inventor::Urdf2Inventor converter(traverser, scaleFactor);

    ROS_INFO("Starting model conversion...");

    std::string outputMaterial = "plastic";  // output material does not really matter for only conversion to IV
    urdf2inventor::Urdf2Inventor::ConversionParametersPtr params
        = converter.getBasicConversionParams(rootLinkName, outputMaterial, addTrans);

    ROS_INFO("Loading and converting...");

    urdf2inventor::Urdf2Inventor::ConversionResultPtr cResult =
        converter.loadAndConvert(urdf_filename, true, params);
    if (!cResult->success)
    {
        ROS_ERROR("Failed to process.");
        return 0;
    }

    ROS_INFO("Conversion done. Now writing files.");

    urdf2inventor::FileIO<urdf2inventor::Urdf2Inventor::MeshFormat> fileIO(outputDir);
    if (!fileIO.write(cResult))
    {
        ROS_ERROR("Could not write files");
        return 0;
    }

    std::stringstream wholeFile;
    wholeFile << outputDir << "/robot/" << traverser->getModelName() << ".iv";
    ROS_INFO_STREAM("Now writing whole robot to " << wholeFile.str());
    if (!converter.writeAsInventor(wholeFile.str(), "", true, addTrans))
    {
        ROS_ERROR("Could not write whole robot file");
        return 0;
    }

    ROS_INFO("Cleaning up...");
    converter.cleanup();

    ROS_INFO("Done.");
    return 0;
}
