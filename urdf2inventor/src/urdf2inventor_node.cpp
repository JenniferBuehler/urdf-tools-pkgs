/**
    Copyright (C) 2015 Jennifer Buehler

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

    std::string outputMaterial = "plastic";
    double scaleFactor = 1;
    
    priv.param<std::string>("output_material", outputMaterial, outputMaterial);
    ROS_INFO("output_material: <%s>", outputMaterial.c_str());

    priv.param<double>("scale_factor", scaleFactor, scaleFactor);
    ROS_INFO("scale_factor: <%f>", scaleFactor);

    // An axis and angle (degrees) can be specified which will transform *all*
    // visuals (not links, but their visuals!) within their local coordinate system.
    // This can be used to correct transformation errors which may have been 
    // introduced in converting meshes from one format to the other, losing orientation information
    // For example, .dae has an "up vector" definition which may have been ignored.
    float visCorrAxX=0;
    priv.param<float>("visual_corr_axis_x", visCorrAxX, visCorrAxX);
    float visCorrAxY=0;
    priv.param<float>("visual_corr_axis_y", visCorrAxY, visCorrAxY);
    float visCorrAxZ=0;
    priv.param<float>("visual_corr_axis_z", visCorrAxZ, visCorrAxZ);
    float visCorrAxAngle=0;
    priv.param<float>("visual_corr_axis_angle", visCorrAxAngle, visCorrAxAngle);
    urdf2inventor::Urdf2Inventor::EigenTransform addTrans(Eigen::AngleAxisd(visCorrAxAngle*M_PI/180, Eigen::Vector3d(visCorrAxX,visCorrAxY,visCorrAxZ)));

    urdf2inventor::Urdf2Inventor::UrdfTraverserPtr traverser(new urdf_traverser::UrdfTraverser());

    urdf2inventor::Urdf2Inventor converter(traverser, scaleFactor);

    ROS_INFO("Starting model conversion...");
        
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
    wholeFile<<outputDir<<"/robot/"<<traverser->getModelName()<<".iv";
    ROS_INFO_STREAM("Now writing whole robot to "<<wholeFile.str());
    if (!converter.writeAsInventor(wholeFile.str(),"",true, addTrans))
    {
        ROS_ERROR("Could not write whole robot file");
        return 0;
    }

    ROS_INFO("Cleaning up...");
    converter.cleanup();

    ROS_INFO("Done.");
    return 0;
}
