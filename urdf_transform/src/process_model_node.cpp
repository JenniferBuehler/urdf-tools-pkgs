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
#include <ros/ros.h>
#include <urdf_traverser/UrdfTraverser.h>
#include <urdf_traverser/PrintModel.h>
#include <urdf_transform/ScaleModel.h>
#include <urdf_transform/JoinFixedLinks.h>
#include <urdf_transform/AlignRotationAxis.h>
#include <string>

using urdf_traverser::UrdfTraverser;

void printHelp(char * progName)
{
    ROS_INFO_STREAM("Test tool for urdf transform.");
    ROS_INFO_STREAM("Usage: " << progName << " <OP> <input-file> [<from-link>]");
    ROS_INFO_STREAM("<OP>: The operation to perform. <print|scale|join|zaxis>");
    ROS_INFO_STREAM("If <from-link> is specified, the URDF is processed from this link down instead of the root link.");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "urdf_traverser_scale_model", ros::init_options::AnonymousName);

    if (argc < 3)
    {
        printHelp(argv[0]);
        return 0;
    }

    enum Operation {PRINT, SCALE, JOIN, ZAXIS};
    Operation op;
    std::string opArg = argv[1];
    if (opArg == "print")
    {
        op = PRINT;
    }
    else if (opArg == "scale")
    {
        op = SCALE;
    }
    else if (opArg == "join")
    {
        op = JOIN;
    }
    else if (opArg == "zaxis")
    {
        op = ZAXIS;
    }
    else
    {
        printHelp(argv[0]);
        ROS_ERROR_STREAM("Unkown operation: " << opArg);
        return 1;
    }

    std::string inputFile = argv[2];
    std::string fromLink;

    if (argc > 3)
    {
        fromLink = argv[3];
    }

    ROS_INFO_STREAM("Traversing model from file " << inputFile << "...");
    if (!fromLink.empty()) ROS_INFO_STREAM("Starting from link " << fromLink);

    bool verbose = false;

    ROS_INFO("Loading file...");
    UrdfTraverser traverser;
    if (!traverser.loadModelFromFile(inputFile))
    {
        ROS_ERROR_STREAM("Could not load file " << inputFile);
        return 0;
    }

    switch (op)
    {
    case PRINT:
    {
        ROS_INFO("###### MODEL #####");
        verbose = true;
        traverser.printModel(verbose);
        ROS_INFO("###### JOINT NAMES #####");
        traverser.printJointNames(fromLink);
        break;
    }
    case SCALE:
    {
        ROS_INFO("###### MODEL BEFORE #####");
        verbose = true;
        traverser.printModel(verbose);
        urdf_transform::scaleModel(traverser, 2);
        break;
    }
    case JOIN:
    {
        ROS_INFO("Join fixed links...");
        if (!urdf_transform::joinFixedLinks(traverser, fromLink))
        {
            ROS_ERROR_STREAM("Could not join fixed links");
            return 0;
        }

        ROS_INFO("$$$--- Joint names after fixed linked joining:");
        traverser.printJointNames(fromLink);
        break;
    }
    case ZAXIS:
    {
        verbose = true;
        ROS_INFO("Align rotation axis...");
        ROS_INFO("###### MODEL BEFORE #####");
        traverser.printModel(verbose);
        Eigen::Vector3d axis(0, 0, 1);
        if (!urdf_transform::allRotationsToAxis(traverser, fromLink, axis))
        {
            ROS_ERROR_STREAM("Could not rotate axes");
            return 0;
        }
        break;
    }
    default:
    {
        ROS_ERROR("Unknown operation.");
        return 1;
    }
    }

    if (op != PRINT)
    {
        ROS_INFO("### Model after processing:");
        traverser.printModel(verbose);
    }

    return 0;
}


