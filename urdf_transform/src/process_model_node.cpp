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


