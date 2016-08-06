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
#include <urdf_traverser/DependencyOrderedJoints.h>

#include <string>

using urdf_traverser::UrdfTraverser;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "urdf_traverser_join_fixed_links", ros::init_options::AnonymousName);
    ros::NodeHandle priv("~");

    if (argc < 2)
    {
        ROS_INFO_STREAM("Usage: " << argv[0] << " <input-file> [<from-link>]");
        ROS_INFO_STREAM("If <from-link> is specified, the URDF is processed from this link down instead of the root link.");
        return 0;
    }

    std::string inputFile = argv[1];
    std::string fromLink;

    if (argc > 2)
    {
        fromLink = argv[2];
    }

    ROS_INFO_STREAM("Traversing model from file " << inputFile << "...");
    if (!fromLink.empty()) ROS_INFO_STREAM("Starting from link " << fromLink);


    ROS_INFO("Loading file...");
    UrdfTraverser traverser;
    if (!traverser.loadModelFromFile(inputFile))
    {
        ROS_ERROR_STREAM("Could not load file " << inputFile);
        return 0;
    }

    bool verbose = true;

    /*
    ROS_INFO("###### MODEL #####");
    traverser.printModel(verbose);
    */
    ROS_INFO("###### MODEL #####");
    traverser.printModel(verbose);

    ROS_INFO("###### JOINT NAMES #####");
    traverser.printJointNames(fromLink);

    /*    std::vector<urdf_traverser::JointPtr> depOrdered;
        if (!urdf_traverser::getDependencyOrderedJoints(traverser,depOrdered, fromLink, true, false))
        {
            ROS_ERROR("Could not get dependency ordered joints");
        }
        else
        {
            ROS_INFO("Dependency ordered joints:");
        }
        for (std::vector<urdf_traverser::JointPtr>::iterator it=depOrdered.begin(); it!=depOrdered.end(); ++it)
        {
            urdf_traverser::JointPtr j=*it;
            std::cout<<j->name<<std::endl;
        }
    */
    return 0;
}


