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

#include <string>

using urdf_traverser::UrdfTraverser;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "urdf_traverser_join_fixed_links", ros::init_options::AnonymousName);
    ros::NodeHandle priv("~");

    if (argc < 2)
    {
        ROS_INFO_STREAM("Usage: " << argv[0] << " <input-file> [<from-link>]" );
        ROS_INFO_STREAM("If <from-link> is specified, the URDF is processed from this link down instead of the root link." );
        return 0;
    }

    std::string inputFile = argv[1];
    std::string fromLink;
 
    if (argc > 2)
    {
        fromLink = argv[2];
    } 
    
    ROS_INFO_STREAM("Traversing model from file "<<inputFile<<"...");
    if (!fromLink.empty()) ROS_INFO_STREAM("Starting from link "<<fromLink);


    ROS_INFO("Loading file...");
    UrdfTraverser traverser;
    if (!traverser.loadModelFromFile(inputFile))
    {
        ROS_ERROR_STREAM("Could not load file "<<inputFile);
        return 0;
    }
    
    bool verbose=true;

    /*
    ROS_INFO("###### MODEL #####");
    traverser.printModel(verbose);
    ROS_INFO("###### JOINT NAMES #####");
    traverser.printJointNames(fromLink);
*/
    ROS_INFO("###### MODEL #####");
    print_model::printModel(traverser,verbose);
    
    return 0;
}


