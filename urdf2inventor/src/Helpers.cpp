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

#include <urdf2inventor/Helpers.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <fcntl.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <string>


/**
 * ################################################################################################################
 * Minor helpers (e.g. file writing)
 * \author Jennifer Buehler
 * \date last edited October 2015
 * ################################################################################################################
 */

// handle for stdout redirect
int stdout_fd;

void urdf2inventor::helpers::resetStdOut()
{
    if (stdout_fd < 0)
    {
        return;
    }
    fflush(stdout);
    if (dup2(stdout_fd, STDOUT_FILENO) < 0)
    {
        ROS_ERROR("Could not restore stdout");
        return;
    }
    close(stdout_fd);

    // setbuf(stdout,NULL);//reset to unnamed buffer
}

// See http://homepage.ntlworld.com/jonathan.deboynepollard/FGA/redirecting-standard-io.html
void urdf2inventor::helpers::redirectStdOut(const char * toFile)
{
    fflush(stdout);

    mode_t mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;
    int file = open(toFile, O_CREAT | O_APPEND | O_WRONLY, mode);
    if (file < 0)
    {
        ROS_ERROR("could not create new output stream %s: %s", toFile, strerror(errno));
        return;
    }
    stdout_fd = dup(STDOUT_FILENO);                // Clone stdout to a new descriptor
    if (dup2(file, STDOUT_FILENO) < 0)
    {
        ROS_ERROR("could not redirect output stream");
        return;      // Change stdout to file
    }
    // close(file);  // stdout is still valid

    // setvbuf(stdout,toString,_IOFBF,stringSize);
}




