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

#include <urdf_viewer/InventorViewer.h>
#include <urdf2inventor/Helpers.h>

#include <Inventor/SoDB.h>  // for file reading
#include <Inventor/SoInput.h>   // for file reading
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/events/SoMouseButtonEvent.h>
#include <Inventor/SoPickedPoint.h>

#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoCone.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/details/SoFaceDetail.h>

#include <vector>
#include <map>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>

#include <ros/ros.h>  // only needed for ROS prints, e.g. ROS_ERROR

using urdf_viewer::InventorViewer;

InventorViewer::InventorViewer(bool _faces_ccw):
    root(NULL), viewWindow(NULL), viewer(NULL),
    faces_ccw(_faces_ccw) {}


InventorViewer::InventorViewer(const InventorViewer& o):
    root(o.root), viewWindow(o.viewWindow), viewer(o.viewer),
    faces_ccw(o.faces_ccw) {}
InventorViewer::~InventorViewer() {
//    SoQt::done();
   if (viewer)
    {
        delete viewer;
    }
//    root->unref();
}


void InventorViewer::init(const char * windowName)
{
    if (viewWindow)
    {
        ROS_ERROR("InventorViewer already initialized");
        return;
    }
    viewWindow = SoQt::init(windowName);
    viewer = new SoQtExaminerViewer(viewWindow);
    root = new SoSelection();
    root->ref();

    SoEventCallback * ecb = new SoEventCallback();
    ecb->addEventCallback(SoMouseButtonEvent::getClassTypeId(), InventorViewer::mouseBtnCB, this);
    root->addChild(ecb);
}

void InventorViewer::loadModel(SoNode * model)
{
    if (model) root->addChild(model);
}

bool InventorViewer::loadModel(const std::string& filename)
{
    SoInput in;
    SoNode  *model = NULL;
    if (!in.openFile(filename.c_str()))
        return false;
    if (!SoDB::read(&in, model) || model == NULL)
        return false;

    root->addChild(model);
    return true;
}

void InventorViewer::runViewer()
{
    viewer->setSceneGraph(root);
    viewer->show();

    SoQt::show(viewWindow);
    SoQt::mainLoop();
}

bool InventorViewer::computeCorrectFaceNormal(const SoPickedPoint * pick, bool ccw_face, Eigen::Vector3d& normal)
{
    const SoDetail *pickDetail = pick->getDetail();
    if ((pickDetail != NULL) && (pickDetail->getTypeId() == SoFaceDetail::getClassTypeId()))
    {
        // Picked object is a face
        const SoFaceDetail * fd = dynamic_cast<const SoFaceDetail*>(pickDetail);
        if (!fd)
        {
            ROS_ERROR("Could not cast to face detail");
            return false;
        }

        // Find the coordinate node that is used for the faces.
        // Assume that it's the last SoCoordinate3 node traversed
        // before the picked shape.
        SoSearchAction  searchCoords;
        searchCoords.setType(SoCoordinate3::getClassTypeId());
        searchCoords.setInterest(SoSearchAction::LAST);
        searchCoords.apply(pick->getPath());

        if (searchCoords.getPath() == NULL)
        {
            ROS_ERROR("If we picked a face, we need a node which stores the coordinates!");
            return false;
        }

        SoCoordinate3 * coordNode = dynamic_cast<SoCoordinate3*>(searchCoords.getPath()->getTail());
        if (!coordNode)
        {
            ROS_ERROR("Could not cast SoCoordinate3");
            return false;
        }

        if (fd->getNumPoints() != 3)
        {
            ROS_INFO_STREAM("Face with " << fd->getNumPoints() <<
                            " points can't be used for normal calculation, only triangles supported.");
        }

        int p1 = fd->getPoint(0)->getCoordinateIndex();
        SbVec3f coord1 = coordNode->point[p1];
        int p2 = fd->getPoint(1)->getCoordinateIndex();
        SbVec3f coord2 = coordNode->point[p2];
        int p3 = fd->getPoint(2)->getCoordinateIndex();
        SbVec3f coord3 = coordNode->point[p3];

        SbVec3f diff1(coord2.getValue());
        diff1 -= coord1;
        SbVec3f diff2(coord3.getValue());
        diff2 -= coord1;
        SbVec3f cross = diff1.cross(diff2);
        if (!ccw_face) cross = -cross;

        float x, y, z;
        cross.getValue(x, y, z);
        double len = sqrt(x * x + y * y + z * z);
        x /= len;
        y /= len;
        z /= len;

        normal = Eigen::Vector3d(x, y, z);

        return true;
    }
    return false;
}






/**
 * string which is used for sscanf to extract following information form an SoNode:
 * the name of the link, and the nubmer of the visual. This sscanf should get first an int (visual number), then a string (link name).
 * it should fail for all nodes except those which contain the visual mesh.
 */
#define VISUAL_SCANF "_visual_%i_%s"

SoNode * InventorViewer::getLinkDesc(const SoPath * path, std::string& linkName, int& visualNum)
{
    for (unsigned int i = path->getLength() - 1; i >= 0;  --i)
    {
        SoNode * n = path->getNode(i);
        std::string name = n->getName().getString();
        // ROS_INFO("Pick path len %s\n",name.c_str());
        char ln[1000];
        int num;
        if (sscanf(name.c_str(), VISUAL_SCANF, &num, ln) < 2) continue;
        // ROS_INFO("num: %i rest: %s\n",num,ln);
        linkName = ln; //urdf2inventor::helpers::getFilename(ln); // take only the name after the last '/'
        visualNum = num;
        return n;
    }
    return NULL;
}


void InventorViewer::mouseBtnCB(void *userData, SoEventCallback *_pEvent)
{
    InventorViewer * obj = static_cast<InventorViewer*>(userData);
    if (!obj)
    {
        ROS_ERROR("Invalid UseData passed into mouseBtnCB");
        return;
    }
    
    const SoEvent  *pEvent  = _pEvent->getEvent();

    // general callback:    
    obj->onMouseBtnClick(_pEvent);

    // also see whether part of the model was clicked:
    const SoQtViewer *pViewer = obj->viewer;

    if (SoMouseButtonEvent::isButtonPressEvent(pEvent, SoMouseButtonEvent::BUTTON1))
    {
        SoRayPickAction rayPick(pViewer->getViewportRegion());
        rayPick.setPoint(pEvent->getPosition());
        rayPick.setPickAll(false);
        // rayPick.setRadius(1.0);
        rayPick.apply(pViewer->getSceneManager()->getSceneGraph());
        const SoPickedPoint *pPickedPt = rayPick.getPickedPoint();
        if (pPickedPt != NULL)
        {
            obj->onClickModel(pPickedPt);

            // see if a URDF link was clicked:
            SoPath *pPickPath = pPickedPt->getPath();
            std::string linkName;
            int visualNum;
            SoNode * linkNode = getLinkDesc(pPickPath, linkName, visualNum);
            if (!linkNode)
            {
                ROS_INFO("Clicked on something other than a link");
                return;
            }
            float x, y, z;
            pPickedPt->getObjectPoint(linkNode).getValue(x, y, z);
            ROS_INFO_STREAM("Clicked on "<<linkName<<", at pos "<<x<<", "<<y<<", "<<z); 
        }
    }
}
