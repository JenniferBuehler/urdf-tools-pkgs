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

#ifndef URDF_VIEWER_INVENTORVIEWER_H
#define URDF_VIEWER_INVENTORVIEWER_H
// Copyright Jennifer Buehler

#include <Inventor/Qt/SoQt.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoSelection.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoMaterial.h>

#include <Eigen/Geometry>

#include <vector>
#include <map>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace urdf_viewer
{
/**
 * \brief Runs an inventor viewer.
 * Allows to click points on the model, which
 * can be handled in different ways by subclasses.
 *
 * \author Jennifer Buehler
 * \date March 2016
 */
class InventorViewer
{
public:

    /**
     * \param _faces_ccw faces are to be treated as counter-clockwise. Needed for normal calculations.
     */
    explicit InventorViewer(bool _faces_ccw = true);

    InventorViewer(const InventorViewer& o);
    ~InventorViewer();


    /**
     * Initialize Inventor and Qt
     * TODO: For some reason I haven't yet further investigated, this has to be called **after**
     * functions as Urdf2Inventor::loadAndGetAsInventor(), or it won't work. Find out why, and fix it.
     */
    void init(const char * windowName = "InventorViewer", float bck_r=0.3, float bck_g=0.3, float bck_b=0.3);

    /**
     * Load a model into the viewer
     */
    void loadModel(SoNode * model);

    /**
     * Load a model from an inventor file
     */
    bool loadModel(const std::string& filename);

    void runViewer();

protected:

    /**
     * For subclasses: Called when user clicks on part of the model
     */
    virtual void onClickModel(const SoPickedPoint * pickPoint) {}
    /**
     * For subclasses: on any mouse button click
     */
    virtual void onMouseBtnClick(SoEventCallback *pNode) {}

    /**
     * Calculates the correct face normal of the pick point.
     * \param shapeIdx output: the index in the \e pick path at which the actual shape resides. 
     *      Normal coordinates are in the frame of this shape.
     */
    static bool computeCorrectFaceNormal(const SoPickedPoint * pick, bool ccw_face, Eigen::Vector3d& normal, int& shapeIdx);

    /**
     * Helper function which can be used to find a specific node along the path which is formatted in
     * a given way, such that a a number and a name can be extracted.
     * The last such node found along the path is returned, along with the extracted number and string as paramter.
     * \param sscanfStr input: the string to use in sscanf, which must extract *first* a number (int),
     *      *then* a string for the name. This could be for example "_visual_%i_%s"
     * \param str output: extracted string / name 
     * \param num output: extracted number
     * \param pathIdx output: index the link has in the path.
     */
    static SoNode * getIntStr(const std::string& sscanfStr, const SoPath * path, std::string& str, int& num, int& pathIdx);

    SoQtExaminerViewer * getViewer()
    {
        return viewer;
    }

    SoSelection * getRoot()
    {
        return root;
    }

    bool isFacesCCW() const
    {
        return faces_ccw;
    }

private:

    static void mouseBtnCB(void *userData, SoEventCallback *pNode);

    QWidget * viewWindow;
    SoQtExaminerViewer * viewer;
    bool faces_ccw;
    SoSelection * root;
};

}  //  namespace urdf_viewer
#endif   // URDF_VIEWER_INVENTORVIEWER_H
