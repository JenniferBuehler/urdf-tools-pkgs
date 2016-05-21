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

#ifndef URDF2INVENTOR_URDF2INVENTOR_H
#define URDF2INVENTOR_URDF2INVENTOR_H
// Copyright Jennifer Buehler

//-----------------------------------------------------
#include <urdf_traverser/UrdfTraverser.h>

#include <iostream>
#include <string>
#include <map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoCylinder.h>

#include <baselib_binding/SharedPtr.h>
#include <urdf2inventor/ConversionResult.h>

namespace urdf2inventor
{

/**
 * \brief This class provides functions to transform a robot described in URDF to the inventor format.
 *
 * Careful: So far, only meshes with .stl and .obj extensions have been tested. There were problems
 * with some .stl meshes and ivcon however (it idles forever, crashes, or even freezes the screen).
 * So far, the best solution is to convert all meshes to .obj beforehand. Package assimp_mesh_converter
 * can be used to do this. At some time (hopefully soon) this will be automated here, and dependency to
 * package ivcon should be removed.
 * 
 * TODO: This should be separated in 2 different hierarchies, one to handle the URDF traversal
 * to apply all sorts of functions, and another to do the operations such as mesh conversion and model scaling.
 *
 * \author Jennifer Buehler
 * \date March 2016
 */
class Urdf2Inventor
{
public:
    // the .iv files are represented as strings
    typedef std::string MeshFormat;
    typedef ConversionResult<MeshFormat> ConversionResultT;
    typedef baselib_binding::shared_ptr<ConversionResultT>::type ConversionResultPtr;
    typedef baselib_binding::shared_ptr<ConversionParameters>::type ConversionParametersPtr;
    
    typedef baselib_binding::shared_ptr<urdf_traverser::UrdfTraverser>::type UrdfTraverserPtr;

    typedef urdf_traverser::EigenTransform EigenTransform;
    typedef urdf_traverser::LinkPtr LinkPtr;
    typedef urdf_traverser::LinkConstPtr LinkConstPtr;
    typedef urdf_traverser::JointPtr JointPtr;
    typedef urdf_traverser::JointConstPtr JointConstPtr;
    //typedef urdf_traverser::;

    // output file format to convert the meshes to
    static std::string OUTPUT_EXTENSION;

    // within the output directory specified in the node, another directory is going to be created
    // to contain the mesh files. The name of this directory can be specified here.
    static std::string MESH_OUTPUT_DIRECTORY_NAME;

    /**
     * \param traverser the URDF traverser. Does not need to have a loaded URDF, as method loadAndConvert() will load the URDF into it. 
     * \param _scaleFactor the graspit model might have to be scaled (the urdf model is in meters, graspit! in millimeters).
     * This can be specified with this scale factor.
     * \param _addAxes default value: add the local coordinate system axes of the links to the inventor nodes. 
     *      z axis is displayed blue, y axis green, x axis red, and the rotation axis pink.
     *      Fixed joints axes will be artificially altered to be slightly longer and thinner, so a distinction is
     *      visible.
     * \param _axesRadius default value: radius of the axes, if \e _addAxes is true 
     * \param _axesLength default value: length of the axes, if \e _addAxes is true
     */
    explicit Urdf2Inventor(const UrdfTraverserPtr& traverser,
            float _scaleFactor = 1, bool _addAxes=false, float _axesRadius = 0.003, float _axesLength=0.015):
        urdf_traverser(traverser),
        scaleFactor(_scaleFactor),
        isScaled(false),
        addAxes(_addAxes),
        axesRadius(_axesRadius),
        axesLength(_axesLength)
     {
         assert(urdf_traverser.get());
     }

    ~Urdf2Inventor()
    {
    }

    /**
     * Removes all fixed links in the model by adding visuals and collision geometry to the first parent link which is
     * attached to a non-fixed link. Model has to be loaded with any of the load() methods first. 
     */
    bool joinFixedLinks(const std::string& from_link);

    /**
     * Transform the URDF such that all rotation axises (in the joint's local reference frame) are this axis
     */
    bool allRotationsToAxis(const std::string& fromLinkName, const Eigen::Vector3d& axis);


    /**
     * Returns an inventor node for all links down from (and including) from_link.
     * IMPORTANT: This will also load the model, so any other URDF model previously loaded will be overwritten.
     *
     * \param useScaleFactor if set to true, the model is scaled up using scale factor set in constructor.
     * \param fromLink if empty string, the root link in the URDF is going to be used. Otherwise, a link
     *      name can be set here which will return the model starting from this link name.
     */
    // SoNode * loadAndGetAsInventor(const std::string& filename, const std::string fromLink="", bool useScaleFactor = true);

    /**
     * Returns an inventor node for all links down from (and including) \e from_link. The model needs to be loaded
     * first by any of the load functions.
     *
     * \param useScaleFactor if set to true, the model is scaled up using scale factor set in constructor.
     * \param fromLink if empty string, the root link in the URDF is going to be used. Otherwise, a link
     *      name can be set here which will return the model starting from this link name.
     *
     * \param addAxes add the local coordinate system axes of the links to the inventor nodes. 
     *      z axis is displayed blue, y axis green, x axis red, and the rotation axis pink.
     *      Fixed joints axes will be artificially altered to be slightly longer and thinner, so a distinction is
     *      visible.
     * \param axesRadius radius of the axes, if \e _addAxes is true 
     * \param axesLength length of the axes, if \e _addAxes is true
     * \param addVisualTransform this transform will be post-multiplied on all links' **visuals** (not links!) local
     *      transform (their "origin"). This can be used to correct transformation errors which may have been 
     *      introduced in converting meshes from one format to the other, losing orientation information
     *      (for example, .dae has an "up vector" definition which may have been ignored)
     */
    SoNode * getAsInventor(const std::string& fromLink, bool useScaleFactor,
        bool addAxes, float axesRadius, float axesLength, const EigenTransform& addVisualTransform);

    /**
     * writes all elements down from \e fromLink to files in inventor format.
     * \param outputFilename has to be an inventor filename
     * \param fromLink if empty string, the root link in the URDF is going to be used as starting point. Otherwise, a link
     *      name can be set here which will write the model starting from this link name.
     * \param addVisualTransform this transform will be post-multiplied on all links' **visuals** (not links!) local
     *      transform (their "origin"). This can be used to correct transformation errors which may have been 
     *      introduced in converting meshes from one format to the other, losing orientation information
     *      (for example, .dae has an "up vector" definition which may have been ignored)
     */
    bool writeAsInventor(const std::string& outputFilename,  const std::string& fromLink /*= ""*/, 
        bool useScaleFactor /*= true*/, const EigenTransform& addVisualTransform);

    /**
     * Loads the URDF file into the UrdfTraverser (instance passed into the constructor),
     * then joins fixed links (only if \e joinFixed) and then converts the URDF file to
     * inventor mesh files by calling convert(). 
     */
    ConversionResultPtr loadAndConvert(const std::string& urdfFilename,
        bool joinFixed,
        const ConversionParametersPtr& params);


    /**
     * Loads the URDF model from file into the UrdfTraverser (instance passed into the constructor),
     */ 
    bool loadModelFromFile(const std::string& urdfFilename);

    /**
     * \param rootLink if empty string, the root link in the URDF is going to be used. Otherwise, a link
     *      name can be set here which will convert the model starting from this link name.
     * \param addVisualTransform this transform will be post-multiplied on all links' **visuals** (not links!) local
     *      transform (their "origin"). This can be used to correct transformation errors which may have been 
     *      introduced in converting meshes from one format to the other, losing orientation information
     *      (for example, .dae has an "up vector" definition which may have been ignored)
     */
    ConversionParametersPtr getBasicConversionParams(const std::string& rootLink/*=""*/,
        const std::string& material/*="plastic"*/, const EigenTransform& addVisualTransform)
    {
        return ConversionParametersPtr(new ConversionParameters(rootLink,material, addVisualTransform));
    }

    /**
     * Method which does the conversion from URDF to Inventor. This
     * fist calls the implementation-specific preConvert(),
     * then scales the model (using the scale factor specified
     * in the constructor, unless the model was already scaled before),
     * then converts the individual links mesh files
     * and finally calls implementation-specific postConvert().
     * To save an inventor file of the *whole* robot, use writeAsInventor() instead.
     *
     * \param params parameters of the conversion
     */
    virtual ConversionResultPtr convert(const ConversionParametersPtr& params);

    /**
     * Prints the structure of the URDF to standard out
     */
    bool printModel();

    /**
     * prints the URDF model to standard out, starting from this link.
     */
    bool printModel(const std::string& fromLink);

    /**
     * Prints the joint names. Can't be const in this version because
     * of the use of a recursive method.
     */
    void printJointNames(const std::string& fromLink);

    /**
     * Returns all joint names in depth-frist search order starting from \e fromLink (or from root if
     * \e fromLink is empty)
     */
    bool getJointNames(const std::string& fromLink, const bool skipFixed, std::vector<std::string>& result);

    /**
     * Cleans up all temporary files written to disk.
     */
    void cleanup();

protected:

  
    /**
     * Method called by convert() which can be implemented by subclasses to return a different type of ConversionResult
     * and do any operations required *before* the main body of convert() is being done.
     * See also postConvert().
     * \param rootLink the conversion is to be done starting from this link.
     * \return NULL/invalid shared ptr if pre-conversion failed.
     */ 
    virtual ConversionResultPtr preConvert(const ConversionParametersPtr& params); 

    /**
     * Method called by convert() which can be implemented by subclasses to do any operations
     * required *after* the main body of convert() has been done.
     * See also preConvert().
     * \param rootLink the conversion is to be done starting from this link.
     * \param result the conversion result so far. This is the object which has been returned by preConvert(),
     *      with changes applied by the main body of convert() applied to the base class fields.
     * \return Has to be the same as \e result, with possible changes applied.
     *      Set ConversionResult::success to false to indicate failure, or to true to indicate success.
     */ 
    virtual ConversionResultPtr postConvert(const ConversionParametersPtr& params, ConversionResultPtr& result); 

    /**
     * scale the model and the meshes
     */
    bool scale();
   
    inline float getScaleFactor() const
    {
        return scaleFactor;
    }

    void addLocalAxes(const LinkConstPtr& link, SoSeparator * addToNode, bool useScaleFactor,
        float _axesRadius, float _axesLength) const;

 
private:

    /**
     * Recursive function which returns an inventor node for all links down from (and including) from_link.
     * See other getAsInventor() function.
     */
    SoNode * getAsInventor(const LinkPtr& from_link, bool useScaleFactor, 
        bool _addAxes, float _axesRadius, float _axesLength, const EigenTransform& addTransform);

    /**
     * Writes the contents of SoNode into the file of given name.
     */
    bool writeInventorFile(SoNode * node, const std::string& filename);

    /**
     * Writes all elements down from from_link to a file in inventor format.
     * \param outFilename has to be an inventor filename
     */
    bool writeAsInventor(const std::string& outFilename, const LinkPtr& from_link,
        bool useScaleFactor /*= true*/, const EigenTransform& addTransform);


    UrdfTraverserPtr urdf_traverser;

    // The graspit model might ahve to be scaled compared to the urdf model, this is the scale factor which does that.
    float scaleFactor;
    bool isScaled;

    bool addAxes;
    float axesRadius;
    float axesLength;
};

}  //  namespace urdf2inventor
#endif   // URDF2INVENTOR_URDF2INVENTOR_H
