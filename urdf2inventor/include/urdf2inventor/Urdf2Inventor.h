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
#ifndef URDF2INVENTOR_URDF2INVENTOR_H
#define URDF2INVENTOR_URDF2INVENTOR_H
// Copyright Jennifer Buehler

//-----------------------------------------------------
#include <urdf2inventor/ConversionResult.h>
#include <urdf2inventor/MeshConvertRecursionParams.h>

#include <urdf_traverser/UrdfTraverser.h>

#include <baselib_binding/SharedPtr.h>

#include <iostream>
#include <string>
#include <map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <Inventor/nodes/SoSeparator.h>


namespace urdf2inventor
{

/**
 * \brief This class provides functions to transform a robot described in URDF to the inventor format.
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

    typedef MeshConvertRecursionParams<MeshFormat> MeshConvertRecursionParamsT;
    typedef baselib_binding::shared_ptr<MeshConvertRecursionParamsT>::type MeshConvertRecursionParamsPtr;

    typedef baselib_binding::shared_ptr<urdf_traverser::UrdfTraverser>::type UrdfTraverserPtr;
    typedef baselib_binding::shared_ptr<const urdf_traverser::UrdfTraverser>::type UrdfTraverserConstPtr;

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
    // within the output directory specified in the node, another directory is going to be created
    // to contain the texture files. The name of this directory can be specified here.
    static std::string TEX_OUTPUT_DIRECTORY_NAME;

    /**
     * \param traverser the URDF traverser.
     *    Does not need to have a loaded URDF, as method loadAndConvert()
     *    will load the URDF into it.
     * \param _scaleFactor the graspit model might have to be scaled (the urdf model is
     *    in meters, graspit! in millimeters). This can be specified with this scale factor.
     */
    explicit Urdf2Inventor(const UrdfTraverserPtr& traverser,
                           float _scaleFactor = 1):
        urdf_traverser(traverser),
        scaleFactor(_scaleFactor),
        isScaled(false)
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
     * \param textureFiles if not NULL, a list of all texture files (absolute paths) in use are returned here.
     */
    SoNode * getAsInventor(const std::string& fromLink, bool useScaleFactor,
                           bool addAxes, float axesRadius, float axesLength,
                           const EigenTransform& addVisualTransform,
                           std::set<std::string> * textureFiles);

    /**
     * writes all elements down from \e fromLink to files in inventor format.
     * \param outputFilename has to be an inventor filename
     * \param fromLink if empty string, the root link in the URDF is going to be used as starting point. Otherwise, a link
     *      name can be set here which will write the model starting from this link name.
     * \param addVisualTransform this transform will be post-multiplied on all links' **visuals** (not links!) local
     *      transform (their "origin"). This can be used to correct transformation errors which may have been
     *      introduced in converting meshes from one format to the other, losing orientation information
     *      (for example, .dae has an "up vector" definition which may have been ignored)
     * \param _addAxes default value: add the local coordinate system axes of the links to the inventor nodes.
     *      z axis is displayed blue, y axis green, x axis red, and the rotation axis pink.
     *      Fixed joints axes will be artificially altered to be slightly longer and thinner, so a distinction is
     *      visible.
     * \param _axesRadius default value: radius of the axes, if \e _addAxes is true
     * \param _axesLength default value: length of the axes, if \e _addAxes is true

     */
    bool writeAsInventor(const std::string& outputFilename,  const std::string& fromLink /*= ""*/,
                         bool useScaleFactor /*= true*/, const EigenTransform& addVisualTransform,
                         bool _addAxes = false, float _axesRadius = 0.003, float _axesLength = 0.015);

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
     * Constructs a new ConversionParameters object with basic parameters.
     * \param rootLink if empty string, the root link in the URDF is going to be used. Otherwise, a link
     *      name can be set here which will convert the model starting from this link name.
     */
    ConversionParametersPtr getBasicConversionParams(const std::string& rootLink,
            const std::string& material,
            const EigenTransform& addVisualTransform)
    {
        return ConversionParametersPtr(new ConversionParameters(rootLink, material, addVisualTransform));
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
     * \param meshParams optional: pre-instantiated MeshConvertRecursionParams. Can be
     *   left null and then default is constructed.
     */
    virtual ConversionResultPtr convert(const ConversionParametersPtr& params,
                                        const MeshConvertRecursionParamsPtr& meshParams=MeshConvertRecursionParamsPtr());

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

    UrdfTraverserPtr getTraverser()
    {
        return urdf_traverser;
    }

    UrdfTraverserConstPtr readTraverser() const
    {
        return urdf_traverser;
    }


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

    EigenTransform getTransform(const LinkPtr& from_link,  const JointPtr& to_joint);

private:

    /**
     * Recursive function which returns an inventor node for all links down from (and including) from_link.
     * See other getAsInventor() function.
     */
    SoNode * getAsInventor(const LinkPtr& from_link, bool useScaleFactor,
                           bool _addAxes, float _axesRadius, float _axesLength,
                           const EigenTransform& addTransform,
                           std::set<std::string> * textureFiles);

    /**
     * Writes the contents of SoNode into the file of given name.
     */
    bool writeInventorFile(SoNode * node, const std::string& filename);

    /**
     * Writes all elements down from from_link to a file in inventor format.
     * \param outFilename has to be an inventor filename
     */
    bool writeAsInventor(const std::string& outFilename, const LinkPtr& from_link,
                         bool useScaleFactor /*= true*/, const EigenTransform& addTransform,
                         bool _addAxes = false, float _axesRadius = 0.003, float _axesLength = 0.015);


    UrdfTraverserPtr urdf_traverser;

    // The graspit model might ahve to be scaled compared to the urdf model, this is the scale factor which does that.
    float scaleFactor;
    bool isScaled;
};

}  //  namespace urdf2inventor
#endif   // URDF2INVENTOR_URDF2INVENTOR_H
