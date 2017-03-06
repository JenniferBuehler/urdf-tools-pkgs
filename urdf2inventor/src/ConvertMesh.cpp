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
#include <urdf_traverser/Functions.h>
#include <urdf2inventor/Helpers.h>
#include <urdf2inventor/IVHelpers.h>
#include <urdf2inventor/ConvertMesh.h>
#include <urdf2inventor/MeshConvertRecursionParams.h>

#include <urdf2inventor/AssimpImport.h>
#include <urdf2inventor/Helpers.h>
#include <boost/filesystem.hpp>

#include <Inventor/nodes/SoGroup.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoSelection.h>
#include <Inventor/nodes/SoTexture2.h>
#include <Inventor/nodekits/SoNodeKit.h>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/actions/SoSearchAction.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/importerdesc.h>
#include <assimp/postprocess.h>


using urdf_traverser::UrdfTraverser;
using urdf_traverser::RecursionParams;
using urdf_traverser::LinkRecursionParams;

typedef urdf_traverser::VisualPtr VisualPtr;
typedef urdf_traverser::CollisionPtr CollisionPtr;
typedef urdf_traverser::GeometryPtr GeometryPtr;
typedef urdf_traverser::MaterialPtr MaterialPtr;
typedef urdf_traverser::MeshPtr MeshPtr;
typedef urdf_traverser::SpherePtr SpherePtr;
typedef urdf_traverser::CylinderPtr CylinderPtr;
typedef urdf_traverser::BoxPtr BoxPtr;
//typedef urdf_traverser::Ptr Ptr;

/**
 * Converts the mesh in this file to the inventor format.
 */
SoNode * convertMeshFile(const std::string& filename, double scale_factor, bool setExplicitMaterial = false, double r = 0.5, double g = 0.5, double b = 0.5, double a = 1)
{
//    ROS_INFO("Reading file...");
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(filename, aiProcess_OptimizeGraph |
                           aiProcess_FindInvalidData);
    /*aiProcess_Triangulate |
    aiProcess_OptimizeMeshes |
    /aiProcess_CalcTangentSpace             |
    aiProcess_Triangulate                        |
    aiProcess_JoinIdenticalVertices    |
    aiProcess_SortByPType);*/
    if (!scene || !scene->mRootNode)
    {
        ROS_ERROR_STREAM("Could not import file " << filename);
        return NULL;
    }

    // scale the meshes if required
    if (fabs(scale_factor - 1.0) > 1e-06)
    {
        ROS_INFO_STREAM("Scaling the mesh " << filename << " with factor " << scale_factor);
        // get the scaling matrix
        aiMatrix4x4 scaleTransform;
        aiMatrix4x4::Scaling(aiVector3D(scale_factor, scale_factor, scale_factor), scaleTransform);
        // apply the scaling matrix
        scene->mRootNode->mTransformation *= scaleTransform;
    }

    std::string sceneDir = boost::filesystem::path(filename).parent_path().string();

    SoMaterial * overrideMaterial = NULL;
    if (setExplicitMaterial)
    {
        overrideMaterial = new SoMaterial();
        overrideMaterial->diffuseColor.setValue(r, g, b);
        overrideMaterial->transparency.setValue(1.0 - a);
    }
//    ROS_INFO("Converting to inventor...");
    SoSeparator * ivScene = Assimp2Inventor(scene, sceneDir, overrideMaterial);
    if (!ivScene)
    {
        ROS_ERROR("Could not convert scene");
        return NULL;
    }
    return ivScene;
}


/**
 * Code from https://grey.colorado.edu/coin3d/classSoTexture2.html
 */
void urdf2inventor::removeTextureCopies(SoNode * root)
{
    ROS_INFO("Removing texture copies in the model");

    SoSearchAction sa;
    sa.setType(SoTexture2::getClassTypeId());
    sa.setInterest(SoSearchAction::ALL);
    sa.setSearchingAll(TRUE);
    sa.apply(root);
    SoPathList & pl = sa.getPaths();
    SbDict namedict;

    for (int i = 0; i < pl.getLength(); i++)
    {
        SoFullPath * p = (SoFullPath*) pl[i];
        if (p->getTail()->isOfType(SoTexture2::getClassTypeId()))
        {
            SoTexture2 * tex = (SoTexture2*) p->getTail();
            if (tex->filename.getValue().getLength())
            {
                SbName name = tex->filename.getValue().getString();
                // ROS_INFO_STREAM("Texture file "<<tex->filename.getValue().getString());
                unsigned long key = (unsigned long)((void*) name.getString());
                void * tmp;
                if (!namedict.find(key, tmp))
                {
                    // new texture. just insert into list
                    (void) namedict.enter(key, tex);
                }
                else if (tmp != (void*) tex)   // replace with node found in dict
                {
                    SoGroup * parent = (SoGroup*) p->getNodeFromTail(1);
                    int idx = p->getIndexFromTail(0);
                    parent->replaceChild(idx, (SoNode*) tmp);
                }
            }
        }
    }
    sa.reset();
}

/**
 * Make all texture paths relative to \e basePath. This assumes that
 * when the IV file is written to file, all textures will be saved
 * relative to the directory of the IV file.
 * \return a list of all filenames (*absolute* paths) which were made relative to
 *      \e basePath.
 */
/*std::set<std::string> replaceAbsoluteTexturePaths(SoNode * root, const std::string& basePath)
{
    std::set<std::string> allFiles;
    SoSearchAction sa;
    sa.setType(SoTexture2::getClassTypeId());
    sa.setInterest(SoSearchAction::ALL);
    sa.setSearchingAll(TRUE);
    sa.apply(root);
    SoPathList & pl = sa.getPaths();

    for (int i = 0; i < pl.getLength(); i++)
    {
        SoFullPath * p = (SoFullPath*) pl[i];
        if (!p->getTail()->isOfType(SoTexture2::getClassTypeId())) continue;

        SoTexture2 * tex = (SoTexture2*) p->getTail();
        if (tex->filename.getValue().getLength() == 0) continue;

        std::string _name(tex->filename.getValue().getString());
        SbName name(_name.c_str());

        // ROS_WARN_STREAM("Abs texture file "<<_name<<" relative to "<<basePath);
        boost::filesystem::path absPath(boost::filesystem::absolute(_name));
        std::string cParentPath=urdf_traverser::helpers::getCommonParentPath(basePath,_name);
        // ROS_INFO_STREAM("Parent path :"<<cParentPath);

        std::string relPath;
        if (!urdf_traverser::helpers::getSubdirPath(cParentPath,_name, relPath))
        {
            ROS_ERROR_STREAM("File "<<_name<<" is not a subdirectory of "<<basePath);
            continue;
        }
        ROS_INFO_STREAM("Relative file: "<<relPath);

        tex->filename.setValue(relPath.c_str());
        ROS_INFO("filename set");
        allFiles.insert(absPath.string());
    }
    sa.reset();
    return allFiles;
}*/



// XXX TODO could specify material as parameter to override any possibly already
// existing materials. Propagate this to other functions using getAllGeometry().
bool addGeometry(SoSeparator * addToNode, const std::string& linkName, double scale_factor,
                   const GeometryPtr& geom,
                   const int geomNum,  // number to assign to this visual
                   const MaterialPtr& mat,
                   const urdf_traverser::EigenTransform& geometryTransform,  // transform to the geometry
                   const urdf_traverser::EigenTransform& addMeshTransform, // transform to add only to mesh shapes
                   const bool scaleUrdfTransforms)
{
    
        urdf_traverser::EigenTransform geomTransform = geometryTransform;
        // ROS_INFO_STREAM("Visual "<<i<<" of link "<<linkName<<" transform: "<<geomTransform);
        urdf_traverser::EigenTransform meshGeomTransform = geomTransform * addMeshTransform;

        if (scaleUrdfTransforms) urdf_traverser::scaleTranslation(geomTransform, scale_factor);

        switch (geom->type)
        {
        case urdf::Geometry::MESH:
        {
            // ROS_INFO_STREAM("Mesh for "<<visual->group_name);
            MeshPtr mesh = baselib_binding_ns::dynamic_pointer_cast<urdf::Mesh>(geom);
            if (!mesh.get())
            {
                ROS_ERROR("Mesh cast error");
                return false;
            }
            std::string meshFilename = urdf_traverser::helpers::packagePathToAbsolute(mesh->filename);

            ROS_INFO_STREAM("Converting mesh file " << meshFilename << " with factor " << scale_factor);
            float r = 0.5;
            float g = 0.5;
            float b = 0.5;
            float a = 1.0;
            if (mat)
            {
                r = mat->color.r;
                g = mat->color.g;
                b = mat->color.b;
                a = mat->color.a;
            }
            SoNode * somesh = convertMeshFile(meshFilename, scale_factor, mat != NULL, r, g, b, a);
            // ROS_INFO("Converted.");
            if (!somesh)
            {
                ROS_ERROR("Mesh could not be read");
                return false;
            }
            std::stringstream str;
            str << "_visual_" << geomNum << "_" << linkName;
            // ROS_INFO_STREAM("Visual name "<<str.str());
            somesh->setName(str.str().c_str());
            urdf2inventor::addSubNode(somesh, addToNode, meshGeomTransform);
            break;
        }
        case urdf::Geometry::SPHERE:
        {
            ROS_INFO("Urdf2Inventor: Model has a Sphere");
            SpherePtr sphere = baselib_binding_ns::dynamic_pointer_cast<urdf::Sphere>(geom);
            if (!sphere.get())
            {
                ROS_ERROR("Sphere cast error");
                return false;
            }

            SoSeparator * sphereNode = new SoSeparator();
            sphereNode->ref();
            urdf2inventor::addSphere(addToNode, geomTransform.translation(), sphere->radius * scale_factor, 1, 0, 0);
            break;
        }
        case urdf::Geometry::BOX:
        {
            ROS_INFO("Urdf2Inventor: Model has a box");
            BoxPtr box = baselib_binding_ns::dynamic_pointer_cast<urdf::Box>(geom);
            if (!box.get())
            {
                ROS_ERROR("Box cast error");
                return false;
            }

            SoSeparator * boxNode = new SoSeparator();
            boxNode->ref();
            ROS_INFO_STREAM("Geometry "<<geomNum<<" of link "<<linkName<<" transform: "<<geometryTransform);
            urdf2inventor::EigenTransform tmpT;
            urdf2inventor::EigenTransform geomTransformInv = geomTransform.inverse(); 
            tmpT.setIdentity();
            tmpT.translate(geomTransform.translation());
            tmpT.rotate(geomTransform.rotation());
            urdf2inventor::addBox(addToNode, tmpT, box->dim.x * scale_factor, box->dim.y * scale_factor, box->dim.z * scale_factor, 1, 0, 0, 0);
            break;
        }
        case urdf::Geometry::CYLINDER:
        {
            ROS_INFO("Urdf2Inventor: Model has a cylinder");
            CylinderPtr cylinder = baselib_binding_ns::dynamic_pointer_cast<urdf::Cylinder>(geom);
            if (!cylinder.get())
            {
                ROS_ERROR("Cylinder cast error");
                return false;
            }

            SoSeparator * cylinderNode = new SoSeparator();
            cylinderNode->ref();
            urdf2inventor::addCylinder(addToNode, geomTransform, cylinder->radius/2, cylinder->length, 1, 0, 0, 0); // for some reason, half the radius is required
            break;
        }
        default:
        {
            ROS_ERROR_STREAM("This geometry type not supported so far: " << geom->type);
            return false;
        }
        }

        return true;
}






// XXX TODO could specify material as parameter to override any possibly already
// existing materials. Propagate this to other functions using getAllGeometry().
SoNode * urdf2inventor::getAllGeometry(const urdf_traverser::LinkPtr link, double scale_factor,
                                      const urdf_traverser::EigenTransform& addVisualTransform,
                                      const bool useVisuals,
                                      const bool scaleUrdfTransforms)
{
    SoNodeKit::init();
    SoSeparator * allVisuals = new SoSeparator();
    allVisuals->ref();
    std::string linkName = link->name;

    if (useVisuals)
    {
        unsigned int i = 0;
        for (std::vector<VisualPtr>::const_iterator vit = link->visual_array.begin();
                vit != link->visual_array.end(); ++vit)
        {
            VisualPtr visual = (*vit);
            GeometryPtr geom = visual->geometry;
            MaterialPtr mat = visual->material;

            // ROS_INFO_STREAM("Visual "<<visual->group_name);
            // if (mat) ROS_INFO_STREAM("Material "<<mat->color.r<<", "<<mat->color.g<<", "<<mat->color.b<<", "<<mat->color.a);

            urdf_traverser::EigenTransform vTransform = urdf_traverser::getTransform(visual->origin);
            // ROS_INFO_STREAM("Visual "<<i<<" of link "<<link->name<<" transform: "<<visual->origin);

            if (!addGeometry(allVisuals, linkName, scale_factor,
                       geom, i, mat, vTransform, addVisualTransform, scaleUrdfTransforms))
            {
                ROS_ERROR_STREAM("Could not add geometry of link "<<link->name);
                return NULL;
            }
            ++i;
        }
    }
    else
    {
        unsigned int i = 0;
        for (std::vector<CollisionPtr>::const_iterator cit = link->collision_array.begin();
                cit != link->collision_array.end(); ++cit)
        {
            CollisionPtr coll = (*cit);
            GeometryPtr geom = coll->geometry;
            // XXX TODO allow to set a material for collision shapes, which inherently don't have materials.
            // Because collision geometries don't match to visual geometries
            // (could be different numbers!), we can't infer from visual.
            MaterialPtr mat(new urdf::Material());
            mat->color.r=0.1;
            mat->color.g=0.1;
            mat->color.b=0.5;
            mat->color.a=1;

            urdf_traverser::EigenTransform cTransform = urdf_traverser::getTransform(coll->origin);
            // ROS_INFO_STREAM("Collision geometry "<<i<<" of link "<<link->name<<" transform: "<<coll->origin);

            if (!addGeometry(allVisuals, linkName, scale_factor,
                       geom, i, mat, cTransform, addVisualTransform, scaleUrdfTransforms))
            {
                ROS_ERROR_STREAM("Could not add geometry of link "<<link->name);
                return NULL;
            }
            ++i;
        }
    }

    std::stringstream str;
    str << "_" << linkName;

    allVisuals->setName(str.str().c_str());

    return allVisuals;
}


/**
 * Helper function which gets all visuals from \e link, converts it to
 * IV format and writes it to \e resultIV (after scaling meshes by \e scale_factor,
 * and optionally also the transforms if \e scaleUrdfTransforms is true).
 * It also returns all \e textureFiles (absolute paths to files) in use
 * by the link.
 */
bool convertMeshToIVString(urdf_traverser::LinkPtr& link,
                           const float scale_factor,
                           const urdf_traverser::EigenTransform& addVisualTransform,
                           const bool useVisuals,
                           const bool scaleUrdfTransforms,
                           std::string& resultIV,
                           std::set<std::string>& textureFiles)
{
    ROS_INFO("Convert mesh for link '%s'", link->name.c_str());

    SoNode * allVisuals = urdf2inventor::getAllGeometry(link, scale_factor, addVisualTransform, useVisuals, scaleUrdfTransforms);
    if (!allVisuals)
    {
        ROS_ERROR("Could not get visuals");
        return false;
    }

    // write the node to IV XML format
    if (!urdf2inventor::writeInventorFileString(allVisuals, resultIV))
    {
        ROS_ERROR("Could not get the mesh file content");
        return false;
    }

    //ROS_INFO_STREAM("Result file content: "<<resultIV);

    // collect all relative texture filenames from the absolute texture paths.
    textureFiles =  urdf2inventor::getAllTexturePaths(allVisuals);
    return true;
}


/**
 * Callback function to be called during recursion incurred in convertMeshes().
 * Only supports string mesh formats with MeshConvertRecursionParams<std::string>.
 */
int convertMeshToIVString(urdf_traverser::RecursionParamsPtr& p)
{
    typedef urdf2inventor::MeshConvertRecursionParams<std::string> MeshConvertRecursionParamsT;
    typename MeshConvertRecursionParamsT::Ptr param = baselib_binding_ns::dynamic_pointer_cast<MeshConvertRecursionParamsT>(p);
    if (!param.get())
    {
        ROS_ERROR("Wrong recursion parameter type");
        return -1;
    }

    bool useVisuals=true;  // XXX TODO: Parameterize
    urdf_traverser::LinkPtr link = param->getLink();
    std::string resultFileContent;
    std::set<std::string> textureFiles;
    if (!convertMeshToIVString(link, param->factor, param->getVisualTransform(), useVisuals, false, resultFileContent, textureFiles))
        return -1;

    //ROS_INFO_STREAM("Result file content: "<<resultFileContent);
    if (!param->resultMeshes.insert(std::make_pair(link->name, resultFileContent)).second)
    {
        ROS_ERROR("Could not insert the resulting mesh file for link %s to the map", link->name.c_str());
        return -1;
    }

    param->textureFiles[link->name].insert(textureFiles.begin(), textureFiles.end());
    return 1;
}

template <class MeshFormat>
bool urdf2inventor::convertMeshes(urdf_traverser::UrdfTraverser& traverser,
                                  const std::string& fromLink,
                                  const float scaleFactor,
                                  const std::string& material,
                                  const std::string& file_extension,
                                  const urdf_traverser::EigenTransform& addVisualTransform,
                                  std::map<std::string, MeshFormat>& meshes,
                                  std::map<std::string, std::set<std::string> >& textureFiles)
{
    std::string startLinkName = fromLink;
    if (startLinkName.empty())
    {
        startLinkName = traverser.getRootLinkName();
    }

    urdf_traverser::LinkPtr startLink = traverser.getLink(startLinkName);
    if (!startLink.get())
    {
        ROS_ERROR("Link %s does not exist", startLinkName.c_str());
        return false;
    }

    // do one call of convertMeshes
    typedef urdf2inventor::MeshConvertRecursionParams<MeshFormat> MeshConvertRecursionParamsT;
    typename MeshConvertRecursionParamsT::Ptr meshParams(
        new MeshConvertRecursionParamsT(scaleFactor, material, file_extension, addVisualTransform));

    urdf_traverser::RecursionParamsPtr p(meshParams);

    // go through entire tree
    if (traverser.traverseTreeTopDown(startLinkName, boost::bind(&convertMeshToIVString, _1), p, true) <= 0)
    {
        ROS_ERROR("Could nto convert meshes.");
        return false;
    }
    meshes = meshParams->resultMeshes;
    textureFiles = meshParams->textureFiles;

    return true;
}


bool urdf2inventor::fixTextureReferences(
    const std::string& relMeshDir,
    const std::string& relTexDir,
    const std::map<std::string, std::set<std::string> >& textureFiles,
    std::map<std::string, std::string>& meshes,
    std::map<std::string, std::set<std::string> >& texturesToCopy)
{
    // determine common parent path of all textures. First, remove texture duplicates
    std::set<std::string> allTextures;
    for (std::map<std::string, std::set<std::string> >::const_iterator it = textureFiles.begin(); it != textureFiles.end(); ++it)
        allTextures.insert(it->second.begin(), it->second.end());

    // no textures to process
    if (allTextures.empty()) return true;

    ROS_INFO_STREAM("Fixing texture references to point from " << relMeshDir << " files to textures in " << relTexDir);
    std::string _relMeshDir(relMeshDir);
    std::string _relTexDir(relTexDir);
    urdf_traverser::helpers::enforceDirectory(_relMeshDir, true);
    urdf_traverser::helpers::enforceDirectory(_relTexDir, true);

    std::string commonParent;
    if (!urdf_traverser::helpers::getCommonParentPath(allTextures, commonParent))
    {
        ROS_ERROR_STREAM("Could not find common parent path of all textures");
        return false;
    }

    ROS_INFO_STREAM("Common parent path for all textures: " << commonParent);

    for (std::map<std::string, std::set<std::string> >::const_iterator it = textureFiles.begin(); it != textureFiles.end(); ++it)
    {
        std::string meshFile = it->first;
        std::string meshDirectory = urdf_traverser::helpers::getDirectory(meshFile);

        std::stringstream relMeshInstallFile;
        relMeshInstallFile << _relMeshDir << meshDirectory;


        typename std::map<std::string, std::string>::iterator meshString = meshes.find(meshFile);
        if (meshString == meshes.end())
        {
            ROS_ERROR_STREAM("Consistency: Mesh " << meshFile << " should have been in meshes map");
            return false;
        }

        if (!urdf2inventor::helpers::fixFileReferences(relMeshInstallFile.str(), _relTexDir, commonParent, it->second,
                meshString->second, texturesToCopy))
        {
            ROS_ERROR_STREAM("Could not fix file references");
            return false;
        }
    }
    return true;
}


// instantiation for string meshes
template bool urdf2inventor::convertMeshes<std::string>(urdf_traverser::UrdfTraverser& traverser,
        const std::string& fromLink,
        const float scaleFactor,
        const std::string& material,
        const std::string& file_extension,
        const urdf_traverser::EigenTransform& addVisualTransform,
        std::map<std::string, std::string>& meshes,
        std::map<std::string, std::set<std::string> >& textureFiles);

