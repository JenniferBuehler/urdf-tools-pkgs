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
SoNode * convertMeshFile(const std::string& filename, double scale_factor, bool setExplicitMaterial = false, double r=0.5, double g=0.5, double b=0.5, double a=1)
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
    if( !scene || !scene->mRootNode)
    {
        ROS_ERROR_STREAM("Could not import file "<<filename);
        return NULL;
    }

    // scale the meshes if required
    if (fabs(scale_factor - 1.0) > 1e-06)
    {
        ROS_INFO_STREAM("Scaling the mesh "<<filename<< " with factor "<<scale_factor);
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
        overrideMaterial->diffuseColor.setValue(r,g,b);
        overrideMaterial->transparency.setValue(1.0-a);
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

    for (int i = 0; i < pl.getLength(); i++) {
      SoFullPath * p = (SoFullPath*) pl[i];
      if (p->getTail()->isOfType(SoTexture2::getClassTypeId())) {
        SoTexture2 * tex = (SoTexture2*) p->getTail();
        if (tex->filename.getValue().getLength()) {
          SbName name = tex->filename.getValue().getString();
          // ROS_INFO_STREAM("Texture file "<<tex->filename.getValue().getString());
          unsigned long key = (unsigned long) ((void*) name.getString());
          void * tmp;
          if (!namedict.find(key, tmp)) {
            // new texture. just insert into list
            (void) namedict.enter(key, tex);
          }
          else if (tmp != (void*) tex) { // replace with node found in dict
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



SoNode * urdf2inventor::getAllVisuals(const urdf_traverser::LinkPtr link, double scale_factor,
    const urdf_traverser::EigenTransform& addVisualTransform,
    bool scaleUrdfTransforms)
{
    SoNodeKit::init();
    SoSeparator * allVisuals = new SoSeparator();
    allVisuals->ref();
    std::string linkName = link->name;
    unsigned int i = 0;
    for (std::vector<VisualPtr>::const_iterator vit = link->visual_array.begin();
            vit != link->visual_array.end(); ++vit)
    {
        VisualPtr visual = (*vit);
        GeometryPtr geom = visual->geometry;
        MaterialPtr mat = visual->material;

        // ROS_INFO_STREAM("Visual "<<visual->group_name);
        if (mat) ROS_INFO_STREAM("Material "<<mat->color.r<<", "<<mat->color.g<<", "<<mat->color.b<<", "<<mat->color.a);

        urdf_traverser::EigenTransform vTransform = urdf_traverser::getTransform(visual->origin);
        // ROS_INFO_STREAM("Visual "<<i<<" of link "<<link->name<<" transform: "<<visual->origin);

        vTransform=vTransform*addVisualTransform;
        
        if (scaleUrdfTransforms) urdf_traverser::scaleTranslation(vTransform, scale_factor);

        switch(geom->type)
        {
            case urdf::Geometry::MESH:
            {
                // ROS_INFO_STREAM("Mesh for "<<visual->group_name);
                MeshPtr mesh = baselib_binding_ns::dynamic_pointer_cast<urdf::Mesh>(geom);
                if (!mesh.get())
                {
                    ROS_ERROR("Mesh cast error");
                    return NULL;
                }
                std::string meshFilename = urdf_traverser::helpers::packagePathToAbsolute(mesh->filename);

                ROS_INFO_STREAM("Converting mesh file "<<meshFilename<<" with factor "<<scale_factor);
                float r = 0.5;
                float g = 0.5;
                float b = 0.5;
                float a = 1.0;
                if (mat)
                {
                    r=mat->color.r;
                    g=mat->color.g;
                    b=mat->color.b;
                    a=mat->color.a;
                }
                SoNode * somesh = convertMeshFile(meshFilename,scale_factor,mat!=NULL,r,g,b,a);
                // ROS_INFO("Converted.");
                if (!somesh)
                {
                    ROS_ERROR("Mesh could not be read");
                    return NULL;
                }
                std::stringstream str;
                str << "_visual_" << i << "_" << linkName;
                // ROS_INFO_STREAM("Visual name "<<str.str());
                somesh->setName(str.str().c_str());
                allVisuals = urdf2inventor::addSubNode(somesh, allVisuals, vTransform);
                break;
            }
            case urdf::Geometry::SPHERE:
            {
                ROS_INFO("Urdf2Inventor debug: Model has a Sphere");
                SpherePtr sphere = baselib_binding_ns::dynamic_pointer_cast<urdf::Sphere>(geom);
                if (!sphere.get())
                {
                    ROS_ERROR("Sphere cast error");
                    return NULL;
                }

                SoSeparator * sphereNode = new SoSeparator();
                sphereNode->ref();
                urdf2inventor::addSphere(allVisuals, vTransform.translation(), sphere->radius * scale_factor, 1,0,0);
                break; 
            }
            case urdf::Geometry::BOX:
            {
                ROS_INFO("Urdf2Inventor debug: Model has a box");
                BoxPtr box = baselib_binding_ns::dynamic_pointer_cast<urdf::Box>(geom);
                if (!box.get())
                {
                    ROS_ERROR("Box cast error");
                    return NULL;
                }

                SoSeparator * boxNode = new SoSeparator();
                boxNode->ref();
                urdf2inventor::addBox(allVisuals, vTransform, box->dim.x * scale_factor, box->dim.y * scale_factor, box->dim.z * scale_factor, 1,0,0,0);
                break; 
            }
            default:
            {
                ROS_ERROR_STREAM("This geometry type not supported so far: "<<geom->type);
                return NULL;
            }
        }
        ++i;
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
        const bool scaleUrdfTransforms,
        std::string& resultIV,
        std::set<std::string>& textureFiles)
{
    ROS_INFO("Convert mesh for link '%s'",link->name.c_str());

    SoNode * allVisuals = urdf2inventor::getAllVisuals(link, scale_factor, addVisualTransform, scaleUrdfTransforms);
    if (!allVisuals)
    {
        ROS_ERROR("Could not get visuals");
        return false;
    }

    // write the node to IV XML format
    std::string resultFileContent;
    if (!urdf2inventor::writeInventorFileString(allVisuals, resultIV))
    {
        ROS_ERROR("Could not get the mesh file content");
        return false;
    }

    //ROS_INFO_STREAM("Result file content: "<<resultIV);
  
    // collect all relative texture filenames from the absolute texture paths. 
    std::set<std::string> allFiles =  urdf2inventor::getAllTexturePaths(allVisuals);
    textureFiles.insert(allFiles.begin(), allFiles.end());
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

    urdf_traverser::LinkPtr link = param->getLink();
    std::string resultFileContent;
    std::set<std::string> textureFiles;
    if (!convertMeshToIVString(link, param->factor, param->addVisualTransform, false, resultFileContent, textureFiles))
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
    std::string startLinkName=fromLink;
    if (startLinkName.empty()){
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
    for (std::map<std::string, std::set<std::string> >::const_iterator it=textureFiles.begin(); it!=textureFiles.end(); ++it)
        allTextures.insert(it->second.begin(), it->second.end());

    ROS_INFO_STREAM("Fixing texture references to point from "<<relMeshDir<<" files to textures in "<<relTexDir);
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

    ROS_INFO_STREAM("Common parent path for all textures: "<<commonParent);

    for (std::map<std::string, std::set<std::string> >::const_iterator it=textureFiles.begin(); it!=textureFiles.end(); ++it)
    {
        std::string meshFile = it->first;
        std::string meshDirectory = urdf_traverser::helpers::getDirectory(meshFile);
        
        std::stringstream relMeshInstallFile;
        relMeshInstallFile << _relMeshDir << meshDirectory;
        
        for (std::set<std::string>::iterator itTex=it->second.begin(); itTex!=it->second.end(); ++itTex)
        {
            std::string absTexFile = *itTex;
            std::string relTexFile;
            if (!urdf_traverser::helpers::getSubdirPath(commonParent, absTexFile, relTexFile))
            {
                ROS_ERROR_STREAM("File "<<absTexFile<<" is not in a subdirectory of "<<commonParent);
                continue;
            }
            // ROS_INFO_STREAM("Relative file: "<<relTexFile);
            // ROS_INFO_STREAM("Mesh file: "<<meshFile<<" tex file: "<<absTexFile);
            
            std::stringstream relTexInstallFile;
            relTexInstallFile << _relTexDir << relTexFile;
        
            std::string newTexReference;
            if (!urdf_traverser::helpers::getRelativeDirectory(relTexInstallFile.str(), relMeshInstallFile.str(), newTexReference))
            {
                ROS_ERROR_STREAM("Could not determine relative directory between "<<relTexInstallFile.str()
                        <<" and "<<relMeshInstallFile.str()<<".");
                continue;
            }

            // replace all occurrences in mesh string
            ROS_INFO_STREAM("Replacing new texture reference: "<<newTexReference);
            typename std::map<std::string, std::string>::iterator meshString = meshes.find(meshFile);
            if (meshString == meshes.end())
            {
                ROS_ERROR_STREAM("Consistency: Mesh "<<meshFile<<" should have been in meshes map");
                return false;
            }
           
            // first, replace all full filenames with paths 
            meshString->second = urdf_traverser::helpers::replaceAll(meshString->second,
                    absTexFile, newTexReference);

            // now, replace all remaining occurrences of the path with a version without
            // path separators (this is only in case there is left-over names made up of the path)
            boost::filesystem::path _absTexMod(absTexFile);
            _absTexMod.replace_extension("");
            boost::filesystem::path _texRefMod(newTexReference);
            _texRefMod.replace_extension("");
            std::string texRefMod = urdf_traverser::helpers::replaceAll(_texRefMod.string(), "/", "_");
            texRefMod = urdf_traverser::helpers::replaceAll(texRefMod, ".", "_");
            meshString->second = urdf_traverser::helpers::replaceAll(meshString->second,
                    _absTexMod.string(), texRefMod);
            
            // add this texture to the result set
            texturesToCopy[relTexInstallFile.str()].insert(absTexFile);
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

