#include <ros/ros.h>
#include <urdf_traverser/UrdfTraverser.h>
#include <urdf_traverser/Functions.h>
#include <urdf2inventor/Helpers.h>
#include <urdf2inventor/IVHelpers.h>
#include <urdf2inventor/ConvertMesh.h>
#include <urdf2inventor/MeshConvertRecursionParams.h>

#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoSelection.h>
#include <Inventor/nodekits/SoNodeKit.h>
#include <Inventor/SoDB.h>      // for file reading
#include <Inventor/SoInput.h>   // for file reading
#include <Inventor/actions/SoWriteAction.h>

#include <ivcon/ivconv.h>

using urdf_traverser::UrdfTraverser;
using urdf_traverser::RecursionParams;
using urdf_traverser::LinkRecursionParams;

typedef urdf_traverser::VisualPtr VisualPtr;
typedef urdf_traverser::GeometryPtr GeometryPtr;
typedef urdf_traverser::MeshPtr MeshPtr;
typedef urdf_traverser::SpherePtr SpherePtr;
typedef urdf_traverser::CylinderPtr CylinderPtr;
typedef urdf_traverser::BoxPtr BoxPtr;
//typedef urdf_traverser::Ptr Ptr;

#define REDIRECT_STDOUT

// Filename for temporary output file
#define TEMP_STDOUT "/tmp/redirectedStdOut"


void cleanupRedirectStdOut()
{
    if (urdf_traverser::helpers::fileExists(TEMP_STDOUT))
    {
        urdf_traverser::helpers::deleteFile(TEMP_STDOUT);
    }
}
/**
 * Uses the ivcon package to conver to inventor file.
 */
SoNode * convertMeshFileIvcon(const std::string& filename, double scale_factor)
{
    // int ssize=10000;
    // char bigOutBuf[ssize];
    //ROS_INFO_STREAM("IVCONV reading "<<filename);
#ifdef REDIRECT_STDOUT
    urdf2inventor::helpers::redirectStdOut(TEMP_STDOUT);
#endif
    // first, convert file to inventor. ivconv writes to file only so we have to
    // temporarily write it to file and then read it again
    IVCONV::SCALE_FACTOR = scale_factor;
    IVCONV ivconv;
    if (!ivconv.read(filename))
    {
        ROS_ERROR("Can't read mesh file %s", filename.c_str());
        return NULL;
    }
    ROS_INFO_STREAM("IVCONV writing "<<filename);

    if (!ivconv.write(TMP_FILE_IV))
    {
        ROS_ERROR("Can't write mesh file %s", TMP_FILE_IV);
        return NULL;
    }

    SoInput in;
    SoNode  *scene = NULL;
    if (!in.openFile(TMP_FILE_IV)) return NULL;
    SoDB::read(&in, scene);

    in.closeFile();

#ifdef REDIRECT_STDOUT
    urdf2inventor::helpers::resetStdOut();
    // ROS_INFO("We got %s",bigOutBuf);
#endif
    //ROS_INFO_STREAM("IVCONV finished reading "<<filename);
    return scene;
}



/**
 * Converts a mesh file (given in filename) to an Inventor structure, to which the root is returned.
 * The model may be scaled at the same time using scale_factor.
 */
SoNode * convertMeshFile(const std::string& filename, double scale_factor)
{
    std::string fileExt = urdf_traverser::helpers::fileExtension(filename.c_str());
//    ROS_INFO_STREAM("Filename extension: "<<fileExt);
    SoNode * result=NULL;
    std::string fileExtLow=fileExt;
    boost::to_lower(fileExtLow);
    if ((fileExtLow != "stl") && (fileExtLow != "obj"))
    {
        ROS_INFO_STREAM("Need to convert mesh file "<<filename<<" to temporary obj/stl first!");
        ROS_ERROR("This still needs to be implemented! See a possibility in package assimp_mesh_converter");
        // could write temporary meshes to TEMP_MESH_DIR
        // after conversion, we need to 
/*
        SoInput in;
        SoNode  *scene = NULL;
        if (!in.openFile(filename.c_str()))
        {
            ROS_ERROR_STREAM("Could not open file "<<filename);
            return NULL;
        }
        scene = SoDB::readAll(&in);
        if (!scene)
        {
            ROS_ERROR_STREAM("Could not read file "<<filename);
            return NULL;

        }
        in.closeFile();
        return scene;*/
    }
    else
    {
        // ROS_INFO_STREAM("Converting mesh "<<filename);
        result = convertMeshFileIvcon(filename,scale_factor);
        // ROS_INFO("Converted.");
    } 
    return result;
}


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
        
        // ROS_INFO_STREAM("Visual "<<visual->group_name);

        urdf_traverser::EigenTransform vTransform = urdf_traverser::getTransform(visual->origin);
        ROS_INFO_STREAM("Visual "<<i<<" of link "<<link->name<<" transform: "<<visual->origin);

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

                // ROS_INFO_STREAM("Converting mesh file "<<meshFilename);
                SoNode * somesh = convertMeshFile(meshFilename, scale_factor);
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
                urdf2inventor::addSphere(allVisuals, vTransform.translation(), sphere->radius, 1,0,0);
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
                urdf2inventor::addBox(allVisuals, vTransform, box->dim.x, box->dim.y, box->dim.z, 1,0,0,0);
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
 * writes the contents of SoNode into the inventor (*.iv) format and returns the file
 * content as a string.
 */
bool writeInventorFileString(SoNode * node, std::string& result)
{
    SoOutput out;
    out.setBinary(false);
    size_t initBufSize = 100;
    void * buffer = malloc(initBufSize * sizeof(char));
    out.setBuffer(buffer, initBufSize, std::realloc);
    SoWriteAction write(&out);
    write.apply(node);

    void * resBuf = NULL;
    size_t resBufSize = 0;

    if (!out.getBuffer(resBuf, resBufSize) || (resBufSize == 0))
    {
        ROS_ERROR("Failed to write file string to buffer.");
        return false;
    }

    result = std::string(static_cast<char*>(resBuf), resBufSize);  // buffer will be copied

    free(resBuf);

    return true;
}


/**
 * Callback function to be called during recursion incurred in convertMeshes().
 * Only supports string mesh formats with MeshConvertRecursionParams<std::string>.
 */
int convertStringMesh(urdf_traverser::RecursionParamsPtr& p)
{
    typedef urdf2inventor::MeshConvertRecursionParams<std::string> MeshConvertRecursionParamsT;
    typename MeshConvertRecursionParamsT::Ptr param = baselib_binding_ns::dynamic_pointer_cast<MeshConvertRecursionParamsT>(p);
    if (!param.get())
    {
        ROS_ERROR("Wrong recursion parameter type");
        return -1;
    }

    urdf_traverser::LinkPtr link = param->getLink();
    
    ROS_INFO("Convert mesh for link '%s'",link->name.c_str());

    SoNode * allVisuals = urdf2inventor::getAllVisuals(link, param->factor, param->addVisualTransform);

    if (!allVisuals)
    {
        ROS_ERROR("Could not get visuals");
        return -1;
    }

    std::string resultFileContent;
    if (!writeInventorFileString(allVisuals, resultFileContent))
    {
        ROS_ERROR("Could not get the mesh file content");
        return -1;
    }

    // ROS_INFO_STREAM("Result file content: "<<resultFileContent);

    if (!param->resultMeshes.insert(std::make_pair(link->name, resultFileContent)).second)
    {
        ROS_ERROR("Could not insert the resulting mesh file for link %s to the map", link->name.c_str());
        return -1;
    }
    return 1;
}

template <class MeshFormat>
bool urdf2inventor::convertMeshes(urdf_traverser::UrdfTraverser& traverser,
                                 const std::string& fromLink,
                                 const float scaleFactor,
                                 const std::string& material,
                                 const std::string& file_extension,
                                 const urdf_traverser::EigenTransform& addVisualTransform,
                                 std::map<std::string, MeshFormat>& meshes)
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
    if (traverser.traverseTreeTopDown(startLinkName, boost::bind(&convertStringMesh, _1), p, true) <= 0)
    {
        ROS_ERROR("Could nto convert meshes.");
        return false;
    }

    meshes = meshParams->resultMeshes;
    
    // can delete output file because it was successful
    cleanupRedirectStdOut();
    return true;
}

// instantiation for string meshes
template bool urdf2inventor::convertMeshes<std::string>(urdf_traverser::UrdfTraverser& traverser,
                                 const std::string& fromLink,
                                 const float scaleFactor,
                                 const std::string& material,
                                 const std::string& file_extension,
                                 const urdf_traverser::EigenTransform& addVisualTransform,
                                 std::map<std::string, std::string>& meshes);
