/*************************************************************************\
   Copyright 2014 Institute of Industrial and Control Engineering (IOC)
                Universitat Politecnica de Catalunya
                BarcelonaTech
   All Rights Reserved.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the
   Free Software Foundation, Inc.,
   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
\*************************************************************************/

/* Author: Nestor Garcia Hidalgo */


#include <urdf2inventor/AssimpImport.h>

#include <assimp/Importer.hpp>
#include <assimp/importerdesc.h>
#include <assimp/postprocess.h>

#include <boost/filesystem.hpp>

#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoIndexedPointSet.h>
#include <Inventor/nodes/SoIndexedLineSet.h>
#include <Inventor/nodes/SoIndexedTriangleStripSet.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoTexture2.h>

#include <iostream>
#include <sstream>


SbName getName(const std::string &name) {
    std::stringstream strs;
    for (std::string::const_iterator it(name.begin());
         it != name.end(); ++it) {
        if (it == name.begin()) {
            if (SbName::isBaseNameStartChar(*it) == TRUE) {
                strs << *it;
            } else if (SbName::isBaseNameChar(*it) == TRUE) {
                strs << "_" << *it;
            }
        } else {
            if (SbName::isBaseNameChar(*it) == TRUE) {
                strs << *it;
            } else {
                strs << "_";
            }
        }
    }

    return SbName(strs.str().c_str());
}


SoTransform *getTransform(const aiMatrix4x4 &matrix) {
    aiVector3D scaling;
    aiQuaternion rotation;
    aiVector3D position;
    matrix.Decompose(scaling,rotation,position);

    SoTransform *transform(new SoTransform);
    transform->translation.setValue(position.x,
                                    position.y,
                                    position.z);
    transform->rotation.setValue(rotation.x,
                                 rotation.y,
                                 rotation.z,
                                 rotation.w);
    transform->scaleFactor.setValue(scaling.x,
                                    scaling.y,
                                    scaling.z);

    return transform;
}


SoTexture2 *getTexture(const aiTexture *const texture) {
    if (texture->mHeight == 0) {//Compressed texture
        std::cout << "Found a compressed embedded texture. "
                  << "It will be ignored." << std::endl;
        ///texture->pcData is a pointer to a memory buffer of
        ///size mWidth containing the compressed texture data
        ///I do not know how to extract this information
        return NULL;
    } else {//Uncompressed texture
        unsigned char pixels[texture->mWidth*texture->mHeight*4];
        for (std::size_t i(0); i < texture->mWidth; ++i) {
            for (std::size_t j(0); j < texture->mHeight; ++j) {
                pixels[4*(texture->mHeight*i+j)+0] = texture->pcData[texture->mHeight*i+j].r;
                pixels[4*(texture->mHeight*i+j)+1] = texture->pcData[texture->mHeight*i+j].g;
                pixels[4*(texture->mHeight*i+j)+2] = texture->pcData[texture->mHeight*i+j].b;
                pixels[4*(texture->mHeight*i+j)+3] = texture->pcData[texture->mHeight*i+j].a;
            }
        }
        SoTexture2 *soTexture(new SoTexture2);
        soTexture->image.setValue(SbVec2s(texture->mWidth,texture->mHeight),4,pixels);

        return soTexture;
    }
}


SoTexture *getTexture(const aiMaterial *const material, const std::string &sceneDir) {
    ///I only know how to deal with aiTextureType_DIFFUSE textures
    ///and with only one texture.
    ///Other types and the other DIFFUSE texture but the first will be ignored.
    ///A texture has a transformation, an UV index, a blend factor, an operation
    ///and some flags but all these properties will also be ignored.
    ///Transform should be removed by the post process step,
    /// ut aiUVTransform => SoTexture2Transform ?
    ///UV index only has a valid value if mapping == aiTextureMapping_UV
    ///Operation should have something related with SoTextureCombiner
    ///How I know when I need to use SoTexture3 or Sotexture2?
    ///I don't know how to check if the image has been loaded correctly
    ///texture->model is always set to DECAL, is that good?
    ///It should be related with the loaded info

    //Check if there is a texture
    unsigned int numTextures(material->GetTextureCount(aiTextureType_DIFFUSE));
    if (numTextures == 0) return NULL;
    if (numTextures > 1) {
        std::cout << "Found a material with " << numTextures
                  << " textures. Only the first one will be used." << std::endl;
    }

    //Get path
    aiString path;
    if (material->Get(AI_MATKEY_TEXTURE(aiTextureType_DIFFUSE,0),path) != aiReturn_SUCCESS) {
        std::cout << "Error while getting the texture path. "
                  << "Texture will be ignored." << std::endl;
        return NULL;
    }

    //Check mapping
    ///If it is not defined, suppose it is aiTextureMapping_UV
    int mapping;
    if (material->Get(AI_MATKEY_MAPPING(aiTextureType_DIFFUSE,0),mapping) == aiReturn_SUCCESS) {
        if (mapping != aiTextureMapping_UV) {
            std::cout << "Invalid texture mapping. Texture will be ignored." << std::endl;
            return NULL;
        }
    }

    //Check UV transform
    aiUVTransform transform;
    if (material->Get(AI_MATKEY_UVTRANSFORM(aiTextureType_DIFFUSE,0),
                      transform) == aiReturn_SUCCESS) {
        std::cout << "Error I did not expect a texture transform. " <<
                     "Property will be ignored." << std::endl;
    }

    SoTexture2 *texture(new SoTexture2);

    //Load image
    std::string filename = boost::filesystem::canonical(path.C_Str()).string();
    
    texture->filename.setValue(filename.c_str());///How to check if image was loaded?
    texture->setName(getName(boost::filesystem::path(filename).filename().string()));

    //Set model
    texture->model.setValue(SoTexture2::DECAL);

    //Set wrap
    int mapMode;
    if (material->Get(AI_MATKEY_MAPPINGMODE_U(aiTextureType_DIFFUSE,0),
                      mapMode) == aiReturn_SUCCESS) {
        switch (mapMode) {
        case aiTextureMapMode_Wrap:
            texture->wrapS.setValue(SoTexture2::REPEAT);
            break;
        case aiTextureMapMode_Clamp:
            texture->wrapS.setValue(SoTexture2::CLAMP);
            break;
        case aiTextureMapMode_Decal:
        case aiTextureMapMode_Mirror:
        default:
            std::cout << "Wrong S texture mapping mode. "
                      << "Property will be ignored." << std::endl;
            break;
        }
    }
    if (material->Get(AI_MATKEY_MAPPINGMODE_V(aiTextureType_DIFFUSE,0),
                      mapMode) == aiReturn_SUCCESS) {
        switch (mapMode) {
        case aiTextureMapMode_Wrap:
            texture->wrapT.setValue(SoTexture2::REPEAT);
            break;
        case aiTextureMapMode_Clamp:
            texture->wrapT.setValue(SoTexture2::CLAMP);
            break;
        case aiTextureMapMode_Decal:
        case aiTextureMapMode_Mirror:
        default:
            std::cout << "Wrong T texture mapping mode. "
                      << "Property will be ignored." << std::endl;
            break;
        }
    }

    return texture;
}


SoMaterial *getMaterial(const aiMaterial *const material) {
    SoMaterial *soMat(new SoMaterial);

    aiString name;
    aiColor3D color;
    float value;

    //Add name
    if (material->Get(AI_MATKEY_NAME,name) == aiReturn_SUCCESS) {
        soMat->setName(getName(name.C_Str()));
    }

    //Add diffuse color
    if (material->Get(AI_MATKEY_COLOR_DIFFUSE,color) == aiReturn_SUCCESS) {
        soMat->diffuseColor.setValue(color.r,
                                     color.g,
                                     color.b);
    }

    //Add specular color
    if (material->Get(AI_MATKEY_COLOR_SPECULAR,color) == aiReturn_SUCCESS) {
        soMat->specularColor.setValue(color.r,
                                      color.g,
                                      color.b);
    }

    //Add ambient color
    if (material->Get(AI_MATKEY_COLOR_AMBIENT,color) == aiReturn_SUCCESS) {
        soMat->ambientColor.setValue(color.r,
                                     color.g,
                                     color.b);
    }

    //Add emissive color
    if (material->Get(AI_MATKEY_COLOR_EMISSIVE,color) == aiReturn_SUCCESS) {
        soMat->emissiveColor.setValue(color.r,
                                      color.g,
                                      color.b);
    }

    //Add transparency
    if (material->Get(AI_MATKEY_OPACITY,value) == aiReturn_SUCCESS) {
        soMat->transparency.setValue(1.0-value);
    }

    //Add shininess
    if (material->Get(AI_MATKEY_SHININESS_STRENGTH,value) == aiReturn_SUCCESS) {
        soMat->shininess.setValue(value);
    }

    return soMat;
}


SoIndexedShape *getShape(const aiMesh *const mesh) {
    if (!mesh->HasPositions() || !mesh->HasFaces()) return NULL; //Mesh is empty

    //Get shape type
    SoIndexedShape *shape;
    std::size_t numIndices;
    switch (mesh->mPrimitiveTypes) {
    case aiPrimitiveType_POINT:
        shape = new SoIndexedPointSet;
        numIndices = 1;
        break;
    case aiPrimitiveType_LINE:
        shape = new SoIndexedLineSet;
        numIndices = 2;
        break;
    case aiPrimitiveType_TRIANGLE:
        shape = new SoIndexedTriangleStripSet;
        numIndices = 3;
        break;
    default:
        //Or the faces are polygons or the shape contains more than one primitive
        //This should have been solved by the triangulate and sort-by-type post processing steps
        std::cout << "Wrong primitive type. Mesh will be ignored." << std::endl;
        return NULL;
        break;
    }

    //Set name
    shape->setName(getName(mesh->mName.C_Str()));

    SoVertexProperty *vertexProperty(new SoVertexProperty);
    shape->vertexProperty.setValue(vertexProperty);

    //Set vertices
    float vertices[mesh->mNumVertices][3];
    for (std::size_t i(0); i < mesh->mNumVertices; ++i) {
        vertices[i][0] = mesh->mVertices[i].x;
        vertices[i][1] = mesh->mVertices[i].y;
        vertices[i][2] = mesh->mVertices[i].z;
    }
    vertexProperty->vertex.setValues(0,mesh->mNumVertices,vertices);

    if (mesh->HasNormals()) {
        //Set normals
        float normals[mesh->mNumVertices][3];
        for (std::size_t i(0); i < mesh->mNumVertices; ++i) {
            normals[i][0] = mesh->mNormals[i].x;
            normals[i][1] = mesh->mNormals[i].y;
            normals[i][2] = mesh->mNormals[i].z;
        }
        vertexProperty->normal.setValues(0,mesh->mNumVertices,normals);
    }

    if (mesh->GetNumColorChannels() > 0) {
        std::cout << "Mesh has " << mesh->GetNumColorChannels()
                  << " vertex color channels. Property will be ignored." << std::endl;
    }

    if (mesh->GetNumUVChannels() > 0) {
        if (mesh->GetNumUVChannels() > 1) {
                std::cout << "Mesh has " << mesh->GetNumUVChannels()
                          << " UV channels. Only the first one will be used." << std::endl;
                ///How to map UV channels to textures (MATKEY_UVWSRC)?
                ///The MATKEY_UVWSRC property is only present if the source format doesn't
                ///specify an explicit mapping from textures to UV channels. Many formats
                ///don't do this and assimp is not aware of a perfect rule either.
                ///
                ///Your handling of UV channels needs to be flexible therefore.
                ///Our recommendation is to use logic like this to handle most cases properly:
                ///
                ///  have only one uv channel?
                ///     assign channel 0 to all textures and break
                ///
                ///  for all textures
                ///     have uvwsrc for this texture?
                ///        assign channel specified in uvwsrc
                ///     else
                ///        assign channels in ascending order for all texture stacks,
                ///            i.e. diffuse1 gets channel 1, opacity0 gets channel 0.
        }

        //Set texture coordinates
        if (mesh->mNumUVComponents[0] == 2) {
            float texCoords[mesh->mNumVertices][2];
            for (std::size_t i(0); i < mesh->mNumVertices; ++i) {
                texCoords[i][0] = mesh->mTextureCoords[0][i].x;
                texCoords[i][1] = mesh->mTextureCoords[0][i].y;
            }
            vertexProperty->texCoord.setValues(0,mesh->mNumVertices,texCoords);
        } else if (mesh->mNumUVComponents[0] == 3) {
            std::cout << "Setting texture coordinates of 3 components "
                      << "but all the loaded textures will be of 2 components." << std::endl;

            float texCoords3[mesh->mNumVertices][3];
            for (std::size_t i(0); i < mesh->mNumVertices; ++i) {
                texCoords3[i][0] = mesh->mTextureCoords[0][i].x;
                texCoords3[i][1] = mesh->mTextureCoords[0][i].y;
                texCoords3[i][2] = mesh->mTextureCoords[0][i].z;
            }
            vertexProperty->texCoord3.setValues(0,mesh->mNumVertices,texCoords3);
        } else {
            std::cout << "Mesh has texture coordinates of " << mesh->mNumUVComponents[0]
                      << " components. Property will be ignored." << std::endl;
        }
    }

    //Set faces
    int indices[mesh->mNumFaces*(numIndices+1)];
    for (std::size_t i(0); i < mesh->mNumFaces; ++i) {
        for (std::size_t j(0); j < numIndices; ++j) {
            indices[i*(numIndices+1)+j] = mesh->mFaces[i].mIndices[j];
        }
        indices[i*(numIndices+1)+numIndices] = -1;
    }
    shape->coordIndex.setValues(0,mesh->mNumFaces*(numIndices+1),indices);

    return shape;
}


SoSeparator *getMesh(const aiMesh *const mesh, const aiMaterial *const material,
                     const std::string& sceneDir, SoSeparator *meshSep = NULL) {
    SoIndexedShape *shape(getShape(mesh));
    if (shape) {
        if (!meshSep) meshSep = new SoSeparator;

        //Add texture
        SoTexture *texture(getTexture(material,sceneDir));
        if (texture) meshSep->addChild(texture);

        //Add material
        meshSep->addChild(getMaterial(material));

        //Add shape
        meshSep->addChild(shape);

        return meshSep;
    } else {
        return NULL;
    }
}


bool hasMesh(const aiNode *node) {
    if (node->mNumMeshes > 0) return true;
    for (std::size_t i(0); i < node->mNumChildren; ++i) {
        if (hasMesh(node->mChildren[i])) return true;
    }
    return false;
}


void addNode(SoSeparator *const parent, const aiNode *const node,
             const aiMaterial *const *const materials, const aiMesh *const *const meshes,
             const aiTexture *const *const textures, const std::string& sceneDir) {
    if (hasMesh(node)) {
        SoSeparator *nodeSep;
        if ((!node->mParent || node->mTransformation.IsIdentity()) &&
                node->mNumMeshes == 0) {
            nodeSep = parent;
        } else {
            //Create separator
            nodeSep = new SoSeparator;
            nodeSep->setName(getName(node->mName.C_Str()));
            parent->addChild(nodeSep);

            //Add transform
            if (node->mParent && !node->mTransformation.IsIdentity())
                nodeSep->addChild(getTransform(node->mTransformation));

            //Add meshes
            if (node->mNumMeshes == 1 && node->mNumChildren == 0) {
                getMesh(meshes[node->mMeshes[0]],
                        materials[meshes[node->mMeshes[0]]->mMaterialIndex],
                        sceneDir,nodeSep);
            } else {
                for (std::size_t i(0); i < node->mNumMeshes; ++i) {
                    SoNode *child(getMesh(meshes[node->mMeshes[i]],
                                  materials[meshes[node->mMeshes[i]]->mMaterialIndex],sceneDir));
                    if (child) nodeSep->addChild(child);
                }
            }
        }

        //Add children nodes
        for (std::size_t i(0); i < node->mNumChildren; ++i) {
            addNode(nodeSep,node->mChildren[i],materials,meshes,textures,sceneDir);
        }
    }
}

SoSeparator *Assimp2Inventor(const aiScene *const scene, const std::string& sceneDir) {
    SoSeparator *root(new SoSeparator);
    std::cout << "I imported a scene with " << scene->mNumTextures << " embedded textures, "
              << scene->mNumMaterials << " materials and "
              << scene->mNumMeshes << " meshes." << std::endl;
    if (scene->mNumTextures > 0) {
        std::cout << "Found a scene with embedded textures. They will be ignored." << std::endl;
        ///I don't know how they will be referenced inside the scene
    }
    addNode(root,scene->mRootNode,scene->mMaterials,
            scene->mMeshes,scene->mTextures,sceneDir);
    return root;
}


std::vector<std::string> tokenize(const std::string &str, const std::string &token) {
    std::vector<std::string> tokenized;
    size_t from(0), size(str.size());
    for (size_t to(std::min(str.find(token,from),size));
         from < to; to = std::min(str.find(token,from),size)) {
        tokenized.push_back(str.substr(from,to-from));
        from = to + token.size();
    }
    return tokenized;
}


std::vector<std::string> assimpImportedExtensions() {
    aiString tmp;
    Assimp::Importer importer;
    importer.GetExtensionList(tmp);
    std::string extensions(tmp.C_Str());

    return tokenize(extensions.substr(2,std::string::npos),";*.");
}


std::vector<std::pair<std::string,std::vector<std::string> > > assimpImportedFormats() {
    std::vector<std::pair<std::string,std::vector<std::string> > > importedFormats;
    const aiImporterDesc *importerDesc;
    Assimp::Importer importer;
    std::string name;
    std::vector<std::string> extensions;
    std::size_t k, pos;
    for (std::size_t i(0); i < importer.GetImporterCount(); ++i) {
        importerDesc = importer.GetImporterInfo(i);

        name = importerDesc->mName;
        pos = name.find(" Importer");
        if (pos != std::string::npos) name.erase(pos,9);
        pos = name.find(" Reader");
        if (pos != std::string::npos) name.erase(pos,7);
        pos = name.find("\n");
        if (pos != std::string::npos) name.erase(pos,std::string::npos);
        while (name.substr(name.size()-1) == " ") {
            name.erase(name.size()-1,1);
        }
        extensions = tokenize(importerDesc->mFileExtensions," ");

        k = 0;
        while (k < importedFormats.size() &&
               importedFormats.at(k).first != name) {
            k++;
        }
        if (k < importedFormats.size()) {
            for (std::size_t j(0); j < extensions.size(); ++j) {
                importedFormats.at(k).second.push_back(extensions.at(j));
            }
        } else {
            std::pair< std::string,std::vector<std::string> > format;
            format.first = name;
            format.second = extensions;
            importedFormats.push_back(format);
        }
    }

    return importedFormats;
}

