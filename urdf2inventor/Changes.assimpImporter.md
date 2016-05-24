### Adopted assimp importer code 

This package has recently adopted importing meshes with assimp.
The initial code is heavily based on the inventor reading code
by Nestor Garcia Hidalgo which can be found in the repository
[coindesigner](https://github.com/iocroblab/coindesigner/) in the file
[assimpImport.cpp](https://github.com/iocroblab/coindesigner/blob/master/src/assimpImport.cpp).    
I have not fully tested this code with various models so please report if there is any issues.


### Changes to the code made so far
- Took out all QtDir references, eg. replacet QtDir::cleanPath with boost::canonical
- Took out USE_ASSIMP as assimp is always used
- removed importScene() from .h and .cpp and made the method 
    SoSeparator *Assimp2Inventor(const aiScene *const scene, const std::string& sceneDir)
  public in the header instead
- changed addNode() such that the aiNode::mTransformation is also considered if the node parent is NULL.
  This is requiered to apply a scale (and any other possible global transformations) at the root.
- added option to override material colour (to use the one specified in the URDF).
