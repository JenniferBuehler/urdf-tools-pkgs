# urdf2inventor

See also [this wiki page](https://github.com/JenniferBuehler/urdf-tools-pkgs/wiki/urdf2inventor)


### Dependencies

- [convenience-pkgs](https://github.com/JenniferBuehler/convenience-pkgs/wiki/urdf2inventor)
- assimp (ubuntu package libassimp-dev)
- Qt4 libs (ubuntu libqt4-dev)
- SoQt and Coin (ubuntu packages libsoqt4-deva and libcoin80-dev)

Note:

Depending catkin packages CMakeLists.txt must:

``add_definitions(${urdf2inventor_DEFINITIONS})``

### Important notes

This package has recently adopted importing meshes with assimp.
The initial code is heavily based on the inventor reading code
by Nestor Garcia Hidalgo which can be found in the repository
[coindesigner](https://github.com/iocroblab/coindesigner/) in the file
[assimpImport.cpp](https://github.com/iocroblab/coindesigner/blob/master/src/assimpImport.cpp).    
I have not fully tested this code with various models so please report if there is any issues.
