^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package urdf2inventor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.2 (2018-01-06)
------------------
* Urdf2Inventor::convert() now accepts MeshRecursionParams so that it additional transforms can be handled per link
* Backup
* Backup before critical changes
* Some tidy up
* Can now use collision geometry instead of visual for meshes, though it still needs to be added as paramter
* Contributors: Jennifer Buehler

1.0.1 (2016-08-06)
------------------
* Changed to BSD 3-clause license
* Fix cylinder and box orientations
* Added function to compute Bounding Box
* Added new transform and cylinder methods to IVHelpers
* Supports Cylinder in urdf2inventor now
* Contributors: Jennifer Buehler

1.0.0 (2016-06-07)
------------------
* Added missing installation of hpp files
* Added installation of launch files
* Contributors: Jennifer Buehler

0.0.4 (2016-06-06)
------------------
* Fixed export depending system libs in cmakelists
* Contributors: Jennifer Buehler

0.0.3 (2016-06-04)
------------------

0.0.1 (2016-05-31)
------------------
* urdf2inventor now copies textures for the whole model as well
* Now exports textures to file for individual meshes, working on getting it to work with overall robot
* Fixed but in loading non-existing materials; Now also can load textures;
* Now can use material from URDF too
* Fixed error in scaling after transforming from assimp
* urdf2inventor now works with assimp
* Added urdf2inventor and urdf_viewer
* Contributors: Jennifer Buehler
