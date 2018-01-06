^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package urdf_traverser
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.2 (2018-01-06)
------------------
* Fixed error in joining fixed links
* Fixed rotation to z axis which was still going the wrong way. Some robots may now need parameter negate_joint_movement inverted
* Some tidy up
* Contributors: Jennifer Buehler

1.0.1 (2016-08-06)
------------------
* Changed to BSD 3-clause license
* Contributors: Jennifer Buehler

1.0.0 (2016-06-07)
------------------

0.0.4 (2016-06-06)
------------------

0.0.3 (2016-06-04)
------------------

0.0.1 (2016-05-31)
------------------
* urdf2inventor now copies textures for the whole model as well
* Now exports textures to file for individual meshes, working on getting it to work with overall robot
* Now can use material from URDF too
* Added urdf2inventor and urdf_viewer
* All traversal callbacks and functionality moved to separate files outside UrdfTraverser. UrdfTraverser now clean.
* Moved more functions out of UrdfTraverser and into callbacks
* Contributors: Jennifer Buehler
