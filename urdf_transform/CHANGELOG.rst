^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package urdf_transform
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.2 (2018-01-06)
------------------
* Fixed error in joining fixed links
* Fixed rotation to z axis which was still going the wrong way. Some robots may now need parameter negate_joint_movement inverted
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
* All traversal callbacks and functionality moved to separate files outside UrdfTraverser. UrdfTraverser now clean.
* Moved more functions out of UrdfTraverser and into callbacks
* Contributors: Jennifer Buehler
