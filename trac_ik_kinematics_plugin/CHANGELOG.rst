^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package trac_ik_kinematics_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.0 (2025-07-28)
------------------
* Add Manipulation3, maximizing smallest singular value (`#46 <https://bitbucket.org/traclabs/trac_ik/pull-requests/46>`_)
* Contributors: Matthijs van der Burgh

2.1.0 (2025-05-20)
------------------
* Switch from ament_target_dependencies to target_link_libraries. (`#43 <https://bitbucket.org/traclabs/trac_ik/pull-requests/43>`_)
* Clean up and format CMakeLists.txt. (`#43 <https://bitbucket.org/traclabs/trac_ik/pull-requests/43>`_)
* Fix deprecated header warnings (`#41 <https://bitbucket.org/traclabs/trac_ik/pull-requests/41>`_)
* Changes the trac_ik_kinematics plugin to relaunch the IK solver with a random seed when the solution callback fails. The same behavior is also done by KDL (see here) and leads to better solver integration when the callback is used for collision checking. (`#38 <https://bitbucket.org/traclabs/trac_ik/pull-requests/38>`_)
* Contributors: Kenji Brameld, Timon Engelke, Roelof

2.0.1 (2024-04-12)
------------------

2.0.0 (2024-03-29)
------------------
* hide parameters header file, as it is implemetation
* remove boost from trac_ik_lib and trac_ik_kinematics_plugin
* Set default solve type to Distance
* Contributors: Ana C. Huaman Quispe, Kenji Brameld

1.6.6 (2021-05-05)
------------------
* removed bad depends
* propagated nlopt deps to sat packages
* Contributors: Stephen Hart

1.6.4 (2021-04-29)
------------------

1.6.2 (2021-03-17)
------------------
* changed package.xmls to format 3
* switched to non-deprecated function for neotic
* Contributors: Stephen Hart
