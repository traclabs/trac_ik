^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package trac_ik_lib
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.0 (2025-07-28)
------------------
* Add Manipulation3, maximizing smallest singular value (`#46 <https://bitbucket.org/traclabs/trac_ik/pull-requests/46>`_)
* Contributors: Matthijs van der Burgh

2.1.0 (2025-05-20)
------------------
* Switch from ament_target_dependencies to target_link_libraries. (`#43 <https://bitbucket.org/traclabs/trac_ik/pull-requests/43>`_)
* Fix deprecated header warnings (`#41 <https://bitbucket.org/traclabs/trac_ik/pull-requests/41>`_)
* Add a constructor TRAC_IK::TRAC_IK that does not require a rclcpp::Node as argument (`#40 <https://bitbucket.org/traclabs/trac_ik/pull-requests/40>`_)
* Contributors: Kenji Brameld, Roelof

2.0.1 (2024-04-12)
------------------
* Add missing geometry_msgs dependency in package.xml
* Contributors: Kenji Brameld

2.0.0 (2024-03-29)
------------------
* change nanoseconds() to seconds()
* remove boost from trac_ik_lib and trac_ik_kinematics_plugin
* Added support for C++17
* added build_export_depends to tracik_lib for nlopt
* fix issues caused by shared references and thread safety
* hotfix for init of solvers using non-thread-safe KDL::Chain
* Contributors: Ana C. Huaman Quispe, Kenji Brameld, Mike Lanighan, Stephen Hart

1.6.6 (2021-05-05)
------------------

1.6.4 (2021-04-29)
------------------
* added nlopt depends to traciklib cmake
* Contributors: Stephen Hart

1.6.2 (2021-03-17)
------------------
