^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package trac_ik_lib
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.7 (2025-05-20)
------------------
* drop requirement for obsolete c++ standard to support compiling on newer systems with log4cxx requiring c++17 (`#39 <https://bitbucket.org/traclabs/trac_ik/pull-requests/39>`_)
* added build_export_depends to tracik_lib for nlopt (`#33 <https://bitbucket.org/traclabs/trac_ik/pull-requests/33>`_)
* hotfix for init of solvers using non-thread-safe KDL::Chain (`#32 <https://bitbucket.org/traclabs/trac_ik/pull-requests/32>`_)
* adding pr2_description dependency for trac_ik_examples (`#32 <https://bitbucket.org/traclabs/trac_ik/pull-requests/32>`_)
* updating to use xacro instead of xacro.py (`#32 <https://bitbucket.org/traclabs/trac_ik/pull-requests/32>`_)
* Contributors: Mike Lanighan, Stephen Hart, v4hn

1.6.6 (2021-05-05)
------------------

1.6.4 (2021-04-29)
------------------
* added nlopt depends to traciklib cmake
* Contributors: Stephen Hart

1.6.2 (2021-03-17)
------------------
