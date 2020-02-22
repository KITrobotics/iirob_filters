^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package iirob_filters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Repository changes

0.9.0 (2020-02-22)
------------------
* Add melodic travis build (`#31 <https://github.com/KITrobotics/iirob_filters/issues/31>`_)
  * Add melodic travis build to kinetic-branch
  * Removed support for indigo
* Use local namespace for parameters
* Contributors: Denis Štogl

0.8.2 (2020-02-17)
------------------
* Added chages to sequential Kalman Filter (`#30 <https://github.com/KITrobotics/iirob_filters/issues/30>`_)
* added handling if At is empty (`#29 <https://github.com/KITrobotics/iirob_filters/issues/29>`_)
* version 0.8.1
* generated changelog
* Contributors: Denis Štogl, GDwag, Gilbert Groten, Daniel Azanov

0.8.1 (2018-12-13)
------------------
* Revert use of doTransform for Wrenches (#24)
* Update README.md (#25)
* Update README.md (#23)
* Use doTransform for Wrenches (#22)
  * Use doTransform for Wrenches
  * Corrected package.xml repository website and moved from Eigen to Eigen3 in CMakeLists.txt
  * Added author
* Added Kalman filter implementation only for target tracking. (#14)
  * Added Kalman filter implementation  with static and in time changing matrices.
* Merge pull request #21 from KITrobotics/info_output
  Added info output for filters. Added world frame for gravity_compensa…
* Added info output for filters. Added world frame for gravity_compensation and small refractoring.
* Merge pull request #19 from KITrobotics/destogl-patch-1
  Update README.md
* Update .travis.yml
* Update .travis.yml
* Update .travis.rosinstall
* Update package.xml
* Update CMakeLists.txt
* Update README.md
* Merge pull request #16 from iirob/gravity_output
  Added output for gravity compenation
* Update gravity_compensation.h
* Added output for gravity compenation
* Merge pull request #13 from iirob/raw_with_fts
  Raw with fts
* changes to work with double and wrench stamped
* use rosparam handler
* xMerge branch 'raw_with_fts' into me
* added moving mean filter
* Update gravity_compensation.h
* Update gravity_compensation.h
* filters working with wrench template and implementing interface from ros filters: need code review
* changes to correctly work with filters plugin, code not reviewed
* Merge pull request #10 from iirob/raw_with_fts
  Raw with fts
* Update CMakeLists.txt
* Filters are now matching to FilterBase interface
* Merge branch 'raw_with_fts' of https://github.com/iirob/iirob_filters into raw_with_fts
* changes on gravity compensation and threshold filter to match the FilterBase interface
* Merge branch 'kinetic-devel' into raw_with_fts
* Merge branch 'indigo-devel' into kinetic-devel
* changes to avoid rosparam_handler error while compiling
* changes to add pluginlib
* Merge pull request #8 from iirob/raw_with_fts
  Raw with fts 2nd time
* Merge branch 'indigo-devel' into raw_with_fts
* Merge pull request #7 from iirob/raw_with_fts
  Raw with fts updated
* Update .travis.rosinstall
* Merge branch 'kinetic-devel' into raw_with_fts
* small change
* used changes from indigo-devel
* adopted changes from indigo-devel
* changes needed to work with rosparam_handler
* Merge pull request #6 from iirob/raw_with_fts
  Raw with fts
* Merge branch 'indigo-devel' into raw_with_fts
* Merge pull request #5 from iirob/raw_with_fts
  Raw with fts
* Update .travis.rosinstall
* Added custom param_handler into travis
* Merge branch 'raw_with_fts' of github.com:iirob/iirob_filters into raw_with_fts
* Reverted to c++11
* Reverted to c++11
* compatibility changes
* merged with kinetic-devel
* ros params for filters working
* Merge pull request #4 from iirob/library_rename_export
  Library rename export
* Merge pull request #3 from iirob/library_rename_export
  Update CMakeLists.txt
* Update CMakeLists.txt
* Merge pull request #2 from iirob/add_install_space
  Added install space for iirob_filters
* Merge pull request #1 from iirob/add_install_space
  Added install space for iirob_filters
* Added install space for iirob_filters
* Removed Eigen warning
* Merge branch 'indigo-devel' into kinetic-devel
* Removed kinetic test from indigo branch
* Added dummy .rosinstall file
* revert to trusty
* Updated travis config to xenial for kinetic
* Added readme file
* Added tf2 dependecy
* Added travis config
* Update .travis.yml
* Updated moving mean filter to correct correctly
* Updated GravityCompensation filter to accept messages in any frame and return them in the same frame. Reduced transformations error output so that every 100th error is shown.
* Renamed median to moving_mean and updated headers in files
* fixed parameters in init
* created 'init'-functions
* update
* Inital coping of files into filters
* Contributors: Andreea Tulbure, Denis Štogl, IIROB Praktikum 1, Timo Leitritz, muritane
