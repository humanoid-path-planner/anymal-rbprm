# Copyright (c) 2012 CNRS
# Author: Florent Lamiraux
#
# This file is part of hpp-rbprm-corba.
# hpp-rbprm-corba is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-rbprm-corba is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-rbprm-corba.  If not, see
# <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CXX_DISABLE_WERROR true)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/idl.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME anymal-rbprm)
SET(PROJECT_DESCRIPTION "Database for anymal robot using hpp-rbprm")
SET(PROJECT_URL "")

SET(CUSTOM_HEADER_DIR "hpp/anymal-rbprm")

SETUP_PROJECT ()

# search for python
FINDPYTHON(2.7 EXACT REQUIRED)

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

SET(CATKIN_PACKAGE_SHARE_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})
  
  
SET(CATKIN_PACKAGE_SHARE_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/anymal-rbprm)
  
  
install(DIRECTORY
  data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
  
  
install (FILES
  src/hpp/corbaserver/rbprm/anymal/robot.py
  src/hpp/corbaserver/rbprm/anymal/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/rbprm/anymal)


install (FILES
  src/hpp/corbaserver/rbprm/anymal_contact6D/robot.py
  src/hpp/corbaserver/rbprm/anymal_contact6D/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/rbprm/anymal_contact6D)

install (FILES
  src/hpp/corbaserver/rbprm/anymal_abstract/robot.py
  src/hpp/corbaserver/rbprm/anymal_abstract/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/rbprm/anymal_abstract)


SETUP_PROJECT_FINALIZE()

