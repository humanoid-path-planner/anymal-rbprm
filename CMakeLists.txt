cmake_minimum_required(VERSION 3.1)

set(PROJECT_NAME anymal-rbprm)
set(PROJECT_DESCRIPTION "Database for anymal robot using hpp-rbprm")
set(PROJECT_URL "")
set(CUSTOM_HEADER_DIR "hpp/anymal-rbprm")
set(CXX_DISABLE_WERROR true)
set(PROJECT_USE_CMAKE_EXPORT TRUE)

include(cmake/base.cmake)
include(cmake/python.cmake)

compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

findpython()

add_library(${PROJECT_NAME} INTERFACE)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)

install(DIRECTORY data/ DESTINATION share/${PROJECT_NAME})

string(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})

set(${PROJECT_NAME}_PYTHON_FILES __init__.py anymal.py anymal_abstract.py
                                 anymal_contact6D.py)

foreach(SOURCE ${${PROJECT_NAME}_PYTHON_FILES})
  python_install_on_site(${PY_NAME} ${SOURCE})
endforeach(SOURCE ${${PROJECT_NAME}_PYTHON_SOURCES})

install(FILES package.xml DESTINATION share/${PROJECT_NAME})
