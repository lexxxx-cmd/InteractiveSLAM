# FindG2O.cmake
# -------
# Finds the g2o (General Graph Optimization) library installed via vcpkg.
#
# This module sets:
#   G2O_FOUND          - TRUE if g2o is found
#   G2O_INCLUDE_DIRS   - g2o include directories
#   G2O_LIBRARIES      - all g2o libraries to link against
#
# Works with vcpkg CMAKE_TOOLCHAIN_FILE (global vcpkg install).

find_path(G2O_INCLUDE_DIRS
    NAMES g2o/core/hyper_graph.h
    HINTS ENV VCPKG_ROOT
    PATHS /usr/local/include /usr/include
    PATH_SUFFIXES include
)

find_library(G2O_CORE_LIBRARY
    NAMES g2o_core g2o
    HINTS ENV VCPKG_ROOT
    PATHS /usr/local/lib /usr/lib
    PATH_SUFFIXES lib
)

find_library(G2O_TYPES_SLAM3D_LIBRARY
    NAMES g2o_types_slam3d
    HINTS ENV VCPKG_ROOT
    PATHS /usr/local/lib /usr/lib
    PATH_SUFFIXES lib
)

find_library(G2O_TYPES_SLAM3D_ADDONS_LIBRARY
    NAMES g2o_types_slam3d_addons
    HINTS ENV VCPKG_ROOT
    PATHS /usr/local/lib /usr/lib
    PATH_SUFFIXES lib
)

# Find ALL solver libraries — graph_slam.cpp uses
# G2O_USE_OPTIMIZATION_LIBRARY for each, so all must be linked
foreach(_solver pcg cholmod csparse)
    find_library(G2O_SOLVER_${_solver}
        NAMES g2o_solver_${_solver}
        HINTS ENV VCPKG_ROOT
        PATHS /usr/local/lib /usr/lib
        PATH_SUFFIXES lib
    )
endforeach()

if(G2O_INCLUDE_DIRS AND G2O_CORE_LIBRARY AND G2O_TYPES_SLAM3D_LIBRARY)
    set(G2O_FOUND TRUE)
    set(G2O_LIBRARIES ${G2O_CORE_LIBRARY} ${G2O_TYPES_SLAM3D_LIBRARY})
    if(G2O_TYPES_SLAM3D_ADDONS_LIBRARY)
        list(APPEND G2O_LIBRARIES ${G2O_TYPES_SLAM3D_ADDONS_LIBRARY})
    endif()
    # Link all available solvers
    foreach(_solver pcg cholmod csparse)
        if(G2O_SOLVER_${_solver})
            list(APPEND G2O_LIBRARIES ${G2O_SOLVER_${_solver}})
        endif()
    endforeach()
    message(STATUS "Found g2o:")
    message(STATUS "  include:  ${G2O_INCLUDE_DIRS}")
    message(STATUS "  libraries: ${G2O_LIBRARIES}")
else()
    set(G2O_FOUND FALSE)
    if(NOT G2O_INCLUDE_DIRS)
        message(STATUS "g2o include dir not found. Install via: vcpkg install g2o")
    endif()
    if(NOT G2O_CORE_LIBRARY)
        message(STATUS "g2o core library not found.")
    endif()
    if(NOT G2O_TYPES_SLAM3D_LIBRARY)
        message(STATUS "g2o types_slam3d library not found.")
    endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(G2O
    REQUIRED_VARS G2O_INCLUDE_DIRS G2O_CORE_LIBRARY G2O_TYPES_SLAM3D_LIBRARY
)
