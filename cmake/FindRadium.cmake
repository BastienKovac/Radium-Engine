# Try to find the radium engine base folder
# Will define
# RADIUM_ROOT_DIR : the root of the radium SDK
# RADIUM_INCLUDE_DIR : the include directory of radium
# RADIUM_PLUGIN_OUTPUT_PATH : output path for radiums plugin


# Radium_FOUND if found

IF(NOT RADIUM_ROOT_DIR)
  FIND_PATH( RADIUM_ROOT_DIR NAMES src/Core/RaCore.hpp
    PATHS
    ${CMAKE_SOURCE_DIR}/extern
    ${CMAKE_SOURCE_DIR}/external
    ${CMAKE_SOURCE_DIR}/3rdPartyLibraries
    ${CMAKE_SOURCE_DIR}/..
    ${CMAKE_SOURCE_DIR}/../..
    ${CMAKE_SOURCE_DIR}/../../..
    ${CMAKE_CURRENT_SOURCE_DIR}/extern
    ${CMAKE_CURRENT_SOURCE_DIR}/external
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
    ${CMAKE_CURRENT_SOURCE_DIR}/../../..
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdPartyLibraries
    PATH_SUFFIXES Radium-Engine
    DOC "The radium engine source folder")
ENDIF(NOT RADIUM_ROOT_DIR)


IF ( RADIUM_ROOT_DIR )
    SET ( RADIUM_INCLUDES "${RADIUM_ROOT_DIR}/src")


    SET ( RADIUM_PLUGIN_OUTPUT_PATH "${RADIUM_ROOT_DIR}/${CMAKE_BUILD_TYPE}/Plugins")

    FIND_LIBRARY( RA_CORE_LIB
        NAMES radiumCore
        PATHS ${RADIUM_ROOT_DIR}/${CMAKE_BUILD_TYPE}/lib
        )

    FIND_LIBRARY( RA_ENGINE_LIB
        NAMES radiumEngine
        PATHS ${RADIUM_ROOT_DIR}/${CMAKE_BUILD_TYPE}/lib
        )

    FIND_LIBRARY ( RA_GUIBASE_LIB
        NAMES radiumGuiBase
        PATHS ${RADIUM_ROOT_DIR}/${CMAKE_BUILD_TYPE}/lib
        )

    SET ( Radium_FOUND TRUE )

    ############################################################################
    # Get dependencies if not already specified
    IF(NOT EIGEN3_INCLUDE_DIR)
        set(EIGEN3_INCLUDE_DIR ${RADIUM_ROOT_DIR}/${CMAKE_BUILD_TYPE}/3rdPartyLibraries/include)
    ENDIF(NOT EIGEN3_INCLUDE_DIR)

    IF (NOT ASSIMP_LIBRARIES)
        FIND_LIBRARY ( ASSIMP_LIBRARIES
            NAMES assimp assimpd
            PATHS ${RADIUM_ROOT_DIR}/${CMAKE_BUILD_TYPE}/3rdPartyLibraries/lib
            )
    ENDIF (NOT ASSIMP_LIBRARIES)

    IF(NOT ASSIMP_INCLUDE_DIR)
        set(ASSIMP_INCLUDE_DIR ${RADIUM_ROOT_DIR}/${CMAKE_BUILD_TYPE}/3rdPartyLibraries/include)
    ENDIF(NOT ASSIMP_INCLUDE_DIR)


    IF (NOT GLBINDING_LIBRARIES)
        FIND_LIBRARY ( GLBINDING_LIBRARIES
            NAMES glbinding glbindingd
            PATHS ${RADIUM_ROOT_DIR}/${CMAKE_BUILD_TYPE}/3rdPartyLibraries/lib
            )
    ENDIF (NOT GLBINDING_LIBRARIES)

    IF(NOT GLBINDING_INCLUDE_DIR)
        set(GLBINDING_INCLUDE_DIR ${RADIUM_ROOT_DIR}/${CMAKE_BUILD_TYPE}/3rdPartyLibraries/include)
    ENDIF(NOT GLBINDING_INCLUDE_DIR)

    # TODO (Mathias) : verify if cmake recommand this (append the include dir of module dependencis in the include dir of the module header)
    # TODO (Nico) : this is unconsistent with the lib behavior: deps *.so are not added in RADIUM_LIBRARIES
    SET( RADIUM_INCLUDE_DIR)
    LIST(APPEND RADIUM_INCLUDE_DIR "${RADIUM_INCLUDES}" "${EIGEN3_INCLUDE_DIR}" "${PATATE_INCLUDE_DIR}" "${ASSIMP_INCLUDE_DIR}" "${GLBINDING_INCLUDE_DIR}")

    # TODO (Mathias) Like above : is it recommended to add all dependencies here ?
    SET( RADIUM_LIBRARIES )
    IF ( RA_CORE_LIB AND RA_ENGINE_LIB AND RA_GUIBASE_LIB AND ASSIMP_LIBRARIES AND GLBINDING_LIBRARIES)
       LIST(APPEND RADIUM_LIBRARIES "${RA_CORE_LIB}" "${RA_ENGINE_LIB}" "${RA_GUIBASE_LIB}")
       SET ( Radium_Libs_FOUND TRUE)
    ENDIF ( RA_CORE_LIB AND RA_ENGINE_LIB AND RA_GUIBASE_LIB AND ASSIMP_LIBRARIES AND GLBINDING_LIBRARIES)
ENDIF( RADIUM_ROOT_DIR)

IF ( Radium_FOUND )
    IF(NOT Radium_FIND_QUIETLY)
      MESSAGE ( STATUS "Found Radium Engine: ${RADIUM_ROOT_DIR}")
      MESSAGE ( STATUS "      Eigen3 includes: ${EIGEN3_INCLUDE_DIR}")
      MESSAGE ( STATUS "      Patate includes: ${PATATE_INCLUDE_DIR}")
      MESSAGE ( STATUS "      Radium libs: ${RADIUM_LIBRARIES}")
      MESSAGE ( STATUS "      Assimp libs: ${ASSIMP_LIBRARIES}")
      MESSAGE ( STATUS "      GlBinding libs: ${GLBINDING_LIBRARIES}")
    ENDIF(NOT Radium_FIND_QUIETLY)
    IF (NOT Radium_Libs_FOUND)
        MESSAGE( WARNING "Could not find Radium libraries. You must compile them first")
    ENDIF (NOT Radium_Libs_FOUND)
ELSE(Radium_FOUND)
    IF(Radium_FIND_REQUIRED)
        MESSAGE( FATAL_ERROR "Could not find Radium Engine root dir")
    ENDIF(Radium_FIND_REQUIRED)
ENDIF(Radium_FOUND)
