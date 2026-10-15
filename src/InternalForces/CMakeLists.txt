project(${BIORBD_NAME}_internal_forces)
set(INTERNAL_FORCES_MODULE_NAME ${BIORBD_NAME}_internal_forces PARENT_SCOPE)

# Add the relevant files
set(SRC_LIST_MODULE
    "${CMAKE_CURRENT_SOURCE_DIR}/PathModifiers.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/ViaPoint.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Geometry.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Compound.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/WrappingHalfCylinder.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/WrappingObject.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/WrappingSphere.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/WrappingSurface.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/WrappingGeodesic.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/WrappingNumericalKernel.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/WrappingNgv.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/WrappingCylinder.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/WrappingEllipsoid.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/WrappingTorus.cpp"
)

if(BIORBD_USE_CASADI_MATH)
    list(APPEND SRC_LIST_MODULE "${CMAKE_CURRENT_SOURCE_DIR}/WrappingCasadi.cpp")
endif()

# Create the library
if (WIN32)
    add_library(${PROJECT_NAME} STATIC "${SRC_LIST_MODULE}")
else()
    if (BUILD_SHARED_LIBS)
        add_library(${PROJECT_NAME} SHARED "${SRC_LIST_MODULE}")
    else()
        add_library(${PROJECT_NAME} STATIC "${SRC_LIST_MODULE}")
    endif()
endif()
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_debug")

# Add the include
target_include_directories(${PROJECT_NAME} PRIVATE
    "${RBDL_INCLUDE_DIR}"
    "${MATH_BACKEND_INCLUDE_DIR}"
    "${EIGEN3_INCLUDE_DIR}"
    "${CMAKE_SOURCE_DIR}/include"
    "${BIORBD_BINARY_DIR}/include"

)
# Modern Eigen config packages provide their include paths on the imported
# target; older discovery modules use EIGEN3_INCLUDE_DIR above.
if(TARGET Eigen3::Eigen)
    target_include_directories(${PROJECT_NAME} PRIVATE
        $<TARGET_PROPERTY:Eigen3::Eigen,INTERFACE_INCLUDE_DIRECTORIES>)
endif()

# Add the dependencies for insuring build order
target_link_libraries(${PROJECT_NAME} PUBLIC
    "${RBDL_LIBRARY}"
    "${MATH_BACKEND_LIBRARIES}"
    "${BIORBD_NAME}_utils"
    "${BIORBD_NAME}_rigidbody"
)
if(BIORBD_WITH_SIMBODY_WRAPPING)
    target_compile_definitions(${PROJECT_NAME} PRIVATE BIORBD_WITH_SIMBODY_WRAPPING=1)
    target_link_libraries(${PROJECT_NAME} PRIVATE SimTKsimbody)
endif()
add_dependencies(${PROJECT_NAME}
    "${BIORBD_NAME}_utils"
    "${BIORBD_NAME}_rigidbody"
)

# Installation
install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION "${${BIORBD_NAME}_LIB_FOLDER}"
    RUNTIME DESTINATION "${${BIORBD_NAME}_BIN_FOLDER}"
    LIBRARY DESTINATION "${${BIORBD_NAME}_LIB_FOLDER}"
)
set_target_properties(${PROJECT_NAME} PROPERTIES
    INSTALL_RPATH "${${BIORBD_NAME}_BIN_FOLDER}"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)
