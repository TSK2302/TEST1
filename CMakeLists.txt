cmake_minimum_required(VERSION 3.16)

project(UI_CHANGES VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Qt
find_package(QT NAMES Qt6 Qt5 REQUIRED Widgets OpenGL OpenGLWidgets Charts Concurrent)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED Widgets OpenGL OpenGLWidgets Charts Concurrent)

# Find OpenGL
find_package(OpenGL REQUIRED)

# Find Assimp
find_package(assimp REQUIRED)

set(Python_EXECUTABLE "C:/Users/tsaya/.conda/envs/BENZ1/python.exe")
find_package(Python REQUIRED COMPONENTS Interpreter Development)

set(BOOST_ROOT "Y:/PCL 1.13.0/3rdParty/Boost")
set(BOOST_LIBRARYDIR "Y:/PCL 1.13.0/3rdParty/Boost/lib")
set(Boost_NO_BOOST_CMAKE TRUE)
set(Boost_DEBUG ON) # Enable debug output for Boost detection
set(Boost_USE_STATIC_LIBS ON) # Use static Boost libraries
set(Boost_USE_MULTITHREADED ON)
set(Boost_NO_SYSTEM_PATHS ON) # Force CMake to use BOOST_ROOT
set(Boost_COMPILER "-vc143") # Specify MSVC 2022

add_definitions(-DNOMINMAX)

# Print Boost library directory contents for debugging
file(GLOB BOOST_LIB_FILES "Y:/PCL 1.13.0/3rdParty/Boost/lib/*.lib")
message(STATUS "Boost library files found: ${BOOST_LIB_FILES}")

# Explicitly find Boost with components required by PCL
find_package(Boost 1.80.0 REQUIRED COMPONENTS system filesystem thread date_time iostreams chrono atomic regex)
if(Boost_FOUND)
    message(STATUS "Boost found: ${Boost_INCLUDE_DIRS}")
    message(STATUS "Boost libraries: ${Boost_LIBRARIES}")
    include_directories(${Boost_INCLUDE_DIRS})
    link_directories(${Boost_LIBRARY_DIRS})
else()
    message(FATAL_ERROR "Boost not found at Y:/PCL 1.13.0/3rdParty/Boost. Please reinstall PCL with Boost or install Boost manually.")
endif()

set(PCL_DIR "Y:/PCL 1.13.0/cmake")
find_package(PCL 1.13 REQUIRED COMPONENTS common io kdtree visualization filters registration segmentation)


set(CILANTRO_DIR "C:/Program Files/cilantro/")
set(CILANTRO_INCLUDE_DIR "C:/Program Files/cilantro/include")

include_directories(${CILANTRO_INCLUDE_DIR})


find_library(CILANTRO_LIBRARY
    NAMES cilantro
    PATHS ${CILANTRO_DIR}
    PATH_SUFFIXES Debug Release lib
    NO_DEFAULT_PATH
)

if(NOT CILANTRO_LIBRARY)
    message(FATAL_ERROR "Cilantro library not found in ${CILANTRO_DIR}. Ensure it's built.")
else()
    message(STATUS "Found Cilantro library: ${CILANTRO_LIBRARY}")
endif()


# Configure pybind11 to use Python 3.9
set(PYBIND11_PYTHON_VERSION 3.9)
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 REQUIRED)
message(STATUS "pybind11_INCLUDE_DIRS: ${pybind11_INCLUDE_DIRS}")

# Specify the nlohmann/json include directory
set(NLOHMANN_JSON_INCLUDE_DIR "C://Program Files (x86)//nlohmann_json//include")
include_directories(${NLOHMANN_JSON_INCLUDE_DIR})

# Define source files
set(PROJECT_SOURCES
    main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
        openglwidget.cpp
        openglwidget.h

        # Grid dialogs
        generate2dgriddialog.cpp
        generate2dgriddialog.h
        generate2dgriddialog.ui
        generate3dgriddialog.cpp
        generate3dgriddialog.h
        generate3dgriddialog.ui
        convert2dto3dgriddialog.cpp
        convert2dto3dgriddialog.h
        convert2dto3dgriddialog.ui
        gridvisibilitydialog.cpp
        gridvisibilitydialog.h
        gridvisibilitydialog.ui
        multigriddialog.cpp
        multigriddialog.h
        multigriddialog.ui

        # Segmentation
        segmentation.cpp
        segmentation.h
        segmentation.ui

        # Sampling
        samplingprocessor.cpp
        samplingprocessor.h
        samplingprocessor.ui

        # Tools
        collision_detection.cpp
        collision_detection.h
        transformmanager.cpp
        transformmanager.h
        cloudcomparison.cpp
        cloudcomparison.h

        cluster.cpp
        cluster.h
        cluster.ui
        identifysimilarclustersdialog.cpp
        identifysimilarclustersdialog.h

        colorchanger.cpp
        colorchanger.h
        colorchanger.ui

        collisiondialog.h
        collisiondialog.cpp

        objectdetection.cpp
        objectdetection.h
        clusterlist.cpp
        clusterlist.h
        clusterlist.ui

        manageobjects.cpp
        manageobjects.h
        manageobjects.ui

        managetwins.cpp
        managetwins.h
        managetwins.ui

        viewportcapturedialog.h
        viewportcapturedialog.cpp

        replacewithtwins.cpp
        replacewithtwins.h
        replacewithtwins.ui

        colorassigner.cpp
        colorassigner.h

        gridpopulationdensitydialog.cpp
        gridpopulationdensitydialog.h
        gridpopulationdensitydialog.ui


        # Add the resource file
        resources.qrc
        fonts.qrc
    )


foreach(source_file ${PROJECT_SOURCES})
    if(NOT EXISTS "${CMAKE_SOURCE_DIR}/${source_file}")
        message(WARNING "Source file not found: ${CMAKE_SOURCE_DIR}/${source_file}")
    endif()
endforeach()

# Qt executable
qt_add_executable(${PROJECT_NAME}
    MANUAL_FINALIZATION
    ${PROJECT_SOURCES}
)
# Link libraries
# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt6::Widgets
    Qt6::OpenGL
    Qt6::OpenGLWidgets
    Qt6::Charts
    Qt6::Concurrent
    ${OPENGL_LIBRARIES}
    assimp::assimp
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${CILANTRO_LIBRARY}
    Python::Python
    pybind11::embed
)

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
    ${ASSIMP_DIR}/include
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CILANTRO_INCLUDE_DIR}
    ${CILANTRO_BUILD_INCLUDE_DIR}
    ${Python_INCLUDE_DIRS}
    ${pybind11_INCLUDE_DIRS}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    ${PCL_DEFINITIONS}
    EIGEN_NO_DEBUG
)

message(STATUS "PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
message(STATUS "PCL_LIBRARIES: ${PCL_LIBRARIES}")
message(STATUS "Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARIES}")

# Platform-specific settings
set_target_properties(${PROJECT_NAME} PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER com.example.${PROJECT_NAME}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

# Install rules
include(GNUInstallDirs)
install(TARGETS ${PROJECT_NAME}
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Finalize Qt executable
if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(${PROJECT_NAME})
endif()

# Optional: Disable deprecated APIs (equivalent to QT_DISABLE_DEPRECATED_BEFORE)
# add_definitions(-DQT_DISABLE_DEPRECATED_BEFORE=0x060000)
