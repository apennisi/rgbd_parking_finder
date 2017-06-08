project(parkingfinder)

set(execName1 planimetry)
set(execName2 parking_training)
set(execName3 bay_detector)
set(CMAKE_CXX_FLAGS "-std=c++11 -g -O3")

cmake_minimum_required(VERSION 2.8)

find_package(OpenCV 3.1 REQUIRED)
find_package(PCL REQUIRED)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") #this is needed for ubuntu 16.04 and pcl 1.7

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_BINARY_DIR}/include/filemanager)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/filemanager)
include_directories(${PROJECT_BINARY_DIR}/include/segmentation)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/segmentation)
include_directories(${PROJECT_BINARY_DIR}/include/utils)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/utils)
include_directories(${PROJECT_BINARY_DIR}/include/ml)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/ml)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

set(SRC_FOLDER src/application )
set(EXECUTABLE_OUTPUT_PATH ../bin)

file(GLOB_RECURSE PLANIMETRY_SRC_FILES "src/applications/planimetry.cpp" "src/utils/homography.cpp" "src/utils/parking_selection.cpp" "src/utils/point.cpp" "src/utils/rectangle.cpp" 
				       "src/filemanager/plyfilereader.cpp" "src/filemanager/filewriter.cpp" "src/segmentation/plane_viewer.cpp")
				       
file(GLOB_RECURSE TRAINING_SRC_FILES "src/applications/parking_training.cpp" "src/utils/homography.cpp" "src/utils/point.cpp" "src/utils/rectangle.cpp" 
				       "src/filemanager/plyfilereader.cpp" "src/utils/planimetry_manager.cpp"
				       "src/ml/svm.cpp" "src/ml/adaboost.cpp" "src/filemanager/featurefilemanager.cpp" "src/filemanager/configmanager.cpp" 
				       "src/segmentation/bayfeatures.cpp" "src/segmentation/combined_features.cpp" "src/segmentation/esf_features.cpp" 
				       "src/segmentation/fpfh_features.cpp" "src/segmentation/pfh_features.cpp" "src/segmentation/vfh_features.cpp")

file(GLOB_RECURSE BAY_SRC_FILES "src/applications/bays_detection.cpp" "src/utils/homography.cpp" "src/utils/point.cpp" "src/utils/rectangle.cpp" 
				       "src/filemanager/plyfilereader.cpp" "src/utils/planimetry_manager.cpp" 
				       "src/ml/svm.cpp" "src/ml/adaboost.cpp" "src/filemanager/featurefilemanager.cpp" "src/filemanager/configmanager.cpp"
				       "src/segmentation/bayfeatures.cpp" "src/segmentation/combined_features.cpp" "src/segmentation/esf_features.cpp" 
				       "src/segmentation/fpfh_features.cpp" "src/segmentation/pfh_features.cpp" "src/segmentation/vfh_features.cpp")				       

add_executable(${execName1} ${PLANIMETRY_SRC_FILES})
add_executable(${execName2} ${TRAINING_SRC_FILES})
add_executable(${execName3} ${BAY_SRC_FILES})


TARGET_LINK_LIBRARIES( ${execName1} -lm -lpthread  ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
TARGET_LINK_LIBRARIES( ${execName2} -lm -lpthread  ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
TARGET_LINK_LIBRARIES( ${execName3} -lm -lpthread  ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
