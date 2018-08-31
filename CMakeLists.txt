cmake_minimum_required(VERSION 2.6)

project(segmatch C CXX)

set(CMAKE_BUILD_TYPE "Debug") 
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")

if(UNIX)
    message(STATUS "os type: linux")
    # find_package
    find_package(PCL 1.8 REQUIRED)
    find_package(OpenCV 2.4.9 REQUIRED)
    find_package(libnabo)

    # include_directories
    include_directories(${PCL_INCLUDE_DIRS})
    include_directories(${OpenCV_INCLUDE_DIRS})
    include_directories(${libnabo_INCLUDE_DIRS})
    include_directories(${PROJECT_SOURCE_DIR}/include)
    include_directories(/usr/include/yaml-cpp/) #yaml header file

    # link_directories
    link_directories(${PCL_LIBRARY_DIRS})
    link_directories(${OpenCV_LIBRARY_DIRS})
    link_directories(/usr/lib/x86_64-linux-gnu/)    #yaml lib link

    # add_definitions
    add_definitions(-std=c++11)
    add_definitions(-DHAVE_PCAP)
    add_definitions(${PCL_DEFINITIONS})
endif()

add_library(segmatch SHARED ${PROJECT_SOURCE_DIR}/src/segmatch/SegMatch.cpp
                            ${PROJECT_SOURCE_DIR}/src/segmatch/segmenters/region_growing_segmenter.cpp
                            ${PROJECT_SOURCE_DIR}/src/segmatch/segmenters/euclidean_segmenter.cpp)                            
target_link_libraries(segmatch ${OpenCV_LIBS})                            


add_executable (test_find_match ${PROJECT_SOURCE_DIR}/test/test_find_match.cpp)
target_link_libraries (test_find_match ${PCL_LIBRARIES} 
                                        ${OpenCV_LIBRARIES} 
                                        ${libnabo_LIBRARIES}
                                        libyaml-cpp.so
                                        segmatch)

add_executable (test_train_forest ${PROJECT_SOURCE_DIR}/test/test_train_forest.cpp)
target_link_libraries (test_train_forest ${PCL_LIBRARIES} 
                                            ${OpenCV_LIBRARIES} 
                                            ${libnabo_LIBRARIES}
                                            libyaml-cpp.so
                                            segmatch)

add_executable (test_code ${PROJECT_SOURCE_DIR}/test/test.cpp)
target_link_libraries (test_code ${PCL_LIBRARIES} 
                                    ${OpenCV_LIBRARIES} 
                                    ${libnabo_LIBRARIES}
                                    libyaml-cpp.so
                                    segmatch)