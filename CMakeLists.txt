cmake_minimum_required (VERSION 3.8)
set(CMAKE_CXX_STANDARD 17)

project ("TOF-RGB_Stereocalibration" LANGUAGES C CXX)
add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
add_definitions(-w)

#copy image data to destination.
file(COPY ${CMAKE_SOURCE_DIR}/image_data DESTINATION .)


#set cmake build mode
if(CMAKE_BUILD_TYPE AND (CMAKE_BUILD_TYPE STREQUAL "Debug"))
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wall -O0")
    message("Debug mode:${CMAKE_C_FLAGS_DEBUG}")
elseif(CMAKE_BUILD_TYPE AND (CMAKE_BUILD_TYPE STREQUAL "Release"))
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Wall -O3")
    message("Release mode:${CMAKE_C_FLAGS_RELEASE}")
else()
    message("else:${CMAKE_BUILD_TYPE}")
    message("else:${CMAKE_C_FLAGS_RELEASE}")
endif()

#config opencv (windows)
include_directories(${CMAKE_SOURCE_DIR}/3rdparty/opencv4.5.5/include/)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(OpenCV_LINK_DIR ${CMAKE_SOURCE_DIR}/3rdparty/opencv4.5.5/lib/debug/)
    file(GLOB_RECURSE OPENCV_DLL ${CMAKE_SOURCE_DIR}/3rdparty/opencv4.5.5/lib/debug/*.dll)
    file(GLOB OpenCV_LIBS ${OpenCV_LINK_DIR}/*.lib)
else()
    set(OpenCV_LINK_DIR ${CMAKE_SOURCE_DIR}/3rdparty/opencv4.5.5/lib/release/)
    file(GLOB_RECURSE OPENCV_DLL ${CMAKE_SOURCE_DIR}/3rdparty/opencv4.5.5/lib/release/*.dll)
    file(GLOB OpenCV_LIBS ${OpenCV_LINK_DIR}/*.lib)
endif()

file(COPY ${OPENCV_DLL} DESTINATION .)
file(GLOB SOURCE ${CMAKE_SOURCE_DIR}/src/*/*.cpp)

add_executable(TOF-RGB_stereocalibration  "src/main.cpp" ${SOURCE} )
target_link_libraries(TOF-RGB_stereocalibration ${OpenCV_LIBS} )
