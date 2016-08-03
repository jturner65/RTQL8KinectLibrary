###############################################
# kinect



PROJECT(kinect)

# INCLUDE_DIRECTORIES(${KinectSDK_INCLUDE_DIR})
# INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/kinectUI_SDK/kinectSDK)
# INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/kinectUI_SDK/speechSDK)
# INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/kinectUI_SDK/kinectSDK/lib)
# INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/kinectUI_SDK/speechSDK/lib)

# Search all header and source files
FILE(GLOB kinect_srcs "*.cpp")
FILE(GLOB kinect_hdrs "*.h")
FILE(GLOB kinect_hpps "*.hpp")

# Library 
ADD_LIBRARY(kinect ${kinect_srcs} ${kinect_hdrs} ${kinect_hpps} )
TARGET_LINK_LIBRARIES(kinect kinematics optimizer yui renderer )
TARGET_LINK_LIBRARIES(kinect ${KinectSDK_LIBRARIES})

SET_TARGET_PROPERTIES(kinect PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
SET_TARGET_PROPERTIES(kinect PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

INSTALL(FILES ${kinect_hdrs} DESTINATION include/rtql8/kinect)
INSTALL(FILES ${kinect_hpps} DESTINATION include/rtql8/kinect)
INSTALL(TARGETS kinect DESTINATION lib)
