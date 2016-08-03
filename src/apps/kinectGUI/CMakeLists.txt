###############################################
# apps/kinectGUI

PROJECT(kinectGUI)
FILE(GLOB kinectGUI_srcs "*.cpp")
FILE(GLOB kinectGUI_hdrs "*.h")
ADD_EXECUTABLE(kinectGUI ${kinectGUI_srcs} ${kinectGUI_hdrs})
LINK_THIRDPARTY_GLUT(kinectGUI)
LINK_THIRDPARTY_TICPP(kinectGUI)

TARGET_LINK_LIBRARIES(kinectGUI kinematics optimizer yui renderer)
TARGET_LINK_LIBRARIES(kinectGUI kinect ${KinectSDK_LIBRARIES})
# comctl32.lib glu32.lib opengl32.lib Kinect10.lib SOIL.lib
# FaceTrackLib.lib KinectFusion170_32.lib KinectInteraction170_32.lib Msdmo.lib dmoguids.lib amstrmid.lib avrt.lib)

SET_TARGET_PROPERTIES(kinectGUI PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
