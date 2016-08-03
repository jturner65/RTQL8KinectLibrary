###############################################
# apps/closedLoop
PROJECT(closedLoop)
FILE(GLOB closedLoop_srcs "*.cpp")
FILE(GLOB closedLoop_hdrs "*.h")
ADD_EXECUTABLE(closedLoop ${closedLoop_srcs} ${closedLoop_hdrs})
TARGET_LINK_LIBRARIES(closedLoop kinematics dynamics yui renderer integration)
LINK_THIRDPARTY_GLUT(closedLoop)
LINK_THIRDPARTY_TICPP(closedLoop)

SET_TARGET_PROPERTIES(closedLoop PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
