###############################################
# apps/hanging
PROJECT(hanging)
FILE(GLOB collision_srcs "*.cpp")
FILE(GLOB collision_hdrs "*.h")
ADD_EXECUTABLE(hanging ${collision_srcs} ${collision_hdrs})
TARGET_LINK_LIBRARIES(hanging lcpsolver collision kinematics dynamics yui renderer integration)
LINK_THIRDPARTY_GLUT(hanging)
LINK_THIRDPARTY_TICPP(hanging)

SET_TARGET_PROPERTIES(hanging PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
