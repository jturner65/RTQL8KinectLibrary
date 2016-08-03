###############################################
# Toolkit

MESSAGE(STATUS "configuring toolkit..")

PROJECT(toolkit)

# Search all header and source files
FILE(GLOB toolkit_srcs "*.cpp")
FILE(GLOB toolkit_hdrs "*.h")
FILE(GLOB toolkit_hpps "*.hpp")

ADD_LIBRARY(toolkit ${toolkit_srcs} ${toolkit_hdrs} ${toolkit_hpps})
TARGET_LINK_LIBRARIES(toolkit kinematics dynamics yui renderer integration)
LINK_THIRDPARTY_GLUT(toolkit)
LINK_THIRDPARTY_TICPP(toolkit)
SET_TARGET_PROPERTIES(toolkit PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# target_link_libraries(toolkit ${PYTHON_LIBRARIES} boost_python)




