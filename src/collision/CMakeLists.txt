###############################################
# Collision
PROJECT(collision)

# Search all header and source files
FILE(GLOB collision_srcs "*.cpp")
FILE(GLOB collision_hdrs "*.h")


# Library 

ADD_LIBRARY(collision ${collision_srcs} ${collision_hdrs})
TARGET_LINK_LIBRARIES(collision kinematics)
SET_TARGET_PROPERTIES(collision PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
SET_TARGET_PROPERTIES(collision PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

INSTALL(FILES ${collision_hdrs} DESTINATION include/rtql8/collision)
INSTALL(TARGETS collision DESTINATION lib)
