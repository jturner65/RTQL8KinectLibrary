###############################################
# geometry
PROJECT(geometry)

# Search all header and source files
FILE(GLOB geometry_srcs "*.cpp")
FILE(GLOB geometry_hdrs "*.h")

# Library 
ADD_LIBRARY(geometry ${geometry_srcs} ${geometry_hdrs})
SET_TARGET_PROPERTIES(geometry PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
SET_TARGET_PROPERTIES(geometry PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

INSTALL(FILES ${geometry_hdrs} DESTINATION include/rtql8/geometry)
INSTALL(TARGETS geometry DESTINATION lib)
