###############################################
# renderer
PROJECT(renderer)

# Search all header and source files
FILE(GLOB renderer_srcs "*.cpp")
FILE(GLOB renderer_hdrs "*.h")

# Library 
ADD_LIBRARY(renderer ${renderer_srcs} ${renderer_hdrs})
SET_TARGET_PROPERTIES(renderer PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
SET_TARGET_PROPERTIES(renderer PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

INSTALL(FILES ${renderer_hdrs} DESTINATION include/rtql8/renderer)
INSTALL(TARGETS renderer DESTINATION lib)
