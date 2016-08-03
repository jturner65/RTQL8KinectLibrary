###############################################
# lcpsolver
PROJECT(lcpsolver)

# Search all header and source files
FILE(GLOB lcpsolver_srcs "*.cpp")
FILE(GLOB lcpsolver_hdrs "*.h")

# Library 
ADD_LIBRARY(lcpsolver ${lcpsolver_srcs} ${lcpsolver_hdrs})
SET_TARGET_PROPERTIES(lcpsolver PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
SET_TARGET_PROPERTIES(lcpsolver PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

INSTALL(FILES ${lcpsolver_hdrs} DESTINATION include/rtql8/lcpsolver)
INSTALL(TARGETS lcpsolver DESTINATION lib)
