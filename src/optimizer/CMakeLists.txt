###############################################
# optimizer
PROJECT(optimizer)

# Search all header and source files
FILE(GLOB optimizer_srcs "*.cpp")
FILE(GLOB optimizer_hdrs "*.h")

# Library 
ADD_LIBRARY(optimizer ${optimizer_srcs} ${optimizer_hdrs})
TARGET_LINK_LIBRARIES(optimizer utils)
SET_TARGET_PROPERTIES(optimizer PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
SET_TARGET_PROPERTIES(optimizer PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

INSTALL(FILES ${optimizer_hdrs} DESTINATION include/rtql8/optimizer)
INSTALL(FILES ${optimizer_hpps} DESTINATION include/rtql8/optimizer)
INSTALL(TARGETS optimizer DESTINATION lib)
