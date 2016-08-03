###############################################
# YUI
PROJECT(yui)
# Search all header and source files
FILE(GLOB yui_srcs "*.cpp")
FILE(GLOB yui_hdrs "*.h")
# Library 
ADD_LIBRARY(yui ${yui_srcs} ${yui_hdrs})
SET_TARGET_PROPERTIES(yui PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

INSTALL(FILES ${yui_hdrs} DESTINATION include/rtql8/yui)
INSTALL(TARGETS yui DESTINATION lib)