###############################################
# Simulation
PROJECT(simulation)

# Search all header and source files
FILE(GLOB simulation_srcs "*.cpp")
FILE(GLOB simulation_hdrs "*.h")

# Library 
ADD_LIBRARY(simulation ${simulation_srcs} ${simulation_hdrs})
TARGET_LINK_LIBRARIES(simulation collision dynamics geometry integration kinematics lcpsolver renderer utils yui)
SET_TARGET_PROPERTIES(simulation PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
SET_TARGET_PROPERTIES(simulation PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

INSTALL(FILES ${simulation_hdrs} DESTINATION include/rtql8/simulation)
INSTALL(TARGETS simulation DESTINATION lib)
