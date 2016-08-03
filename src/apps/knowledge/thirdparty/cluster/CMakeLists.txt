###############################################
# apps/clust

message(STATUS "configuring thirparty/cluster for knowledge")
project(clust)

file(GLOB clust_srcs "*.c")
file(GLOB clust_hdrs "*.h")

add_library(clust ${clust_srcs} ${clust_hdrs} )

# target_link_libraries(clust utils renderer kinematics dynamics geometry)


set_target_properties(clust PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

