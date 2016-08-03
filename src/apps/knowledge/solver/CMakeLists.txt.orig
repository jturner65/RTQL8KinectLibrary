###############################################
# apps/solver

message(STATUS "configuring solver for knowledge")
project(solver)

file(GLOB solver_srcs "*.cpp")
file(GLOB solver_hdrs "*.h")

add_library(solver ${solver_srcs} ${solver_hdrs})

target_link_libraries(solver utils renderer kinematics dynamics geometry)
target_link_libraries(solver toolkit controller)
target_link_libraries(solver ${Boost_LIBRARIES})
target_link_libraries(solver mysqlcppconn)
target_link_libraries(solver shark)
target_link_libraries(solver clust)

set_target_properties(solver PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

