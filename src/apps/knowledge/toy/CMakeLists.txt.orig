 ###############################################
# apps/toy

message(STATUS "configuring toy for knowledge")
project(toy)

file(GLOB toy_srcs "*.cpp")
file(GLOB toy_hdrs "*.h")

add_executable(toy ${toy_srcs} ${toy_hdrs})

target_link_libraries(toy utils renderer kinematics dynamics geometry)
target_link_libraries(toy toolkit)
target_link_libraries(toy controller)
target_link_libraries(toy clust)
target_link_libraries(toy ${Boost_LIBRARIES})


set_target_properties(toy PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

