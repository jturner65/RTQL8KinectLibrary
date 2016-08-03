###############################################
# apps/benchmark

message(STATUS "configuring benchmark for knowledge")
project(benchmark)

file(GLOB benchmark_srcs "*.cpp")
file(GLOB benchmark_hdrs "*.h")

add_executable(benchmark ${benchmark_srcs} ${benchmark_hdrs})

target_link_libraries(benchmark utils renderer kinematics dynamics geometry)
target_link_libraries(benchmark toolkit)
target_link_libraries(benchmark controller)
target_link_libraries(benchmark ${Boost_LIBRARIES})
target_link_libraries(benchmark 
                      ${Boost_FILESYSTEM_LIBRARIES}
                      ${Boost_SYSTEM_LIBRARIES}
                      )

set_target_properties(benchmark PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

