###############################################
# apps/operation

message(STATUS "configuring operation for knowledge")
project(operation)

file(GLOB operation_srcs "*.cpp")
file(GLOB operation_hdrs "*.h")

add_library(operation 
  ${operation_srcs} ${operation_hdrs}
  )

target_link_libraries(operation utils renderer kinematics dynamics geometry)
target_link_libraries(operation toolkit controller)
target_link_libraries(operation ${Boost_LIBRARIES})


set_target_properties(operation PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

