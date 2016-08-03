###############################################
# apps/controller

message(STATUS "configuring controller for knowledge")
project(controller)

file(GLOB controller_srcs "*.cpp")
file(GLOB controller_hdrs "*.h")

add_library(controller 
  ${controller_srcs} ${controller_hdrs}
  )

target_link_libraries(controller utils renderer kinematics dynamics geometry)
# target_link_libraries(controller optimizer snopt gfortran)
# target_link_libraries(controller glog gflags)
target_link_libraries(controller toolkit)
target_link_libraries(controller ${Boost_LIBRARIES})


set_target_properties(controller PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

