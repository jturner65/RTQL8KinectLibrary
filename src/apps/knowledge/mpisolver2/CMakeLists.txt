###############################################
# apps/mpisolver2

message(STATUS "configuring mpisolver2 for knowledge")
project(mpisolver2)
message(STATUS "set CMAKE_CXX_COMPILER as mpicxx")
set(CMAKE_CXX_COMPILER mpicxx)

file(GLOB mpisolver2_srcs "*.cpp")
file(GLOB mpisolver2_hdrs "*.h")

add_executable(mpisolver2
  ${mpisolver2_srcs} ${mpisolver2_hdrs}
  )

target_link_libraries(mpisolver2 utils renderer kinematics dynamics geometry)
# target_link_libraries(mpisolver2 optimizer snopt gfortran)
# target_link_libraries(mpisolver2 optimizer)
# target_link_libraries(mpisolver2 glog gflags)
target_link_libraries(mpisolver2 toolkit)
target_link_libraries(mpisolver2 controller)
target_link_libraries(mpisolver2 shark)
target_link_libraries(mpisolver2 clust)

target_link_libraries(mpisolver2 ${QT_LIBRARIES})
target_link_libraries(mpisolver2 ${Boost_LIBRARIES})

set_target_properties(mpisolver2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

