###############################################
# apps/mpisolver

message(STATUS "configuring mpisolver for knowledge")
project(mpisolver)
message(STATUS "set CMAKE_CXX_COMPILER as mpicxx")
set(CMAKE_CXX_COMPILER mpicxx)

file(GLOB mpisolver_srcs "*.cpp")
file(GLOB mpisolver_hdrs "*.h")

add_executable(mpisolver
  ${mpisolver_srcs} ${mpisolver_hdrs}
  )

target_link_libraries(mpisolver utils renderer kinematics dynamics geometry)
# target_link_libraries(mpisolver optimizer snopt gfortran)
# target_link_libraries(mpisolver optimizer)
# target_link_libraries(mpisolver glog gflags)
target_link_libraries(mpisolver toolkit)
target_link_libraries(mpisolver controller)
target_link_libraries(mpisolver shark)

target_link_libraries(mpisolver ${QT_LIBRARIES})
target_link_libraries(mpisolver ${Boost_LIBRARIES})

set_target_properties(mpisolver PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

