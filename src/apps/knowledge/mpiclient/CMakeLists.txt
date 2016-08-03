###############################################
# apps/mpiclient

message(STATUS "configuring mpiclient for knowledge")
project(mpiclient)

file(GLOB mpiclient_srcs "*.cpp")
file(GLOB mpiclient_hdrs "*.h")

add_library(mpiclient SHARED
  ${mpiclient_srcs} ${mpiclient_hdrs}
  )

target_link_libraries(mpiclient utils renderer kinematics dynamics geometry)
# target_link_libraries(mpiclient glog gflags)
target_link_libraries(mpiclient toolkit)
target_link_libraries(mpiclient ${Boost_LIBRARIES})
target_link_libraries(mpiclient ssh)

set_target_properties(mpiclient PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

