###############################################
# apps/server


message(STATUS "configuring server for knowledge")
project(server)
if (KN_USE_MPI)
message(STATUS "set CMAKE_CXX_COMPILER mpicxx")
set(CMAKE_CXX_COMPILER mpicxx)
endif()

add_definitions(-DOMPI_IMPORTS)
add_definitions(-DOPAL_IMPORTS)
add_definitions(-DORTE_IMPORTS)

file(GLOB server_srcs "*.cpp")
file(GLOB server_hdrs "*.h")

add_executable(server ${server_srcs} ${server_hdrs})

target_link_libraries(server utils renderer kinematics dynamics geometry)
target_link_libraries(server toolkit)
target_link_libraries(server controller)
target_link_libraries(server ${Boost_LIBRARIES})
target_link_libraries(server mysqlcppconn)

set_target_properties(server PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
