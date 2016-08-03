###############################################
# apps/gui

message(STATUS "configuring gui for knowledge")
project(gui)

file(GLOB gui_srcs "*.cpp")
file(GLOB gui_hdrs "*.h")

if (NOT KN_USE_KINECT)
  list(REMOVE_ITEM gui_srcs "${CMAKE_CURRENT_SOURCE_DIR}/GuiKinController.cpp")
  message(STATUS "remove: ${CMAKE_CURRENT_SOURCE_DIR}/GuiKinController.cpp")
  message(STATUS "gui_srcs = ${gui_srcs}")
endif()


qt4_wrap_cpp(gui_moc_files window.h glwidget.h widget_opt_explorer.h)

add_executable(gui
  ${gui_srcs} ${gui_hdrs} ${gui_moc_files}
  )

target_link_libraries(gui utils renderer kinematics dynamics geometry)
if (KN_USE_KINECT)
target_link_libraries(gui kinect)
endif()

target_link_libraries(gui toolkit)
target_link_libraries(gui controller operation)
if (KN_COMPILE_SOLVER)
message(STATUS "configuring gui for knowledge -- add solver")
target_link_libraries(gui solver)
endif()

target_link_libraries(gui ${QT_LIBRARIES})
target_link_libraries(gui ${Boost_LIBRARIES})
# target_link_libraries(gui ssh)
# target_link_libraries(gui 
#                       ${Boost_FILESYSTEM_LIBRARIES}
#                       ${Boost_SYSTEM_LIBRARIES}
#                       )

# set_target_properties(gui PROPERTIES LINK_FLAGS " -pagezero_size 10000 -image_base 100000000")
set_target_properties(gui PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

