# A list of applications

IF (WIN32)
MESSAGE(STATUS "Configure kinectGUI since OS is Win32")
ADD_SUBDIRECTORY(kinectGUI)
ELSE()
MESSAGE(STATUS "Do not configure kinectGUI since OS is not Win32")
ENDIF()

# ADD_SUBDIRECTORY(knowledge)

# ADD_SUBDIRECTORY(viewer)
# ADD_SUBDIRECTORY(forwardSim)
# ADD_SUBDIRECTORY(cubes)
# ADD_SUBDIRECTORY(motionAnalysis)
ADD_SUBDIRECTORY(pdController)
ADD_SUBDIRECTORY(balance)
ADD_SUBDIRECTORY(ik)
# ADD_SUBDIRECTORY(hybrid)
# ADD_SUBDIRECTORY(hanging)
# ADD_SUBDIRECTORY(closedLoop)
