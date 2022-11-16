# Install script for directory: /home/brky/workspaces/fjnunes_ws/src/ompl/demos

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE FILE FILES
    "/home/brky/workspaces/fjnunes_ws/src/ompl/demos/CForestCircleGridBenchmark.cpp"
    "/home/brky/workspaces/fjnunes_ws/src/ompl/demos/GeometricCarPlanning.cpp"
    "/home/brky/workspaces/fjnunes_ws/src/ompl/demos/HybridSystemPlanning.cpp"
    "/home/brky/workspaces/fjnunes_ws/src/ompl/demos/HypercubeBenchmark.cpp"
    "/home/brky/workspaces/fjnunes_ws/src/ompl/demos/KinematicChainBenchmark.cpp"
    "/home/brky/workspaces/fjnunes_ws/src/ompl/demos/LTLWithTriangulation.cpp"
    "/home/brky/workspaces/fjnunes_ws/src/ompl/demos/OpenDERigidBodyPlanning.cpp"
    "/home/brky/workspaces/fjnunes_ws/src/ompl/demos/OptimalPlanning.cpp"
    "/home/brky/workspaces/fjnunes_ws/src/ompl/demos/PlannerData.cpp"
    "/home/brky/workspaces/fjnunes_ws/src/ompl/demos/PlannerProgressProperties.cpp"
    "/home/brky/workspaces/fjnunes_ws/src/ompl/demos/Point2DPlanning.cpp"
    "/home/brky/workspaces/fjnunes_ws/src/ompl/demos/RigidBodyPlanning.cpp"
    "/home/brky/workspaces/fjnunes_ws/src/ompl/demos/RigidBodyPlanningWithControls.cpp"
    "/home/brky/workspaces/fjnunes_ws/src/ompl/demos/RigidBodyPlanningWithIK.cpp"
    "/home/brky/workspaces/fjnunes_ws/src/ompl/demos/RigidBodyPlanningWithIntegrationAndControls.cpp"
    "/home/brky/workspaces/fjnunes_ws/src/ompl/demos/RigidBodyPlanningWithODESolverAndControls.cpp"
    "/home/brky/workspaces/fjnunes_ws/src/ompl/demos/StateSampling.cpp"
    "/home/brky/workspaces/fjnunes_ws/src/ompl/demos/ThunderLightning.cpp"
    "/home/brky/workspaces/fjnunes_ws/src/ompl/demos/TriangulationDemo.cpp"
    "/home/brky/workspaces/fjnunes_ws/src/ompl/demos/OptimalPlanning.py"
    "/home/brky/workspaces/fjnunes_ws/src/ompl/demos/PlannerData.py"
    "/home/brky/workspaces/fjnunes_ws/src/ompl/demos/Point2DPlanning.py"
    "/home/brky/workspaces/fjnunes_ws/src/ompl/demos/RandomWalkPlanner.py"
    "/home/brky/workspaces/fjnunes_ws/src/ompl/demos/RigidBodyPlanning.py"
    "/home/brky/workspaces/fjnunes_ws/src/ompl/demos/RigidBodyPlanningWithControls.py"
    "/home/brky/workspaces/fjnunes_ws/src/ompl/demos/RigidBodyPlanningWithODESolverAndControls.py"
    "/home/brky/workspaces/fjnunes_ws/src/ompl/demos/StateSampling.py"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE DIRECTORY FILES "/home/brky/workspaces/fjnunes_ws/src/ompl/demos/Koules")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE DIRECTORY FILES "/home/brky/workspaces/fjnunes_ws/src/ompl/demos/VFRRT")
endif()

