add_library(footbot_warehouse SHARED
  footbot_warehouse.h
  footbot_warehouse.cpp
  footbot_navigation.cpp
  footbot_rescue.cpp
  footbot_comms.cpp
  footbot_task_allocation.cpp
  footbot_docking.cpp)
target_link_libraries(footbot_warehouse
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
