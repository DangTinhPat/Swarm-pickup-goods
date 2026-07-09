link_directories(${CMAKE_BINARY_DIR}/controllers/footbot_warehouse)
set(warehouse_loop_functions_SOURCES
  warehouse_loop_functions.cpp
  warehouse_floor_render.cpp
  warehouse_spawning.cpp
  warehouse_robot_update.cpp
  warehouse_metrics.cpp)

if(ARGOS_QTOPENGL_FOUND)
  include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
  set(warehouse_loop_functions_SOURCES
    ${warehouse_loop_functions_SOURCES}
    warehouse_qt_user_functions.cpp)
endif(ARGOS_QTOPENGL_FOUND)

add_library(warehouse_loop_functions MODULE ${warehouse_loop_functions_SOURCES})
target_link_libraries(warehouse_loop_functions
  footbot_warehouse
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media)

if(ARGOS_QTOPENGL_FOUND)
  target_link_libraries(warehouse_loop_functions argos3plugin_simulator_qtopengl)
endif(ARGOS_QTOPENGL_FOUND)
