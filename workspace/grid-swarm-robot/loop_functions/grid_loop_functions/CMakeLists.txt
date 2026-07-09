# Controller goi truc tiep API bang den cua loop functions (phu thuoc
# 2 chieu) nen ca hai bien dich vao MOT thu vien .so duy nhat — bo nap
# dong cua ARGoS khong phan giai duoc typeinfo cheo giua 2 thu vien
# nap tuan tu.
set(grid_loop_functions_SOURCES
  ${CMAKE_SOURCE_DIR}/controllers/footbot_grid/footbot_grid.cpp
  ${CMAKE_SOURCE_DIR}/controllers/footbot_grid/footbot_grid_fsm.cpp
  ${CMAKE_SOURCE_DIR}/controllers/footbot_grid/footbot_grid_localization.cpp
  ${CMAKE_SOURCE_DIR}/controllers/footbot_grid/footbot_grid_pathfinding.cpp
  ${CMAKE_SOURCE_DIR}/controllers/footbot_grid/footbot_grid_steering.cpp
  ${CMAKE_SOURCE_DIR}/controllers/footbot_grid/footbot_grid_traffic.cpp
  ${CMAKE_SOURCE_DIR}/controllers/footbot_grid/footbot_grid_detour.cpp
  ${CMAKE_SOURCE_DIR}/controllers/footbot_grid/footbot_grid_comms.cpp
  grid_loop_functions.cpp
  grid_reservations.cpp
  grid_task_board.cpp
  grid_docks.cpp
  grid_energy_safety.cpp
  grid_metrics.cpp
  grid_floor_render.cpp)

if(ARGOS_QTOPENGL_FOUND)
  include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
  set(grid_loop_functions_SOURCES
    ${grid_loop_functions_SOURCES}
    grid_qt_user_functions.cpp)
endif(ARGOS_QTOPENGL_FOUND)

add_library(grid_loop_functions MODULE ${grid_loop_functions_SOURCES})
target_link_libraries(grid_loop_functions
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media)

if(ARGOS_QTOPENGL_FOUND)
  target_link_libraries(grid_loop_functions argos3plugin_simulator_qtopengl)
endif(ARGOS_QTOPENGL_FOUND)
