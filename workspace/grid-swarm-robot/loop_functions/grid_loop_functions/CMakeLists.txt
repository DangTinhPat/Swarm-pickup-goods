# Controller goi truc tiep API "bang den" cua loop functions (dat cho o,
# nhan viec, hoi dock trong...) nen hai phia phu thuoc lan nhau. De bo
# nap dong cua ARGoS phan giai du symbol, ca hai duoc bien dich vao MOT
# thu vien .so duy nhat (file nguon van tach thu muc ro rang).
set(grid_loop_functions_SOURCES
  ${CMAKE_SOURCE_DIR}/controllers/footbot_grid/footbot_grid.cpp
  ${CMAKE_SOURCE_DIR}/controllers/footbot_grid/footbot_grid_nav.cpp
  ${CMAKE_SOURCE_DIR}/controllers/footbot_grid/footbot_grid_traffic.cpp
  grid_loop_functions.cpp
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
