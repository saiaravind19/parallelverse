find_package(nlohmann_json 3.2 REQUIRED)

add_library(sim2d_core STATIC
    src/sim.cpp src/sim.hpp
    src/mgr.cpp src/mgr.hpp
    src/config.cpp src/config.hpp
    src/map.cpp src/map.hpp
    src/profile.hpp
    src/types.hpp
)

target_include_directories(sim2d_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_link_libraries(sim2d_core
    PUBLIC
        madrona_mw_core
        madrona_mw_cpu
        nlohmann_json::nlohmann_json
)

add_executable(sim2d_collision_test src/collision_test.cpp)
target_link_libraries(sim2d_collision_test PRIVATE sim2d_core)

# ---- GUI app (top-down view, ZeroMQ, Record) ----
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../external/madrona/external/imgui)
add_library(sim2d_imgui STATIC
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(sim2d_imgui PUBLIC ${IMGUI_DIR} ${IMGUI_DIR}/backends)
target_link_libraries(sim2d_imgui PUBLIC glfw OpenGL::GL)

add_executable(sim2d_app src/app.cpp)
target_link_libraries(sim2d_app PRIVATE sim2d_core sim2d_imgui zmq)
