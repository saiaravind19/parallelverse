// sim2d_app: batch sim with a top-down GUI, ZeroMQ state/command channels,
// and a Record button that writes an MP4 via ffmpeg.
//
// Usage: sim2d_app [scene.json]
//   state   PUB tcp://*:5555  topic "state/<world>"  JSON
//   command SUB tcp://*:5556  topic "cmd"            JSON
//           {"world":0,"robot":1,"waypoints":[[x,y],...]}

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <zmq.h>
#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>
#include <vector>
#include <sys/stat.h>

#include "config.hpp"
#include "mgr.hpp"

using json = nlohmann::json;
using namespace sim2d;

namespace {

constexpr int kWinW = 1280;
constexpr int kWinH = 800;
constexpr int kRecordFps = 25;

void publishState(void *pub, uint32_t world, const json &msg)
{
    std::string topic = "state/" + std::to_string(world);
    std::string body = msg.dump();
    zmq_send(pub, topic.data(), topic.size(), ZMQ_SNDMORE);
    zmq_send(pub, body.data(), body.size(), 0);
}

// Returns a discarded json when no command is waiting.
json pollCommand(void *sub)
{
    char buf[65536];
    int n = zmq_recv(sub, buf, sizeof(buf), ZMQ_DONTWAIT);
    if (n < 0) return json(json::value_t::discarded);
    if (n != 3 || std::memcmp(buf, "cmd", 3) != 0) {
        return json(json::value_t::discarded);
    }
    int m = zmq_recv(sub, buf, sizeof(buf), 0);
    if (m < 0) return json(json::value_t::discarded);
    return json::parse(std::string(buf, m), nullptr, false);
}

struct Recording {
    FILE *pipe = nullptr;
    std::string path;

    bool start(int w, int h, uint32_t world)
    {
        mkdir("recordings", 0755);
        char stamp[32];
        std::time_t t = std::time(nullptr);
        std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&t));
        path = "recordings/world" + std::to_string(world) + "_" + stamp + ".mp4";
        std::string cmd =
            "ffmpeg -loglevel error -y -f rawvideo -pix_fmt rgba -s " +
            std::to_string(w) + "x" + std::to_string(h) +
            " -r " + std::to_string(kRecordFps) +
            " -i - -vf vflip -c:v libx264 -pix_fmt yuv420p -preset veryfast " +
            path;
        pipe = popen(cmd.c_str(), "w");
        return pipe != nullptr;
    }

    void writeFrame(int w, int h, std::vector<unsigned char> &buf)
    {
        buf.resize((size_t)w * h * 4);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, buf.data());
        fwrite(buf.data(), 1, buf.size(), pipe);
    }

    void stop()
    {
        if (pipe) pclose(pipe);
        pipe = nullptr;
    }
};

} // namespace

int main(int argc, char **argv)
{
    std::string scene_path = argc > 1 ? argv[1] : "scenes/demo.json";

    float dt = 0.04f;
    std::vector<Sim::WorldInit> worlds;
    std::vector<std::string> names;
    std::string err;
    if (!loadScene(scene_path, dt, worlds, names, err)) {
        fprintf(stderr, "scene error: %s\n", err.c_str());
        return 1;
    }
    const uint32_t nW = (uint32_t)worlds.size();

    Manager mgr({ .dt = dt }, worlds);

    // Host-side copy of each robot's planned path, for drawing.
    std::vector<std::vector<std::vector<Vec2>>> paths(
        nW, std::vector<std::vector<Vec2>>(limits::maxRobotsPerWorld));
    auto robotCount = [&](uint32_t w) {
        int n = 0;
        for (CountT r = 0; r < limits::maxRobotsPerWorld; r++) {
            n += worlds[w].spawns[r].active;
        }
        return n;
    };

    void *zctx = zmq_ctx_new();
    void *pub = zmq_socket(zctx, ZMQ_PUB);
    void *sub = zmq_socket(zctx, ZMQ_SUB);
    if (zmq_bind(pub, "tcp://*:5555") != 0 || zmq_bind(sub, "tcp://*:5556") != 0) {
        fprintf(stderr, "could not bind ZeroMQ ports 5555/5556\n");
        return 1;
    }
    zmq_setsockopt(sub, ZMQ_SUBSCRIBE, "cmd", 3);

    if (!glfwInit()) {
        fprintf(stderr, "glfwInit failed\n");
        return 1;
    }
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    GLFWwindow *window = glfwCreateWindow(kWinW, kWinH, "sim2d", nullptr, nullptr);
    if (!window) {
        fprintf(stderr, "could not create window (is DISPLAY set?)\n");
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    bool playing = true;
    float speed = 1.f;
    float accum = 0.f;
    int sel_world = 0;
    int sel_robot = 0;
    Recording rec;
    std::string status = "Ready";
    std::vector<unsigned char> frame;
    double last = glfwGetTime();

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // Waypoint commands from ZeroMQ.
        for (json cmd = pollCommand(sub); !cmd.is_discarded();
             cmd = pollCommand(sub)) {
            uint32_t w = cmd.value("world", 0u);
            uint32_t r = cmd.value("robot", 0u);
            if (w >= nW || r >= limits::maxRobotsPerWorld) continue;
            std::vector<Vec2> pts;
            for (const auto &p : cmd.value("waypoints", json::array())) {
                pts.push_back(Vec2 { p[0].get<float>(), p[1].get<float>() });
            }
            paths[w][r] = pts;
            mgr.setWaypoints(w, r, pts);
        }

        // Step at real time * speed, and publish state after each step.
        double now = glfwGetTime();
        accum += (float)(now - last) * speed;
        last = now;
        if (playing) {
            while (accum >= dt) {
                mgr.step();
                accum -= dt;
                for (uint32_t w = 0; w < nW; w++) {
                    json robots = json::array();
                    for (uint32_t r = 0; r < (uint32_t)robotCount(w); r++) {
                        RobotState s = mgr.robotState(w, r);
                        robots.push_back({ {"id", r}, {"x", s.x}, {"y", s.y},
                            {"theta", s.theta}, {"v", s.v},
                            {"stopped", s.stopped}, {"collided", s.collided} });
                    }
                    publishState(pub, w, { {"world", w}, {"robots", robots} });
                }
            }
        } else {
            accum = 0.f;
        }

        // ---- GUI ----
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Once);
        ImGui::SetNextWindowSize(ImVec2(280, 260), ImGuiCond_Once);
        ImGui::Begin("Controls");
        ImGui::Checkbox("Playing", &playing);
        ImGui::SliderFloat("Speed", &speed, 0.1f, 5.f, "%.1fx");
        int nR = robotCount(sel_world);
        if (sel_robot >= nR) sel_robot = 0;
        ImGui::SliderInt("Robot", &sel_robot, 0, nR - 1);
        ImGui::TextWrapped("Click the map to add a waypoint for the selected robot.");
        if (ImGui::Button("Clear waypoints")) {
            paths[sel_world][sel_robot].clear();
            mgr.setWaypoints(sel_world, sel_robot, {});
        }
        ImGui::Separator();
        if (!rec.pipe) {
            if (ImGui::Button("Record")) {
                status = rec.start(kWinW, kWinH, sel_world)
                    ? "Recording " + rec.path : "Could not start ffmpeg";
            }
        } else if (ImGui::Button("Stop recording")) {
            rec.stop();
            status = "Saved " + rec.path;
        }
        ImGui::TextWrapped("%s", status.c_str());
        ImGui::End();

        ImGui::SetNextWindowPos(ImVec2(300, 10), ImGuiCond_Once);
        ImGui::SetNextWindowSize(ImVec2(kWinW - 310, kWinH - 20), ImGuiCond_Once);
        ImGui::Begin("Map");
        if (ImGui::BeginTabBar("worlds")) {
            for (uint32_t w = 0; w < nW; w++) {
                if (ImGui::BeginTabItem(names[w].c_str())) {
                    if ((int)w != sel_world) {
                        sel_world = (int)w;
                        sel_robot = 0;
                    }
                    ImGui::EndTabItem();
                }
            }
            ImGui::EndTabBar();
        }
        const MapGrid &map = worlds[sel_world].map;
        ImVec2 p0 = ImGui::GetCursorScreenPos();
        ImVec2 avail = ImGui::GetContentRegionAvail();
        float scale = std::fmin(avail.x / map.width, avail.y / map.height) * 0.95f;
        auto toScreen = [&](float x, float y) {
            return ImVec2(p0.x + x * scale, p0.y + avail.y - y * scale);
        };

        ImDrawList *dl = ImGui::GetWindowDrawList();
        dl->AddRectFilled(p0, ImVec2(p0.x + avail.x, p0.y + avail.y),
                          IM_COL32(25, 25, 30, 255));
        for (int32_t y = 0; y < map.height; y++) {
            for (int32_t x = 0; x < map.width; x++) {
                if (!map.cells[y * map.width + x]) continue;
                dl->AddRectFilled(toScreen(x, y + 1), toScreen(x + 1, y),
                                  IM_COL32(90, 90, 100, 255));
            }
        }

        for (int r = 0; r < nR; r++) {
            const auto &pts = paths[sel_world][r];
            for (size_t i = 0; i < pts.size(); i++) {
                ImVec2 p = toScreen(pts[i].x, pts[i].y);
                dl->AddCircleFilled(p, 4.f, IM_COL32(120, 200, 120, 255));
                if (i > 0) {
                    ImVec2 q = toScreen(pts[i - 1].x, pts[i - 1].y);
                    dl->AddLine(q, p, IM_COL32(120, 200, 120, 120), 1.f);
                }
            }
            RobotState s = mgr.robotState(sel_world, r);
            ImU32 col = (s.collided || s.stopped)
                ? IM_COL32(230, 90, 80, 255) : IM_COL32(80, 160, 240, 255);
            float rad = worlds[sel_world].spawns[r].radius * scale;
            ImVec2 c = toScreen(s.x, s.y);
            dl->AddCircleFilled(c, rad, col);
            ImVec2 tip = toScreen(s.x + std::cos(s.theta) * 0.3f,
                                  s.y + std::sin(s.theta) * 0.3f);
            dl->AddLine(c, tip, IM_COL32(255, 255, 255, 255), 2.f);
        }

        ImGui::InvisibleButton("canvas", avail);
        if (ImGui::IsItemClicked(ImGuiMouseButton_Left)) {
            ImVec2 m = ImGui::GetIO().MousePos;
            Vec2 wp { (m.x - p0.x) / scale, (p0.y + avail.y - m.y) / scale };
            auto &list = paths[sel_world][sel_robot];
            if (list.size() < (size_t)limits::maxWaypoints) {
                list.push_back(wp);
                mgr.setWaypoints(sel_world, sel_robot, list);
            }
        }
        ImGui::End();

        ImGui::Render();
        int fbw, fbh;
        glfwGetFramebufferSize(window, &fbw, &fbh);
        glViewport(0, 0, fbw, fbh);
        glClearColor(0.1f, 0.1f, 0.12f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        if (rec.pipe) rec.writeFrame(fbw, fbh, frame);
        glfwSwapBuffers(window);
    }

    rec.stop();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    zmq_close(pub);
    zmq_close(sub);
    zmq_ctx_term(zctx);
    return 0;
}
