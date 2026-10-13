// sim2d_app: batch sim with a top-down GUI, ZeroMQ state/command channels,
// and a Record button that writes an MP4 via ffmpeg.
//
// Usage: sim2d_app [scene.json]
//   state   PUB tcp://*:5555  topic "/world_<w>/state"  protobuf WorldState
//   command SUB tcp://*:5556  topic "/cmd_path"          protobuf PathCommand
//           (proto/sim2d_msgs.proto)

#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <zmq.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include "sim2d_msgs.pb.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>
#include <vector>
#include <sys/stat.h>

#include "config.hpp"
#include "mgr.hpp"

using namespace sim2d;

namespace {

constexpr int kWinW = 1280;
constexpr int kWinH = 800;
constexpr int kRecordFps = 25;

// Topics use ROS-style names: /world_<w>/state and /cmd_path.
// Bodies are protobuf (proto/sim2d_msgs.proto), encoded with nanopb.

struct StateCtx {
    const Manager *mgr;
    uint32_t world;
    uint32_t robots;
};

bool encodeRobots(pb_ostream_t *stream, const pb_field_t *field, void *const *arg)
{
    const StateCtx *ctx = (const StateCtx *)*arg;
    for (uint32_t r = 0; r < ctx->robots; r++) {
        RobotState s = ctx->mgr->robotState(ctx->world, r);
        sim2d_msgs_RobotState rs = sim2d_msgs_RobotState_init_zero;
        rs.robot_id = r;
        rs.has_pose = true;
        rs.pose.x = s.x;
        rs.pose.y = s.y;
        rs.pose.theta = s.theta;
        rs.v = s.v;
        rs.stopped = s.stopped != 0;
        rs.hit_robot = s.hitRobot != 0;
        rs.finished = s.count > 0 && s.next >= s.count;
        rs.next_index = (uint32_t)s.next;
        rs.waypoint_count = (uint32_t)s.count;
        if (!pb_encode_tag_for_field(stream, field)) return false;
        if (!pb_encode_submessage(stream, sim2d_msgs_RobotState_fields, &rs)) return false;
    }
    return true;
}

void publishWorldState(void *pub, const Manager &mgr, uint32_t world,
                       uint32_t robots, uint64_t step, float dt,
                       std::vector<uint8_t> &buf)
{
    sim2d_msgs_WorldState ws = sim2d_msgs_WorldState_init_zero;
    ws.has_header = true;
    ws.header.seq = step;
    ws.header.stamp_ms = (uint64_t)(step * dt * 1000.0);
    std::snprintf(ws.header.frame_id, sizeof(ws.header.frame_id), "map");
    ws.world_id = world;
    StateCtx ctx { &mgr, world, robots };
    ws.robots.funcs.encode = encodeRobots;
    ws.robots.arg = &ctx;

    buf.resize(128 + (size_t)robots * 64);  // each robot encodes in under 64 bytes
    pb_ostream_t os = pb_ostream_from_buffer(buf.data(), buf.size());
    if (!pb_encode(&os, sim2d_msgs_WorldState_fields, &ws)) {
        fprintf(stderr, "could not encode WorldState: %s\n", PB_GET_ERROR(&os));
        return;
    }
    std::string topic = "/world_" + std::to_string(world) + "/state";
    zmq_send(pub, topic.data(), topic.size(), ZMQ_SNDMORE);
    zmq_send(pub, buf.data(), os.bytes_written, 0);
}

bool decodePose(pb_istream_t *stream, const pb_field_t *, void **arg)
{
    std::vector<Vec2> *pts = (std::vector<Vec2> *)*arg;
    sim2d_msgs_Pose2D p = sim2d_msgs_Pose2D_init_zero;
    if (!pb_decode(stream, sim2d_msgs_Pose2D_fields, &p)) return false;
    pts->push_back(Vec2 { p.x, p.y });
    return true;
}

// Reads one PathCommand if one is waiting. Returns 0 if none, 1 if a command
// was read, -1 if a message was received but could not be decoded. The socket
// only receives "/cmd_path" messages, so the topic frame is skipped.
int pollPathCommand(void *sub, uint32_t &world, uint32_t &robot,
                    std::vector<Vec2> &pts)
{
    char buf[65536];
    int n = zmq_recv(sub, buf, sizeof(buf), ZMQ_DONTWAIT);
    if (n < 0) return 0;
    int m = zmq_recv(sub, buf, sizeof(buf), 0);
    if (m < 0 || m > (int)sizeof(buf)) return -1;

    pts.clear();
    sim2d_msgs_PathCommand cmd = sim2d_msgs_PathCommand_init_zero;
    cmd.path.poses.funcs.decode = decodePose;
    cmd.path.poses.arg = &pts;
    pb_istream_t is = pb_istream_from_buffer((const pb_byte_t *)buf, (size_t)m);
    if (!pb_decode(&is, sim2d_msgs_PathCommand_fields, &cmd)) {
        fprintf(stderr, "ignored bad PathCommand: %s\n", PB_GET_ERROR(&is));
        return -1;
    }
    world = cmd.world_id;
    robot = cmd.robot_id;
    return 1;
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
    zmq_setsockopt(sub, ZMQ_SUBSCRIBE, "/cmd_path", 9);

    if (!glfwInit()) {
        fprintf(stderr, "glfwInit failed\n");
        return 1;
    }
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
#ifdef __APPLE__
    // macOS only offers OpenGL 3.2+ core profiles.
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#endif
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
#ifdef __APPLE__
    ImGui_ImplOpenGL3_Init("#version 150");
#else
    ImGui_ImplOpenGL3_Init("#version 130");
#endif

    bool playing = true;
    float speed = 1.f;
    float accum = 0.f;
    int sel_world = 0;
    Recording rec;
    std::string status = "Ready";
    std::vector<unsigned char> frame;
    double last = glfwGetTime();
    uint64_t stepCount = 0;
    std::vector<uint8_t> stateBuf;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // Path commands from ZeroMQ.
        std::vector<Vec2> cmdPts;
        uint32_t cmdWorld = 0, cmdRobot = 0;
        for (int got = pollPathCommand(sub, cmdWorld, cmdRobot, cmdPts); got != 0;
             got = pollPathCommand(sub, cmdWorld, cmdRobot, cmdPts)) {
            if (got < 0) continue;
            if (cmdWorld >= nW || cmdRobot >= limits::maxRobotsPerWorld) continue;
            if (cmdPts.size() > (size_t)limits::maxWaypoints) {
                fprintf(stderr, "ignored command: %zu waypoints, limit is %d\n",
                        cmdPts.size(), (int)limits::maxWaypoints);
                continue;
            }
            paths[cmdWorld][cmdRobot] = cmdPts;
            mgr.setWaypoints(cmdWorld, cmdRobot, cmdPts);
        }

        // Step at real time * speed, and publish state after each step.
        double now = glfwGetTime();
        accum += (float)(now - last) * speed;
        last = now;
        if (playing) {
            while (accum >= dt) {
                mgr.step();
                accum -= dt;
                stepCount++;
                for (uint32_t w = 0; w < nW; w++) {
                    publishWorldState(pub, mgr, w, (uint32_t)robotCount(w),
                                      stepCount, dt, stateBuf);
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
        ImGui::TextWrapped("Waypoints come in over ZeroMQ (topic cmd).");
        ImGui::TextWrapped("Blue: moving   Cyan: finished   Orange: blocked by wall   Red: robot contact");
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
        ImGui::Begin("Map", nullptr, ImGuiWindowFlags_NoCollapse);
        if (ImGui::BeginTabBar("worlds")) {
            for (uint32_t w = 0; w < nW; w++) {
                if (ImGui::BeginTabItem(names[w].c_str())) {
                    sel_world = (int)w;
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
        // Grid: one line per cell edge, so every CSV cell is visible.
        const ImU32 gridCol = IM_COL32(60, 60, 70, 255);
        for (int32_t x = 0; x <= map.width; x++) {
            dl->AddLine(toScreen(x, 0), toScreen(x, map.height), gridCol);
        }
        for (int32_t y = 0; y <= map.height; y++) {
            dl->AddLine(toScreen(0, y), toScreen(map.width, y), gridCol);
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
            ImU32 col = IM_COL32(80, 160, 240, 255);                        // moving / idle
            if (s.count > 0 && s.next >= s.count) {
                col = IM_COL32(80, 200, 200, 255);                          // finished
            }
            if (s.stopped) col = IM_COL32(240, 160, 40, 255);               // wall stop
            if (s.hitRobot) col = IM_COL32(230, 60, 60, 255);               // robot contact
            float rad = worlds[sel_world].spawns[r].radius * scale;
            ImVec2 c = toScreen(s.x, s.y);
            dl->AddCircleFilled(c, rad, col);
            ImVec2 tip = toScreen(s.x + std::cos(s.theta) * 0.3f,
                                  s.y + std::sin(s.theta) * 0.3f);
            dl->AddLine(c, tip, IM_COL32(255, 255, 255, 255), 2.f);
            char label[8];
            std::snprintf(label, sizeof(label), "%d", r);
            ImVec2 ts = ImGui::CalcTextSize(label);
            dl->AddText(ImVec2(c.x - ts.x * 0.5f, c.y - rad - ts.y - 2.f),
                        IM_COL32(255, 255, 255, 255), label);
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
