// sim2d_app: batch sim with a top-down GUI, ZeroMQ state/command channels,
// and a Record button that writes an MP4 via ffmpeg.
//
// Usage: sim2d_app [scene.json] [--robots N] [--max-robots M]
//   --robots N      start every world with N automatically placed robots
//   --max-robots M  robot slots per world (default: every movable cell)
//   state   PUB tcp://*:5555  topic "/world_<w>/state"  protobuf WorldState
//   command SUB tcp://*:5556  topics "/cmd_path"   protobuf PathCommand
//                                    "/cmd_spawn"  protobuf SpawnCommand
//                                    "/cmd_vel"    protobuf VelocityCommand
//   service REP tcp://*:5557  ControlModeRequest -> ControlModeResponse
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

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
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
        rs.control_mode = s.mode == kModeVelocity
            ? sim2d_msgs_ControlMode_CONTROL_MODE_VELOCITY
            : sim2d_msgs_ControlMode_CONTROL_MODE_WAYPOINT;
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

struct Command {
    enum Kind { Path, Spawn, Velocity } kind = Path;
    uint32_t world = 0;
    // Path
    uint32_t robot = 0;
    std::vector<Vec2> pts;
    // Velocity
    float linear = 0, angular = 0;
    // Spawn
    uint32_t count = 0;
    bool hasPose = false;
    float x = 0, y = 0, theta = 0;
    float radius = 0, vmax = 0, amax = 0, omegamax = 0;  // 0 = default
};

// Reads one command if one is waiting. Returns 0 if none, 1 if a command was
// read, -1 if a message was received but could not be decoded.
int pollCommand(void *sub, Command &cmd)
{
    char topic[64];
    int tn = zmq_recv(sub, topic, sizeof(topic), ZMQ_DONTWAIT);
    if (tn < 0) return 0;
    static char buf[65536];
    int m = zmq_recv(sub, buf, sizeof(buf), 0);
    if (m < 0 || m > (int)sizeof(buf) || tn > (int)sizeof(topic)) return -1;
    pb_istream_t is = pb_istream_from_buffer((const pb_byte_t *)buf, (size_t)m);

    if (tn == 9 && std::memcmp(topic, "/cmd_path", 9) == 0) {
        cmd.kind = Command::Path;
        cmd.pts.clear();
        sim2d_msgs_PathCommand pc = sim2d_msgs_PathCommand_init_zero;
        pc.path.poses.funcs.decode = decodePose;
        pc.path.poses.arg = &cmd.pts;
        if (!pb_decode(&is, sim2d_msgs_PathCommand_fields, &pc)) {
            fprintf(stderr, "ignored bad PathCommand: %s\n", PB_GET_ERROR(&is));
            return -1;
        }
        cmd.world = pc.world_id;
        cmd.robot = pc.robot_id;
        return 1;
    }
    if (tn == 8 && std::memcmp(topic, "/cmd_vel", 8) == 0) {
        cmd.kind = Command::Velocity;
        sim2d_msgs_VelocityCommand vc = sim2d_msgs_VelocityCommand_init_zero;
        if (!pb_decode(&is, sim2d_msgs_VelocityCommand_fields, &vc)) {
            fprintf(stderr, "ignored bad VelocityCommand: %s\n", PB_GET_ERROR(&is));
            return -1;
        }
        cmd.world = vc.world_id;
        cmd.robot = vc.robot_id;
        cmd.linear = vc.linear;
        cmd.angular = vc.angular;
        return 1;
    }
    if (tn == 10 && std::memcmp(topic, "/cmd_spawn", 10) == 0) {
        cmd.kind = Command::Spawn;
        sim2d_msgs_SpawnCommand sc = sim2d_msgs_SpawnCommand_init_zero;
        if (!pb_decode(&is, sim2d_msgs_SpawnCommand_fields, &sc)) {
            fprintf(stderr, "ignored bad SpawnCommand: %s\n", PB_GET_ERROR(&is));
            return -1;
        }
        cmd.world = sc.world_id;
        cmd.count = sc.count;
        cmd.hasPose = sc.has_pose;
        cmd.x = sc.pose.x;
        cmd.y = sc.pose.y;
        cmd.theta = sc.pose.theta;
        cmd.radius = sc.radius;
        cmd.vmax = sc.vmax;
        cmd.amax = sc.amax;
        cmd.omegamax = sc.omegamax;
        return 1;
    }
    return -1;
}

// Answers one ControlModeRequest if one is waiting. Always replies, so the
// REQ/REP pair never gets stuck.
void serveControlMode(void *rep, Manager &mgr, const std::vector<std::string> &names)
{
    char buf[1024];
    int n = zmq_recv(rep, buf, sizeof(buf), ZMQ_DONTWAIT);
    if (n < 0) return;

    sim2d_msgs_ControlModeResponse resp = sim2d_msgs_ControlModeResponse_init_zero;
    resp.mode = sim2d_msgs_ControlMode_CONTROL_MODE_UNSPECIFIED;
    auto fail = [&](const char *why) {
        resp.success = false;
        std::snprintf(resp.message, sizeof(resp.message), "%s", why);
    };

    sim2d_msgs_ControlModeRequest req = sim2d_msgs_ControlModeRequest_init_zero;
    pb_istream_t is = pb_istream_from_buffer((const pb_byte_t *)buf,
                                             n <= (int)sizeof(buf) ? (size_t)n : 0);
    if (n > (int)sizeof(buf) || !pb_decode(&is, sim2d_msgs_ControlModeRequest_fields, &req)) {
        fail("could not read the request");
    } else {
        // World by name, or by id when no name is given.
        int world = -1;
        if (req.world_name[0]) {
            for (size_t i = 0; i < names.size(); i++) {
                if (names[i] == req.world_name) { world = (int)i; break; }
            }
        } else if (req.world_id < mgr.numWorlds()) {
            world = (int)req.world_id;
        }
        if (world < 0) {
            fail("no such world");
        } else {
            resp.world_id = (uint32_t)world;
            std::snprintf(resp.world_name, sizeof(resp.world_name), "%s",
                          names[(size_t)world].c_str());
            resp.robot_id = req.robot_id;
            int32_t cur = mgr.controlMode((uint32_t)world, req.robot_id);
            if (cur < 0) {
                fail("no such robot");
            } else {
                std::string err;
                bool ok = true;
                if (req.mode == sim2d_msgs_ControlMode_CONTROL_MODE_WAYPOINT) {
                    ok = mgr.setControlMode((uint32_t)world, req.robot_id, kModeWaypoint, err);
                } else if (req.mode == sim2d_msgs_ControlMode_CONTROL_MODE_VELOCITY) {
                    ok = mgr.setControlMode((uint32_t)world, req.robot_id, kModeVelocity, err);
                }
                cur = mgr.controlMode((uint32_t)world, req.robot_id);
                if (ok) {
                    resp.success = true;
                    std::snprintf(resp.message, sizeof(resp.message), "%s",
                        req.mode == sim2d_msgs_ControlMode_CONTROL_MODE_UNSPECIFIED
                            ? "current mode" : "mode is now set");
                } else {
                    fail(err.c_str());
                }
            }
            if (cur >= 0) {
                resp.mode = cur == kModeVelocity
                    ? sim2d_msgs_ControlMode_CONTROL_MODE_VELOCITY
                    : sim2d_msgs_ControlMode_CONTROL_MODE_WAYPOINT;
            }
        }
    }

    uint8_t out[512];
    pb_ostream_t os = pb_ostream_from_buffer(out, sizeof(out));
    if (!pb_encode(&os, sim2d_msgs_ControlModeResponse_fields, &resp)) {
        fprintf(stderr, "could not encode ControlModeResponse\n");
        zmq_send(rep, "", 0, 0);  // keep the REQ/REP pair in step
        return;
    }
    zmq_send(rep, out, os.bytes_written, 0);
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
    std::string scene_path = "scenes/demo.json";
    int robots_override = -1;
    int max_robots = 0;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if (a == "--robots" && i + 1 < argc) {
            robots_override = std::atoi(argv[++i]);
        } else if (a == "--max-robots" && i + 1 < argc) {
            max_robots = std::atoi(argv[++i]);
        } else if (!a.empty() && a[0] != '-') {
            scene_path = a;
        } else {
            fprintf(stderr, "usage: %s [scene.json] [--robots N] [--max-robots M]\n",
                    argv[0]);
            return 1;
        }
    }

    float dt = 0.04f;
    std::vector<WorldSpec> worlds;
    std::vector<std::string> names;
    std::string err;
    if (!loadScene(scene_path, dt, worlds, names, err, robots_override)) {
        fprintf(stderr, "scene error: %s\n", err.c_str());
        return 1;
    }
    const uint32_t nW = (uint32_t)worlds.size();

    // Slots: --max-robots, else every movable cell (never above the movable cells).
    Manager mgr({ .dt = dt, .maxRobots = max_robots }, worlds);
    printf("%u world(s), room for %d robots per world (--max-robots to change)\n",
           nW, mgr.maxRobots());
    if (max_robots > mgr.maxRobots()) {
        printf("note: --max-robots %d lowered to %d, the most movable cells in any world\n",
               max_robots, mgr.maxRobots());
    }

    // Host-side copy of each robot's planned path, for drawing.
    std::vector<std::vector<std::vector<Vec2>>> paths(
        nW, std::vector<std::vector<Vec2>>((size_t)mgr.maxRobots()));
    auto robotCount = [&](uint32_t w) { return (int)mgr.robotCount(w); };

    void *zctx = zmq_ctx_new();
    void *pub = zmq_socket(zctx, ZMQ_PUB);
    void *sub = zmq_socket(zctx, ZMQ_SUB);
    void *rep = zmq_socket(zctx, ZMQ_REP);
    if (zmq_bind(pub, "tcp://*:5555") != 0 || zmq_bind(sub, "tcp://*:5556") != 0 ||
        zmq_bind(rep, "tcp://*:5557") != 0) {
        fprintf(stderr, "could not bind ZeroMQ ports 5555/5556/5557\n");
        return 1;
    }
    zmq_setsockopt(sub, ZMQ_SUBSCRIBE, "/cmd_path", 9);
    zmq_setsockopt(sub, ZMQ_SUBSCRIBE, "/cmd_spawn", 10);
    zmq_setsockopt(sub, ZMQ_SUBSCRIBE, "/cmd_vel", 8);

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
    int add_count = 10;
    uint64_t ignoredCmds = 0;
    // Map view per world: zoom (1 = whole map fits) and pan in pixels.
    std::vector<float> zoom(nW, 1.f), panX(nW, 0.f), panY(nW, 0.f);

    // Add robots to a world at run time. Used by the GUI and /cmd_spawn.
    auto spawnRobots = [&](uint32_t w, int count, const Sim::RobotSpawn *one) {
        std::string why;
        int added = 0;
        std::vector<Sim::RobotSpawn> list;
        if (one) {
            list.push_back(*one);
        } else {
            RobotDefaults d;
            list = mgr.autoPlace(w, count, d);
        }
        for (const Sim::RobotSpawn &sp : list) {
            int id = mgr.addRobot(w, sp, why);
            if (id < 0) break;
            paths[w][(size_t)id].clear();
            added++;
        }
        int wanted = one ? 1 : count;
        if (added == wanted) {
            status = "Added " + std::to_string(added) + " robot(s) to " + names[w];
        } else {
            if (why.empty()) {
                if (mgr.robotCount(w) >= mgr.movableCells(w)) {
                    why = "no free cell: layout has " + std::to_string(mgr.movableCells(w)) +
                          " movable cells";
                } else if (mgr.robotCount(w) >= mgr.maxRobots()) {
                    why = "world is full; relaunch with a larger --max-robots";
                } else {
                    why = "no free cell left";
                }
            }
            status = "Added " + std::to_string(added) + " of " +
                     std::to_string(wanted) + ": " + why;
        }
        fprintf(stderr, "%s\n", status.c_str());
    };

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        serveControlMode(rep, mgr, names);

        // Commands from ZeroMQ: paths, velocities and run-time spawns.
        Command cmd;
        for (int got = pollCommand(sub, cmd); got != 0; got = pollCommand(sub, cmd)) {
            if (got < 0 || cmd.world >= nW) continue;
            if (cmd.kind == Command::Path) {
                std::string why;
                if (mgr.setWaypoints(cmd.world, cmd.robot, cmd.pts, why)) {
                    paths[cmd.world][cmd.robot] = cmd.pts;
                } else if (ignoredCmds++ % 100 == 0) {
                    fprintf(stderr, "ignored path for robot %u: %s\n", cmd.robot, why.c_str());
                }
            } else if (cmd.kind == Command::Velocity) {
                std::string why;
                if (!mgr.setVelocity(cmd.world, cmd.robot, cmd.linear, cmd.angular, why) &&
                    ignoredCmds++ % 100 == 0) {
                    fprintf(stderr, "ignored velocity for robot %u: %s\n", cmd.robot, why.c_str());
                }
            } else if (cmd.hasPose) {
                RobotDefaults d;
                Sim::RobotSpawn sp {};
                sp.x = cmd.x;
                sp.y = cmd.y;
                sp.theta = cmd.theta;
                sp.radius = cmd.radius > 0 ? cmd.radius : d.radius;
                sp.vMax = cmd.vmax > 0 ? cmd.vmax : d.vMax;
                sp.aMax = cmd.amax > 0 ? cmd.amax : d.aMax;
                sp.omegaMax = cmd.omegamax > 0 ? cmd.omegamax : d.omegaMax;
                sp.active = 1;
                spawnRobots(cmd.world, 1, &sp);
            } else if (cmd.count > 0) {
                spawnRobots(cmd.world, (int)cmd.count, nullptr);
            }
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
        ImGui::SetNextWindowSize(ImVec2(280, 420), ImGuiCond_Once);
        ImGui::Begin("Controls");
        ImGui::Checkbox("Playing", &playing);
        ImGui::SliderFloat("Speed", &speed, 0.1f, 5.f, "%.1fx");
        int nR = robotCount(sel_world);
        ImGui::TextWrapped("Waypoints come in over ZeroMQ (topic cmd).");
        ImGui::TextWrapped("Blue: moving   Cyan: finished   Orange: blocked by wall   Red: robot contact");
        ImGui::Separator();
        ImGui::Text("Robots: %d / %d", nR, std::min(mgr.maxRobots(), mgr.movableCells(sel_world)));
        ImGui::SetNextItemWidth(100.f);
        ImGui::InputInt("##add", &add_count);
        if (add_count < 1) add_count = 1;
        ImGui::SameLine();
        if (ImGui::Button("Add robots")) {
            spawnRobots((uint32_t)sel_world, add_count, nullptr);
        }
        ImGui::Separator();
        ImGui::Text("View: wheel = zoom, drag = pan");
        float &zm = zoom[sel_world];
        if (ImGui::Button("Zoom +")) zm = std::fmin(zm * 1.25f, 60.f);
        ImGui::SameLine();
        if (ImGui::Button("Zoom -")) zm = std::fmax(zm / 1.25f, 0.2f);
        ImGui::SameLine();
        if (ImGui::Button("Fit")) {
            zm = 1.f;
            panX[sel_world] = panY[sel_world] = 0.f;
        }
        ImGui::Text("Zoom: %.0f%%", zm * 100.f);
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
        ImVec2 p1(p0.x + avail.x, p0.y + avail.y);

        // The canvas catches the mouse: wheel zooms around the cursor, drag pans.
        ImGui::InvisibleButton("canvas", avail,
                               ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonMiddle);
        const float fit = std::fmin(avail.x / map.width, avail.y / map.height) * 0.95f;
        float &Z = zoom[sel_world];
        float &PX = panX[sel_world];
        float &PY = panY[sel_world];
        ImGuiIO &io = ImGui::GetIO();
        if (ImGui::IsItemHovered() && io.MouseWheel != 0.f) {
            float old_scale = fit * Z;
            float wx = (io.MousePos.x - (p0.x + PX)) / old_scale;
            float wy = ((p0.y + avail.y + PY) - io.MousePos.y) / old_scale;
            Z *= std::pow(1.15f, io.MouseWheel);
            Z = std::fmin(std::fmax(Z, 0.2f), 60.f);
            float ns = fit * Z;
            PX = io.MousePos.x - p0.x - wx * ns;
            PY = io.MousePos.y - p0.y - avail.y + wy * ns;
        }
        if (ImGui::IsItemActive() && (ImGui::IsMouseDragging(ImGuiMouseButton_Left, 0.f) ||
                                      ImGui::IsMouseDragging(ImGuiMouseButton_Middle, 0.f))) {
            PX += io.MouseDelta.x;
            PY += io.MouseDelta.y;
        }
        const float scale = fit * Z;
        auto toScreen = [&](float x, float y) {
            return ImVec2(p0.x + PX + x * scale, p0.y + avail.y + PY - y * scale);
        };
        // Map cells inside the canvas.
        auto cellX = [&](float sx) { return (sx - (p0.x + PX)) / scale; };
        auto cellY = [&](float sy) { return ((p0.y + avail.y + PY) - sy) / scale; };
        int32_t cx0 = std::max(0, (int32_t)std::floor(cellX(p0.x)));
        int32_t cx1 = std::min(map.width - 1, (int32_t)std::floor(cellX(p1.x)));
        int32_t cy0 = std::max(0, (int32_t)std::floor(cellY(p1.y)));
        int32_t cy1 = std::min(map.height - 1, (int32_t)std::floor(cellY(p0.y)));

        ImDrawList *dl = ImGui::GetWindowDrawList();
        dl->PushClipRect(p0, p1, true);
        dl->AddRectFilled(p0, p1, IM_COL32(25, 25, 30, 255));
        for (int32_t y = cy0; y <= cy1; y++) {
            for (int32_t x = cx0; x <= cx1; x++) {
                if (!map.cells[y * map.width + x]) continue;
                dl->AddRectFilled(toScreen(x, y + 1), toScreen(x + 1, y),
                                  IM_COL32(90, 90, 100, 255));
            }
        }
        // Grid: one line per cell edge, so every CSV cell is visible.
        const ImU32 gridCol = IM_COL32(60, 60, 70, 255);
        if (scale >= 4.f) {
            for (int32_t x = cx0; x <= cx1 + 1; x++) {
                dl->AddLine(toScreen(x, 0), toScreen(x, map.height), gridCol);
            }
            for (int32_t y = cy0; y <= cy1 + 1; y++) {
                dl->AddLine(toScreen(0, y), toScreen(map.width, y), gridCol);
            }
        }

        const float wp_r = std::fmin(4.f, std::fmax(1.5f, scale * 0.1f));
        for (int r = 0; r < nR; r++) {
            RobotState s = mgr.robotState(sel_world, r);
            ImVec2 c = toScreen(s.x, s.y);
            float rad = std::fmax(mgr.robotRadius(sel_world, r) * scale, 1.5f);
            bool visible = c.x + rad >= p0.x && c.x - rad <= p1.x &&
                           c.y + rad >= p0.y && c.y - rad <= p1.y;

            const auto &pts = paths[sel_world][r];
            for (size_t i = 0; i < pts.size(); i++) {
                ImVec2 p = toScreen(pts[i].x, pts[i].y);
                dl->AddCircleFilled(p, wp_r, IM_COL32(120, 200, 120, 255));
                if (i > 0) {
                    ImVec2 q = toScreen(pts[i - 1].x, pts[i - 1].y);
                    dl->AddLine(q, p, IM_COL32(120, 200, 120, 120), 1.f);
                }
            }
            if (!visible) continue;

            ImU32 col = IM_COL32(80, 160, 240, 255);                        // moving / idle
            if (s.count > 0 && s.next >= s.count) {
                col = IM_COL32(80, 200, 200, 255);                          // finished
            }
            if (s.stopped) col = IM_COL32(240, 160, 40, 255);               // wall stop
            if (s.hitRobot) col = IM_COL32(230, 60, 60, 255);               // robot contact
            dl->AddCircleFilled(c, rad, col);
            ImVec2 tip = toScreen(s.x + std::cos(s.theta) * 0.3f,
                                  s.y + std::sin(s.theta) * 0.3f);
            dl->AddLine(c, tip, IM_COL32(255, 255, 255, 255), 2.f);
            if (rad >= 5.f) {  // labels only when they fit
                char label[8];
                std::snprintf(label, sizeof(label), "%d", r);
                ImVec2 ts = ImGui::CalcTextSize(label);
                dl->AddText(ImVec2(c.x - ts.x * 0.5f, c.y - rad - ts.y - 2.f),
                            IM_COL32(255, 255, 255, 255), label);
            }
        }
        dl->PopClipRect();

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
    zmq_close(rep);
    zmq_ctx_term(zctx);
    return 0;
}
