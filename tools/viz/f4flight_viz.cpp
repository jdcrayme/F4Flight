// f4flight - tools/f4flight_viz.cpp
//
// Interactive 3D maneuver viewer using RayLib.
//
// Usage:
//   f4flight_viz                          # launches with empty scene
//   f4flight_viz trace.json               # loads a trace file
//
// The viewer can also run scenarios in-process and visualize them live:
//   - Pick an aircraft from the dropdown (scans tests/fixtures/*.json)
//   - Pick a scenario from the dropdown
//   - Click "Run" to generate a trace in-memory and display it
//
// Controls:
//   Tab         Switch camera mode (top-down, side, chase, free orbit)
//   Space       Play/pause
//   ←/→         Step frame backward/forward
//   ↑/↓         Speed up / slow down playback
//   R           Reset to start
//   Mouse drag  Orbit camera (free orbit mode)
//   Mouse wheel Zoom
//
// Build: requires F4FLIGHT_BUILD_VIZ=ON and RayLib (fetched via CMake).
//   cmake -DF4FLIGHT_BUILD_VIZ=ON ..
//   make f4flight_viz

#include "f4flight/flight/f4flight.h"
#include "scenario_framework.h"
#include "trace.h"

#include <raylib.h>
#include <rlgl.h>
#include <raymath.h>

#include <cstdio>
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>
#include <filesystem>
#include <unistd.h>  // getcwd

// Note: do NOT use `using namespace f4flight` — it conflicts with RayLib's
// Quaternion typedef. Use explicit f4flight:: qualifiers instead.
using namespace manuver_test;

// ---------------------------------------------------------------------------
// Globals (the viewer is single-window, so globals are fine)
// ---------------------------------------------------------------------------

static f4flight::Trace g_trace;
static int g_currentFrame = 0;
static bool g_playing = false;
static double g_playbackSpeed = 1.0;
static double g_frameAccum = 0.0;

enum class ViewMode { TopDown, Side, Chase, FreeOrbit };
static ViewMode g_camMode = ViewMode::TopDown;
static const char* g_camModeNames[] = {"Top-Down", "Side", "Chase", "Free Orbit"};

static Camera3D g_camera = {};

// UI state
static int g_selectedAircraft = 0;
static int g_selectedScenario = 0;
static std::vector<std::string> g_aircraftList;
static std::vector<std::string> g_scenarioList;
static std::string g_statusMsg = "Ready. Select aircraft + scenario, then click Run.";
static bool g_showHelp = false;

// ---------------------------------------------------------------------------
// Color helpers
// ---------------------------------------------------------------------------

static Color modeColor(const std::string& mode) {
    if (mode == "Takeoff")       return SKYBLUE;
    if (mode == "Landing")       return GREEN;
    if (mode == "MissileDefeat") return RED;
    if (mode == "GunsJink")      return ORANGE;
    if (mode == "WVREngage")     return PURPLE;
    if (mode == "Waypoint")      return GRAY;
    if (mode == "GroundAvoid")   return PINK;
    return DARKGRAY;
}

// Convert world coordinates (NED, feet) to RayLib world coordinates.
// NED: x=North, y=East, z=Down. RayLib: x=East, y=Up, z=North.
// Scale: 1 unit = 100 ft (so 10000 ft = 100 units).
static constexpr float WORLD_SCALE = 0.01f;  // feet to "units"

static Vector3 worldToRl(double x, double y, double z) {
    return {
        (float)(y * WORLD_SCALE),    // East → x
        (float)(-z * WORLD_SCALE),   // Up = -Down → y
        (float)(x * WORLD_SCALE)     // North → z
    };
}

// ---------------------------------------------------------------------------
// Load trace from a file
// ---------------------------------------------------------------------------

static bool loadTraceFile(const std::string& path) {
    std::string err;
    f4flight::Trace t;
    if (!readTrace(path, t, err)) {
        g_statusMsg = "Error loading: " + err;
        return false;
    }
    g_trace = std::move(t);
    g_currentFrame = 0;
    g_frameAccum = 0.0;
    g_statusMsg = "Loaded " + std::to_string(g_trace.frames.size()) + " frames from " + path;
    return true;
}

// ---------------------------------------------------------------------------
// Run a scenario in-process and load the trace
// ---------------------------------------------------------------------------

static void runScenario() {
    if (g_aircraftList.empty() || g_scenarioList.empty()) return;
    if (g_selectedAircraft < 0 || g_selectedAircraft >= (int)g_aircraftList.size()) return;
    if (g_selectedScenario < 0 || g_selectedScenario >= (int)g_scenarioList.size()) return;

    std::string acPath = g_aircraftList[g_selectedAircraft];
    std::string scName = g_scenarioList[g_selectedScenario];

    // Load aircraft
    f4flight::AircraftConfig cfg;
    auto result = f4flight::json::readFile(acPath, cfg);
    if (!result.ok) {
        g_statusMsg = "Failed to load aircraft config";
        return;
    }

    // Find the scenario
    auto& registry = manuver_test::ScenarioRegistry::instance();
    auto scenario = registry.create(scName);
    if (!scenario) {
        g_statusMsg = "Unknown scenario: " + scName;
        return;
    }

    // Set up flight model + steering controller
    f4flight::FlightModel fm;
    const double initCs = cfg.geometry.cornerVcas_kts > 0 ? cfg.geometry.cornerVcas_kts : 330.0;
    fm.init(cfg, 10000, initCs * f4flight::KNOTS_TO_FTPSEC, 0.0, true);

    f4flight::SteeringController sc;
    sc.setCornerSpeed(initCs);
    sc.setMaxGs(cfg.geometry.maxGs);
    sc.setMaxBank(45.0);
    sc.setAltitude(10000.0);
    sc.setHeading(0.0);
    sc.setMaxGamma(15.0);

    manuver_test::ScenarioContext sctx{cfg};

    // Run the scenario with a trace recorder
    f4flight::TraceRecorder tracer;
    std::string acName = std::filesystem::path(acPath).stem().string();
    tracer.start(acName, scName);

    const double dt = 1.0 / 60.0;
    auto tests = scenario->StartScenario(fm, sctx);

    for (auto& test : tests) {
        sc.softReset();
        test->Init(sc, fm);
        const double phaseStart = tracer.trace().frames.empty() ? 0.0
            : tracer.trace().frames.back().t;

        while (!test->IsFinished()) {
            f4flight::PilotInput input;
            if (test->inputOverride(input, fm.state())) {
                // direct input
            } else {
                input = sc.compute(fm.state(), dt, 0.0, fm.fcs(), fm.state().fcs);
                const double bankCmd = test->bankOverride_rad();
                if (bankCmd >= 0.0) {
                    input.rstick = f4flight::limit((bankCmd - fm.state().kin.phi) * 2.0, -1.0, 1.0);
                }
            }
            fm.update(dt, input, 0.0, f4flight::Vec3{0.0, 0.0, 1.0});
            test->Evaluate(fm.state(), input, dt);

            // Record frame
            const double t = tracer.trace().frames.empty() ? dt
                : tracer.trace().frames.back().t + dt;
            std::string modeName = f4flight::digi::digiModeName(sc.brain().activeMode());
            std::string phaseName;
            const auto& go = sc.brain().state().ag.groundOps;
            switch (go.phase) {
                case f4flight::digi::GroundOpsPhase::TakeoffRoll:    phaseName = "TakeoffRoll"; break;
                case f4flight::digi::GroundOpsPhase::Rotation:       phaseName = "Rotation"; break;
                case f4flight::digi::GroundOpsPhase::AfterTakeoff:   phaseName = "AfterTakeoff"; break;
                case f4flight::digi::GroundOpsPhase::Approach:       phaseName = "Approach"; break;
                case f4flight::digi::GroundOpsPhase::Flare:          phaseName = "Flare"; break;
                case f4flight::digi::GroundOpsPhase::Touchdown:      phaseName = "Touchdown"; break;
                case f4flight::digi::GroundOpsPhase::Rollout:        phaseName = "Rollout"; break;
                case f4flight::digi::GroundOpsPhase::VacatingRunway: phaseName = "VacatingRunway"; break;
                default: break;
            }
            tracer.record(t, fm.state(), input, modeName, phaseName);
        }

        const double phaseEnd = tracer.trace().frames.empty() ? 0.0
            : tracer.trace().frames.back().t;
        tracer.markPhase(test->name(), phaseStart, phaseEnd, test->IsPassed(), test->ShouldSkip());
    }

    tracer.finish(tracer.trace().frames.empty() ? 0.0 : tracer.trace().frames.back().t);
    g_trace = tracer.trace();
    g_currentFrame = 0;
    g_frameAccum = 0.0;
    g_statusMsg = "Ran " + acName + " / " + scName + " (" +
                  std::to_string(g_trace.frames.size()) + " frames, " +
                  std::to_string((int)g_trace.duration_s) + "s)";
}

// ---------------------------------------------------------------------------
// Scan for aircraft fixtures and scenarios
// ---------------------------------------------------------------------------

static void scanAircraft() {
    g_aircraftList.clear();
    // Look for JSON files in tests/fixtures/ relative to CWD or known paths
    std::vector<std::string> searchPaths = {
        "tests/fixtures/",
        "../tests/fixtures/",
        "../../tests/fixtures/",
    };
    for (const auto& dir : searchPaths) {
        if (std::filesystem::exists(dir)) {
            for (const auto& entry : std::filesystem::directory_iterator(dir)) {
                if (entry.path().extension() == ".json") {
                    g_aircraftList.push_back(entry.path().string());
                }
            }
            if (!g_aircraftList.empty()) break;
        }
    }
    std::sort(g_aircraftList.begin(), g_aircraftList.end());
}

static void scanScenarios() {
    g_scenarioList = manuver_test::ScenarioRegistry::instance().list();
}

// ---------------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------------

static void drawGroundGrid() {
    // Draw a grid at y=0 (ground level)
    const float gridSize = 100.0f;  // 10000 ft in world units
    const int gridLines = 20;
    const float step = gridSize * 2.0f / gridLines;

    rlBegin(RL_LINES);
    rlColor3f(0.3f, 0.3f, 0.3f);
    for (int i = 0; i <= gridLines; ++i) {
        float pos = -gridSize + i * step;
        rlVertex3f(pos, 0.0f, -gridSize);
        rlVertex3f(pos, 0.0f,  gridSize);
        rlVertex3f(-gridSize, 0.0f, pos);
        rlVertex3f( gridSize, 0.0f, pos);
    }
    rlEnd();
}

static void drawTrackTrail(int fromFrame, int toFrame, Color color) {
    if (fromFrame < 0) fromFrame = 0;
    if (toFrame >= (int)g_trace.frames.size()) toFrame = g_trace.frames.size() - 1;
    if (toFrame <= fromFrame) return;

    rlBegin(RL_LINES);
    rlColor4ub(color.r, color.g, color.b, color.a);
    for (int i = fromFrame; i < toFrame; ++i) {
        const auto& f1 = g_trace.frames[i];
        const auto& f2 = g_trace.frames[i + 1];
        Vector3 p1 = worldToRl(f1.x, f1.y, f1.z);
        Vector3 p2 = worldToRl(f2.x, f2.y, f2.z);
        rlVertex3f(p1.x, p1.y, p1.z);
        rlVertex3f(p2.x, p2.y, p2.z);
    }
    rlEnd();
}

static void drawAircraftMarker(const f4flight::TraceFrame& f) {
    Vector3 pos = worldToRl(f.x, f.y, f.z);
    Color col = modeColor(f.mode);

    // Draw a small arrow pointing in the heading direction
    float heading = (float)f.psi;
    float dx = std::sin(heading) * 3.0f;
    float dz = std::cos(heading) * 3.0f;

    DrawLine3D(pos, {pos.x + dx, pos.y, pos.z + dz}, col);
    DrawSphere(pos, 1.5f, col);
}

static void drawThreats(const f4flight::TraceFrame& f) {
    for (const auto& t : f.threats) {
        Vector3 pos = worldToRl(t.x, t.y, t.z);
        Color col = (t.type == "missile") ? RED : (t.type == "guns") ? ORANGE : PURPLE;
        DrawSphere(pos, 1.0f, col);
        DrawLine3D(pos, {pos.x, 0, pos.z}, Fade(col, 0.3f));
    }
}

static void updateCamera() {
    if (g_trace.frames.empty()) return;
    const auto& f = g_trace.frames[g_currentFrame];
    Vector3 acPos = worldToRl(f.x, f.y, f.z);

    switch (g_camMode) {
        case ViewMode::TopDown:
            g_camera.position = {acPos.x, 80.0f, acPos.z};
            g_camera.target = acPos;
            g_camera.up = {0, 0, 1};
            g_camera.fovy = 45.0f;
            g_camera.projection = CAMERA_PERSPECTIVE;
            break;
        case ViewMode::Side:
            g_camera.position = {acPos.x + 60.0f, acPos.y + 10.0f, acPos.z};
            g_camera.target = acPos;
            g_camera.up = {0, 1, 0};
            break;
        case ViewMode::Chase: {
            float heading = (float)f.psi;
            g_camera.position = {
                acPos.x - std::sin(heading) * 30.0f,
                acPos.y + 10.0f,
                acPos.z - std::cos(heading) * 30.0f
            };
            g_camera.target = acPos;
            g_camera.up = {0, 1, 0};
            break;
        }
        case ViewMode::FreeOrbit:
            // Camera stays where the user put it
            break;
    }
}

static void drawHUD() {
    if (g_trace.frames.empty()) {
        DrawText("No trace loaded. Select aircraft + scenario, click Run.",
                 20, 20, 16, RAYWHITE);
        return;
    }

    const auto& f = g_trace.frames[g_currentFrame];
    double alt = -f.z;
    double speed = f.vcas;
    double t = f.t;

    // HUD background
    DrawRectangle(10, 10, 320, 140, Fade(BLACK, 0.7f));

    DrawText(TextFormat("Aircraft: %s", g_trace.aircraft.c_str()), 20, 18, 14, RAYWHITE);
    DrawText(TextFormat("Scenario: %s", g_trace.scenario.c_str()), 20, 36, 14, RAYWHITE);
    DrawText(TextFormat("t=%.1fs / %.1fs  frame %d/%d",
             t, g_trace.duration_s, g_currentFrame + 1, (int)g_trace.frames.size()),
             20, 54, 14, RAYWHITE);
    DrawText(TextFormat("ALT: %.0f ft   VCAS: %.0f kts   G: %.1f",
             alt, speed, f.nzcgs), 20, 72, 14, RAYWHITE);
    DrawText(TextFormat("Mode: %s   Phase: %s", f.mode.c_str(), f.phase.c_str()),
             20, 90, 14, modeColor(f.mode));

    // Phase results
    int py = 108;
    for (const auto& p : g_trace.phases) {
        const char* res = p.skipped ? "SKIP" : (p.passed ? "PASS" : "FAIL");
        Color c = p.skipped ? GRAY : (p.passed ? GREEN : RED);
        DrawText(TextFormat("  %s [%s]", p.name.c_str(), res), 20, py, 12, c);
        py += 14;
    }

    // Camera mode + playback state
    DrawText(TextFormat("Camera: %s  %s  %.1fx",
             g_camModeNames[(int)g_camMode],
             g_playing ? "PLAYING" : "PAUSED",
             g_playbackSpeed),
             20, py + 4, 12, YELLOW);

    // Timeline bar
    int barY = 170;
    int barW = 600;
    DrawRectangle(10, barY, barW, 20, Fade(BLACK, 0.7f));
    float progress = g_trace.frames.empty() ? 0 :
        (float)g_currentFrame / (float)(g_trace.frames.size() - 1);
    DrawRectangle(10, barY, (int)(barW * progress), 20, BLUE);
    DrawText(TextFormat("Timeline"), 15, barY + 24, 10, GRAY);

    // Status message
    DrawText(g_statusMsg.c_str(), 10, barY + 40, 12, LIME);

    // Help text
    if (g_showHelp) {
        DrawRectangle(10, 240, 400, 180, Fade(BLACK, 0.85f));
        DrawText("CONTROLS", 20, 248, 14, YELLOW);
        DrawText("Tab     Switch camera mode", 20, 268, 12, RAYWHITE);
        DrawText("Space   Play/pause", 20, 282, 12, RAYWHITE);
        DrawText("Left/Right  Step frame", 20, 296, 12, RAYWHITE);
        DrawText("Up/Down  Playback speed", 20, 310, 12, RAYWHITE);
        DrawText("R       Reset to start", 20, 324, 12, RAYWHITE);
        DrawText("H       Toggle help", 20, 338, 12, RAYWHITE);
        DrawText("Mouse drag  Orbit (free orbit mode)", 20, 352, 12, RAYWHITE);
        DrawText("Mouse wheel  Zoom", 20, 366, 12, RAYWHITE);
    } else {
        DrawText("Press H for help", 10, 240, 12, GRAY);
    }
}

// ---------------------------------------------------------------------------
// Screenshot mode — render a single frame to PNG and exit (for headless use)
// ---------------------------------------------------------------------------

static bool screenshotMode = false;
static std::string screenshotPath;
static int screenshotFrame = -1;  // -1 = middle frame

static void takeScreenshot() {
    if (g_trace.frames.empty()) return;

    // Jump to the requested frame (or middle)
    if (screenshotFrame >= 0 && screenshotFrame < (int)g_trace.frames.size()) {
        g_currentFrame = screenshotFrame;
    } else {
        g_currentFrame = (int)g_trace.frames.size() / 2;
    }

    // Update camera for this frame
    updateCamera();

    // Render one frame
    BeginDrawing();
    ClearBackground({15, 15, 25, 255});
    BeginMode3D(g_camera);
    drawGroundGrid();

    // Draw full track
    drawTrackTrail(0, (int)g_trace.frames.size() - 1, Fade(GRAY, 0.3f));
    // Draw recent trail colored by mode
    int trailStart = std::max(0, g_currentFrame - 300);
    int i = trailStart;
    while (i < g_currentFrame) {
        std::string mode = g_trace.frames[i].mode;
        int j = i;
        while (j < g_currentFrame && g_trace.frames[j + 1].mode == mode) ++j;
        drawTrackTrail(i, j + 1, modeColor(mode));
        i = j + 1;
    }
    drawAircraftMarker(g_trace.frames[g_currentFrame]);
    drawThreats(g_trace.frames[g_currentFrame]);

    EndMode3D();
    drawHUD();
    EndDrawing();

    // Save screenshot — RayLib's TakeScreenshot saves relative to CWD,
    // so we need to resolve to an absolute path first.
    std::string absPath = screenshotPath;
    if (!absPath.empty() && absPath[0] != '/') {
        // Resolve relative to CWD
        char* cwd = getcwd(nullptr, 0);
        if (cwd) {
            absPath = std::string(cwd) + "/" + absPath;
            free(cwd);
        }
    }
    // RayLib's TakeScreenshot uses the filename as-is, but on some platforms
    // it strips the directory. We'll save to a temp file then rename.
    TakeScreenshot(screenshotPath.c_str());

    // RayLib saves to CWD/filename — move it to the requested path
    // Extract just the filename from the path
    std::string filename = screenshotPath;
    auto lastSlash = filename.find_last_of("/\\");
    if (lastSlash != std::string::npos) filename = filename.substr(lastSlash + 1);

    if (filename != screenshotPath) {
        std::rename(filename.c_str(), screenshotPath.c_str());
    }

    std::printf("Screenshot saved to %s (frame %d)\n",
                screenshotPath.c_str(), g_currentFrame);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    // Parse args
    std::string tracePath;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--screenshot" && i + 1 < argc) {
            screenshotMode = true;
            screenshotPath = argv[++i];
        } else if (a == "--frame" && i + 1 < argc) {
            screenshotFrame = std::atoi(argv[++i]);
        } else if (a == "--help" || a == "-h") {
            std::printf(
                "Usage: f4flight_viz [trace.json] [--screenshot out.png] [--frame N]\n\n"
                "Interactive 3D maneuver viewer.\n\n"
                "Options:\n"
                "  --screenshot PATH   Render one frame to PNG and exit (headless)\n"
                "  --frame N           Frame number to screenshot (default: middle)\n"
                "  --help              Show this help\n\n"
                "Controls (interactive mode):\n"
                "  Tab       Switch camera mode\n"
                "  Space     Play/pause\n"
                "  Left/Right  Step frame\n"
                "  Up/Down   Playback speed\n"
                "  R         Reset to start\n"
                "  Enter     Run selected scenario\n"
                "  H         Toggle help\n");
            return 0;
        } else if (a[0] != '-' && tracePath.empty()) {
            tracePath = a;
        }
    }

    // Init window
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(1280, 800, "f4flight viz");
    SetTargetFPS(60);

    // Camera defaults
    g_camera.position = {0, 80, 0};
    g_camera.target = {0, 0, 0};
    g_camera.up = {0, 0, 1};
    g_camera.fovy = 45.0f;
    g_camera.projection = CAMERA_PERSPECTIVE;

    // Scan for aircraft + scenarios
    scanAircraft();
    scanScenarios();

    // Load trace from command line if provided
    if (!tracePath.empty()) {
        loadTraceFile(tracePath);
    }

    // Screenshot mode: render one frame and exit
    if (screenshotMode) {
        takeScreenshot();
        CloseWindow();
        return 0;
    }

    // Main loop
    while (!WindowShouldClose()) {
        // --- Input ---
        if (IsKeyPressed(KEY_TAB)) {
            g_camMode = (ViewMode)(((int)g_camMode + 1) % 4);
        }
        if (IsKeyPressed(KEY_SPACE)) g_playing = !g_playing;
        if (IsKeyPressed(KEY_H)) g_showHelp = !g_showHelp;
        if (IsKeyPressed(KEY_R)) { g_currentFrame = 0; g_frameAccum = 0; }
        if (IsKeyPressed(KEY_LEFT)) {
            g_playing = false;
            if (g_currentFrame > 0) g_currentFrame--;
        }
        if (IsKeyPressed(KEY_RIGHT)) {
            g_playing = false;
            if (g_currentFrame < (int)g_trace.frames.size() - 1) g_currentFrame++;
        }
        if (IsKeyPressed(KEY_UP)) g_playbackSpeed = std::min(8.0, g_playbackSpeed * 2.0);
        if (IsKeyPressed(KEY_DOWN)) g_playbackSpeed = std::max(0.125, g_playbackSpeed / 2.0);

        // Run key (R+A combination or button)
        if (IsKeyPressed(KEY_ENTER)) {
            runScenario();
        }

        // Mouse wheel zoom
        float wheel = GetMouseWheelMove();
        if (wheel != 0) {
            g_camera.position.x += (g_camera.target.x - g_camera.position.x) * 0.1f * wheel;
            g_camera.position.y += (g_camera.target.y - g_camera.position.y) * 0.1f * wheel;
            g_camera.position.z += (g_camera.target.z - g_camera.position.z) * 0.1f * wheel;
        }

        // --- Playback ---
        if (g_playing && !g_trace.frames.empty()) {
            g_frameAccum += GetFrameTime() * g_playbackSpeed * 60.0;  // frames
            while (g_frameAccum >= 1.0 && g_currentFrame < (int)g_trace.frames.size() - 1) {
                g_currentFrame++;
                g_frameAccum -= 1.0;
            }
            if (g_currentFrame >= (int)g_trace.frames.size() - 1) {
                g_playing = false;
                g_currentFrame = (int)g_trace.frames.size() - 1;
            }
        }

        // --- Camera update ---
        if (g_camMode == ViewMode::FreeOrbit) {
            UpdateCamera(&g_camera, CAMERA_ORBITAL);
        } else {
            updateCamera();
        }

        // --- Draw ---
        BeginDrawing();
        ClearBackground({15, 15, 25, 255});

        BeginMode3D(g_camera);
        drawGroundGrid();

        if (!g_trace.frames.empty()) {
            // Draw full track trail (faded)
            drawTrackTrail(0, (int)g_trace.frames.size() - 1, Fade(GRAY, 0.3f));

            // Draw recent trail (last 300 frames) colored by mode
            int trailStart = std::max(0, g_currentFrame - 300);
            // Draw mode-colored segments
            int i = trailStart;
            while (i < g_currentFrame) {
                std::string mode = g_trace.frames[i].mode;
                int j = i;
                while (j < g_currentFrame && g_trace.frames[j + 1].mode == mode) ++j;
                drawTrackTrail(i, j + 1, modeColor(mode));
                i = j + 1;
            }

            // Draw current aircraft
            drawAircraftMarker(g_trace.frames[g_currentFrame]);

            // Draw threats at current frame
            drawThreats(g_trace.frames[g_currentFrame]);

            // Draw runway threshold marker (at origin)
            DrawLine3D({-0.5f, 0, 0}, {0.5f, 0, 0}, GOLD);
            DrawLine3D({0, 0, -0.5f}, {0, 0, 0.5f}, GOLD);
        }

        EndMode3D();

        // HUD overlay
        drawHUD();

        // Aircraft/scenario selection (simple on-screen text)
        if (!g_aircraftList.empty()) {
            DrawText("Aircraft:", 10, GetScreenHeight() - 80, 12, GRAY);
            DrawText(TextFormat("  [%d] %s (UP/DN to change)",
                     g_selectedAircraft,
                     std::filesystem::path(g_aircraftList[g_selectedAircraft]).stem().string().c_str()),
                     10, GetScreenHeight() - 65, 12, RAYWHITE);
            DrawText("Scenario:", 10, GetScreenHeight() - 45, 12, GRAY);
            DrawText(TextFormat("  [%d] %s (PGUP/PGDN to change)",
                     g_selectedScenario,
                     g_scenarioList.empty() ? "" : g_scenarioList[g_selectedScenario].c_str()),
                     10, GetScreenHeight() - 30, 12, RAYWHITE);
        }

        if (IsKeyPressed(KEY_PAGE_UP)) {
            g_selectedScenario = std::max(0, g_selectedScenario - 1);
        }
        if (IsKeyPressed(KEY_PAGE_DOWN)) {
            g_selectedScenario = std::min((int)g_scenarioList.size() - 1, g_selectedScenario + 1);
        }
        if (IsKeyPressed(KEY_UP) && IsKeyDown(KEY_LEFT_SHIFT)) {
            g_selectedAircraft = std::min((int)g_aircraftList.size() - 1, g_selectedAircraft + 1);
        }
        if (IsKeyPressed(KEY_DOWN) && IsKeyDown(KEY_LEFT_SHIFT)) {
            g_selectedAircraft = std::max(0, g_selectedAircraft - 1);
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
