#pragma once
// Ian: TronGridSource — "The Grid" first-person Tron-style virtual field camera.
//
// Architecture:
//   - Pure software renderer using the same pixel-buffer + Bresenham pattern as
//     SimCameraSource.  Zero OSG/OpenGL dependency — avoids the crash risk of
//     creating GPU contexts on worker threads.
//   - Renders a first-person driver's-eye view of a 54×27 ft FRC field as a
//     glowing cyan wireframe grid on a black background, inspired by the classic
//     Tron light-cycle arena.
//   - Reads robot position (X_ft, Y_ft, Heading) from SmartDashboard each frame
//     to position the virtual camera at robot height (~2 ft) looking in the
//     robot's heading direction.
//   - 3D perspective projection via a software pinhole camera model: world-space
//     points are transformed to camera-space, then projected to screen via
//     focal-length division.
//   - Field boundary walls are rendered as bright cyan wireframe.
//   - The Squid Games FIRST logo (red circle, white triangle, blue square) is
//     rendered as vector art on the field floor at center court.
//   - When the robot goes off-field, an arrow points back toward the field.
//   - Depth fog: lines fade from bright cyan to dim blue with distance.
//
// Color palette (classic Tron):
//   - Background: pure black (0,0,0)
//   - Near grid lines: bright cyan (0, 255, 255)
//   - Far grid lines: dim blue (0, 40, 80)
//   - Field walls: white-cyan (200, 255, 255)
//   - HUD text: cyan with slight glow
//
// Ian: LESSON LEARNED — OSG offscreen FBO rendering on a worker thread crashes
// on many Windows GPU drivers because the OpenGL context must be created on
// the thread that owns the window.  The pure-software approach is immune to
// this — it only needs CPU and std::vector<uint8_t>.

#include <atomic>
#include <cstdint>
#include <functional>
#include <thread>
#include <vector>

class MjpegServer;

class TronGridSource
{
public:
	TronGridSource();
	~TronGridSource();

	// Ian: Position callback — called each frame to get the robot's current
	// position and heading.  The callback returns (x_ft, y_ft, heading_deg).
	// This decouples TronGridSource from SmartDashboard/NT4 directly.
	using PositionCallback = std::function<bool(double& x_ft, double& y_ft, double& heading_deg)>;
	void SetPositionCallback(PositionCallback cb);

	void Start(MjpegServer* server, int width = 320, int height = 240, int targetFps = 15);
	void Stop();
	bool IsRunning() const;
	int GetFrameCount() const;

private:
	void WorkerThread();
	void RenderFrame(int frameIndex);

	// --- 3D projection pipeline ---
	struct Vec3 { double x, y, z; };
	struct Vec2 { double x, y; };

	// Transform world point to camera-relative coordinates
	Vec3 WorldToCamera(const Vec3& worldPt) const;
	// Project camera-space point to screen pixel coordinates
	// Returns false if the point is behind the camera
	bool CameraToScreen(const Vec3& camPt, int& sx, int& sy) const;
	// Combined: world to screen.  Returns false if behind camera.
	bool WorldToScreen(const Vec3& worldPt, int& sx, int& sy) const;

	// --- Drawing primitives (same pattern as SimCameraSource) ---
	void SetPixel(int x, int y, uint8_t r, uint8_t g, uint8_t b);
	void DrawLine(int x0, int y0, int x1, int y1, uint8_t r, uint8_t g, uint8_t b);
	void DrawRect(int x, int y, int w, int h, uint8_t r, uint8_t g, uint8_t b);
	void DrawChar(int x, int y, char ch, uint8_t r, uint8_t g, uint8_t b, int scale = 1);
	void DrawString(int x, int y, const char* str, uint8_t r, uint8_t g, uint8_t b, int scale = 1);

	// --- 3D line drawing with depth fog ---
	// Draws a line between two world-space points with perspective projection.
	// Color fades from (r,g,b) toward black based on distance from camera.
	void DrawLine3D(const Vec3& a, const Vec3& b, uint8_t r, uint8_t g, uint8_t b_color);

	// --- Scene elements ---
	void DrawGrid();
	void DrawFieldWalls();
	void DrawSquidGamesEmblem();
	void DrawHUD(int frameIndex);
	void DrawOffFieldArrow();
	void DrawBackgroundClouds(int frameIndex);
	void DrawMCPTower();

	// --- Clipping helpers ---
	// Clips a 3D line segment against the near plane (z > nearClip in camera space).
	// Returns false if entirely behind camera.
	bool ClipLineNearPlane(Vec3& a, Vec3& b, double nearClip = 0.1) const;
	// Clips a 2D screen-space line segment to the visible screen rectangle.
	// Uses Cohen-Sutherland algorithm.  Returns false if entirely off-screen.
	bool ClipLineToScreen(int& x0, int& y0, int& x1, int& y1) const;

	// --- State ---
	MjpegServer* m_server = nullptr;
	std::thread m_workerThread;
	std::atomic<bool> m_running{false};
	std::atomic<int> m_frameCount{0};
	PositionCallback m_positionCallback;

	int m_width = 320;
	int m_height = 240;
	int m_targetFps = 15;
	std::vector<uint8_t> m_pixelBuffer;

	// --- Camera state (updated each frame from position callback) ---
	double m_camX = 0.0;      // feet — robot X position on field
	double m_camY = 0.0;      // feet — robot Y position on field
	double m_camZ = 2.0;      // feet — camera height (fixed at ~2ft)
	double m_headingDeg = 0.0; // degrees — robot heading (0 = +Y axis)

	// --- Field constants ---
	// Ian: CENTERED COORDINATE SYSTEM — The field is centered at the origin.
	//   X ∈ [-kFieldHalfLength, +kFieldHalfLength]  (along field length)
	//   Y ∈ [-kFieldHalfWidth,  +kFieldHalfWidth]   (along field width)
	//   Robot (0,0) from simulator = field center.  No offset mapping needed.
	static constexpr double kFieldLength     = 54.0;  // feet (X axis, total)
	static constexpr double kFieldWidth      = 27.0;  // feet (Y axis, total)
	static constexpr double kFieldHalfLength = 27.0;  // kFieldLength / 2
	static constexpr double kFieldHalfWidth  = 13.5;  // kFieldWidth / 2
	static constexpr double kWallHeight      = 1.5;   // feet — field perimeter wall height
	static constexpr double kFOVDeg          = 90.0;  // horizontal field of view
};
