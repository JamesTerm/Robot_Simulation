// Ian: TronGridSource.cpp — "The Grid" first-person Tron-style virtual field camera.
// See TronGridSource.h for architecture notes.
//
// This file contains:
//   1. Worker thread loop (position query + render + JPEG encode + push)
//   2. 3D perspective projection pipeline (world → camera → screen)
//   3. Scene renderers: floor grid, field walls, Squid Games emblem, HUD, off-field arrow
//   4. Bresenham line drawing with depth-fog color attenuation
//   5. Near-plane line clipping to handle geometry behind the camera
//
// Ian: COORDINATE SYSTEM (CENTERED)
//   World space: X = along field length (54 ft), Y = along field width (27 ft), Z = up
//   Field is centered at the origin:
//     X ∈ [-27, +27],  Y ∈ [-13.5, +13.5],  Z = 0 (floor)
//   Robot (0,0) from the simulator = field center.
//   Robot heading 0° = looking along +Y axis (toward far wall)
//   Camera is at (robot_X, robot_Y, 2.0) looking in heading direction
//
// Ian: The robot position from SmartDashboard (X_ft, Y_ft) maps directly to
//   field world coordinates with no offset.  Robot at (0,0) is field center.

#include "TronGridSource.h"
#include "MjpegServer.h"
#include "stb_image_write.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>

#ifdef _WIN32
#include <Windows.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace
{
	void DebugLog(const char* fmt, ...)
	{
		char buf[512];
		va_list args;
		va_start(args, fmt);
		vsnprintf(buf, sizeof(buf), fmt, args);
		va_end(args);
#ifdef _WIN32
		OutputDebugStringA(buf);
#endif
	}

	void StbWriteCallback(void* context, void* data, int size)
	{
		auto* output = static_cast<std::vector<uint8_t>*>(context);
		const auto* bytes = static_cast<const uint8_t*>(data);
		output->insert(output->end(), bytes, bytes + size);
	}

	// =========================================================================
	// Minimal 5x7 bitmap font (same as SimCameraSource)
	// =========================================================================
	static constexpr int kFontFirstChar = 32;
	static constexpr int kFontLastChar = 90;
	static constexpr int kGlyphWidth = 5;
	static constexpr int kGlyphHeight = 7;

	// clang-format off
	static const uint8_t kFontData[][5] =
	{
		{0x00, 0x00, 0x00, 0x00, 0x00}, // ' '
		{0x00, 0x00, 0x5F, 0x00, 0x00}, // '!'
		{0x00, 0x07, 0x00, 0x07, 0x00}, // '"'
		{0x14, 0x7F, 0x14, 0x7F, 0x14}, // '#'
		{0x24, 0x2A, 0x7F, 0x2A, 0x12}, // '$'
		{0x23, 0x13, 0x08, 0x64, 0x62}, // '%'
		{0x36, 0x49, 0x55, 0x22, 0x50}, // '&'
		{0x00, 0x05, 0x03, 0x00, 0x00}, // '''
		{0x00, 0x1C, 0x22, 0x41, 0x00}, // '('
		{0x00, 0x41, 0x22, 0x1C, 0x00}, // ')'
		{0x14, 0x08, 0x3E, 0x08, 0x14}, // '*'
		{0x08, 0x08, 0x3E, 0x08, 0x08}, // '+'
		{0x00, 0x50, 0x30, 0x00, 0x00}, // ','
		{0x08, 0x08, 0x08, 0x08, 0x08}, // '-'
		{0x00, 0x60, 0x60, 0x00, 0x00}, // '.'
		{0x20, 0x10, 0x08, 0x04, 0x02}, // '/'
		{0x3E, 0x51, 0x49, 0x45, 0x3E}, // '0'
		{0x00, 0x42, 0x7F, 0x40, 0x00}, // '1'
		{0x42, 0x61, 0x51, 0x49, 0x46}, // '2'
		{0x21, 0x41, 0x45, 0x4B, 0x31}, // '3'
		{0x18, 0x14, 0x12, 0x7F, 0x10}, // '4'
		{0x27, 0x45, 0x45, 0x45, 0x39}, // '5'
		{0x3C, 0x4A, 0x49, 0x49, 0x30}, // '6'
		{0x01, 0x71, 0x09, 0x05, 0x03}, // '7'
		{0x36, 0x49, 0x49, 0x49, 0x36}, // '8'
		{0x06, 0x49, 0x49, 0x29, 0x1E}, // '9'
		{0x00, 0x36, 0x36, 0x00, 0x00}, // ':'
		{0x00, 0x56, 0x36, 0x00, 0x00}, // ';'
		{0x08, 0x14, 0x22, 0x41, 0x00}, // '<'
		{0x14, 0x14, 0x14, 0x14, 0x14}, // '='
		{0x00, 0x41, 0x22, 0x14, 0x08}, // '>'
		{0x02, 0x01, 0x51, 0x09, 0x06}, // '?'
		{0x32, 0x49, 0x79, 0x41, 0x3E}, // '@'
		{0x7E, 0x11, 0x11, 0x11, 0x7E}, // 'A'
		{0x7F, 0x49, 0x49, 0x49, 0x36}, // 'B'
		{0x3E, 0x41, 0x41, 0x41, 0x22}, // 'C'
		{0x7F, 0x41, 0x41, 0x22, 0x1C}, // 'D'
		{0x7F, 0x49, 0x49, 0x49, 0x41}, // 'E'
		{0x7F, 0x09, 0x09, 0x09, 0x01}, // 'F'
		{0x3E, 0x41, 0x49, 0x49, 0x7A}, // 'G'
		{0x7F, 0x08, 0x08, 0x08, 0x7F}, // 'H'
		{0x00, 0x41, 0x7F, 0x41, 0x00}, // 'I'
		{0x20, 0x40, 0x41, 0x3F, 0x01}, // 'J'
		{0x7F, 0x08, 0x14, 0x22, 0x41}, // 'K'
		{0x7F, 0x40, 0x40, 0x40, 0x40}, // 'L'
		{0x7F, 0x02, 0x0C, 0x02, 0x7F}, // 'M'
		{0x7F, 0x04, 0x08, 0x10, 0x7F}, // 'N'
		{0x3E, 0x41, 0x41, 0x41, 0x3E}, // 'O'
		{0x7F, 0x09, 0x09, 0x09, 0x06}, // 'P'
		{0x3E, 0x41, 0x51, 0x21, 0x5E}, // 'Q'
		{0x7F, 0x09, 0x19, 0x29, 0x46}, // 'R'
		{0x46, 0x49, 0x49, 0x49, 0x31}, // 'S'
		{0x01, 0x01, 0x7F, 0x01, 0x01}, // 'T'
		{0x3F, 0x40, 0x40, 0x40, 0x3F}, // 'U'
		{0x1F, 0x20, 0x40, 0x20, 0x1F}, // 'V'
		{0x3F, 0x40, 0x38, 0x40, 0x3F}, // 'W'
		{0x63, 0x14, 0x08, 0x14, 0x63}, // 'X'
		{0x07, 0x08, 0x70, 0x08, 0x07}, // 'Y'
		{0x61, 0x51, 0x49, 0x45, 0x43}, // 'Z'
	};
	// clang-format on

	// Ian: Clamp a value to [lo, hi]
	inline int Clamp(int val, int lo, int hi)
	{
		return val < lo ? lo : (val > hi ? hi : val);
	}

	// Ian: Interpolate color component based on depth fog
	// dist = distance from camera, maxDist = distance at which color goes to minimum
	inline uint8_t FogColor(uint8_t bright, uint8_t dim, double dist, double maxDist)
	{
		if (dist >= maxDist) return dim;
		if (dist <= 0.0) return bright;
		double t = dist / maxDist;
		return static_cast<uint8_t>(bright + t * (static_cast<double>(dim) - bright));
	}
}

// =========================================================================
// Construction / Destruction
// =========================================================================

TronGridSource::TronGridSource() = default;

TronGridSource::~TronGridSource()
{
	Stop();
}

void TronGridSource::SetPositionCallback(PositionCallback cb)
{
	m_positionCallback = std::move(cb);
}

void TronGridSource::Start(MjpegServer* server, int width, int height, int targetFps)
{
	Stop();

	m_server = server;
	m_width = width;
	m_height = height;
	m_targetFps = targetFps;
	m_frameCount.store(0);
	m_pixelBuffer.resize(static_cast<size_t>(m_width) * m_height * 3, 0);

	m_running.store(true);
	m_workerThread = std::thread(&TronGridSource::WorkerThread, this);

	DebugLog("[TronGridSource] Started: %dx%d @ %d fps\n", m_width, m_height, m_targetFps);
}

void TronGridSource::Stop()
{
	m_running.store(false);
	if (m_workerThread.joinable())
	{
		m_workerThread.join();
	}
	m_server = nullptr;
	DebugLog("[TronGridSource] Stopped (total frames: %d)\n", m_frameCount.load());
}

bool TronGridSource::IsRunning() const
{
	return m_running.load();
}

int TronGridSource::GetFrameCount() const
{
	return m_frameCount.load();
}

// =========================================================================
// Worker Thread
// =========================================================================

void TronGridSource::WorkerThread()
{
	const auto frameDuration = std::chrono::microseconds(1000000 / m_targetFps);
	auto nextFrameTime = std::chrono::steady_clock::now();
	int frameIndex = 0;

	while (m_running.load())
	{
		// Query robot position each frame
		if (m_positionCallback)
		{
			double x = 0.0, y = 0.0, heading = 0.0;
			if (m_positionCallback(x, y, heading))
			{
			// Ian: CENTERED COORDINATE SYSTEM — Robot (0,0) from the simulator
			// maps directly to field center (0,0).  No offset needed.
			// The field spans X ∈ [-27, +27], Y ∈ [-13.5, +13.5].
			m_camX = x;
			m_camY = y;
				// Ian: HEADING INVERSION — The published Drive/Heading uses atan2(x,y)
			// which gives CW-positive from +Y.  But our WorldToCamera() rotation
			// expects CCW-positive (standard math convention).  Negating the heading
			// fixes the rotation so CW robot rotation gives CW view rotation.
			m_headingDeg = -heading;
			}
		}

		RenderFrame(frameIndex);

		// JPEG encode
		std::vector<uint8_t> jpegOutput;
		jpegOutput.reserve(32 * 1024);
		const int quality = 75;
		int result = stbi_write_jpg_to_func(
			StbWriteCallback,
			&jpegOutput,
			m_width,
			m_height,
			3,
			m_pixelBuffer.data(),
			quality);

		if (result != 0 && m_server != nullptr)
		{
			m_server->PushFrame(jpegOutput);
			m_frameCount.fetch_add(1);
		}

		++frameIndex;

		nextFrameTime += frameDuration;
		auto now = std::chrono::steady_clock::now();
		if (nextFrameTime > now)
		{
			std::this_thread::sleep_until(nextFrameTime);
		}
		else
		{
			nextFrameTime = now;
		}
	}
}

// =========================================================================
// 3D Projection Pipeline
// =========================================================================

// Ian: Transform a world-space point into camera-relative coordinates.
// Camera is at (m_camX, m_camY, m_camZ), looking in heading direction.
//
// Heading convention: 0° = looking along +Y axis (toward far wall)
//   Heading increases clockwise when viewed from above:
//     0° = +Y, 90° = +X, 180° = -Y, 270° = -X
//
// Camera coordinate system (right-handed):
//   cam_X = right, cam_Y = up, cam_Z = forward (into screen)
TronGridSource::Vec3 TronGridSource::WorldToCamera(const Vec3& worldPt) const
{
	// Translate to camera origin
	const double dx = worldPt.x - m_camX;
	const double dy = worldPt.y - m_camY;
	const double dz = worldPt.z - m_camZ;

	// Rotate by heading.  heading=0 means looking along +Y.
	// We want +Y world direction to map to +Z camera (forward).
	const double headingRad = m_headingDeg * M_PI / 180.0;
	const double cosH = std::cos(headingRad);
	const double sinH = std::sin(headingRad);

	// Rotation matrix (heading rotates in XY plane):
	//   cam_right   (X) =  cosH * dx + sinH * dy    (perpendicular to heading, rightward)
	//   cam_forward (Z) = -sinH * dx + cosH * dy     (along heading direction)
	//   cam_up      (Y) = dz                          (world Z = camera Y)
	// Ian: With heading=0: cam_right = dx, cam_forward = dy — looking along +Y. Correct.
	//      With heading=90: cam_right = dy, cam_forward = -dx — looking along +X. Correct.
	//      Negating sinH for right-hand: heading increases CW from above.
	Vec3 cam;
	cam.x = cosH * dx + sinH * dy;      // right
	cam.y = dz;                           // up
	cam.z = -sinH * dx + cosH * dy;      // forward
	return cam;
}

// Ian: Project camera-space point to screen coordinates using pinhole model.
// Returns false if point is behind camera (z <= 0).
bool TronGridSource::CameraToScreen(const Vec3& camPt, int& sx, int& sy) const
{
	if (camPt.z <= 0.01)
		return false;

	// Focal length from horizontal FOV
	const double fovRad = kFOVDeg * M_PI / 180.0;
	const double focalX = (m_width * 0.5) / std::tan(fovRad * 0.5);
	const double focalY = focalX;  // Square pixels

	// Project: screen = focal * (cam / cam.z) + center
	const double px = focalX * (camPt.x / camPt.z) + m_width * 0.5;
	const double py = -focalY * (camPt.y / camPt.z) + m_height * 0.5;  // flip Y: up is negative screen Y

	sx = static_cast<int>(px + 0.5);
	sy = static_cast<int>(py + 0.5);
	return true;
}

bool TronGridSource::WorldToScreen(const Vec3& worldPt, int& sx, int& sy) const
{
	Vec3 cam = WorldToCamera(worldPt);
	return CameraToScreen(cam, sx, sy);
}

// =========================================================================
// Near-Plane Line Clipping
// =========================================================================
// Ian: Lines that cross behind the camera cause wild projection artifacts.
// We clip them against z = nearClip in camera space using parametric
// interpolation.  If both endpoints are behind the camera, the line is
// fully culled.  If one endpoint is behind, we interpolate to the clip plane.
bool TronGridSource::ClipLineNearPlane(Vec3& a, Vec3& b, double nearClip) const
{
	const bool aFront = (a.z > nearClip);
	const bool bFront = (b.z > nearClip);

	if (aFront && bFront)
		return true;   // Both in front — no clipping needed

	if (!aFront && !bFront)
		return false;  // Both behind — fully culled

	// One behind, one in front — clip the behind point
	// Parametric: P(t) = a + t*(b - a), solve for P(t).z = nearClip
	const double t = (nearClip - a.z) / (b.z - a.z);
	Vec3 clipped;
	clipped.x = a.x + t * (b.x - a.x);
	clipped.y = a.y + t * (b.y - a.y);
	clipped.z = nearClip;

	if (!aFront)
		a = clipped;
	else
		b = clipped;

	return true;
}

// =========================================================================
// Screen-Space Line Clipping (Cohen-Sutherland)
// =========================================================================
// Ian: Clips a 2D line segment to the screen rectangle [0, width) x [0, height).
// This prevents Bresenham from wasting time on pixels outside the framebuffer,
// and more importantly prevents lines with one endpoint projected far offscreen
// from creating absurdly long Bresenham runs that lock up the frame.
//
// Without this, a grid line with one end 5000 pixels offscreen would try to
// draw thousands of clipped-away pixels.

bool TronGridSource::ClipLineToScreen(int& x0, int& y0, int& x1, int& y1) const
{
	// Cohen-Sutherland region codes
	enum : int { INSIDE = 0, LEFT = 1, RIGHT = 2, BOTTOM = 4, TOP = 8 };

	const int xMin = 0, yMin = 0;
	const int xMax = m_width - 1, yMax = m_height - 1;

	auto computeCode = [&](int x, int y) -> int {
		int code = INSIDE;
		if (x < xMin) code |= LEFT;
		else if (x > xMax) code |= RIGHT;
		if (y < yMin) code |= TOP;
		else if (y > yMax) code |= BOTTOM;
		return code;
	};

	int code0 = computeCode(x0, y0);
	int code1 = computeCode(x1, y1);

	for (int iter = 0; iter < 20; ++iter)
	{
		if ((code0 | code1) == 0)
			return true;   // Both inside — trivially accept

		if ((code0 & code1) != 0)
			return false;  // Both on same outside side — trivially reject

		// Pick the point that's outside
		const int codeOut = (code0 != 0) ? code0 : code1;
		int x = 0, y = 0;

		// Ian: Using double arithmetic for the interpolation to avoid integer
		// overflow on large offscreen coordinates.
		if (codeOut & TOP)
		{
			x = x0 + static_cast<int>(static_cast<double>(x1 - x0) * (yMin - y0) / (y1 - y0));
			y = yMin;
		}
		else if (codeOut & BOTTOM)
		{
			x = x0 + static_cast<int>(static_cast<double>(x1 - x0) * (yMax - y0) / (y1 - y0));
			y = yMax;
		}
		else if (codeOut & RIGHT)
		{
			y = y0 + static_cast<int>(static_cast<double>(y1 - y0) * (xMax - x0) / (x1 - x0));
			x = xMax;
		}
		else if (codeOut & LEFT)
		{
			y = y0 + static_cast<int>(static_cast<double>(y1 - y0) * (xMin - x0) / (x1 - x0));
			x = xMin;
		}

		if (codeOut == code0)
		{
			x0 = x; y0 = y;
			code0 = computeCode(x0, y0);
		}
		else
		{
			x1 = x; y1 = y;
			code1 = computeCode(x1, y1);
		}
	}

	return false;  // Should not reach here, but safety bail-out
}

// =========================================================================
// Frame Rendering
// =========================================================================

void TronGridSource::RenderFrame(int frameIndex)
{
	// Clear to pure black (Tron void)
	std::memset(m_pixelBuffer.data(), 0,
		static_cast<size_t>(m_width) * m_height * 3);

	// Check if robot is on the field (centered: X ∈ [-27,27], Y ∈ [-13.5,13.5])
	const bool onField =
		m_camX >= -(kFieldHalfLength + 2.0) && m_camX <= kFieldHalfLength + 2.0 &&
		m_camY >= -(kFieldHalfWidth + 2.0)  && m_camY <= kFieldHalfWidth + 2.0;

	// Draw scene elements (back to front, painter's algorithm)
	DrawBackgroundClouds(frameIndex);
	DrawMCPTower();
	DrawGrid();
	DrawFieldWalls();
	DrawSquidGamesEmblem();

	if (!onField)
		DrawOffFieldArrow();

	DrawHUD(frameIndex);
}

// =========================================================================
// Grid Floor
// =========================================================================
// Ian: Draw a 1-foot-spaced grid on the floor plane (Z=0).
// Lines parallel to both X and Y axes, spanning the field plus some margin.
// Color fades with distance (depth fog) for that classic Tron depth feel.
// Field is centered: X ∈ [-27, +27], Y ∈ [-13.5, +13.5].

void TronGridSource::DrawGrid()
{
	// Extend grid slightly beyond the field for visual continuity
	const double margin = 5.0;
	const double xMin = -kFieldHalfLength - margin;
	const double xMax =  kFieldHalfLength + margin;
	const double yMin = -kFieldHalfWidth - margin;
	const double yMax =  kFieldHalfWidth + margin;

	// Lines parallel to X axis (varying Y)
	for (double y = std::floor(yMin); y <= yMax; y += 1.0)
	{
		// Field boundary lines are brighter
		const bool isBoundary = (std::abs(y - kFieldHalfWidth) < 0.01 ||
		                         std::abs(y + kFieldHalfWidth) < 0.01);
		if (isBoundary)
			DrawLine3D({xMin, y, 0.0}, {xMax, y, 0.0}, 0, 220, 220);
		else
			DrawLine3D({xMin, y, 0.0}, {xMax, y, 0.0}, 0, 140, 160);
	}

	// Lines parallel to Y axis (varying X)
	for (double x = std::floor(xMin); x <= xMax; x += 1.0)
	{
		const bool isBoundary = (std::abs(x - kFieldHalfLength) < 0.01 ||
		                         std::abs(x + kFieldHalfLength) < 0.01);
		if (isBoundary)
			DrawLine3D({x, yMin, 0.0}, {x, yMax, 0.0}, 0, 220, 220);
		else
			DrawLine3D({x, yMin, 0.0}, {x, yMax, 0.0}, 0, 140, 160);
	}
}

// =========================================================================
// Field Walls
// =========================================================================
// Ian: Draw the field perimeter walls as bright wireframe rectangles.
// Each wall is a vertical rectangle at the field boundary, kWallHeight tall.
// Field centered: X ∈ [-27, +27], Y ∈ [-13.5, +13.5].

void TronGridSource::DrawFieldWalls()
{
	// Wall color: bright cyan-white (stands out from grid)
	const uint8_t wr = 180, wg = 255, wb = 255;

	const double xL = -kFieldHalfLength;  // left
	const double xR =  kFieldHalfLength;  // right
	const double yN = -kFieldHalfWidth;   // near (south)
	const double yF =  kFieldHalfWidth;   // far (north)
	const double zB = 0.0;                // bottom
	const double zT = kWallHeight;        // top

	// Near wall (Y = -13.5, south — closest to robot starting position)
	DrawLine3D({xL, yN, zB}, {xR, yN, zB}, wr, wg, wb);
	DrawLine3D({xL, yN, zT}, {xR, yN, zT}, wr, wg, wb);
	DrawLine3D({xL, yN, zB}, {xL, yN, zT}, wr, wg, wb);
	DrawLine3D({xR, yN, zB}, {xR, yN, zT}, wr, wg, wb);

	// Far wall (Y = +13.5, north)
	DrawLine3D({xL, yF, zB}, {xR, yF, zB}, wr, wg, wb);
	DrawLine3D({xL, yF, zT}, {xR, yF, zT}, wr, wg, wb);
	DrawLine3D({xL, yF, zB}, {xL, yF, zT}, wr, wg, wb);
	DrawLine3D({xR, yF, zB}, {xR, yF, zT}, wr, wg, wb);

	// Left wall (X = -27)
	DrawLine3D({xL, yN, zB}, {xL, yF, zB}, wr, wg, wb);
	DrawLine3D({xL, yN, zT}, {xL, yF, zT}, wr, wg, wb);

	// Right wall (X = +27)
	DrawLine3D({xR, yN, zB}, {xR, yF, zB}, wr, wg, wb);
	DrawLine3D({xR, yN, zT}, {xR, yF, zT}, wr, wg, wb);

	// Vertical midpoint posts for visual structure
	DrawLine3D({0.0, yN, zB}, {0.0, yN, zT}, 100, 180, 180);
	DrawLine3D({0.0, yF, zB}, {0.0, yF, zT}, 100, 180, 180);
	DrawLine3D({xL, 0.0, zB}, {xL, 0.0, zT}, 100, 180, 180);
	DrawLine3D({xR, 0.0, zB}, {xR, 0.0, zT}, 100, 180, 180);
}

// =========================================================================
// Squid Games FIRST Emblem
// =========================================================================
// Ian: The SmartDashboard icon is a red circle, white/light triangle, and blue
// square — arranged in the style of the Squid Games invitation card symbols.
// We render these as vector outlines on the field floor at center court.
// Field center is now (0, 0) in the centered coordinate system.
//
// Layout (centered at origin, Z=0.01 to prevent z-fighting with grid):
//   Circle (red) — left of center
//   Triangle (white) — center, above
//   Square (blue) — right of center

void TronGridSource::DrawSquidGamesEmblem()
{
	const double cx = 0.0;    // field center X
	const double cy = 0.0;    // field center Y
	const double ez = 0.01;                 // Slightly above floor to avoid z-fighting
	const double emblemSize = 3.0;          // Size of each shape in feet
	const double spacing = 4.0;             // Spacing between shape centers

	// --- Red Circle (left) ---
	{
		const double circCx = cx - spacing;
		const double circCy = cy;
		const double radius = emblemSize * 0.45;
		const int segments = 24;
		for (int i = 0; i < segments; ++i)
		{
			const double a0 = (i * 2.0 * M_PI) / segments;
			const double a1 = ((i + 1) * 2.0 * M_PI) / segments;
			Vec3 p0 = {circCx + radius * std::cos(a0), circCy + radius * std::sin(a0), ez};
			Vec3 p1 = {circCx + radius * std::cos(a1), circCy + radius * std::sin(a1), ez};
			DrawLine3D(p0, p1, 255, 50, 50);  // Red
		}
	}

	// --- White Triangle (center, above) ---
	{
		const double triCx = cx;
		const double triCy = cy;
		const double r = emblemSize * 0.5;  // Circumradius
		// Equilateral triangle pointing up (+Y in world = "up" on the floor plane)
		Vec3 top    = {triCx,                     triCy + r,              ez};
		Vec3 left   = {triCx - r * std::cos(M_PI / 6.0), triCy - r * std::sin(M_PI / 6.0), ez};
		Vec3 right  = {triCx + r * std::cos(M_PI / 6.0), triCy - r * std::sin(M_PI / 6.0), ez};
		DrawLine3D(top, left, 240, 240, 240);    // White
		DrawLine3D(left, right, 240, 240, 240);
		DrawLine3D(right, top, 240, 240, 240);
	}

	// --- Blue Square (right) ---
	{
		const double sqCx = cx + spacing;
		const double sqCy = cy;
		const double half = emblemSize * 0.4;
		Vec3 bl = {sqCx - half, sqCy - half, ez};
		Vec3 br = {sqCx + half, sqCy - half, ez};
		Vec3 tr = {sqCx + half, sqCy + half, ez};
		Vec3 tl = {sqCx - half, sqCy + half, ez};
		DrawLine3D(bl, br, 50, 80, 255);  // Blue
		DrawLine3D(br, tr, 50, 80, 255);
		DrawLine3D(tr, tl, 50, 80, 255);
		DrawLine3D(tl, bl, 50, 80, 255);
	}
}

// =========================================================================
// Background Clouds
// =========================================================================
// Ian: Distant geometric square/rectangular shapes floating in the black void,
// inspired by the scene where Flynn falls into the digital world in Tron (1982).
// These are dim wireframe rectangles at various depths beyond the field,
// slowly drifting to give a sense of scale and depth.  They're drawn first
// (farthest from camera) so the grid and walls paint over them.
//
// Field is centered at origin.  Clouds placed 20-50 ft beyond field edges
// so they stay within the fog draw distance (~60 ft from camera at center).
// Colors are brighter than before so they survive depth fog attenuation.

void TronGridSource::DrawBackgroundClouds(int frameIndex)
{
	struct CloudDef
	{
		double cx, cy, cz;     // center in world coords
		double width, height;  // size of the rectangle
		double driftSpeed;     // radians per frame for gentle rotation
		uint8_t r, g, b;      // color (should be visible after fog)
	};

	// Ian: Clouds placed 25-50 ft from origin at various heights.
	// Fog maxes at 60 ft, so these need to be within ~50 ft of the camera
	// (which starts at origin) to be visible.  Colors are bright enough
	// that even after fog attenuation they'll show as dim shapes.
	static const CloudDef clouds[] =
	{
		// North clouds (beyond far wall at Y=+13.5)
		{  0.0,  30.0, 20.0,  10.0,  6.0, 0.0003,   0, 100, 140},
		{-15.0,  35.0, 28.0,   7.0, 10.0, 0.0005,   0,  80, 120},
		{ 18.0,  32.0, 15.0,  12.0,  5.0, 0.0002,   0,  90, 130},

		// South clouds (behind near wall at Y=-13.5)
		{  5.0, -28.0, 18.0,   8.0,  8.0, 0.0004,   0,  85, 125},
		{-12.0, -32.0, 25.0,  11.0,  6.0, 0.0006,   0,  75, 115},

		// East/West clouds (beyond side walls at X=±27)
		{-38.0,   0.0, 22.0,   8.0,  9.0, 0.0003,   0,  80, 120},
		{ 40.0,   3.0, 30.0,   9.0,  7.0, 0.0005,   0,  90, 130},

		// Higher distant clouds
		{  0.0,  45.0, 35.0,  14.0,  8.0, 0.0001,   0,  70, 110},
		{-25.0,  20.0, 32.0,  10.0,  7.0, 0.0002,   0,  75, 115},
		{ 30.0, -15.0, 26.0,  12.0,  6.0, 0.0004,   0,  80, 120},

		// Closer smaller clouds for visual density
		{ 10.0,  22.0, 12.0,   5.0,  3.0, 0.0007,   0, 110, 150},
		{-20.0, -18.0, 10.0,   4.0,  4.0, 0.0006,   0, 100, 140},
	};

	const int numClouds = static_cast<int>(sizeof(clouds) / sizeof(clouds[0]));

	for (int i = 0; i < numClouds; ++i)
	{
		const CloudDef& c = clouds[i];

		// Gentle drift: rotate the rectangle slightly over time
		const double angle = c.driftSpeed * frameIndex;
		const double cosA = std::cos(angle);
		const double sinA = std::sin(angle);

		// Half-extents
		const double hw = c.width * 0.5;
		const double hh = c.height * 0.5;

		// Four corners in local space (XZ plane at the cloud's Y position),
		// rotated by the drift angle around the cloud center.
		// The rectangle floats in the XZ plane (horizontal in world space).
		Vec3 corners[4];
		const double localX[4] = {-hw,  hw,  hw, -hw};
		const double localZ[4] = {-hh, -hh,  hh,  hh};

		for (int j = 0; j < 4; ++j)
		{
			// Rotate in the XY plane (world horizontal) around center
			const double rx = cosA * localX[j] - sinA * localZ[j];
			const double ry = sinA * localX[j] + cosA * localZ[j];
			corners[j] = {c.cx + rx, c.cy + ry, c.cz};
		}

		// Draw the four edges of the rectangle
		for (int j = 0; j < 4; ++j)
		{
			DrawLine3D(corners[j], corners[(j + 1) % 4], c.r, c.g, c.b);
		}
	}
}

// =========================================================================
// MCP Tower
// =========================================================================
// Ian: The Master Control Program from Tron — rendered as a tall red wireframe
// column far to the north (beyond the far wall).  Visible when looking in the
// heading=0 direction (toward +Y / far wall).  It's a menacing red rectangular
// monolith, like the MCP's spinning column in the movie.
//
// Field is centered.  Far wall is at Y=+13.5.  MCP placed at Y=40 so it's
// ~26 ft beyond the far wall — within fog range from field center (~40 ft).

void TronGridSource::DrawMCPTower()
{
	// Ian: MCP tower position — beyond far (north) wall, centered on X
	const double mcpX = 0.0;         // field center X
	const double mcpY = 40.0;        // 40 ft north — ~26 ft beyond far wall, within fog range
	const double mcpBaseZ = 0.0;     // Ground level
	const double mcpTopZ = 40.0;     // 40 ft tall (visible above walls)
	const double mcpHalfW = 5.0;     // 10 ft wide
	const double mcpHalfD = 2.0;     // 4 ft deep

	// MCP color: ominous red
	const uint8_t mr = 180, mg = 20, mb = 20;
	// Dimmer red for the base/distant edges
	const uint8_t dr = 120, dg = 10, db = 10;

	// Front face (closer to field)
	const double frontY = mcpY - mcpHalfD;
	const double backY  = mcpY + mcpHalfD;

	// Front face — 4 edges
	DrawLine3D({mcpX - mcpHalfW, frontY, mcpBaseZ}, {mcpX + mcpHalfW, frontY, mcpBaseZ}, mr, mg, mb);
	DrawLine3D({mcpX - mcpHalfW, frontY, mcpTopZ},  {mcpX + mcpHalfW, frontY, mcpTopZ},  mr, mg, mb);
	DrawLine3D({mcpX - mcpHalfW, frontY, mcpBaseZ}, {mcpX - mcpHalfW, frontY, mcpTopZ},  mr, mg, mb);
	DrawLine3D({mcpX + mcpHalfW, frontY, mcpBaseZ}, {mcpX + mcpHalfW, frontY, mcpTopZ},  mr, mg, mb);

	// Back face — 4 edges (dimmer, farther away)
	DrawLine3D({mcpX - mcpHalfW, backY, mcpBaseZ}, {mcpX + mcpHalfW, backY, mcpBaseZ}, dr, dg, db);
	DrawLine3D({mcpX - mcpHalfW, backY, mcpTopZ},  {mcpX + mcpHalfW, backY, mcpTopZ},  dr, dg, db);
	DrawLine3D({mcpX - mcpHalfW, backY, mcpBaseZ}, {mcpX - mcpHalfW, backY, mcpTopZ},  dr, dg, db);
	DrawLine3D({mcpX + mcpHalfW, backY, mcpBaseZ}, {mcpX + mcpHalfW, backY, mcpTopZ},  dr, dg, db);

	// Connecting edges (front to back) — 4 horizontal edges
	DrawLine3D({mcpX - mcpHalfW, frontY, mcpBaseZ}, {mcpX - mcpHalfW, backY, mcpBaseZ}, dr, dg, db);
	DrawLine3D({mcpX + mcpHalfW, frontY, mcpBaseZ}, {mcpX + mcpHalfW, backY, mcpBaseZ}, dr, dg, db);
	DrawLine3D({mcpX - mcpHalfW, frontY, mcpTopZ},  {mcpX - mcpHalfW, backY, mcpTopZ},  mr, mg, mb);
	DrawLine3D({mcpX + mcpHalfW, frontY, mcpTopZ},  {mcpX + mcpHalfW, backY, mcpTopZ},  mr, mg, mb);

	// Horizontal stripe bands across the front face for visual interest
	// (like the MCP's glowing bands)
	const int numBands = 5;
	for (int i = 1; i <= numBands; ++i)
	{
		const double bandZ = mcpBaseZ + (mcpTopZ - mcpBaseZ) * (static_cast<double>(i) / (numBands + 1));
		// Alternating brightness for the bands
		const uint8_t br = (i % 2 == 0) ? 220 : 160;
		const uint8_t bg = (i % 2 == 0) ? 30  : 15;
		const uint8_t bb = (i % 2 == 0) ? 30  : 15;
		DrawLine3D({mcpX - mcpHalfW, frontY, bandZ}, {mcpX + mcpHalfW, frontY, bandZ}, br, bg, bb);
	}
}

// =========================================================================
// HUD (Heads-Up Display)
// =========================================================================
// Ian: All text removed per user request — pure black background with no burn-in.
// Only the CRT scanline effect remains for retro aesthetic.

void TronGridSource::DrawHUD(int /*frameIndex*/)
{
	// Ian: No text drawn — user wants pure black background with no burn-in.
	// Keeping the scanline pass only for that classic CRT feel.

	// Subtle scanline effect — horizontal lines every 4 pixels for CRT feel
	// Ian: Very light dimming, doesn't obscure the scene.  Just adds texture.
	for (int y = 0; y < m_height; y += 4)
	{
		const size_t rowStart = static_cast<size_t>(y) * m_width * 3;
		for (int x = 0; x < m_width; ++x)
		{
			const size_t idx = rowStart + static_cast<size_t>(x) * 3;
			// Dim by ~15%
			m_pixelBuffer[idx + 0] = static_cast<uint8_t>(m_pixelBuffer[idx + 0] * 0.85);
			m_pixelBuffer[idx + 1] = static_cast<uint8_t>(m_pixelBuffer[idx + 1] * 0.85);
			m_pixelBuffer[idx + 2] = static_cast<uint8_t>(m_pixelBuffer[idx + 2] * 0.85);
		}
	}
}

// =========================================================================
// Off-Field Arrow
// =========================================================================
// Ian: When the robot is outside the field bounds, draw a screen-space arrow
// pointing in the direction of the field center.

void TronGridSource::DrawOffFieldArrow()
{
	// Ian: Field center is now (0,0) in the centered coordinate system.
	const double fieldCenterX = 0.0;
	const double fieldCenterY = 0.0;

	// Direction from robot to field center, in world coords
	double toFieldX = fieldCenterX - m_camX;
	double toFieldY = fieldCenterY - m_camY;
	double dist = std::sqrt(toFieldX * toFieldX + toFieldY * toFieldY);
	if (dist < 0.01) return;
	toFieldX /= dist;
	toFieldY /= dist;

	// Convert to camera-relative direction (just the XY angle)
	const double headingRad = m_headingDeg * M_PI / 180.0;
	const double cosH = std::cos(headingRad);
	const double sinH = std::sin(headingRad);

	// Camera-space direction (X=right, Z=forward)
	const double camDirX = cosH * toFieldX + sinH * toFieldY;
	const double camDirZ = -sinH * toFieldX + cosH * toFieldY;

	// Screen-space angle
	const double screenAngle = std::atan2(-camDirX, camDirZ);

	// Draw arrow at screen center
	const int acx = m_width / 2;
	const int acy = m_height / 2;
	const int arrowLen = 40;
	const int tipX = acx + static_cast<int>(arrowLen * std::sin(-screenAngle));
	const int tipY = acy - static_cast<int>(arrowLen * std::cos(-screenAngle));

	// Arrow shaft
	DrawLine(acx, acy, tipX, tipY, 255, 200, 0);  // Orange-yellow

	// Arrowhead (two short lines from tip, angled back)
	const double headAngle = 0.4;  // radians, ~23 degrees
	const int headLen = 12;
	for (int side = -1; side <= 1; side += 2)
	{
		const double ha = -screenAngle + M_PI + side * headAngle;
		const int hx = tipX + static_cast<int>(headLen * std::sin(ha));
		const int hy = tipY - static_cast<int>(headLen * std::cos(ha));
		DrawLine(tipX, tipY, hx, hy, 255, 200, 0);
	}

	// "OFF FIELD" text below arrow
	DrawString(acx - 30, acy + 50, "OFF FIELD", 255, 200, 0);
}

// =========================================================================
// 3D Line Drawing with Depth Fog
// =========================================================================
// Ian: This is the core rendering function.  Takes two world-space points,
// transforms to camera space, clips against the near plane, projects to
// screen, then clips to screen bounds using Cohen-Sutherland, and draws
// with Bresenham.  Color is attenuated by distance.

void TronGridSource::DrawLine3D(const Vec3& a, const Vec3& b, uint8_t r, uint8_t g, uint8_t b_color)
{
	// Transform to camera space
	Vec3 camA = WorldToCamera(a);
	Vec3 camB = WorldToCamera(b);

	// Clip against near plane
	if (!ClipLineNearPlane(camA, camB, 0.2))
		return;  // Entire line is behind camera

	// Project to screen
	int sx0, sy0, sx1, sy1;
	if (!CameraToScreen(camA, sx0, sy0))
		return;
	if (!CameraToScreen(camB, sx1, sy1))
		return;

	// Ian: Cohen-Sutherland 2D line clipping against the screen rectangle.
	// This properly handles lines that are partially or fully outside the FOV
	// by clipping them to the visible screen area before drawing.  Without this,
	// lines with one endpoint far offscreen produce absurdly long Bresenham runs.
	if (!ClipLineToScreen(sx0, sy0, sx1, sy1))
		return;  // Entire line is off-screen

	// Depth fog: attenuate color based on average distance from camera
	const double avgDist = (camA.z + camB.z) * 0.5;
	const double maxFogDist = 60.0;  // Lines fully dim at 60 feet
	const uint8_t fr = FogColor(r, static_cast<uint8_t>(r / 6), avgDist, maxFogDist);
	const uint8_t fg = FogColor(g, static_cast<uint8_t>(g / 6), avgDist, maxFogDist);
	const uint8_t fb = FogColor(b_color, static_cast<uint8_t>(b_color / 6), avgDist, maxFogDist);

	// Don't draw lines that have fogged to near-black
	if (fr < 3 && fg < 3 && fb < 3)
		return;

	DrawLine(sx0, sy0, sx1, sy1, fr, fg, fb);
}

// =========================================================================
// Pixel-level Drawing (same as SimCameraSource)
// =========================================================================

void TronGridSource::SetPixel(int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
	if (x < 0 || x >= m_width || y < 0 || y >= m_height)
		return;
	const size_t idx = (static_cast<size_t>(y) * m_width + x) * 3;
	m_pixelBuffer[idx + 0] = r;
	m_pixelBuffer[idx + 1] = g;
	m_pixelBuffer[idx + 2] = b;
}

void TronGridSource::DrawLine(int x0, int y0, int x1, int y1, uint8_t r, uint8_t g, uint8_t b)
{
	int dx = x1 - x0;
	int dy = y1 - y0;
	const int sx = (dx >= 0) ? 1 : -1;
	const int sy = (dy >= 0) ? 1 : -1;
	dx = (dx >= 0) ? dx : -dx;
	dy = (dy >= 0) ? dy : -dy;

	// Ian: Safety limit — if the line is absurdly long (bad projection),
	// bail out to avoid locking up the frame.
	if (dx + dy > 5000)
		return;

	int err = dx - dy;

	while (true)
	{
		SetPixel(x0, y0, r, g, b);
		if (x0 == x1 && y0 == y1)
			break;
		const int e2 = 2 * err;
		if (e2 > -dy)
		{
			err -= dy;
			x0 += sx;
		}
		if (e2 < dx)
		{
			err += dx;
			y0 += sy;
		}
	}
}

void TronGridSource::DrawRect(int x, int y, int w, int h, uint8_t r, uint8_t g, uint8_t b)
{
	DrawLine(x, y, x + w, y, r, g, b);
	DrawLine(x + w, y, x + w, y + h, r, g, b);
	DrawLine(x + w, y + h, x, y + h, r, g, b);
	DrawLine(x, y + h, x, y, r, g, b);
}

void TronGridSource::DrawChar(int x, int y, char ch, uint8_t r, uint8_t g, uint8_t b, int scale)
{
	if (ch >= 'a' && ch <= 'z')
		ch = ch - 'a' + 'A';

	const int idx = static_cast<int>(ch) - kFontFirstChar;
	if (idx < 0 || idx >= static_cast<int>(sizeof(kFontData) / sizeof(kFontData[0])))
		return;

	const uint8_t* glyph = kFontData[idx];
	for (int col = 0; col < kGlyphWidth; ++col)
	{
		const uint8_t colData = glyph[col];
		for (int row = 0; row < kGlyphHeight; ++row)
		{
			if (colData & (1 << row))
			{
				for (int sy = 0; sy < scale; ++sy)
				{
					for (int sx = 0; sx < scale; ++sx)
					{
						SetPixel(x + col * scale + sx, y + row * scale + sy, r, g, b);
					}
				}
			}
		}
	}
}

void TronGridSource::DrawString(int x, int y, const char* str, uint8_t r, uint8_t g, uint8_t b, int scale)
{
	while (*str)
	{
		DrawChar(x, y, *str, r, g, b, scale);
		x += (kGlyphWidth + 1) * scale;
		++str;
	}
}
