// Ian: SimCameraSource.cpp — Synthetic camera frame generator.
// See SimCameraSource.h for architecture notes.
//
// This file contains:
//   1. Worker thread loop (frame generation + JPEG encoding + push to MjpegServer)
//   2. Synthetic test pattern renderer (black bg, rotating line, crosshair, text)
//   3. Minimal 5x7 bitmap font for rendering frame counter / label text
//   4. Bresenham line drawing for the radar sweep and crosshair

// Ian: STB_IMAGE_WRITE_IMPLEMENTATION must be defined in exactly ONE .cpp file.
// We define it here because SimCameraSource is the only consumer of JPEG encoding.
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include "SimCameraSource.h"
#include "MjpegServer.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>

#ifdef _WIN32
#include <Windows.h>  // OutputDebugStringA
#endif

// Ian: Pi constant — MSVC's _USE_MATH_DEFINES + <cmath> is unreliable across
// translation units.  Just define it.
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

	// Ian: stb_image_write callback for writing JPEG data to a std::vector.
	// stbi_write_jpg_to_func calls this potentially multiple times as it
	// encodes the image.  We append each chunk to the output vector.
	void StbWriteCallback(void* context, void* data, int size)
	{
		auto* output = static_cast<std::vector<uint8_t>*>(context);
		const auto* bytes = static_cast<const uint8_t*>(data);
		output->insert(output->end(), bytes, bytes + size);
	}

	// =========================================================================
	// Minimal 5x7 bitmap font
	// =========================================================================
	// Ian: Each character is encoded as 5 columns of 7 bits each (LSB = top row).
	// This is a standard format for tiny embedded fonts.  We only define the
	// printable ASCII range we actually use: digits 0-9, letters A-Z (upper),
	// a few punctuation marks, and space.
	//
	// Glyph data: 5 bytes per character, each byte is a column (7 bits used).

	// Font covers ASCII 32 (' ') through 90 ('Z') = 59 characters
	static constexpr int kFontFirstChar = 32;   // ' '
	static constexpr int kFontLastChar = 90;    // 'Z'
	static constexpr int kGlyphWidth = 5;
	static constexpr int kGlyphHeight = 7;

	// clang-format off
	static const uint8_t kFontData[][5] =
	{
		// ' ' (32)
		{0x00, 0x00, 0x00, 0x00, 0x00},
		// '!' (33)
		{0x00, 0x00, 0x5F, 0x00, 0x00},
		// '"' (34)
		{0x00, 0x07, 0x00, 0x07, 0x00},
		// '#' (35)
		{0x14, 0x7F, 0x14, 0x7F, 0x14},
		// '$' (36)
		{0x24, 0x2A, 0x7F, 0x2A, 0x12},
		// '%' (37)
		{0x23, 0x13, 0x08, 0x64, 0x62},
		// '&' (38)
		{0x36, 0x49, 0x55, 0x22, 0x50},
		// ''' (39)
		{0x00, 0x05, 0x03, 0x00, 0x00},
		// '(' (40)
		{0x00, 0x1C, 0x22, 0x41, 0x00},
		// ')' (41)
		{0x00, 0x41, 0x22, 0x1C, 0x00},
		// '*' (42)
		{0x14, 0x08, 0x3E, 0x08, 0x14},
		// '+' (43)
		{0x08, 0x08, 0x3E, 0x08, 0x08},
		// ',' (44)
		{0x00, 0x50, 0x30, 0x00, 0x00},
		// '-' (45)
		{0x08, 0x08, 0x08, 0x08, 0x08},
		// '.' (46)
		{0x00, 0x60, 0x60, 0x00, 0x00},
		// '/' (47)
		{0x20, 0x10, 0x08, 0x04, 0x02},
		// '0' (48)
		{0x3E, 0x51, 0x49, 0x45, 0x3E},
		// '1' (49)
		{0x00, 0x42, 0x7F, 0x40, 0x00},
		// '2' (50)
		{0x42, 0x61, 0x51, 0x49, 0x46},
		// '3' (51)
		{0x21, 0x41, 0x45, 0x4B, 0x31},
		// '4' (52)
		{0x18, 0x14, 0x12, 0x7F, 0x10},
		// '5' (53)
		{0x27, 0x45, 0x45, 0x45, 0x39},
		// '6' (54)
		{0x3C, 0x4A, 0x49, 0x49, 0x30},
		// '7' (55)
		{0x01, 0x71, 0x09, 0x05, 0x03},
		// '8' (56)
		{0x36, 0x49, 0x49, 0x49, 0x36},
		// '9' (57)
		{0x06, 0x49, 0x49, 0x29, 0x1E},
		// ':' (58)
		{0x00, 0x36, 0x36, 0x00, 0x00},
		// ';' (59)
		{0x00, 0x56, 0x36, 0x00, 0x00},
		// '<' (60)
		{0x08, 0x14, 0x22, 0x41, 0x00},
		// '=' (61)
		{0x14, 0x14, 0x14, 0x14, 0x14},
		// '>' (62)
		{0x00, 0x41, 0x22, 0x14, 0x08},
		// '?' (63)
		{0x02, 0x01, 0x51, 0x09, 0x06},
		// '@' (64)
		{0x32, 0x49, 0x79, 0x41, 0x3E},
		// 'A' (65)
		{0x7E, 0x11, 0x11, 0x11, 0x7E},
		// 'B' (66)
		{0x7F, 0x49, 0x49, 0x49, 0x36},
		// 'C' (67)
		{0x3E, 0x41, 0x41, 0x41, 0x22},
		// 'D' (68)
		{0x7F, 0x41, 0x41, 0x22, 0x1C},
		// 'E' (69)
		{0x7F, 0x49, 0x49, 0x49, 0x41},
		// 'F' (70)
		{0x7F, 0x09, 0x09, 0x09, 0x01},
		// 'G' (71)
		{0x3E, 0x41, 0x49, 0x49, 0x7A},
		// 'H' (72)
		{0x7F, 0x08, 0x08, 0x08, 0x7F},
		// 'I' (73)
		{0x00, 0x41, 0x7F, 0x41, 0x00},
		// 'J' (74)
		{0x20, 0x40, 0x41, 0x3F, 0x01},
		// 'K' (75)
		{0x7F, 0x08, 0x14, 0x22, 0x41},
		// 'L' (76)
		{0x7F, 0x40, 0x40, 0x40, 0x40},
		// 'M' (77)
		{0x7F, 0x02, 0x0C, 0x02, 0x7F},
		// 'N' (78)
		{0x7F, 0x04, 0x08, 0x10, 0x7F},
		// 'O' (79)
		{0x3E, 0x41, 0x41, 0x41, 0x3E},
		// 'P' (80)
		{0x7F, 0x09, 0x09, 0x09, 0x06},
		// 'Q' (81)
		{0x3E, 0x41, 0x51, 0x21, 0x5E},
		// 'R' (82)
		{0x7F, 0x09, 0x19, 0x29, 0x46},
		// 'S' (83)
		{0x46, 0x49, 0x49, 0x49, 0x31},
		// 'T' (84)
		{0x01, 0x01, 0x7F, 0x01, 0x01},
		// 'U' (85)
		{0x3F, 0x40, 0x40, 0x40, 0x3F},
		// 'V' (86)
		{0x1F, 0x20, 0x40, 0x20, 0x1F},
		// 'W' (87)
		{0x3F, 0x40, 0x38, 0x40, 0x3F},
		// 'X' (88)
		{0x63, 0x14, 0x08, 0x14, 0x63},
		// 'Y' (89)
		{0x07, 0x08, 0x70, 0x08, 0x07},
		// 'Z' (90)
		{0x61, 0x51, 0x49, 0x45, 0x43},
	};
	// clang-format on
}

SimCameraSource::SimCameraSource() = default;

SimCameraSource::~SimCameraSource()
{
	Stop();
}

void SimCameraSource::Start(MjpegServer* server, int width, int height, int targetFps)
{
	Stop();

	m_server = server;
	m_width = width;
	m_height = height;
	m_targetFps = targetFps;
	m_frameCount.store(0);
	m_pixelBuffer.resize(static_cast<size_t>(m_width) * m_height * 3, 0);

	m_running.store(true);
	m_workerThread = std::thread(&SimCameraSource::WorkerThread, this);

	DebugLog("[SimCameraSource] Started: %dx%d @ %d fps\n", m_width, m_height, m_targetFps);
}

void SimCameraSource::Stop()
{
	m_running.store(false);
	if (m_workerThread.joinable())
	{
		m_workerThread.join();
	}
	m_server = nullptr;
	DebugLog("[SimCameraSource] Stopped (total frames: %d)\n", m_frameCount.load());
}

bool SimCameraSource::IsRunning() const
{
	return m_running.load();
}

int SimCameraSource::GetFrameCount() const
{
	return m_frameCount.load();
}

void SimCameraSource::WorkerThread()
{
	// Ian: Frame timing.  We target m_targetFps but allow the actual interval
	// to vary based on how long rendering + encoding takes.  sleep_until gives
	// us consistent frame timing even if individual frames take varying amounts
	// of time.
	const auto frameDuration = std::chrono::microseconds(1000000 / m_targetFps);
	auto nextFrameTime = std::chrono::steady_clock::now();
	int frameIndex = 0;

	while (m_running.load())
	{
		// Render the synthetic frame
		RenderFrame(frameIndex);

		// JPEG encode via stb_image_write
		std::vector<uint8_t> jpegOutput;
		jpegOutput.reserve(32 * 1024);  // Pre-allocate ~32KB (typical for 320x240 JPEG)

		// Ian: Quality 75 is a good balance for FRC camera streams.
		// Higher quality = larger frames = more bandwidth.  At 320x240
		// this produces ~10-20KB per frame, well within typical network capacity.
		const int quality = 75;
		int result = stbi_write_jpg_to_func(
			StbWriteCallback,
			&jpegOutput,
			m_width,
			m_height,
			3,  // RGB components
			m_pixelBuffer.data(),
			quality);

		if (result != 0 && m_server != nullptr)
		{
			m_server->PushFrame(jpegOutput);
			m_frameCount.fetch_add(1);
		}

		++frameIndex;

		// Sleep until the next frame time
		nextFrameTime += frameDuration;
		auto now = std::chrono::steady_clock::now();
		if (nextFrameTime > now)
		{
			std::this_thread::sleep_until(nextFrameTime);
		}
		else
		{
			// Ian: We're behind schedule — skip the sleep and render the next
			// frame immediately.  Reset the target time to now to avoid a burst
			// of catch-up frames.
			nextFrameTime = now;
		}
	}
}

// =========================================================================
// Rendering
// =========================================================================

void SimCameraSource::RenderFrame(int frameIndex)
{
	// Clear to dark background (not pure black — slight blue tint for aesthetics)
	const size_t totalBytes = static_cast<size_t>(m_width) * m_height * 3;
	for (size_t i = 0; i < totalBytes; i += 3)
	{
		m_pixelBuffer[i + 0] = 10;   // R
		m_pixelBuffer[i + 1] = 10;   // G
		m_pixelBuffer[i + 2] = 20;   // B (slight blue)
	}

	const int cx = m_width / 2;
	const int cy = m_height / 2;

	// --- Border rectangle ---
	DrawRect(0, 0, m_width - 1, m_height - 1, 40, 80, 40);  // dim green border

	// --- Crosshair at center ---
	// Horizontal line
	DrawLine(cx - 20, cy, cx + 20, cy, 0, 200, 0);  // green
	// Vertical line
	DrawLine(cx, cy - 20, cx, cy + 20, 0, 200, 0);  // green

	// --- Rotating radar sweep line ---
	// Ian: The angle advances with each frame.  At 15fps, one full rotation
	// takes 360/3 = 120 frames = 8 seconds.  Adjust the divisor to change speed.
	const double angle = (frameIndex * 3.0) * M_PI / 180.0;
	const int sweepLen = (m_width < m_height ? m_width : m_height) / 2 - 10;
	const int ex = cx + static_cast<int>(sweepLen * std::cos(angle));
	const int ey = cy + static_cast<int>(sweepLen * std::sin(angle));
	DrawLine(cx, cy, ex, ey, 0, 255, 0);  // bright green

	// --- "SIM CAM" label at top ---
	DrawString(5, 5, "SIM CAM", 200, 200, 200, 2);

	// --- Frame counter at bottom-left ---
	char frameBuf[64];
	snprintf(frameBuf, sizeof(frameBuf), "FRAME %d", frameIndex);
	DrawString(5, m_height - 14, frameBuf, 180, 180, 0);  // yellow-ish

	// --- FPS indicator at top-right ---
	char fpsBuf[32];
	snprintf(fpsBuf, sizeof(fpsBuf), "%dFPS", m_targetFps);
	// Right-align: each char is (kGlyphWidth+1)*scale pixels wide at scale 1
	const int fpsTextWidth = static_cast<int>(strlen(fpsBuf)) * (kGlyphWidth + 1);
	DrawString(m_width - fpsTextWidth - 5, 5, fpsBuf, 180, 180, 180);

	// --- Resolution text at bottom-right ---
	char resBuf[32];
	snprintf(resBuf, sizeof(resBuf), "%dX%d", m_width, m_height);
	const int resTextWidth = static_cast<int>(strlen(resBuf)) * (kGlyphWidth + 1);
	DrawString(m_width - resTextWidth - 5, m_height - 14, resBuf, 120, 120, 120);

	// --- Concentric range rings around center ---
	// Ian: Simple circle approximation using line segments.
	// Draws rings at 1/4 and 1/2 of the sweep radius.
	for (int ring = 1; ring <= 2; ++ring)
	{
		const int radius = (sweepLen * ring) / 2;
		const int segments = 36;
		for (int s = 0; s < segments; ++s)
		{
			const double a0 = (s * 2.0 * M_PI) / segments;
			const double a1 = ((s + 1) * 2.0 * M_PI) / segments;
			const int x0 = cx + static_cast<int>(radius * std::cos(a0));
			const int y0 = cy + static_cast<int>(radius * std::sin(a0));
			const int x1 = cx + static_cast<int>(radius * std::cos(a1));
			const int y1 = cy + static_cast<int>(radius * std::sin(a1));
			DrawLine(x0, y0, x1, y1, 30, 60, 30);  // dim green
		}
	}
}

// =========================================================================
// Pixel-level drawing
// =========================================================================

void SimCameraSource::SetPixel(int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
	if (x < 0 || x >= m_width || y < 0 || y >= m_height)
		return;
	const size_t idx = (static_cast<size_t>(y) * m_width + x) * 3;
	m_pixelBuffer[idx + 0] = r;
	m_pixelBuffer[idx + 1] = g;
	m_pixelBuffer[idx + 2] = b;
}

// Ian: Bresenham's line algorithm — classic integer-only line drawing.
void SimCameraSource::DrawLine(int x0, int y0, int x1, int y1, uint8_t r, uint8_t g, uint8_t b)
{
	int dx = x1 - x0;
	int dy = y1 - y0;
	const int sx = (dx >= 0) ? 1 : -1;
	const int sy = (dy >= 0) ? 1 : -1;
	dx = (dx >= 0) ? dx : -dx;
	dy = (dy >= 0) ? dy : -dy;

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

void SimCameraSource::DrawRect(int x, int y, int w, int h, uint8_t r, uint8_t g, uint8_t b)
{
	DrawLine(x, y, x + w, y, r, g, b);         // top
	DrawLine(x + w, y, x + w, y + h, r, g, b); // right
	DrawLine(x + w, y + h, x, y + h, r, g, b); // bottom
	DrawLine(x, y + h, x, y, r, g, b);         // left
}

void SimCameraSource::DrawChar(int x, int y, char ch, uint8_t r, uint8_t g, uint8_t b, int scale)
{
	// Convert to uppercase for our limited font range
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
				// Fill a scale x scale block
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

void SimCameraSource::DrawString(int x, int y, const char* str, uint8_t r, uint8_t g, uint8_t b, int scale)
{
	while (*str)
	{
		DrawChar(x, y, *str, r, g, b, scale);
		x += (kGlyphWidth + 1) * scale;  // 1 pixel gap between characters
		++str;
	}
}
