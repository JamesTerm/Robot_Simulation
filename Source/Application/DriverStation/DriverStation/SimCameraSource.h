#pragma once
// Ian: SimCameraSource generates JPEG frames for the MJPEG server.
// Default mode: synthetic vector graphics on a black background (works on any
// machine without camera hardware).  Future mode: USB camera + overlay.
//
// Architecture:
//   - Owns a worker thread that generates frames at a target FPS.
//   - Each frame is rendered into an RGB pixel buffer, JPEG-encoded via
//     stb_image_write, and pushed to the MjpegServer via PushFrame().
//   - The synthetic test pattern includes:
//       * Black background
//       * Rotating "radar sweep" line (verifies animation is working)
//       * Frame counter and timestamp text (verifies data is updating)
//       * A crosshair at center (verifies pixel-level rendering)
//   - Resolution is 320x240 by default (matches typical FRC camera).
//
// Ian: stb_image_write's JPEG encoder writes via a callback function.
// We use stbi_write_jpg_to_func to write directly into a std::vector<uint8_t>
// buffer, avoiding temporary files.

#include <atomic>
#include <cstdint>
#include <thread>
#include <vector>

class MjpegServer;

class SimCameraSource
{
public:
	SimCameraSource();
	~SimCameraSource();

	// Ian: Start/Stop control the frame generation thread.
	// The MjpegServer pointer is stored (non-owning) — caller must ensure
	// the MjpegServer outlives the SimCameraSource (or call Stop() first).
	void Start(MjpegServer* server, int width = 320, int height = 240, int targetFps = 15);
	void Stop();

	bool IsRunning() const;
	int GetFrameCount() const;

private:
	void WorkerThread();

	// Ian: Render the synthetic test pattern into m_pixelBuffer.
	// frameIndex drives the animation (rotating line angle, etc.).
	void RenderFrame(int frameIndex);

	// Ian: Simple pixel-level drawing helpers.  No font library needed —
	// we draw numbers/text using hardcoded 5x7 bitmap font glyphs.
	void SetPixel(int x, int y, uint8_t r, uint8_t g, uint8_t b);
	void DrawLine(int x0, int y0, int x1, int y1, uint8_t r, uint8_t g, uint8_t b);
	void DrawRect(int x, int y, int w, int h, uint8_t r, uint8_t g, uint8_t b);
	void DrawChar(int x, int y, char ch, uint8_t r, uint8_t g, uint8_t b, int scale = 1);
	void DrawString(int x, int y, const char* str, uint8_t r, uint8_t g, uint8_t b, int scale = 1);

	MjpegServer* m_server = nullptr;
	std::thread m_workerThread;
	std::atomic<bool> m_running{false};
	std::atomic<int> m_frameCount{0};

	int m_width = 320;
	int m_height = 240;
	int m_targetFps = 15;

	// Ian: RGB pixel buffer, m_width * m_height * 3 bytes.
	// Reused across frames to avoid per-frame allocation.
	std::vector<uint8_t> m_pixelBuffer;
};
