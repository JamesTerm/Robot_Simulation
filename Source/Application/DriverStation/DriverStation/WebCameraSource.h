#pragma once
// Ian: WebCameraSource captures frames from a USB webcam using Video for Windows
// (VFW / capCreateCaptureWindow) and pushes JPEG-encoded frames to the MjpegServer.
//
// Architecture:
//   - Uses a hidden capture window (capCreateCaptureWindow) to grab frames from
//     the default video capture device (index 0).
//   - A worker thread runs a message pump and a timed polling loop that calls
//     capGrabFrameNoStop to request frames at the target framerate.  Each grab
//     triggers the capSetCallbackOnFrame callback synchronously, which converts
//     the frame to top-down RGB (from YUY2 or bottom-up BGR), JPEG-encodes via
//     stb_image_write, and pushes to the MjpegServer.
//   - Resolution defaults to 320x240 @ 15fps to match SimCameraSource.
//
// Ian: VFW is the simplest Windows camera API — just a few Win32 calls, no COM,
// no Media Foundation boilerplate.  It works with virtually all USB webcams on
// Windows 7+.  The capture window must live on a thread with a message pump,
// which is why we create it on the worker thread rather than on the main UI thread.
//
// Ian: LESSON LEARNED — capPreview(TRUE) + capSetCallbackOnFrame relies on the
// window processing WM_PAINT to trigger callbacks.  Hidden windows never get
// WM_PAINT, so no frames arrive.  Use capGrabFrameNoStop in a polling loop
// instead — it triggers the callback synchronously regardless of window visibility.

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>
#include <windows.h>
#include <vfw.h>
#pragma comment(lib, "vfw32.lib")

class MjpegServer;

class WebCameraSource
{
public:
	WebCameraSource();
	~WebCameraSource();

	// Ian: Start/Stop control the capture thread.
	// The MjpegServer pointer is stored (non-owning) — caller must ensure
	// the MjpegServer outlives the WebCameraSource (or call Stop() first).
	void Start(MjpegServer* server, int width = 320, int height = 240, int targetFps = 15);
	void Stop();

	bool IsRunning() const;
	int GetFrameCount() const;

	// Ian: Wait for the worker thread to finish startup (driver connect attempt).
	// Returns true if the camera connected successfully, false if it failed or
	// timed out.  Call this after Start() to detect "no camera" scenarios.
	// Typical timeout: 2000ms (VFW driver probing can take ~1s on some systems).
	bool WaitForStartup(int timeoutMs = 2000);

private:
	void WorkerThread();

	// Ian: VFW frame callback — called on the capture thread when a frame arrives.
	// We use the lParam trick to pass 'this' via capSetUserData / capGetUserData.
	static LRESULT CALLBACK FrameCallbackProc(HWND hWnd, LPVIDEOHDR lpVHdr);
	void OnFrame(const uint8_t* data, int dataSize, int width, int height,
		int bitsPerPixel, DWORD compression, bool topDown);

	MjpegServer* m_server = nullptr;
	std::thread m_workerThread;
	std::atomic<bool> m_running{false};
	std::atomic<bool> m_stopRequested{false};
	std::atomic<int> m_frameCount{0};

	// Ian: Startup synchronization — lets the caller wait for the worker thread
	// to finish its capDriverConnect attempt and know if it succeeded.
	std::mutex m_startupMutex;
	std::condition_variable m_startupCv;
	bool m_startupDone = false;     // protected by m_startupMutex
	bool m_startupSuccess = false;  // protected by m_startupMutex

	int m_requestedWidth = 320;
	int m_requestedHeight = 240;
	int m_targetFps = 15;

	// Ian: Reusable buffers to avoid per-frame allocation.
	std::vector<uint8_t> m_rgbBuffer;     // Flipped/converted RGB for JPEG encoding
	std::vector<uint8_t> m_jpegBuffer;    // JPEG output
	std::mutex m_bufferMutex;             // Protects m_rgbBuffer and m_jpegBuffer
};
