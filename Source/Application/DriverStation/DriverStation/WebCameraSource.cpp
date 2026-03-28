// Ian: WebCameraSource — VFW-based USB webcam capture for the MJPEG server.
//
// LESSON LEARNED: VFW's capCreateCaptureWindow must be created on a thread that
// runs a message pump (GetMessage/DispatchMessage), because WM_CAP_* messages
// are posted to the window.  We create the capture window on our worker thread
// and run a minimal message loop there.
//
// LESSON LEARNED: capPreview(TRUE) + capSetCallbackOnFrame relies on the window
// receiving WM_PAINT messages, which never arrive for hidden windows.  Instead we
// use capGrabFrameNoStop in a timed polling loop — it synchronously triggers the
// frame callback regardless of window visibility.
//
// LESSON LEARNED: Most USB cameras on Windows deliver YUY2 (packed YCbCr 4:2:2)
// through VFW, NOT BI_RGB.  capSetVideoFormat to request RGB24 often fails
// because the VFW driver doesn't support format conversion.  We handle both
// YUY2 and BI_RGB (24/32 bpp) formats in the frame callback.
//
// The frame callback converts the frame data to top-down RGB, JPEG-encodes it,
// and pushes to the MjpegServer.
//
// We do NOT define STB_IMAGE_WRITE_IMPLEMENTATION here — it's already defined
// in SimCameraSource.cpp.  We only need the stb declarations.

#include "stdafx.h"
#include "WebCameraSource.h"
#include "MjpegServer.h"

// Ian: stb_image_write is already compiled in SimCameraSource.cpp with
// STB_IMAGE_WRITE_IMPLEMENTATION.  We only need the declarations here.
#include "stb_image_write.h"

namespace
{
	// Ian: stbi_write callback that appends to a std::vector<uint8_t>.
	// Same pattern as SimCameraSource.
	void StbiWriteCallback(void* context, void* data, int size)
	{
		auto* buffer = static_cast<std::vector<uint8_t>*>(context);
		const auto* bytes = static_cast<const uint8_t*>(data);
		buffer->insert(buffer->end(), bytes, bytes + size);
	}

	// Ian: YUY2 FourCC — 0x32595559 = 'Y','U','Y','2' in little-endian.
	// Every 4 bytes encode 2 pixels: [Y0, U, Y1, V].
	// Each pixel pair shares U,V chrominance.
	constexpr DWORD FOURCC_YUY2 = 0x32595559;

	// Ian: Clamp an int to [0, 255] and return as uint8_t.
	inline uint8_t ClampByte(int v)
	{
		return static_cast<uint8_t>(v < 0 ? 0 : (v > 255 ? 255 : v));
	}

	// Ian: Convert YUY2 (packed YCbCr 4:2:2) to top-down RGB.
	// Every 4 bytes = 2 pixels: [Y0, U, Y1, V].
	// Uses BT.601 conversion with fixed-point arithmetic (<<10):
	//   R = 1.164*(Y-16) + 1.596*(V-128)
	//   G = 1.164*(Y-16) - 0.391*(U-128) - 0.813*(V-128)
	//   B = 1.164*(Y-16) + 2.018*(U-128)
	void ConvertYUY2toRGB(const uint8_t* src, int srcStride,
	                       uint8_t* dst, int width, int height,
	                       bool topDown)
	{
		for (int y = 0; y < height; ++y)
		{
			int srcY = topDown ? y : (height - 1 - y);
			const uint8_t* srcRow = src + static_cast<size_t>(srcY) * srcStride;
			uint8_t* dstRow = dst + static_cast<size_t>(y) * width * 3;

			for (int x = 0; x < width; x += 2)
			{
				uint8_t y0 = srcRow[x * 2 + 0];
				uint8_t u  = srcRow[x * 2 + 1];
				uint8_t y1 = srcRow[x * 2 + 2];
				uint8_t v  = srcRow[x * 2 + 3];

				int C0 = (static_cast<int>(y0) - 16) * 1192;  // 1.164 * 1024
				int C1 = (static_cast<int>(y1) - 16) * 1192;
				int D  = static_cast<int>(u) - 128;
				int E  = static_cast<int>(v) - 128;

				int rAdd = 1634 * E;              // 1.596 * 1024
				int gAdd = -401 * D - 833 * E;    // -0.391, -0.813 * 1024
				int bAdd = 2066 * D;              // 2.018 * 1024

				// Pixel 0
				dstRow[x * 3 + 0] = ClampByte((C0 + rAdd + 512) >> 10);
				dstRow[x * 3 + 1] = ClampByte((C0 + gAdd + 512) >> 10);
				dstRow[x * 3 + 2] = ClampByte((C0 + bAdd + 512) >> 10);

				// Pixel 1
				dstRow[x * 3 + 3] = ClampByte((C1 + rAdd + 512) >> 10);
				dstRow[x * 3 + 4] = ClampByte((C1 + gAdd + 512) >> 10);
				dstRow[x * 3 + 5] = ClampByte((C1 + bAdd + 512) >> 10);
			}
		}
	}
}

WebCameraSource::WebCameraSource() = default;

WebCameraSource::~WebCameraSource()
{
	Stop();
}

void WebCameraSource::Start(MjpegServer* server, int width, int height, int targetFps)
{
	Stop();

	m_server = server;
	m_requestedWidth = width;
	m_requestedHeight = height;
	m_targetFps = targetFps;
	m_frameCount = 0;
	m_stopRequested = false;

	// Ian: Reset startup synchronization state before launching the worker.
	{
		std::lock_guard<std::mutex> lock(m_startupMutex);
		m_startupDone = false;
		m_startupSuccess = false;
	}

	m_running = true;
	m_workerThread = std::thread(&WebCameraSource::WorkerThread, this);
}

void WebCameraSource::Stop()
{
	if (!m_running)
		return;

	m_stopRequested = true;

	if (m_workerThread.joinable())
		m_workerThread.join();

	m_running = false;
	m_server = nullptr;
}

bool WebCameraSource::IsRunning() const
{
	return m_running;
}

int WebCameraSource::GetFrameCount() const
{
	return m_frameCount;
}

// Ian: Wait for the worker thread to report whether capDriverConnect succeeded.
// Returns true if the camera connected, false on failure or timeout.
bool WebCameraSource::WaitForStartup(int timeoutMs)
{
	std::unique_lock<std::mutex> lock(m_startupMutex);
	if (m_startupCv.wait_for(lock, std::chrono::milliseconds(timeoutMs),
		[this] { return m_startupDone; }))
	{
		return m_startupSuccess;
	}
	// Timed out — treat as failure
	OutputDebugStringW(L"[WebCam] WaitForStartup timed out\n");
	return false;
}

// Ian: VFW frame callback — called on the capture thread for each frame.
// capSetUserData stores our 'this' pointer; capGetUserData retrieves it.
//
// LESSON LEARNED: Some VFW drivers set dwBytesUsed=0 even for valid frames.
// We must NOT reject the frame based solely on dwBytesUsed.  Instead, compute
// the expected data size from the BITMAPINFOHEADER (biSizeImage, or stride × height).
//
// LESSON LEARNED: Do NOT call capSetVideoFormat to resize — the driver lies
// about success and the confused state produces zeroed frame data.  Always
// capture at native resolution (see WorkerThread comments for details).
LRESULT CALLBACK WebCameraSource::FrameCallbackProc(HWND hWnd, LPVIDEOHDR lpVHdr)
{
	if (!lpVHdr || !lpVHdr->lpData)
		return TRUE;

	// Ian: capGetUserData retrieves the pointer we stored via capSetUserData.
	auto* self = reinterpret_cast<WebCameraSource*>(capGetUserData(hWnd));
	if (!self)
		return TRUE;

	// Ian: We need the BITMAPINFOHEADER to know the actual frame dimensions,
	// pixel format, and compression type (BI_RGB vs YUY2).
	DWORD biSize = capGetVideoFormatSize(hWnd);
	if (biSize < sizeof(BITMAPINFOHEADER))
		return TRUE;

	std::vector<uint8_t> formatBuf(biSize);
	capGetVideoFormat(hWnd, formatBuf.data(), biSize);
	const auto* bih = reinterpret_cast<const BITMAPINFOHEADER*>(formatBuf.data());

	int width = bih->biWidth;
	int height = std::abs(bih->biHeight);
	const int bpp = bih->biBitCount;
	const DWORD compression = bih->biCompression;
	// Ian: LESSON LEARNED — VFW YUY2 data is always in top-down scan order
	// regardless of the sign of biHeight.  The biHeight sign only controls
	// scan direction for BI_RGB.  For YUY2, the first row in the buffer is
	// always the top row of the image.  If we respect the biHeight sign for
	// YUY2, the image comes out upside-down.
	const bool topDown = (compression == FOURCC_YUY2)
		? true
		: (bih->biHeight < 0);

	// Ian: Determine actual data size.  Prefer dwBytesUsed if the driver set it;
	// otherwise fall back to biSizeImage, or compute from stride × height.
	// LESSON LEARNED: Some VFW drivers set dwBytesUsed=0 even for valid frames.
	// This is NOT a reason to reject the frame — the data may still be valid.
	int dataSize = static_cast<int>(lpVHdr->dwBytesUsed);
	if (dataSize <= 0)
		dataSize = static_cast<int>(bih->biSizeImage);
	if (dataSize <= 0)
	{
		const int bytesPerPixel = bpp / 8;
		const int stride = ((width * bytesPerPixel + 3) & ~3);
		dataSize = stride * height;
	}

	self->OnFrame(lpVHdr->lpData, dataSize,
		width, height, bpp, compression, topDown);

	return TRUE;
}

void WebCameraSource::OnFrame(const uint8_t* data, int dataSize,
	int width, int height, int bitsPerPixel, DWORD compression, bool topDown)
{
	if (!m_server || m_stopRequested)
		return;

	std::lock_guard<std::mutex> lock(m_bufferMutex);

	// Allocate/reuse RGB buffer
	const int rgbSize = width * height * 3;
	m_rgbBuffer.resize(rgbSize);

	if (compression == FOURCC_YUY2 && bitsPerPixel == 16)
	{
		// Ian: YUY2 packed YCbCr 4:2:2.  Stride = width * 2, DWORD-aligned.
		const int srcStride = ((width * 2 + 3) & ~3);
		if (dataSize < srcStride * height)
			return;
		ConvertYUY2toRGB(data, srcStride, m_rgbBuffer.data(), width, height, topDown);
	}
	else if (compression == BI_RGB && (bitsPerPixel == 24 || bitsPerPixel == 32))
	{
		// Ian: BI_RGB 24-bit BGR or 32-bit BGRA, typically bottom-up.
		const int bytesPerPixel = bitsPerPixel / 8;
		const int srcStride = ((width * bytesPerPixel + 3) & ~3);
		if (dataSize < srcStride * height)
			return;

		for (int y = 0; y < height; ++y)
		{
			int srcY = topDown ? y : (height - 1 - y);
			const uint8_t* srcRow = data + static_cast<size_t>(srcY) * srcStride;
			uint8_t* dstRow = m_rgbBuffer.data() + static_cast<size_t>(y) * width * 3;

			for (int x = 0; x < width; ++x)
			{
				const uint8_t* srcPixel = srcRow + static_cast<size_t>(x) * bytesPerPixel;
				uint8_t* dstPixel = dstRow + static_cast<size_t>(x) * 3;
				dstPixel[0] = srcPixel[2];  // R <- B
				dstPixel[1] = srcPixel[1];  // G <- G
				dstPixel[2] = srcPixel[0];  // B <- R
			}
		}
	}
	else
	{
		// Ian: Unsupported format — log once and skip.
		OutputDebugStringA("[WebCam] Unsupported pixel format, skipping frame\n");
		return;
	}

	// JPEG encode
	m_jpegBuffer.clear();
	m_jpegBuffer.reserve(32 * 1024);
	stbi_write_jpg_to_func(StbiWriteCallback, &m_jpegBuffer,
		width, height, 3, m_rgbBuffer.data(), 75);

	if (!m_jpegBuffer.empty())
	{
		m_server->PushFrame(m_jpegBuffer);
		m_frameCount.fetch_add(1, std::memory_order_relaxed);
	}
}

void WebCameraSource::WorkerThread()
{
	OutputDebugStringW(L"[WebCam] Worker thread starting\n");

	// Ian: Create a hidden capture window on this thread.
	// The window must be on a thread with a message pump.
	HWND hCapWnd = capCreateCaptureWindowW(
		L"WebCamCapture",
		WS_CHILD,           // hidden (no WS_VISIBLE, child of desktop)
		0, 0,
		m_requestedWidth,
		m_requestedHeight,
		GetDesktopWindow(),  // parent
		0);                  // child ID

	if (!hCapWnd)
	{
		OutputDebugStringW(L"[WebCam] capCreateCaptureWindow failed\n");
		// Ian: Signal startup failure so WaitForStartup returns false.
		{
			std::lock_guard<std::mutex> lock(m_startupMutex);
			m_startupDone = true;
			m_startupSuccess = false;
		}
		m_startupCv.notify_one();
		m_running = false;
		return;
	}

	// Ian: Connect to the first available video capture device (index 0).
	if (!capDriverConnect(hCapWnd, 0))
	{
		OutputDebugStringW(L"[WebCam] capDriverConnect failed — no camera found\n");
		DestroyWindow(hCapWnd);
		// Ian: Signal startup failure so WaitForStartup returns false.
		{
			std::lock_guard<std::mutex> lock(m_startupMutex);
			m_startupDone = true;
			m_startupSuccess = false;
		}
		m_startupCv.notify_one();
		m_running = false;
		return;
	}

	OutputDebugStringW(L"[WebCam] Driver connected\n");

	// Ian: Signal startup success — camera driver connected OK.
	{
		std::lock_guard<std::mutex> lock(m_startupMutex);
		m_startupDone = true;
		m_startupSuccess = true;
	}
	m_startupCv.notify_one();

	// Ian: LESSON LEARNED — Do NOT call capSetVideoFormat to resize.
	// The VFW driver (Microsoft WDM Image Capture Win32) lies about supporting
	// non-native resolutions.  capSetVideoFormat(320x240) returns TRUE but the
	// driver doesn't actually resize — it keeps the native 640x480 buffer.
	// Worse, the confused state causes capGrabFrameNoStop to return frames
	// with dwBytesUsed=0 and potentially zeroed/invalid pixel data (producing
	// solid green output after YUY2→RGB conversion).
	//
	// Instead, always capture at native resolution.  The MJPEG server and
	// SmartDashboard don't care about the frame size — they handle whatever
	// we send.
	{
		DWORD biSize = capGetVideoFormatSize(hCapWnd);
		if (biSize >= sizeof(BITMAPINFOHEADER))
		{
			std::vector<uint8_t> formatBuf(biSize);
			capGetVideoFormat(hCapWnd, formatBuf.data(), biSize);
			const auto* bih = reinterpret_cast<const BITMAPINFOHEADER*>(formatBuf.data());

			// Ian: Log native format to OutputDebugString for diagnostics.
			char dbgBuf[128];
			snprintf(dbgBuf, sizeof(dbgBuf),
				"[WebCam] Native format: %ldx%d, %d bpp, compression=0x%08lX\n",
				bih->biWidth, std::abs(bih->biHeight), bih->biBitCount,
				bih->biCompression);
			OutputDebugStringA(dbgBuf);
		}
	}

	// Ian: Set up the frame callback and user data.
	// capGrabFrameNoStop triggers this callback synchronously when a frame is grabbed.
	capSetUserData(hCapWnd, reinterpret_cast<LONG_PTR>(this));
	capSetCallbackOnFrame(hCapWnd, FrameCallbackProc);

	// Ian: LESSON LEARNED — capPreview(TRUE) + capSetCallbackOnFrame relies on the
	// capture window processing WM_PAINT messages to trigger frame callbacks.  A hidden
	// WS_CHILD window of the desktop often never receives WM_PAINT, so the callback
	// never fires and no frames arrive.
	//
	// Instead we use capGrabFrameNoStop in a timed polling loop.  capGrabFrameNoStop
	// grabs a single frame from the driver and triggers the capSetCallbackOnFrame
	// callback synchronously during the call.  This works regardless of window
	// visibility.  We still need the message pump for VFW internal message processing.
	const DWORD frameIntervalMs = 1000 / m_targetFps;

	OutputDebugStringW(L"[WebCam] Capture started, entering grab loop\n");

	DWORD nextGrabTime = GetTickCount();
	while (!m_stopRequested)
	{
		// Ian: Pump messages for VFW internal processing.
		MSG msg;
		while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE))
		{
			TranslateMessage(&msg);
			DispatchMessageW(&msg);
		}

		DWORD now = GetTickCount();
		if (static_cast<long>(now - nextGrabTime) >= 0)
		{
			// Ian: capGrabFrameNoStop grabs one frame from the driver.
			// It triggers the capSetCallbackOnFrame callback synchronously
			// before returning (if a callback is installed).
			// Grab can fail transiently (driver busy, etc.) — just skip.
			if (capGrabFrameNoStop(hCapWnd))
			{
				// Frame callback fires synchronously inside this call.
			}
			nextGrabTime = now + frameIntervalMs;
		}

		// Ian: Sleep briefly to avoid busy-spinning.  MsgWaitForMultipleObjects
		// handles both the message pump and the timed sleep in one call.
		DWORD sleepMs = 1;
		DWORD timeUntilGrab = nextGrabTime - GetTickCount();
		if (timeUntilGrab > 0 && timeUntilGrab <= frameIntervalMs)
			sleepMs = timeUntilGrab;
		MsgWaitForMultipleObjects(0, nullptr, FALSE, sleepMs, QS_ALLINPUT);
	}

	// Ian: Clean shutdown — disconnect driver, destroy window.
	capSetCallbackOnFrame(hCapWnd, nullptr);
	capDriverDisconnect(hCapWnd);
	DestroyWindow(hCapWnd);

	OutputDebugStringW(L"[WebCam] Worker thread exiting\n");
}
