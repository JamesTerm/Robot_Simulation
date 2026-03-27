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
// The frame callback converts the DIB data to top-down RGB, JPEG-encodes it,
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

// Ian: VFW frame callback — called on the capture thread for each frame.
// capSetUserData stores our 'this' pointer; capGetUserData retrieves it.
LRESULT CALLBACK WebCameraSource::FrameCallbackProc(HWND hWnd, LPVIDEOHDR lpVHdr)
{
	if (!lpVHdr || !lpVHdr->lpData || lpVHdr->dwBytesUsed == 0)
		return TRUE;

	// Ian: capGetUserData retrieves the pointer we stored via capSetUserData.
	auto* self = reinterpret_cast<WebCameraSource*>(capGetUserData(hWnd));
	if (!self)
		return TRUE;

	// Ian: We need the BITMAPINFOHEADER to know the actual frame dimensions and
	// pixel format.  capGetVideoFormat returns it.
	DWORD biSize = capGetVideoFormatSize(hWnd);
	if (biSize < sizeof(BITMAPINFOHEADER))
		return TRUE;

	std::vector<uint8_t> formatBuf(biSize);
	capGetVideoFormat(hWnd, formatBuf.data(), biSize);
	const auto* bih = reinterpret_cast<const BITMAPINFOHEADER*>(formatBuf.data());

	self->OnFrame(lpVHdr->lpData, static_cast<int>(lpVHdr->dwBytesUsed),
		bih->biWidth, std::abs(bih->biHeight), bih->biBitCount);

	return TRUE;
}

void WebCameraSource::OnFrame(const uint8_t* data, int dataSize, int width, int height, int bitsPerPixel)
{
	if (!m_server || m_stopRequested)
		return;

	// Ian: VFW typically delivers frames as bottom-up BGR24 (BI_RGB, 24bpp).
	// We need to flip vertically and convert BGR→RGB for JPEG encoding.
	const int bytesPerPixel = bitsPerPixel / 8;
	if (bytesPerPixel != 3 && bytesPerPixel != 4)
	{
		// Ian: We only handle 24-bit BGR or 32-bit BGRA.  Other formats
		// (YUV, compressed) would need conversion — punt for now.
		OutputDebugStringA("[WebCam] Unsupported pixel format, skipping frame\n");
		return;
	}

	const int srcStride = ((width * bytesPerPixel + 3) & ~3);  // DWORD-aligned
	if (dataSize < srcStride * height)
		return;

	std::lock_guard<std::mutex> lock(m_bufferMutex);

	// Allocate/reuse RGB buffer
	const int rgbSize = width * height * 3;
	m_rgbBuffer.resize(rgbSize);

	// Ian: Convert bottom-up BGR(A) to top-down RGB.
	for (int y = 0; y < height; ++y)
	{
		const uint8_t* srcRow = data + static_cast<size_t>(height - 1 - y) * srcStride;
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
		m_running = false;
		return;
	}

	// Ian: Connect to the first available video capture device (index 0).
	if (!capDriverConnect(hCapWnd, 0))
	{
		OutputDebugStringW(L"[WebCam] capDriverConnect failed — no camera found\n");
		DestroyWindow(hCapWnd);
		m_running = false;
		return;
	}

	// Ian: Try to set the requested resolution.  Not all cameras support 320x240,
	// so we read back what the driver actually set.
	{
		DWORD biSize = capGetVideoFormatSize(hCapWnd);
		if (biSize >= sizeof(BITMAPINFOHEADER))
		{
			std::vector<uint8_t> formatBuf(biSize);
			capGetVideoFormat(hCapWnd, formatBuf.data(), biSize);
			auto* bih = reinterpret_cast<BITMAPINFOHEADER*>(formatBuf.data());
			bih->biWidth = m_requestedWidth;
			bih->biHeight = m_requestedHeight;
			capSetVideoFormat(hCapWnd, bih, biSize);
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
			if (!capGrabFrameNoStop(hCapWnd))
			{
				// Ian: Grab can fail transiently (driver busy, etc.).
				// Don't spam the log — just skip this frame.
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
