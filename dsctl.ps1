# dsctl.ps1 — Automate DriverStation via Win32 SendMessage
# Usage: powershell -File dsctl.ps1 <command>
# Commands: start, auton, tele, test, stop, auton-enable (start + auton)

param([string]$Command = "auton-enable")

Add-Type @"
using System;
using System.Runtime.InteropServices;

public class Win32 {
    [DllImport("user32.dll", SetLastError = true)]
    public static extern IntPtr FindWindow(string lpClassName, string lpWindowName);

    [DllImport("user32.dll", SetLastError = true)]
    public static extern bool PostMessage(IntPtr hWnd, uint Msg, IntPtr wParam, IntPtr lParam);

    [DllImport("user32.dll")]
    public static extern IntPtr SendMessage(IntPtr hWnd, uint Msg, IntPtr wParam, IntPtr lParam);

    [DllImport("user32.dll")]
    [return: MarshalAs(UnmanagedType.Bool)]
    public static extern bool EnumWindows(EnumWindowsProc lpEnumFunc, IntPtr lParam);

    [DllImport("user32.dll")]
    public static extern uint GetWindowThreadProcessId(IntPtr hWnd, out uint lpdwProcessId);

    public delegate bool EnumWindowsProc(IntPtr hWnd, IntPtr lParam);
}
"@

$WM_COMMAND = 0x0111
$IDC_Start = 1006
$IDC_Stop  = 1005
$IDC_Auton = 1003
$IDC_Tele  = 1002
$IDC_Test  = 1004

# Find DriverStation process
$proc = Get-Process -Name "DriverStation" -ErrorAction SilentlyContinue
if (-not $proc) {
    Write-Error "DriverStation.exe is not running"
    exit 1
}

$pid_target = $proc.Id
Write-Host "Found DriverStation PID=$pid_target"

# Find the dialog window belonging to this process
$hwnd = [IntPtr]::Zero
$found = $false

[Win32]::EnumWindows({
    param($h, $l)
    $wpid = [uint32]0
    [Win32]::GetWindowThreadProcessId($h, [ref]$wpid) | Out-Null
    if ($wpid -eq $script:pid_target) {
        $script:hwnd = $h
        $script:found = $true
        return $false  # stop enumerating
    }
    return $true
}, [IntPtr]::Zero) | Out-Null

if (-not $found -or $hwnd -eq [IntPtr]::Zero) {
    Write-Error "Could not find DriverStation window"
    exit 1
}

Write-Host "Found window handle: $hwnd"

function Send-Command($controlId) {
    # WM_COMMAND: LOWORD(wParam) = control ID, HIWORD(wParam) = BN_CLICKED (0)
    $wparam = [IntPtr]$controlId
    [Win32]::PostMessage($hwnd, $WM_COMMAND, $wparam, [IntPtr]::Zero) | Out-Null
    Write-Host "Sent WM_COMMAND for control $controlId"
}

switch ($Command) {
    "start" {
        Send-Command $IDC_Start
    }
    "stop" {
        Send-Command $IDC_Stop
    }
    "auton" {
        Send-Command $IDC_Auton
    }
    "tele" {
        Send-Command $IDC_Tele
    }
    "test" {
        Send-Command $IDC_Test
    }
    "auton-enable" {
        Send-Command $IDC_Start
        Start-Sleep -Milliseconds 500
        Send-Command $IDC_Auton
        Write-Host "Sent Start + Auton"
    }
    default {
        Write-Error "Unknown command: $Command. Use: start, stop, auton, tele, test, auton-enable"
        exit 1
    }
}

Write-Host "Done."
