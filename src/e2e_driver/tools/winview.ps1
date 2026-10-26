# MJPEG view of the W1 simulator window: viewer.py's twin for EVAL_SIM=w1 (eval_manager.py). http://127.0.0.1:PORT/ on
# winpc, reached from the Mac through an ssh forward (a loopback listener needs no firewall rule). The client area is
# captured with PrintWindow (works while the window is covered), scaled to WIDTH, only while someone is watching.
# Needs the desktop session: eval_manager.py runs it through remote_ps.py --interactive, which passes -OutputDir; the
# script then starts itself detached from C:\morai-par\winview.ps1 and returns. -Stop ends the running ones.
# A window wider than WIDTH is resized to it (W1 opens at the 4K screen size, where one capture takes 100 ms); Unity keeps
# that size in HKCU\Software\MORAI\Simulator when it closes, so the caller restores the size afterwards.
param([int]$Port = 8090, [double]$Fps = 15, [int]$Width = 1280, [string]$Exe = 'C:\morai\multi-03-20261007\',
      [string]$OutputDir = '', [switch]$Stop)
if ($Stop) {
  Get-CimInstance Win32_Process -Filter "Name = 'powershell.exe'" |
    Where-Object { $_.CommandLine -like '*morai-par\winview.ps1*' -and $_.CommandLine -notlike '*-Stop*' } |
    ForEach-Object { Stop-Process -Id $_.ProcessId -Force; "stopped winview pid $($_.ProcessId)" }
  return
}
if ($OutputDir) {
  New-Item -ItemType Directory -Force -Path $OutputDir | Out-Null
  $p = Start-Process powershell -WindowStyle Hidden -PassThru -ArgumentList @('-NoProfile', '-ExecutionPolicy', 'Bypass',
        '-File', 'C:\morai-par\winview.ps1', '-Port', $Port, '-Fps', $Fps, '-Width', $Width, '-Exe', $Exe)
  @{ok = $true; pid = $p.Id; port = $Port} | ConvertTo-Json -Compress | Set-Content -Encoding UTF8 (Join-Path $OutputDir 'result.json')
  return
}
Add-Type -ReferencedAssemblies System.Drawing -TypeDefinition @'
using System;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Drawing.Imaging;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;

public static class WinView {
  [DllImport("user32.dll")] static extern bool PrintWindow(IntPtr h, IntPtr dc, uint flags);
  [DllImport("user32.dll")] static extern bool GetClientRect(IntPtr h, out Rect r);
  [DllImport("user32.dll")] static extern bool IsWindow(IntPtr h);
  [DllImport("user32.dll")] static extern bool SetProcessDPIAware();
  [DllImport("user32.dll")] static extern bool GetWindowRect(IntPtr h, out Rect r);
  [DllImport("user32.dll")] static extern bool ShowWindow(IntPtr h, int cmd);
  [DllImport("user32.dll")] static extern bool SetWindowPos(IntPtr h, IntPtr after, int x, int y, int cx, int cy, uint flags);
  struct Rect { public int L, T, R, B; }
  static byte[] frame;
  static int viewers;
  static readonly object lk = new object();

  static IntPtr Find(string prefix) {   // the Simulator copy under prefix (W1), not another copy
    foreach (var p in Process.GetProcessesByName("Simulator")) {
      try {
        if (p.MainModule.FileName.StartsWith(prefix, StringComparison.OrdinalIgnoreCase) && p.MainWindowHandle != IntPtr.Zero)
          return p.MainWindowHandle;
      } catch (Exception) {}
    }
    return IntPtr.Zero;
  }

  static void Capture(string prefix, double fps, int width) {
    ImageCodecInfo jpeg = null;
    foreach (var c in ImageCodecInfo.GetImageEncoders()) if (c.MimeType == "image/jpeg") jpeg = c;
    var q = new EncoderParameters(1);
    q.Param[0] = new EncoderParameter(System.Drawing.Imaging.Encoder.Quality, 70L);
    IntPtr h = IntPtr.Zero;
    var sw = Stopwatch.StartNew();
    while (true) {
      double t0 = sw.Elapsed.TotalSeconds;
      if (viewers == 0) { Thread.Sleep(200); continue; }
      try {
        if (h == IntPtr.Zero || !IsWindow(h)) h = Find(prefix);
        Rect r;
        if (h == IntPtr.Zero || !GetClientRect(h, out r) || r.R <= 0 || r.B <= 0) { h = IntPtr.Zero; Thread.Sleep(1000); continue; }
        int hh = (int)Math.Round(r.B * (double)width / r.R);
        if (r.R > width) {                       // client area WIDTH x hh: restore (un-maximize), keep the border
          Rect w;
          GetWindowRect(h, out w);
          ShowWindow(h, 9);
          SetWindowPos(h, IntPtr.Zero, 0, 0, width + (w.R - w.L - r.R), hh + (w.B - w.T - r.B), 0x0014);   // NOZORDER|NOACTIVATE
          Thread.Sleep(500);
          continue;
        }
        using (var full = new Bitmap(r.R, r.B, PixelFormat.Format32bppRgb))
        using (var small = new Bitmap(width, hh, PixelFormat.Format24bppRgb)) {
          using (var g = Graphics.FromImage(full)) {
            IntPtr dc = g.GetHdc();
            try { PrintWindow(h, dc, 3); } finally { g.ReleaseHdc(dc); }   // 3: client area, full (DirectX) content
          }
          using (var g = Graphics.FromImage(small)) {
            g.InterpolationMode = InterpolationMode.Bilinear;
            g.DrawImage(full, 0, 0, width, hh);
          }
          var ms = new MemoryStream();
          small.Save(ms, jpeg, q);
          lock (lk) { frame = ms.ToArray(); Monitor.PulseAll(lk); }
        }
      } catch (Exception e) {                    // the simulator is (re)starting: look for its window again
        Console.Error.WriteLine("capture: " + e.Message);
        h = IntPtr.Zero;
        Thread.Sleep(1000);
      }
      int wait = (int)((1.0 / fps - (sw.Elapsed.TotalSeconds - t0)) * 1000);
      if (wait > 0) Thread.Sleep(wait);
    }
  }

  static void Client(TcpClient c) {
    Interlocked.Increment(ref viewers);
    try {
      using (c) {
        var s = c.GetStream();
        var head = Encoding.ASCII.GetBytes("HTTP/1.0 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n" +
                                           "Cache-Control: no-cache\r\n\r\n");
        s.Write(head, 0, head.Length);
        byte[] last = null;
        while (true) {
          byte[] f;
          lock (lk) { if (frame == last) Monitor.Wait(lk, 5000); f = frame; }
          if (f == null || f == last) continue;
          last = f;
          var hd = Encoding.ASCII.GetBytes("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + f.Length + "\r\n\r\n");
          s.Write(hd, 0, hd.Length);
          s.Write(f, 0, f.Length);
          s.Write(new byte[] {13, 10}, 0, 2);
        }
      }
    } catch (Exception) {                        // the viewer went away
    } finally { Interlocked.Decrement(ref viewers); }
  }

  public static void Run(int port, double fps, int width, string prefix) {
    SetProcessDPIAware();
    new Thread(() => Capture(prefix, fps, width)) { IsBackground = true }.Start();
    var l = new TcpListener(IPAddress.Loopback, port);
    l.Start();
    while (true) {
      var c = l.AcceptTcpClient();
      new Thread(() => Client(c)) { IsBackground = true }.Start();
    }
  }
}
'@
[WinView]::Run($Port, $Fps, $Width, $Exe)
