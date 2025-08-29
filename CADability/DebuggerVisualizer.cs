using CADability.Curve2D;
using CADability.GeoObject;
using CADability.Shapes;
using CADability.UserInterface;
using Microsoft.Win32;
using netDxf;
using System;
using System.Buffers.Binary;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.IO.Pipes;
using System.Linq;
using System.Runtime.InteropServices.ComTypes;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace CADability
{
    public static class D
    {
        public static string Show(object toVisualize)
        {
            var enumInterface = toVisualize.GetType().GetInterfaces().FirstOrDefault(i => i.IsGenericType &&
                         i.GetGenericTypeDefinition() == typeof(IEnumerable<>));
            // Convert the IEnumerable type to an array of the same type
            if (enumInterface != null)
            {
                object[] arr = ((IEnumerable)toVisualize).Cast<object>().ToArray();
                GeoObjectList toShow = new GeoObjectList();
                for (int i = 0; i < arr.Length; i++)
                {
                    object g = ToGeoObject(arr[i]);
                    if (g is IGeoObject geoObject)
                    {
                        geoObject.UserData.Add("ListIndex", new IntegerProperty(i, "Debug.ListIndex"));
                        toShow.Add(geoObject);
                    }
                    else if (g is GeoObjectList l) { toShow.AddRange(l); }
                }
                toVisualize = toShow;
            }
            else
            {
                toVisualize = ToGeoObject(toVisualize);
                if (toVisualize is IGeoObject geoObject)
                {
                    // make a GeoObjectlist, because single GeoObjects don't serialize correctely
                    toVisualize = new GeoObjectList(geoObject);
                }
            }
            if (toVisualize != null)
            {
                using (MemoryStream ms = new MemoryStream())
                {
                    JsonSerialize js = new JsonSerialize();
                    js.ToStream(ms, toVisualize, false); // don't close the memory stream
                    ms.Seek(0, SeekOrigin.Begin);
                    StreamReader sr = new StreamReader(ms);
                    string json = sr.ReadToEnd();
                    sr.Close();
                    if (DebuggerVisualizer.TrySend(json)) return "look in CADability.DebuggerViewer";
                    return "CADability.DebuggerViewer is not running";
                }
            }
            else return "no visualizable oject";
        }
        private static object ToGeoObject(object obj)
        {
            if (obj is IGeoObject go)
            {
                IGeoObject ret = go.Clone();
                ret.UserData.Clear(); // Clear user data, because it might contain references to other obejcts
                // which would also be serialized
                if (go is Face fc) ret.UserData.Add("OriginalHashcode", new IntegerProperty(fc.GetHashCode(), "Debug.HashCode"));
                return ret;
            }
            else if (obj is Edge edge)
            {
                IGeoObject ret = edge.Curve3D.Clone() as IGeoObject;
                ret.UserData.Add("OriginalHashcode", new IntegerProperty(edge.GetHashCode(), "Debug.HashCode"));
                return ret;
            }
            else if (obj is ICurve2D curve2d)
            {
                return curve2d.MakeGeoObject(Plane.XYPlane);
            }
            else if (obj is Border border)
            {
                return border.MakePath(Plane.XYPlane);
            }
            else if (obj is SimpleShape ss)
            {
                return ss.DebugList;
            }
            else if (obj is CompoundShape cs)
            {
                return cs.DebugList;
            }
            else if (obj is GeoPoint geoPoint)
            {
                return Point.MakePoint(geoPoint);
            }
            return null;
        }
    }
    public static class DebuggerVisualizer
    {
        private static NamedPipeClientStream _pipe;               // volatile genügt in der Praxis
        private static int _connecting;                            // 0/1 – verhindert paralleles Connect
        private static long _disabledUntilTicks;                   // Backoff bis …

        public static bool TrySend(string json)
        {
            if (json == null) return false;
            if (DateTime.UtcNow.Ticks < Interlocked.Read(ref _disabledUntilTicks)) return false;

            try
            {
                EnsureConnectedNonBlocking();
                var p = _pipe;
                if (p == null || !p.IsConnected) return false;

                UTF8Encoding enc = new UTF8Encoding(encoderShouldEmitUTF8Identifier: false);
                string msg = $@"{{""ver"":1,""type"":""Show"",""payload"":{json}}}";
                byte[] data = enc.GetBytes(msg);

                byte[] len = new byte[4];
                BinaryPrimitives.WriteInt32LittleEndian(len, data.Length);

                p.Write(len, 0, len.Length);
                p.Write(data, 0, data.Length);
                p.Flush();
                return true;
            }
            catch
            {
                // 5s Ruhe – verhindert Dauerversuche im Break-Zustand
                Interlocked.Exchange(ref _disabledUntilTicks, DateTime.UtcNow.AddSeconds(5).Ticks);
                SafeDispose();
                return false;
            }
        }
        static string GetCadDebuggerExe()
        {
            using (var key = Registry.CurrentUser.OpenSubKey(@"Software\CADability\DebuggerView"))
                return key?.GetValue("ExePath") as string;
        }

        static bool TryStartCadDebugger(string arg = "")
        {
            bool running;
            try
            {
                using (var m = Mutex.OpenExisting("Cadability.DebuggerViewer_Running")) // wirft, wenn nicht vorhanden
                { running = true; }
            }
            catch (WaitHandleCannotBeOpenedException)
            {
                running = false;
            }

            if (!running)
            {
                var exe = GetCadDebuggerExe();
                if (string.IsNullOrWhiteSpace(exe)) return false;

                var psi = new ProcessStartInfo(exe)
                {
                    UseShellExecute = true,
                    Arguments = string.IsNullOrEmpty(arg) ? "" : arg
                };
                Process.Start(psi);
            }
            return true;
        }
        private static void EnsureConnectedNonBlocking()
        {
            if (_pipe != null && _pipe.IsConnected) return;
            if (Interlocked.Exchange(ref _connecting, 1) != 0) return; // jemand anders versucht schon

            try
            {
                //TryStartCadDebugger();
                SafeDispose();
                var c = new NamedPipeClientStream(".", "cadability.visualizer.v1", PipeDirection.InOut, PipeOptions.None);
                try { c.Connect(50); }                           // max. 50 ms
                catch { c.Dispose(); return; }

                c.ReadMode = PipeTransmissionMode.Byte;
                //c.ReadTimeout = 100; c.WriteTimeout = 100;

                // Handshake (sync, klein)
                SendHello(c);
                if (!ExpectHelloAck(c)) { c.Dispose(); return; }

                Interlocked.Exchange(ref _pipe, c);
            }
            catch
            {
                SafeDispose();
                // kein Re-throw – Property darf nie Fehlermeldungen aufspringen lassen
            }
            finally
            {
                Volatile.Write(ref _connecting, 0);
            }
        }

        private static void SendHello(Stream s)
        {
            byte[] data = new UTF8Encoding(encoderShouldEmitUTF8Identifier: false)
               .GetBytes("{\"ver\":1,\"type\":\"Hello\",\"payload\":{}}");
            byte[] len = new byte[4];
            BinaryPrimitives.WriteInt32LittleEndian(len, data.Length);
            s.Write(len, 0, len.Length);
            s.Write(data, 0, data.Length);
            s.Flush();
        }

        private static bool ExpectHelloAck(Stream s)
        {
            byte[] len = new byte[4];
            if (!ReadExactly(s, len)) return false;
            int n = BinaryPrimitives.ReadInt32LittleEndian(len);
            if (n <= 0 || n > 1_000_000) return false;

            byte[] buf = new byte[n];
            if (!ReadExactly(s, buf)) return false;

            string answer = Encoding.UTF8.GetString(buf);
            return answer.Contains("HelloAck");
        }

        private static bool ReadExactly(Stream s, byte[] buffer)
        {
            int off = 0, need = buffer.Length;
            while (need > 0)
            {
                int r = s.Read(buffer, off, need);
                if (r == 0) return false;
                off += r; need -= r;
            }
            return true;
        }

        private static void SafeDispose()
        {
            try { _pipe?.Dispose(); } catch { }
            _pipe = null;
        }
    }
}
