using System;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using Avalonia.Controls;
using Avalonia.Platform.Storage;
using CADability;
using CADability.Attribute;
using CADability.Avalonia;
using CADability.GeoObject;
using SysPath = System.IO.Path;

namespace ShapeIt.Browser
{
    /// <summary>
    /// Browser-confined File / Import / Export implementation for the ShapeIt WebAssembly head.
    ///
    /// The desktop file flow inside CADability/CADability.Avalonia ultimately calls
    /// <c>File.Open(path)</c> on the <c>IStorageFile.Path.LocalPath</c> returned by the browser
    /// storage picker. That path is NOT a real local filesystem path in WASM, so the desktop
    /// flow cannot read/write the user's file. Instead, the menu commands are intercepted in
    /// <see cref="MainView"/> and routed here, where we work directly with the picker's
    /// <c>Stream</c>s (open/save of the native project) or buffer the bytes through the
    /// in-memory (Emscripten MEMFS) virtual filesystem for the format-specific
    /// importers/exporters that only expose a file-path API.
    ///
    /// Nothing in CADability/, ShapeIt/ or CADability.Avalonia/ is modified.
    /// </summary>
    internal sealed class BrowserFileService
    {
        private readonly CadFrame _frame;
        private readonly Control _owner;

        // Re-entrancy guard: a single browser file picker may be in flight at a time.
        private bool _busy;

        public BrowserFileService(CadFrame frame, Control owner)
        {
            _frame = frame;
            _owner = owner;
        }

        // ── command entry points ────────────────────────────────────────────

        /// <summary>New project. Replaces the current project without a (blocking) save prompt.</summary>
        public void NewProject()
        {
            try
            {
                Project newProject = Project.CreateSimpleProject();
                _frame.Project = newProject;          // setter wires up views + ActiveView
                RefreshView();
                Console.WriteLine("ShapeIt.Browser: new project created.");
            }
            catch (Exception ex)
            {
                Console.WriteLine("ShapeIt.Browser NewProject failed: " + ex);
            }
        }

        public async Task OpenAsync()
        {
            if (!Enter()) return;
            try
            {
                IStorageProvider? sp = GetStorageProvider();
                if (sp == null) return;

                var files = await sp.OpenFilePickerAsync(new FilePickerOpenOptions
                {
                    Title = "Open Project",
                    AllowMultiple = false,
                    FileTypeFilter = new[]
                    {
                        new FilePickerFileType("CADability Projects") { Patterns = new[] { "*.cdb.json", "*.cdb" } },
                        new FilePickerFileType("All Files") { Patterns = new[] { "*" } },
                    },
                });
                if (files == null || files.Count == 0) return;

                IStorageFile picked = files[0];
                byte[] bytes = await ReadAllBytesAsync(picked);

                Project? project = ReadProjectFromBytes(bytes);
                if (project == null)
                {
                    Console.WriteLine("ShapeIt.Browser Open: could not read project from " + picked.Name);
                    return;
                }

                project.FileName = picked.Name;
                _frame.Project = project;     // wires views + ActiveView
                RefreshView();
                Console.WriteLine("ShapeIt.Browser: opened project " + picked.Name);
            }
            catch (Exception ex)
            {
                Console.WriteLine("ShapeIt.Browser Open failed: " + ex);
            }
            finally { Leave(); }
        }

        /// <param name="saveAs">Ignored in the browser: there is no silent re-save, every save
        /// goes through the download picker. Kept so the two menu ids map cleanly.</param>
        // Save serializes the project to a .cdb.json and triggers a browser DOWNLOAD
        // (Blob + anchor). We deliberately do NOT use StorageProvider.SaveFilePickerAsync:
        // that relies on the Chromium-only File System Access API and silently returns null
        // in other browsers (and even in Chromium it needs an intact user gesture). The
        // download path works everywhere. (saveAs is irrelevant in the browser — every save
        // is a download to the browser's download location.)
        public Task SaveAsync(bool saveAs)
        {
            if (!Enter()) return Task.CompletedTask;
            try
            {
                if (_frame.Project == null) return Task.CompletedTask;

                // The desktop convention is a ".cdb" file whose CONTENT is JSON (binary
                // ISerializable is no longer written). Open auto-detects by content, not
                // extension, so ".cdb" + JSON loads everywhere (web and desktop).
                string baseName = string.IsNullOrEmpty(_frame.Project.FileName)
                    ? "project" : SysPath.GetFileName(_frame.Project.FileName);
                if (baseName.EndsWith(".cdb.json", StringComparison.OrdinalIgnoreCase)) baseName = baseName[..^9];
                else if (baseName.EndsWith(".json", StringComparison.OrdinalIgnoreCase)) baseName = baseName[..^5];
                else if (baseName.EndsWith(".cdb", StringComparison.OrdinalIgnoreCase)) baseName = baseName[..^4];
                if (string.IsNullOrEmpty(baseName)) baseName = "project";
                string name = baseName + ".cdb";

                byte[] bytes;
                using (var ms = new MemoryStream())
                {
                    new JsonSerialize().ToStream(ms, _frame.Project, false);
                    bytes = ms.ToArray();
                }

                BrowserHostInterop.DownloadFile(name, Convert.ToBase64String(bytes));
                _frame.Project.FileName = name;
                _frame.Project.IsModified = false;
                Console.WriteLine($"ShapeIt.Browser: saved (download) {name}, {bytes.Length} bytes");
            }
            catch (Exception ex)
            {
                Console.WriteLine("ShapeIt.Browser Save failed: " + ex);
            }
            finally { Leave(); }
            return Task.CompletedTask;
        }

        public async Task ImportAsync()
        {
            if (!Enter()) return;
            string? tempPath = null;
            try
            {
                IStorageProvider? sp = GetStorageProvider();
                if (sp == null) return;

                var files = await sp.OpenFilePickerAsync(new FilePickerOpenOptions
                {
                    Title = "Import File",
                    AllowMultiple = false,
                    FileTypeFilter = new[]
                    {
                        new FilePickerFileType("All Supported") { Patterns = new[] { "*.dxf", "*.stp", "*.step", "*.stl", "*.cdb", "*.cdb.json" } },
                        new FilePickerFileType("DXF Files") { Patterns = new[] { "*.dxf" } },
                        new FilePickerFileType("STEP Files") { Patterns = new[] { "*.stp", "*.step" } },
                        new FilePickerFileType("STL Files") { Patterns = new[] { "*.stl" } },
                        new FilePickerFileType("CADability Projects") { Patterns = new[] { "*.cdb", "*.cdb.json" } },
                        new FilePickerFileType("All Files") { Patterns = new[] { "*" } },
                    },
                });
                if (files == null || files.Count == 0) return;

                IStorageFile picked = files[0];
                byte[] bytes = await ReadAllBytesAsync(picked);
                string ext = GetExtension(picked.Name);

                Project? imported = null;
                switch (ext)
                {
                    case ".stl":
                        imported = ImportStl(bytes);
                        break;
                    case ".dxf":
                        tempPath = WriteTempFile(bytes, ".dxf");
                        // Bypass Project.ReadFromFile's ImportDXF (which spawns an external
                        // ConvertToDxfAutoCad2000 process – impossible in WASM). The DXF.Import
                        // ctor only does File.Open + DxfDocument.Load, which works on MEMFS.
                        imported = new CADability.DXF.Import(tempPath).Project;
                        break;
                    case ".stp":
                    case ".step":
                        tempPath = WriteTempFile(bytes, ".stp");
                        imported = Project.ReadFromFile(tempPath, "stp"); // uses ImportStep
                        break;
                    case ".cdb":
                    case ".json":   // matches the ".json" of "name.cdb.json"
                        imported = ReadProjectFromBytes(bytes);
                        break;
                    default:
                        Console.WriteLine("ShapeIt.Browser Import: unsupported extension '" + ext + "'");
                        return;
                }

                if (imported == null)
                {
                    Console.WriteLine("ShapeIt.Browser Import: nothing imported from " + picked.Name);
                    return;
                }

                MergeIntoActiveProject(imported);
                RefreshView();
                Console.WriteLine("ShapeIt.Browser: imported " + picked.Name);
            }
            catch (Exception ex)
            {
                Console.WriteLine("ShapeIt.Browser Import failed: " + ex);
            }
            finally
            {
                TryDeleteTemp(tempPath);
                Leave();
            }
        }

        /// <summary>
        /// Exports the project as STL / DXF / STEP and triggers a browser DOWNLOAD (Blob).
        /// The desktop's save dialog (with a format dropdown) has no browser equivalent that
        /// works everywhere — SaveFilePickerAsync needs the Chromium-only File System Access
        /// API and silently returns null elsewhere — so the format is chosen by MainView's
        /// in-app export overlay and passed in here. <paramref name="format"/> is one of
        /// "stl", "dxf", "stp"/"step".
        /// </summary>
        public Task ExportAsync(string format)
        {
            if (!Enter()) return Task.CompletedTask;
            string? tempPath = null;
            try
            {
                if (_frame.Project == null) return Task.CompletedTask;

                string baseName = _frame.Project.FileName;
                baseName = string.IsNullOrEmpty(baseName) ? "export" : StripExtensions(baseName);

                byte[]? data = null;
                string ext;
                switch ((format ?? "stl").ToLowerInvariant())
                {
                    case "dxf":
                        ext = ".dxf";
                        data = new CADability.DXF.Export(netDxf.Header.DxfVersion.AutoCad2000)
                            .WriteToByteArray(_frame.Project);
                        break;
                    case "stp":
                    case "step":
                        ext = ".stp";
                        data = new ExportStep().WriteToByteArray(_frame.Project);
                        break;
                    case "stl":
                    default:
                        ext = ".stl";
                        // PaintToSTL only writes to a file path; round-trip through MEMFS.
                        tempPath = TempFilePath(".stl");
                        _frame.Project.Export(tempPath, "stl");
                        data = File.ReadAllBytes(tempPath);
                        break;
                }

                if (data == null) return Task.CompletedTask;
                string name = baseName + ext;
                BrowserHostInterop.DownloadFile(name, Convert.ToBase64String(data));
                Console.WriteLine($"ShapeIt.Browser: exported (download) {name}, {data.Length} bytes");
            }
            catch (Exception ex)
            {
                Console.WriteLine("ShapeIt.Browser Export failed: " + ex);
            }
            finally
            {
                TryDeleteTemp(tempPath);
                Leave();
            }
            return Task.CompletedTask;
        }

        // ── helpers ─────────────────────────────────────────────────────────

        /// <summary>
        /// Reads a CADability project from a buffer, auto-detecting JSON (.cdb.json, first byte '{')
        /// vs. the legacy binary (.cdb) format – the same detection Project.ReadFromFile uses.
        /// </summary>
        private static Project? ReadProjectFromBytes(byte[] bytes)
        {
            if (bytes.Length == 0) return null;
            if (bytes[0] == (byte)'{')
            {
                using var ms = new MemoryStream(bytes, writable: false);
                var js = new JsonSerialize();
                Project? res = js.FromStream(ms) as Project;
                if (res != null) AttributeListContainer.UpdateLists(res, true); // mirrors ReadFromJson
                return res;
            }
            using (var ms = new MemoryStream(bytes, writable: false))
            {
                return Project.ReadFromStream(ms);
            }
        }

        private static Project ImportStl(byte[] bytes)
        {
            var importStl = new ImportSTL();
            Shell[] shells = importStl.Read(bytes);
            Project project = Project.CreateSimpleProject();
            if (shells != null)
            {
                Model model = project.GetActiveModel();
                foreach (Shell shell in shells)
                {
                    project.SetDefaults(shell);
                    if (shell.HasOpenEdgesExceptPoles())
                    {
                        model.Add(shell);
                    }
                    else
                    {
                        Solid sld = Solid.Construct();
                        sld.SetShell(shell);
                        project.SetDefaults(sld);
                        model.Add(sld);
                    }
                }
            }
            return project;
        }

        /// <summary>
        /// Copies the geometry of an imported project's active model into the current project's
        /// active model, re-homing attributes onto the target project so colors/layers resolve.
        /// </summary>
        private void MergeIntoActiveProject(Project imported)
        {
            if (_frame.Project == null)
            {
                _frame.Project = imported;
                return;
            }

            Project target = _frame.Project;
            Model targetModel = target.GetActiveModel();
            Model sourceModel = imported.GetActiveModel();

            foreach (IGeoObject go in sourceModel.AllObjects.Clone())
            {
                target.SetDefaults(go);
                AttributeListContainer.UpdateObjectAttrinutes(target, go);
                go.UpdateAttributes(target);
                targetModel.Add(go);
            }
            target.IsModified = true;
        }

        private void RefreshView()
        {
            var view = _frame.ActiveView;
            if (view == null) return;
            try { view.ZoomTotal(1.2); } catch { /* extent may be empty */ }
            try { view.InvalidateAll(); } catch { /* best effort repaint */ }
        }

        private IStorageProvider? GetStorageProvider()
        {
            var topLevel = TopLevel.GetTopLevel(_owner);
            if (topLevel == null)
            {
                Console.WriteLine("ShapeIt.Browser: no TopLevel / StorageProvider available.");
                return null;
            }
            return topLevel.StorageProvider;
        }

        private static async Task<byte[]> ReadAllBytesAsync(IStorageFile file)
        {
            using Stream stream = await file.OpenReadAsync();
            using var ms = new MemoryStream();
            await stream.CopyToAsync(ms);
            return ms.ToArray();
        }

        private static string GetExtension(string name)
        {
            string ext = SysPath.GetExtension(name ?? string.Empty);
            return ext.ToLowerInvariant();
        }

        // Strips both ".cdb.json" and a single extension to derive an export base name.
        private static string StripExtensions(string name)
        {
            string n = SysPath.GetFileName(name);
            if (n.EndsWith(".cdb.json", StringComparison.OrdinalIgnoreCase))
                return n.Substring(0, n.Length - ".cdb.json".Length);
            return SysPath.GetFileNameWithoutExtension(n);
        }

        private static string TempDir()
        {
            string dir = SysPath.GetTempPath();
            if (string.IsNullOrEmpty(dir)) dir = "/tmp";
            try { Directory.CreateDirectory(dir); } catch { /* MEMFS root usually exists */ }
            return dir;
        }

        private static string TempFilePath(string extension)
            => SysPath.Combine(TempDir(), "shapeit_" + Guid.NewGuid().ToString("N") + extension);

        private static string WriteTempFile(byte[] bytes, string extension)
        {
            string path = TempFilePath(extension);
            File.WriteAllBytes(path, bytes);
            return path;
        }

        private static void TryDeleteTemp(string? path)
        {
            if (string.IsNullOrEmpty(path)) return;
            try { if (File.Exists(path)) File.Delete(path); } catch { /* best effort */ }
        }

        private bool Enter()
        {
            if (_busy)
            {
                Console.WriteLine("ShapeIt.Browser: a file operation is already in progress.");
                return false;
            }
            _busy = true;
            return true;
        }

        private void Leave() => _busy = false;
    }
}
