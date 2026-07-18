using CADability.GeoObject;
using System;
using System.Globalization;
using System.IO;

namespace CADability
{
    /// <summary>
    /// Imports STL files. The triangles are collected in a lightweight <see cref="StlTriangleMesh"/> (no BRep
    /// objects are created for the individual triangles, which used to make big STL files unusable). The mesh is
    /// then passed to <see cref="StlSurfaceReconstruction"/>, which segments the triangles into regions of small
    /// bending angles and tries to recognize standard surfaces (plane, cylinder, cone, sphere, torus).
    /// This is work in progress (stage 1 of the STL reverse engineering): <see cref="Read(string)"/> currently
    /// returns an empty array, the recognized surfaces are provided as raw faces (bounded by their uv extent, not
    /// by the true outline) in <see cref="RecognizedFaces"/> for visual inspection.
    /// </summary>
    public class ImportSTL
    {
        private StreamReader sr;
        private bool isASCII;
        private BinaryReader br;
        private int numdec = 0, numnum = 0; // to estimate the coordinate resolution of ASCII files from the number of decimal places

        public ImportSTL()
        {
        }
        /// <summary>
        /// After a call to <see cref="Read(string)"/> this list contains the recognition result: for each recognized
        /// region a raw face on the fitted surface, colored by surface type (plane: green, cylinder: blue,
        /// cone: orange, sphere: red, torus: violet); unrecognized regions as gray triangles.
        /// </summary>
        public GeoObjectList RecognizedFaces { get; private set; }
        /// <summary>
        /// The triangle mesh created by the last call to <see cref="Read(string)"/>.
        /// </summary>
        public StlTriangleMesh Mesh { get; private set; }
        /// <summary>
        /// The result of the surface recognition of the last call to <see cref="Read(string)"/>.
        /// </summary>
        public StlSurfaceReconstruction Reconstruction { get; private set; }
        /// <summary>
        /// When true (the default), <see cref="Read(string)"/> also builds the raw faces (<see cref="RecognizedFaces"/>).
        /// Set to false to only run the surface recognition (populates <see cref="StlSurfaceReconstruction.Regions"/>),
        /// which skips the still fragile / incomplete stage 2 face building.
        /// </summary>
        public bool BuildRawFaces { get; set; } = true;

        public Shell[] Read(string fileName)
        {
            using (FileStream fs = File.OpenRead(fileName)) // may throw exceptions like "file not found" etc.
            {
                return Read(fs);
            }
        }

        public Shell[] Read(byte[] byteArray)
        {
            using (MemoryStream ms = new MemoryStream(byteArray))
            {
                return Read(ms);
            }
        }

        public Shell[] Read(Stream stream)
        {
            long startPosition = stream.Position;
            byte[] head = new byte[5];
            if (stream.Read(head, 0, 5) != 5) throw new ApplicationException("cannot read from stream");
            isASCII = head[0] == 's' && head[1] == 'o' && head[2] == 'l' && head[3] == 'i' && head[4] == 'd';
            stream.Position = startPosition;
            StlTriangleMesh mesh = ReadMesh(stream);
            if (mesh.TriangleCount == 0 && isASCII)
            {   // some binary STL files also start with "solid": try again in binary mode
                isASCII = false;
                numdec = numnum = 0;
                stream.Position = startPosition;
                mesh = ReadMesh(stream);
            }
            mesh.Finish();
            Mesh = mesh;
            RecognizedFaces = new GeoObjectList();
            Reconstruction = null;
            if (mesh.TriangleCount > 0)
            {
                double extent = mesh.Extent.Size;
                double precision;
                if (numnum > 0) precision = Math.Pow(10, -numdec / (double)numnum); // numdec/numnum is the average number of decimal places
                else precision = extent * 1e-5; // binary STL: single precision floats
                precision = Math.Max(Math.Min(precision, extent * 1e-3), extent * 1e-7);
                Reconstruction = new StlSurfaceReconstruction(mesh, precision);
                if (BuildRawFaces)
                {
                    GeoObjectList rawFaces = Reconstruction.CreateRawFaces(); // inspect this list in the debugger to see the result
                    RecognizedFaces = rawFaces;
                }
                else
                {
                    Reconstruction.Recognize(); // populate Regions only, skip the (stage 2) raw face building
                }
            }
            // TODO (stage 2): build a Shell from the recognized surfaces with proper edges (intersection curves of
            // adjacent surfaces) and return it here
            return new Shell[0];
        }

        private StlTriangleMesh ReadMesh(Stream stream)
        {
            StlTriangleMesh mesh = new StlTriangleMesh();
            uint expectedTriangles = uint.MaxValue;
            if (isASCII)
            {
                sr = new StreamReader(stream);
                br = null;
                sr.ReadLine(); // the "solid ..." title line
            }
            else
            {
                br = new BinaryReader(stream);
                sr = null;
                br.ReadBytes(80); // the header
                expectedTriangles = br.ReadUInt32();
            }
            for (uint i = 0; i < expectedTriangles; i++)
            {
                if (!GetNextTriangle(out GeoPoint p1, out GeoPoint p2, out GeoPoint p3, out GeoVector normal)) break;
                // the normal of the STL file determines the orientation, the vertex order in the mesh is made consistent with it
                if (((p2 - p1) ^ (p3 - p2)) * normal >= 0) mesh.AddTriangle(p1, p2, p3);
                else mesh.AddTriangle(p1, p3, p2);
            }
            sr = null;
            br = null; // the underlying stream is closed by the caller
            return mesh;
        }

        private void accumulatePrecision(params string[] number)
        {
            for (int i = 0; i < number.Length; i++)
            {
                if (number[i].IndexOf('.') > 0)
                {
                    ++numnum;
                    numdec += number[i].Length - number[i].IndexOf('.') - 1;
                }
            }
        }

        private bool ParseVertex(string line, string expectedTag, out GeoPoint p)
        {
            p = GeoPoint.Origin;
            if (line == null) return false;
            string[] parts = line.Trim().Split(new char[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length != 4 || parts[0] != expectedTag) return false;
            NumberStyles style = NumberStyles.AllowDecimalPoint | NumberStyles.AllowExponent | NumberStyles.AllowLeadingSign;
            if (!double.TryParse(parts[1], style, CultureInfo.InvariantCulture, out double x)) return false;
            if (!double.TryParse(parts[2], style, CultureInfo.InvariantCulture, out double y)) return false;
            if (!double.TryParse(parts[3], style, CultureInfo.InvariantCulture, out double z)) return false;
            accumulatePrecision(parts[1], parts[2], parts[3]);
            p = new GeoPoint(x, y, z);
            return true;
        }

        private bool GetNextTriangle(out GeoPoint p1, out GeoPoint p2, out GeoPoint p3, out GeoVector normal)
        {
            p1 = p2 = p3 = GeoPoint.Origin;
            normal = GeoVector.NullVector;
            if (isASCII)
            {
                try
                {
                    if (sr.EndOfStream) return false;
                    string line = sr.ReadLine();
                    if (line == null) return false;
                    string[] facet = line.Trim().Split(new char[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
                    if (facet.Length != 5 || facet[0] != "facet" || facet[1] != "normal") return false; // e.g. "endsolid"
                    NumberStyles style = NumberStyles.AllowDecimalPoint | NumberStyles.AllowExponent | NumberStyles.AllowLeadingSign;
                    if (!double.TryParse(facet[2], style, CultureInfo.InvariantCulture, out double nx)) return false;
                    if (!double.TryParse(facet[3], style, CultureInfo.InvariantCulture, out double ny)) return false;
                    if (!double.TryParse(facet[4], style, CultureInfo.InvariantCulture, out double nz)) return false;
                    if (sr.ReadLine()?.Trim() != "outer loop") return false;
                    if (!ParseVertex(sr.ReadLine(), "vertex", out p1)) return false;
                    if (!ParseVertex(sr.ReadLine(), "vertex", out p2)) return false;
                    if (!ParseVertex(sr.ReadLine(), "vertex", out p3)) return false;
                    if (sr.ReadLine()?.Trim() != "endloop") return false;
                    if (sr.ReadLine()?.Trim() != "endfacet") return false;
                    normal = new GeoVector(nx, ny, nz);
                    return true;
                }
                catch (IOException)
                {
                    return false;
                }
            }
            else
            {
                try
                {
                    if (br.BaseStream.Position + 50 > br.BaseStream.Length) return false; // 12 floats + 2 bytes attribute
                    normal = new GeoVector(br.ReadSingle(), br.ReadSingle(), br.ReadSingle());
                    p1 = new GeoPoint(br.ReadSingle(), br.ReadSingle(), br.ReadSingle());
                    p2 = new GeoPoint(br.ReadSingle(), br.ReadSingle(), br.ReadSingle());
                    p3 = new GeoPoint(br.ReadSingle(), br.ReadSingle(), br.ReadSingle());
                    br.ReadUInt16(); // attribute byte count, unused
                    return true;
                }
                catch (EndOfStreamException)
                {
                    return false;
                }
            }
        }
    }
}
