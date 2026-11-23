using CADability.Attribute;
using CADability.GeoObject;
using Color = CADability.Substitutes.Color;
using System.Text.RegularExpressions;
using IOPath = System.IO.Path;

namespace CADability.Tests
{
    /// <summary>
    /// The STEP export (ExportStep) writes the active model as a single part: one PRODUCT, whose shape representation is
    /// related to the representations of the solids, the shells and the curves. Blocks are resolved, colors are styled
    /// items of a presentation representation, names are encoded as STEP strings.
    /// </summary>
    [TestClass]
    public class StepExportTests
    {
        private const string bodyName = "Grundk\u00f6rper";
        private const string quotedName = "it's a \\ box";

        private static Project TwoSolidsAndCurves()
        {
            Project project = Project.CreateSimpleProject();
            Model model = project.GetActiveModel();
            Solid body = Make3D.MakeBox(GeoPoint.Origin, 10 * GeoVector.XAxis, 20 * GeoVector.YAxis, 30 * GeoVector.ZAxis);
            body.Name = bodyName;
            body.ColorDef = new ColorDef("Rot", Color.FromArgb(255, 0, 0));
            model.Add(body);
            Solid box = Make3D.MakeBox(new GeoPoint(50, 0, 0), 5 * GeoVector.XAxis, 5 * GeoVector.YAxis, 5 * GeoVector.ZAxis);
            box.Name = quotedName;
            box.ColorDef = new ColorDef("Blau", Color.FromArgb(0, 0, 255));
            model.Add(box);
            // the curves are inside a block, which is resolved by the export
            Line line = Line.TwoPoints(new GeoPoint(0, -10, 0), new GeoPoint(10, -10, 0));
            line.ColorDef = new ColorDef("Gruen", Color.FromArgb(0, 128, 0));
            Ellipse arc = Ellipse.Construct();
            arc.SetArcPlaneCenterRadiusAngles(Plane.XYPlane, new GeoPoint(0, -20, 0), 5, 0, Math.PI / 2);
            Block block = Block.Construct();
            block.Name = "Skizze";
            block.Add(line);
            block.Add(arc);
            model.Add(block);
            return project;
        }

        private static GeoObjectList ExportAndImport(Project project, out string step, out ImportStep import)
        {
            string tmp = IOPath.Combine(IOPath.GetTempPath(), "StepExport_" + Guid.NewGuid().ToString("N") + ".stp");
            try
            {
                new ExportStep().WriteToFile(tmp, project);
                step = File.ReadAllText(tmp);
                import = new ImportStep();
                return import.Read(tmp);
            }
            finally
            {
                if (File.Exists(tmp)) File.Delete(tmp);
            }
        }

        [TestMethod]
        public void the_model_is_written_as_a_single_part()
        {
            ExportAndImport(TwoSolidsAndCurves(), out string step, out _);
            Assert.AreEqual(1, Regex.Matches(step, @"=PRODUCT\(").Count, "one product");
            Assert.AreEqual(1, Regex.Matches(step, @"=SHAPE_DEFINITION_REPRESENTATION\(").Count, "one shape");
            Assert.IsFalse(step.Contains("MAPPED_ITEM"), "no mapped items");
            Assert.IsFalse(step.Contains("NEXT_ASSEMBLY_USAGE_OCCURRENCE"), "no assembly");
            Assert.IsTrue(step.Contains("APPLICATION_PROTOCOL_DEFINITION("), "the application protocol");
            Assert.AreEqual(1, Regex.Matches(step, @"=ADVANCED_BREP_SHAPE_REPRESENTATION\(").Count, "the solids");
            Assert.AreEqual(1, Regex.Matches(step, @"=GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION\(").Count, "the curves");
            Assert.AreEqual(2, Regex.Matches(step, @"=SHAPE_REPRESENTATION_RELATIONSHIP\(").Count, "solids and curves related to the shape");
            Assert.AreEqual(2, Regex.Matches(step, @"=MANIFOLD_SOLID_BREP\(").Count, "two solids");
            Assert.AreEqual(1, Regex.Matches(step, @"=MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION\(").Count, "the styled items are presented");
            Assert.IsTrue(step.Contains("'Grundk\\X2\\00F6\\X0\\rper'"), "a non ASCII character is encoded");
            Assert.IsTrue(step.Contains("'it''s a \\\\ box'"), "apostrophe and backslash are doubled");
            Assert.IsFalse(step.Contains("%2"), "no placeholder in the header");
            foreach (char c in step) Assert.IsTrue(c == '\r' || c == '\n' || (c >= ' ' && c <= '~'), "only printable ASCII characters");
        }

        [TestMethod]
        public void the_part_is_imported_again_with_names_and_colors()
        {
            GeoObjectList imported = ExportAndImport(TwoSolidsAndCurves(), out _, out ImportStep import);
            Assert.AreEqual(0, import.Problems.Count, "import problems: " + string.Join("; ", import.Problems.Select(p => $"#{p.Key} {p.Value}")));
            Assert.IsFalse(imported.Any(go => go is Block), "no blocks");

            List<Solid> solids = imported.OfType<Solid>().ToList();
            Assert.AreEqual(2, solids.Count, "two solids");
            Solid body = solids.Single(s => s.Name == bodyName);
            Solid box = solids.Single(s => s.Name == quotedName);
            Assert.AreEqual(6000.0, body.Volume(1e-3), 1e-6, "volume of the body");
            Assert.AreEqual(125.0, box.Volume(1e-3), 1e-6, "volume of the box");
            AssertColor(255, 0, 0, body.ColorDef, "body");
            AssertColor(0, 0, 255, box.ColorDef, "box");

            List<ICurve> curves = imported.OfType<ICurve>().ToList();
            Assert.AreEqual(2, curves.Count, "line and arc");
            ICurve line = curves.Single(c => c is Line);
            Assert.AreEqual(10.0, line.Length, 1e-9, "length of the line");
            AssertColor(0, 128, 0, (line as IColorDef).ColorDef, "line");
            ICurve arc = curves.Single(c => c is Ellipse);
            Assert.AreEqual(5 * Math.PI / 2, arc.Length, 1e-9, "length of the arc");
        }

        /// <summary>
        /// A single solid names the part, so it keeps its name, although the import names a single solid after its product.
        /// </summary>
        [TestMethod]
        public void a_single_solid_keeps_its_name()
        {
            Project project = Project.CreateSimpleProject();
            Solid box = Make3D.MakeBox(GeoPoint.Origin, 10 * GeoVector.XAxis, 10 * GeoVector.YAxis, 10 * GeoVector.ZAxis);
            box.Name = "Wuerfel";
            project.GetActiveModel().Add(box);
            GeoObjectList imported = ExportAndImport(project, out string step, out _);
            Assert.IsTrue(step.Contains("PRODUCT('Wuerfel','Wuerfel'"), "the product is named after the solid");
            Assert.AreEqual("Wuerfel", imported.OfType<Solid>().Single().Name);
        }

        /// <summary>
        /// A bar with three spherical cavities, as in the RPC case HollowBarCutCavities: the cavities are connected components of
        /// the shell of the solid. STEP writes such a solid as BREP_WITH_VOIDS, the import restores the cavities.
        /// </summary>
        [TestMethod]
        public void a_solid_with_cavities_is_a_brep_with_voids()
        {
            Solid bar = Make3D.MakeBox(GeoPoint.Origin, 30 * GeoVector.XAxis, 10 * GeoVector.YAxis, 10 * GeoVector.ZAxis);
            foreach (double x in new[] { 5.0, 15.0, 25.0 })
            {
                Solid[] res = Solid.Subtract(bar, Make3D.MakeSphere(new GeoPoint(x, 5, 5), 3));
                Assert.AreEqual(1, res.Length, "the sphere becomes a cavity");
                bar = res[0];
            }
            Assert.AreEqual(3, bar.Shells[0].GetHullAndHoles().holes.Length, "three cavities before the export");
            bar.Name = "Hohlstab";
            Project project = Project.CreateSimpleProject();
            project.GetActiveModel().Add(bar);

            GeoObjectList imported = ExportAndImport(project, out string step, out ImportStep import);
            Assert.AreEqual(1, Regex.Matches(step, @"=BREP_WITH_VOIDS\(").Count, "one solid with voids");
            Assert.AreEqual(3, Regex.Matches(step, @"=ORIENTED_CLOSED_SHELL\('',\*,#\d+,\.F\.\)").Count, "three voids");
            Assert.AreEqual(4, Regex.Matches(step, @"=CLOSED_SHELL\(").Count, "the hull and the three voids");
            Assert.IsFalse(step.Contains("=MANIFOLD_SOLID_BREP("), "no simple solid");
            Assert.AreEqual(0, import.Problems.Count, "import problems: " + string.Join("; ", import.Problems.Select(p => $"#{p.Key} {p.Value}")));

            Solid solid = imported.OfType<Solid>().Single();
            Assert.AreEqual("Hohlstab", solid.Name);
            Assert.AreEqual(0, solid.Shells[0].OpenEdgesExceptPoles.Length, "closed");
            (HashSet<Face> hull, HashSet<Face>[] holes) = solid.Shells[0].GetHullAndHoles();
            Assert.AreEqual(3, holes.Length, "three cavities after the import");
            double expected = 3000 - 3 * 4.0 / 3.0 * Math.PI * 27; // 3000 - 108 pi
            Assert.AreEqual(expected, solid.Volume(1e-3), expected * 1e-4, "the volume of the cavities is missing");
            foreach (HashSet<Face> hole in holes)
                Assert.IsTrue(Shell.SignedVolume(hole, 1e-3) < 0, "the normals of a cavity point into the cavity");
        }

        [TestMethod]
        public void step_strings_are_decoded()
        {
            Assert.AreEqual("it's", ImportStep.DecodeString("it''s"));
            Assert.AreEqual("a\\b", ImportStep.DecodeString("a\\\\b"));
            Assert.AreEqual("Volumenk\u00f6rper1", ImportStep.DecodeString("Volumenk\\X\\F6rper1"));
            Assert.AreEqual("Grundk\u00f6rper", ImportStep.DecodeString("Grundk\\X2\\00F6\\X0\\rper"));
            Assert.AreEqual("\u00c4\u00d6", ImportStep.DecodeString("\\X2\\00C400D6\\X0\\"));
            Assert.AreEqual("\U0001F600", ImportStep.DecodeString("\\X4\\0001F600\\X0\\"));
            Assert.AreEqual("\u00e9", ImportStep.DecodeString("\\S\\i"));
            Assert.AreEqual("C:\\temp", ImportStep.DecodeString("C:\\temp"), "a single backslash is kept");
            foreach (string s in new[] { "Grundk\u00f6rper", "it's a \\ box", "\U0001F600 x\u00e4", "plain" })
            {
                string literal = ExportStep.StepString(s);
                Assert.AreEqual(s, ImportStep.DecodeString(literal.Substring(1, literal.Length - 2)), $"round trip of {s}");
            }
        }

        private static void AssertColor(int r, int g, int b, ColorDef colorDef, string what)
        {
            Assert.IsNotNull(colorDef, $"color of the {what}");
            Assert.AreEqual(r, (int)colorDef.Color.R, $"red of the {what}");
            Assert.AreEqual(g, (int)colorDef.Color.G, $"green of the {what}");
            Assert.AreEqual(b, (int)colorDef.Color.B, $"blue of the {what}");
        }
    }
}
