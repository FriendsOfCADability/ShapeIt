using CADability.Curve2D;
using System.Linq;

namespace CADability.Tests
{
	[TestClass]
	public class Arc2DTests
	{
		[TestMethod]
		public void Trim_ZeroToZero_ReturnsNull()
		{
			var arc = new Arc2D(new GeoPoint2D(0, 0), 10, new Angle(0), new SweepAngle(90));
			var trimmed = arc.Trim(0.0, 0.0);
			Assert.IsNull(trimmed);
		}

		[TestMethod]
		public void Trim_FirstHalf_ReturnsValidArc()
		{
			var arc = new Arc2D(new GeoPoint2D(0, 0), 10, new Angle(0), new SweepAngle(90));
			var trimmed = arc.Trim(0.0, 0.5);
			Assert.IsNotNull(trimmed);
			Assert.IsTrue(trimmed.Length > 0);
		}

		[TestMethod]
		public void Trim_ReversedHalf_ReturnsValidArc()
		{
			var arc = new Arc2D(new GeoPoint2D(0, 0), 10, new Angle(0), new SweepAngle(90));
			var trimmed = arc.Trim(0.5, 0.0);
			Assert.IsNotNull(trimmed);
			Assert.IsTrue(trimmed.Length > 0);
		}

		[TestMethod]
		public void Trim_AlmostZeroSweep_ReturnsNull()
		{
			var arc = new Arc2D(new GeoPoint2D(0, 0), 10, new Angle(0), new SweepAngle(90));
			var trimmed = arc.Trim(0.5, 0.500000001);
			Assert.IsNull(trimmed);
		}

		/// <summary>
		/// TryPointDeriv2At must describe the same curve as PointAt. For a negative sweep it used to evaluate the
		/// circle at the mirrored angle, which made Derivative2At of surfaces built on arcs wrong.
		/// </summary>
		[TestMethod]
		public void TryPointDeriv2At_MatchesDifferencesOfPointAt()
		{
			double h = 1e-6;
			foreach (double sweep in new double[] { 2.5, -2.5, 5.0, -5.0 })
			{
				var arc = new Arc2D(new GeoPoint2D(10, 2), 3.0, 0.4, sweep);
				foreach (double pos in new double[] { 0.1, 0.45, 0.8 })
				{
					Assert.IsTrue(arc.TryPointDeriv2At(pos, out GeoPoint2D point, out GeoVector2D deriv1, out GeoVector2D deriv2));
					GeoVector2D fd1 = (1.0 / (2 * h)) * (arc.PointAt(pos + h) - arc.PointAt(pos - h));
					GeoVector2D fd2 = (1.0 / (2 * h)) * (arc.DirectionAt(pos + h) - arc.DirectionAt(pos - h));
					string at = $"sweep {sweep}, position {pos}";
					Assert.IsTrue((point | arc.PointAt(pos)) < 1e-12, $"point, {at}: {point} instead of {arc.PointAt(pos)}");
					Assert.IsTrue((fd1 - deriv1).Length < 1e-7 * fd1.Length, $"first derivative, {at}: {deriv1} instead of {fd1}");
					Assert.IsTrue((fd1 - arc.DirectionAt(pos)).Length < 1e-7 * fd1.Length, $"DirectionAt, {at}: {arc.DirectionAt(pos)} instead of {fd1}");
					Assert.IsTrue((fd2 - deriv2).Length < 1e-6 * fd2.Length, $"second derivative, {at}: {deriv2} instead of {fd2}");
				}
			}
		}
	}
}
