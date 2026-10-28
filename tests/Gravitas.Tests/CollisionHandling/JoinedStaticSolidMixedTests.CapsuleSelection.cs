using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests;

/// <content>Checks native exits and finite cut alternatives for joined capsule response.</content>
public sealed partial class JoinedStaticSolidMixedTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void CapsuleUnequalHeightJoin_ShouldPreserveUnforcedAxesDuringAFullStep(bool reverse, bool remote)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        LSCollider2D first = reverse ? Rectangle(scene, 0, 2, -2, 0, Fixed64.One, 3 * Fixed64.Half)
            : Rectangle(scene, -2, 0, -2, 0, (Fixed64)3);
        LSCollider2D second = reverse ? Rectangle(scene, -2, 0, -2, 0, (Fixed64)3)
            : Rectangle(scene, 0, 2, -2, 0, Fixed64.One, 3 * Fixed64.Half);
        var members = remote ? new[] { first, second, Rectangle(scene, -2, 0, -4, -2, Fixed64.One, -Fixed64.Two) }
            : new[] { first, second };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var mover = new LSCapsuleCollider
        { Radius = Fixed64.Half, Size = new(1, 6, 1), Material = PhysicsMaterial.Frictionless };
        SolidBody body = scene.CreateBody(mover, Vector3d.Forward * Fixed64.Quarter, FixedQuaternion.Identity).Body;
        body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        body.SleepEnabled = false; body.UseManualGrounding();
        body.AddLinearImpulse(Vector3d.Backward * Fixed64.FromFraction(1, 16));
        scene.Context.Simulate(); scene.Context.LateSimulate();
        Vector3d position = body.Position3d, velocity = body.LinearVelocity;

        // Check the complete finite capsule after the measured step: the
        // common front plane is exited horizontally without any X/Y motion.
        AssertWholeShapeExit(mover, members, Vector3d.Forward * Fixed64.Half, Vector3d.Forward);
        Assert.Equal(Fixed64.Zero, position.X);
        Assert.Equal(Fixed64.Zero, position.Y);
        Assert.InRange(position.Z.m_rawValue, Fixed64.Quarter.m_rawValue, Fixed64.Half.m_rawValue);
        Assert.Equal(Vector3d.Zero, velocity);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void TallCapsuleConcaveCorner_ShouldRetainBothRequiredLateralCuts(bool asymmetric)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -2, 0, -2, 2, Fixed64.Two),
            Rectangle(scene, 0, 2, -2, 0, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        Fixed64 x = asymmetric ? Fixed64.FromFraction(1, 8) : Fixed64.Quarter;
        var mover = new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 6, 1) };
        scene.CreateBody(mover, new(-x, Fixed64.Zero, -Fixed64.Quarter), FixedQuaternion.Identity);
        bool left = false, back = false;
        foreach (var sample in Rows(mover, members))
        {
            Fixed64 expected;
            if (sample.Row.Normal == Vector3d.Left)
            { left = true; expected = x + FixedMath.Sqrt(Fixed64.Quarter - Fixed64.Quarter * Fixed64.Quarter); }
            else
            {
                Assert.Equal(Vector3d.Backward, sample.Row.Normal); back = true;
                expected = Fixed64.Quarter + FixedMath.Sqrt(Fixed64.Quarter - x * x);
            }
            Assert.InRange(sample.Row.Depth.m_rawValue, expected.m_rawValue - 4, expected.m_rawValue + 4);
        }
        Assert.True(left && back);
    }

    [Fact]
    public void CapsuleOutsideAConcaveWindow_ShouldRetainItsDeepestFiniteBoundaryPeak()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var wall = Rectangle(scene, -2, 2, -4, 0, Fixed64.Two);
        var members = new[] { wall, Rectangle(scene, -2, -1, 0, 2, Fixed64.Two), Rectangle(scene, 1, 2, 0, 2, Fixed64.Two),
            Rectangle(scene, -1, 1, 0, 2, Fixed64.Half, -3 * Fixed64.Half),
            Rectangle(scene, -1, 1, 0, 2, Fixed64.Half, 3 * Fixed64.Half) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var mover = new LSCapsuleCollider { Radius = Fixed64.Two, Size = new(4, 5, 4) };
        var center = new Vector3d(5 * Fixed64.Quarter, Fixed64.Zero, Fixed64.Quarter);
        scene.CreateBody(mover, center, FixedQuaternion.Identity);
        var rows = Rows(mover, wall);
        Assert.NotEmpty(rows);
        Assert.True(rows.Count <= ContactManifold.MaxContactsPerGroup);
        var regions = new HashSet<int>();
        bool deepest = false, finiteCorner = false;
        foreach (var sample in rows)
        {
            Assert.Equal(Vector3d.Backward, sample.Row.Normal);
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(Fixed64.Zero, source.Z);
            Assert.InRange(source.X.m_rawValue, (-Fixed64.One).m_rawValue, Fixed64.One.m_rawValue);
            Assert.InRange(source.Y.m_rawValue, (-Fixed64.One).m_rawValue, Fixed64.One.m_rawValue);
            // The center projects into the joined right tab. Its exposed
            // window rays reach the finite core, whose endpoints are Y=+/-1/2.
            Fixed64 dx = source.X - center.X;
            Fixed64 dy = FixedMath.Max(Fixed64.Zero, source.Y.Abs() - Fixed64.Half);
            Fixed64 expected = FixedMath.Sqrt(4 * Fixed64.One - dx * dx - dy * dy) - Fixed64.Quarter;
            Assert.InRange(sample.Row.Depth.m_rawValue, expected.m_rawValue - 4, expected.m_rawValue + 4);
            Assert.Equal(source.X, moving.X); Assert.Equal(source.Y, moving.Y);
            Assert.True((moving.Z + sample.Row.Depth).Abs() <= Fixed64.MinIncrement);
            deepest |= source.X == Fixed64.One && source.Y.Abs() <= Fixed64.Half;
            finiteCorner |= source.X == Fixed64.One && source.Y.Abs() == Fixed64.One;
            regions.Add(sample.Region);
        }
        // The core-aligned right rim is deeper than either corner; both the
        // peak and a genuine corner must survive the bounded contact reduction.
        Assert.True(deepest && finiteCorner);
        Assert.Single(regions);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void CapsuleNativeAlternative_ShouldCompeteWithTheCompleteSourceCutSet(int heightCase)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var source = Rectangle(scene, -2, 2, -2, 2, Fixed64.Half);
        var members = new[] { source, Rectangle(scene, 2, 4, 0, 2, Fixed64.Half),
            Rectangle(scene, 0, 2, 2, 4, Fixed64.Half) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        Fixed64 height = heightCase == 2 ? (Fixed64)7 : heightCase == 1 ? (Fixed64)6 : 11 * Fixed64.Half;
        var mover = new LSCapsuleCollider { Radius = 5 * Fixed64.Half, Size = new((Fixed64)5, height, (Fixed64)5) };
        scene.CreateBody(mover, new(Fixed64.Half, Fixed64.Half, Fixed64.Two), FixedQuaternion.Identity);
        bool left = false, back = false, cap = false;
        foreach (var sample in Rows(mover, members))
        {
            if (sample.Supplier != source.Id + 1) continue;
            if (sample.Row.Normal == Vector3d.Left) { left = true; Assert.Equal((Fixed64)3, sample.Row.Depth); }
            else if (sample.Row.Normal == Vector3d.Backward)
            {
                back = true; Fixed64 expected = FixedMath.Sqrt((Fixed64)6);
                Assert.InRange(sample.Row.Depth.m_rawValue, expected.m_rawValue - 4, expected.m_rawValue + 4);
            }
            else
            { cap = true; Assert.Equal(Vector3d.Down, sample.Row.Normal); Assert.Equal(height * Fixed64.Half, sample.Row.Depth); }
        }
        // Native 2.75 and its exact tie at 3 beat MAX(sqrt(6),3); 3.5 loses.
        if (heightCase == 2) Assert.True(left && back);
        else { Assert.False(left || back); Assert.True(cap); }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CapsulePartlyParentedCut_ShouldKeepItsIndependentFragmentAndContractTheWallOverlap(bool mirrored)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var wall = Rectangle(scene, -2, 2, -2, 0, Fixed64.Two);
        var members = new[] { wall, Rectangle(scene, -1, 1, 0, 2, Fixed64.One, -Fixed64.One),
            mirrored ? Rectangle(scene, -2, -1, 0, 2, Fixed64.One, Fixed64.One)
                : Rectangle(scene, 1, 2, 0, 2, Fixed64.One, Fixed64.One) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        Fixed64 side = mirrored ? -Fixed64.One : Fixed64.One;
        var mover = CapsuleMover(scene, new(side * 3 * Fixed64.Quarter, 5 * Fixed64.Quarter, Fixed64.Quarter), false, Fixed64.Two);
        Fixed64 expected = FixedMath.Sqrt(Fixed64.FromFraction(1, 8)) - Fixed64.Quarter;
        bool fragment = false;
        foreach (var sample in Rows(mover, wall))
        {
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d point));
            if (sample.Row.Normal != Vector3d.Backward || point != new Vector3d(side, Fixed64.Zero, Fixed64.Zero)) continue;
            fragment = true;
            Assert.InRange(sample.Row.Depth.m_rawValue, expected.m_rawValue - 4, expected.m_rawValue + 4);
        }
        Assert.True(fragment);
        SolidBody body = mover.Body!; body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        body.SleepEnabled = false; body.UseManualGrounding();
        for (int frame = 0; frame < 16; frame++) { scene.Context.Simulate(); scene.Context.LateSimulate(); }
        // The lower neighbor really overlaps the bottom cap by 1/4, so a
        // vertical correction is required here, unlike the front-wall fixture.
        Assert.InRange(body.Position3d.Y.m_rawValue, (5 * Fixed64.Quarter).m_rawValue, (3 * Fixed64.Half).m_rawValue);
        // A finite source row is not a whole-union exit. Check the complete
        // response after all members solve, using the authoritative final pose.
        SetPosition(mover, body.Position3d);
        var ordinary = new Gravitas.CollisionHandling.ContactManifold();
        if (Gravitas.CollisionHandling.CollisionDetectionMixed.TryBuildManifold(mover, wall, ordinary))
            Assert.InRange(ordinary.PrimaryContact.Depth.m_rawValue, 0, Fixed64.FromFraction(1, 100).m_rawValue);
    }
}
