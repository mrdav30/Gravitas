using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests;

/// <content>Checks finite cylinder and cone supports against joined native solids.</content>
public sealed partial class JoinedStaticSolidMixedTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void CurvedSeamSupport_ShouldMatchTheContinuousWallAndClearTheWholeShape(bool cone, bool positiveX)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.Two),
            Rectangle(scene, 0, 4, -2, 0, Fixed64.Two) };
        LSCollider2D continuous = Rectangle(scene, -4, 4, -2, 0, Fixed64.Two);
        Fixed64 x = positiveX ? Fixed64.Quarter : -Fixed64.Quarter;
        LSCollider mover = CurvedMover(scene, cone, new(x, Fixed64.Zero, Fixed64.Quarter));
        var ordinary = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, continuous, ordinary));
        ManifoldContact expected = ordinary.PrimaryContact;
        Assert.Equal(Vector3d.Backward, expected.Normal); Assert.Equal(Fixed64.Quarter, expected.Depth);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.Equal(expected.Normal, sample.Row.Normal); Assert.Equal(expected.Depth, sample.Row.Depth);
            Assert.False(sample.Row.DepthIsClamped);
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(new Vector3d(x, source.Y, -Fixed64.Quarter), moving);
            Assert.Equal(x, source.X); Assert.Equal(Fixed64.Zero, source.Z);
            if (cone) Assert.Equal(-Fixed64.One, source.Y);
            else Assert.InRange(source.Y.m_rawValue, (-Fixed64.One).m_rawValue, Fixed64.One.m_rawValue);
            LSCollider2D supplier = sample.Supplier == members[0].Id + 1 ? members[0] : members[1];
            Assert.Equal(supplier.Id + 1, sample.Supplier);
            Assert.InRange(source.X.m_rawValue, supplier.Bounds.Min.X.m_rawValue, supplier.Bounds.Max.X.m_rawValue);
        }
        AssertWholeShapeExit(mover, members, new(x, Fixed64.Zero, Fixed64.Half), Vector3d.Forward);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CurvedJoinedWallCrossing_ShouldKeepTangentialVelocityAndMemberEvents(bool cone)
    {
        var control = Trace(joined: false);
        var expected = Trace(joined: true);
        Assert.Equal(expected, Trace(joined: true));
        for (int frame = 0; frame < control.Count; frame++)
            Assert.Equal(control[frame].Position.X, expected[frame].Position.X);

        List<(Vector3d Position, Vector3d Linear, Vector3d Angular, FixedQuaternion Rotation)> Trace(bool joined)
        {
            using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
            var members = joined
                ? new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.Two), Rectangle(scene, 0, 4, -2, 0, Fixed64.Two) }
                : new[] { Rectangle(scene, -4, 4, -2, 0, Fixed64.Two) };
            using var surface = joined ? scene.Context.Physics2D.CreateStaticSurface(members) : null;
            int entered = 0, exited = 0;
            foreach (LSCollider2D member in members)
            {
                member.OnMixedContactEnter += _ => entered++;
                member.OnMixedContactExit += _ => exited++;
            }
            LSCollider mover = CurvedMover(scene, cone, new(-Fixed64.Two, Fixed64.Zero, Fixed64.Half));
            SolidBody body = mover.Body!;
            body.FreezeAxes = BodyFreezeAxes3D.PositionY | BodyFreezeAxes3D.Rotation;
            body.SleepEnabled = false; body.UseManualGrounding();
            for (int frame = 0; frame < 32; frame++) Step();
            body.AddLinearImpulse(Vector3d.Right);
            var trace = new List<(Vector3d Position, Vector3d Linear, Vector3d Angular, FixedQuaternion Rotation)>();
            // A 1/16 inward impulse over a 1/16 step leaves steady excess
            // (1-.8)/.8 * 1/256 = 1/1024 above positional slop.
            Fixed64 clearance = CollisionResponseMixed.PenetrationSlop + Fixed64.FromFraction(1, 1024) + Fixed64.FromRaw(2);
            for (int frame = 0; frame < 64; frame++)
            {
                Step();
                Vector3d p = body.Position3d;
                FixedBoundBox bounds = cone
                    ? FixedBoundBox.FromCenteredFiniteConeClippedToDomain(p, body.Rotation, Vector3d.Up, Fixed64.Two, Fixed64.Half)
                    : FixedBoundBox.FromCenteredFiniteCylinderClippedToDomain(p, body.Rotation, Vector3d.Up, Fixed64.Two, Fixed64.Half);
                string state = $"cone={cone}, joined={joined}, frame={frame}, position={p}, velocity={body.LinearVelocity}";
                Assert.True(body.LinearVelocity.X == Fixed64.One, state);
                Assert.Equal(Fixed64.Zero, p.Y); Assert.Equal(Vector3d.Zero, body.AngularVelocity);
                // Response updates body pose before the next collider refresh.
                Assert.True(bounds.Min.Z >= -clearance, $"{state}, minimumZRaw={bounds.Min.Z.m_rawValue}");
                trace.Add((p, body.LinearVelocity, body.AngularVelocity, body.Rotation));
            }
            Assert.True(body.Position3d.X > Fixed64.One);
            Assert.Equal(joined ? 2 : 1, entered); Assert.Equal(joined ? 1 : 0, exited);
            return trace;

            void Step()
            {
                body.AddLinearImpulse(Vector3d.Backward * Fixed64.FromFraction(1, 16));
                scene.Context.Simulate(); scene.Context.LateSimulate();
            }
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CurvedConcaveCut_ShouldRetainBothClosedPhysicalFaceMaxima(bool cone)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -2, 0, -2, 2, Fixed64.Two),
            Rectangle(scene, 0, 2, -2, 0, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCollider mover = CurvedMover(scene, cone, new(-Fixed64.Quarter, Fixed64.Zero, -Fixed64.Quarter));
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        bool left = false, back = false;
        Fixed64 expected = Fixed64.Quarter + FixedMath.Sqrt(Fixed64.FromFraction(3, 16));
        foreach (var sample in rows)
        {
            Assert.True(sample.Row.Normal == Vector3d.Left || sample.Row.Normal == Vector3d.Backward);
            left |= sample.Row.Normal == Vector3d.Left; back |= sample.Row.Normal == Vector3d.Backward;
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(Fixed64.Zero, source.X); Assert.Equal(Fixed64.Zero, source.Z);
            if (cone) Assert.Equal(-Fixed64.One, source.Y);
            else Assert.InRange(source.Y.m_rawValue, (-Fixed64.One).m_rawValue, Fixed64.One.m_rawValue);
            // A radius-1/2 circular section at the retained cut has transverse
            // offset 1/4; four raw units cover sqrt and final witness rounding.
            Assert.InRange(sample.Row.Depth.m_rawValue, expected.m_rawValue - 4, expected.m_rawValue + 4);
            Vector3d paired = moving - sample.Row.Normal * sample.Row.Depth;
            Assert.True((paired - source).MagnitudeSquared <= Fixed64.FromRaw(4));
        }
        Assert.True(left && back);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CurvedTallBody_ShouldPreserveTheContinuousSolidMinimum(bool cone)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.One),
            Rectangle(scene, 0, 4, -2, 0, Fixed64.One) };
        LSCollider2D continuous = Rectangle(scene, -4, 4, -2, 0, Fixed64.One);
        LSCollider mover = cone
            ? new LSConeCollider { Radius = Fixed64.Half, Size = new(1, 6, 1) }
            : new LSCylinderCollider { Radius = Fixed64.Half, Size = new(1, 6, 1) };
        mover.Material = PhysicsMaterial.Frictionless;
        scene.CreateBody(mover, new(Fixed64.Quarter, Fixed64.Zero, Fixed64.Quarter), FixedQuaternion.Identity);
        var ordinary = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, continuous, ordinary));
        ManifoldContact expected = ordinary.PrimaryContact;
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        // A convex solid's minimum must not become opposing cap constraints
        // merely because its unchanged volume is split into declared members.
        foreach (var sample in rows)
        {
            Assert.Equal(expected.Normal, sample.Row.Normal);
            Assert.InRange(sample.Row.Depth.m_rawValue, expected.Depth.m_rawValue - 2, expected.Depth.m_rawValue + 2);
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void CurvedObliqueSupport_ShouldRetainTheNativeFaceOrOuterRim(bool cone, bool corner)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.Two),
            Rectangle(scene, 0, 4, -2, 0, Fixed64.Two) };
        LSCollider2D continuous = Rectangle(scene, -4, 4, -2, 0, Fixed64.Two);
        FixedQuaternion rotation = new(Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Two);
        LSCollider mover = cone
            ? new LSConeCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) }
            : new LSCylinderCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) };
        scene.CreateBody(mover, corner ? new(4 * Fixed64.One + Fixed64.Quarter, Fixed64.Zero, Fixed64.One)
            : new(Fixed64.Quarter, Fixed64.Zero, Fixed64.One), rotation);
        var ordinary = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, continuous, ordinary));
        ManifoldContact expected = ordinary.PrimaryContact;
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.Equal(expected.Normal, sample.Row.Normal);
            Assert.InRange(sample.Row.Depth.m_rawValue, expected.Depth.m_rawValue - 2, expected.Depth.m_rawValue + 2);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(Fixed64.Zero, source.Z);
            if (corner) Assert.Equal((Fixed64)4, source.X);
            Assert.InRange(source.Y.m_rawValue, (-Fixed64.Two).m_rawValue, Fixed64.Two.m_rawValue);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void HorizontalCurvedSeam_ShouldUseTheInverseAuthoredFan(bool cone)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.Two),
            Rectangle(scene, 0, 4, -2, 0, Fixed64.Two) };
        LSCollider2D continuous = Rectangle(scene, -4, 4, -2, 0, Fixed64.Two);
        // +90 degrees about Z sends the local axis to world -X. World +X
        // fan covectors therefore project to local -Y, not local +Y.
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One);
        LSCollider mover = cone
            ? new LSConeCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) }
            : new LSCylinderCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) };
        scene.CreateBody(mover, new(Fixed64.Quarter, Fixed64.Zero, Fixed64.Quarter), rotation);
        var ordinary = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, continuous, ordinary));
        ManifoldContact expected = ordinary.PrimaryContact;
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.Equal(expected.Normal, sample.Row.Normal);
            Assert.InRange(sample.Row.Depth.m_rawValue, expected.Depth.m_rawValue - 2, expected.Depth.m_rawValue + 2);
        }
    }

    [Fact]
    public void ConeGenerator_ShouldSelectTheInitialPhysicalInteriorOfItsSupportFamily()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, 0, 2, -8, 0, (Fixed64)5),
            Rectangle(scene, 0, 2, 0, 8, (Fixed64)5) };
        LSCollider2D continuous = Rectangle(scene, 0, 2, -8, 8, (Fixed64)5);
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(-1358187913), Fixed64.FromRaw(4074563739));
        var mover = new LSConeCollider { Radius = (Fixed64)3, Size = new(6, 4, 6) };
        scene.CreateBody(mover, new(-1, 0, -2), rotation);
        var ordinary = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, continuous, ordinary));
        Assert.Equal(Vector3d.Right, ordinary.PrimaryContact.Normal);
        Assert.InRange(ordinary.PrimaryContact.Depth.m_rawValue,
            Fixed64.FromFraction(1, 5).m_rawValue - 1, Fixed64.FromFraction(1, 5).m_rawValue + 1);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.Equal(Vector3d.Right, sample.Row.Normal);
            Assert.Equal(ordinary.PrimaryContact.Depth, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(Fixed64.Zero, source.X); Assert.Equal((Fixed64)(-2), source.Z);
            // With exact cos=4/5,sin=-3/5 the translated generator spans
            // Y=[-17/5,8/5], but its initial-body section is [-13/4,37/24].
            Assert.InRange(source.Y.m_rawValue, Fixed64.FromFraction(-13, 4).m_rawValue - 2,
                Fixed64.FromFraction(37, 24).m_rawValue + 2);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CurvedThinSlabSupport_ShouldUseTheNonoverlappingPhysicalSupplier(bool cone)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        Fixed64 half = Fixed64.FromFraction(1, 8);
        var members = new[] { Rectangle(scene, -3, -1, -2, 2, half),
            Rectangle(scene, -1, 1, -2, 2, half), Rectangle(scene, 1, 3, -2, 2, half) };
        LSCollider2D continuous = Rectangle(scene, -3, 3, -2, 2, half);
        LSCollider mover = cone
            ? new LSConeCollider { Radius = Fixed64.Quarter, Size = new(1, 4, 1) }
            : new LSCylinderCollider { Radius = Fixed64.Quarter, Size = new(1, 4, 1) };
        // Break the exact Up/Down tie so this fixture has one geometric exit;
        // its purpose is supplier ownership, not cross-representation tie order.
        scene.CreateBody(mover, Vector3d.Up * Fixed64.FromRaw(8),
            new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.Two).Normalized);
        var ordinary = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, members[1], ordinary));
        Assert.False(CollisionDetectionMixed.TryBuildManifold(mover, members[0], ordinary));
        Assert.False(CollisionDetectionMixed.TryBuildManifold(mover, members[2], ordinary));
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, continuous, ordinary));
        ManifoldContact expected = ordinary.PrimaryContact;
        Assert.Equal(Fixed64.One, FixedMath.Abs(expected.Normal.Y));
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.Equal(expected.Normal, sample.Row.Normal); Assert.Equal(expected.Depth, sample.Row.Depth);
            Assert.NotEqual(members[1].Id + 1, sample.Supplier);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.True(FixedMath.Abs(source.X) > Fixed64.One);
            Assert.Equal(half, FixedMath.Abs(source.Y));
        }
    }

    [Fact]
    public void CylinderOuterCapRim_ShouldIgnoreHeightCutsBelowItsPhysicalEdge()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var wall = Rectangle(scene, -3, 3, -2, 0, Fixed64.Two);
        var back = Rectangle(scene, -3, 3, -4, -2, Fixed64.Two);
        var lowTab = Rectangle(scene, -1, 1, 0, 2, Fixed64.One);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { wall, back });
        var mover = new LSCylinderCollider { Radius = Fixed64.Half, Size = new(1, 4, 1) };
        scene.CreateBody(mover, new(Fixed64.Zero, 9 * Fixed64.Quarter, Fixed64.Quarter),
            new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized);
        var expected = Capture(); Assert.NotEmpty(expected);
        foreach (var row in expected)
        {
            Assert.Equal(wall.Id + 1, row.Supplier);
            Assert.Equal(Fixed64.Two, row.Source.Y); Assert.Equal(Fixed64.Zero, row.Source.Z);
            Assert.True(row.Normal.Y < Fixed64.Zero && row.Normal.Z < Fixed64.Zero);
        }
        var ordinary = new ContactManifold();
        Assert.False(CollisionDetectionMixed.TryBuildManifold(mover, lowTab, ordinary));
        surface.ReplaceMembers(new[] { wall, back, lowTab });
        Assert.Equal(expected, Capture());

        List<(Vector3d Normal, Fixed64 Depth, Vector3d Moving, Vector3d Source, int Supplier)> Capture()
        {
            var result = new List<(Vector3d Normal, Fixed64 Depth, Vector3d Moving, Vector3d Source, int Supplier)>();
            foreach (var sample in Rows(mover, wall, back, lowTab))
            {
                Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
                Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
                result.Add((sample.Row.Normal, sample.Row.Depth, moving, source, sample.Supplier));
            }
            return result;
        }
    }

    private static LSCollider CurvedMover(PhysicsScenarioBuilder scene, bool cone, Vector3d position)
    {
        LSCollider mover = cone
            ? new LSConeCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) }
            : new LSCylinderCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) };
        mover.Material = PhysicsMaterial.Frictionless;
        scene.CreateBody(mover, position, FixedQuaternion.Identity);
        return mover;
    }
}
