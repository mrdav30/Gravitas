using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests;

/// <summary>Exercises native full-solid contacts against declared embedded surfaces.</summary>
/// <content>Checks geometry, finite suppliers and complete host crossings.</content>
public sealed partial class JoinedStaticSolidMixedTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void GenuineOuterCapAndSide_ShouldRetainTheirFiniteWholeShapeExit(bool hull, bool cap)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.One),
            Rectangle(scene, 0, 4, -2, 0, Fixed64.One) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        Vector3d start = cap ? new(Fixed64.Zero, 3 * Fixed64.Quarter, -Fixed64.One)
            : new(4 * Fixed64.One + Fixed64.Quarter, Fixed64.Zero, -Fixed64.One);
        LSCollider mover = Mover(scene, hull, start, FixedQuaternion.Identity);
        Vector3d normal = cap ? Vector3d.Down : Vector3d.Left;
        Fixed64 depth = cap ? 3 * Fixed64.Quarter : Fixed64.Quarter;
        var rows = Rows(mover, members);
        Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            ManifoldContact row = sample.Row;
            Assert.Equal(normal, row.Normal); Assert.Equal(depth, row.Depth);
            Assert.False(row.DepthIsClamped);
            Assert.True(row.AnchorA.TryGetWorldPoint(out Vector3d moving));
            Assert.True(row.AnchorB.TryGetWorldPoint(out Vector3d source));
            if (cap) Assert.Equal(moving - normal * depth, source);
            else
            {
                Assert.Equal(4 * Fixed64.One - depth, moving.X);
                Assert.InRange(moving.Y.m_rawValue, (-Fixed64.Half).m_rawValue, Fixed64.Half.m_rawValue);
                Assert.InRange(moving.Z.m_rawValue, (-3 * Fixed64.Half).m_rawValue, (-Fixed64.Half).m_rawValue);
                Assert.InRange(source.Y.m_rawValue, (-Fixed64.One).m_rawValue, Fixed64.One.m_rawValue);
                Assert.InRange(source.Z.m_rawValue, (-Fixed64.Two).m_rawValue, Fixed64.Zero.m_rawValue);
            }
            Assert.Equal(cap ? Fixed64.One : (Fixed64)4, cap ? source.Y : source.X);
        }
        AssertWholeShapeExit(mover, members, start - normal * depth, -normal);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ObliqueOuterCorner_ShouldRetainTheNativeNonfaceExit(bool hull, bool step)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        var members = step
            ? new[] { Rectangle(scene, -2, 0, -2, 2, Fixed64.Two),
                Rectangle(scene, 0, 2, -2, 2, Fixed64.One, centerY: -Fixed64.One) }
            : new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.Two), Rectangle(scene, 0, 4, -2, 0, Fixed64.Two) };
        LSCollider2D member = members[step ? 0 : 1];
        FixedQuaternion rotation = step
            ? FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)45)
            : FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)45, Fixed64.Zero);
        Fixed64 offset = Fixed64.FromFraction(3, 10);
        LSCollider mover = Mover(scene, hull, step ? new(offset, 2 * Fixed64.One + offset, Fixed64.Zero)
            : new(4 * Fixed64.One + offset, Fixed64.Zero, offset), rotation, step ? new(3, 3, 1) : new(3, 1, 3));
        var ordinary = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, member, ordinary));
        Assert.False(CollisionDetectionMixed.TryCollide(mover, members[step ? 1 : 0], out _));
        Assert.True(ordinary.PrimaryContact.Normal.X < Fixed64.Zero
            && (step ? ordinary.PrimaryContact.Normal.Y : ordinary.PrimaryContact.Normal.Z) < Fixed64.Zero);
        // The larger mover's support query reaches the declared neighbor,
        // although the original solids do not overlap there: exercise the joined fan.
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var rows = Rows(mover, members);
        Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            ManifoldContact row = sample.Row;
            Assert.Equal(ordinary.PrimaryContact.Depth, row.Depth);
            Assert.True(row.Normal.X < Fixed64.Zero && (step ? row.Normal.Y : row.Normal.Z) < Fixed64.Zero);
            Assert.True(row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(step ? Fixed64.Zero : (Fixed64)4, source.X);
            Assert.Equal(step ? Fixed64.Two : Fixed64.Zero, step ? source.Y : source.Z);
            Assert.InRange((step ? source.Z : source.Y).m_rawValue, (-Fixed64.Half).m_rawValue, Fixed64.Half.m_rawValue);
        }
        SetPosition(mover, step ? new(2, 4, 0) : new(6, 0, 2));
        foreach (LSCollider2D source in members) Assert.False(CollisionDetectionMixed.TryBuildManifold(mover, source, ordinary));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CornerSupport_ShouldNotBeMistakenForItsParallelOppositeFace(bool hull)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        var corner = new LSPolygonCollider2D(new(0, 0), new(4, 0), new(0, 4))
        { MixedHalfThicknessOverride = Fixed64.Two };
        var far = new LSPolygonCollider2D(new(4, 0), new(4, 4), new(0, 4))
        { MixedHalfThicknessOverride = Fixed64.Two };
        corner.InitializeWithNoBody(new TestMatterAgent(scene.Context));
        far.InitializeWithNoBody(new TestMatterAgent(scene.Context));
        Assert.Equal(Vector2d.Zero, corner.GetWorldVertex(0));
        Fixed64 offset = -Fixed64.FromFraction(3, 10);
        LSCollider mover = Mover(scene, hull, new(offset, Fixed64.Zero, offset),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)45, Fixed64.Zero));
        var ordinary = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, corner, ordinary));
        Assert.True(ordinary.PrimaryContact.Normal.X > Fixed64.Zero && ordinary.PrimaryContact.Normal.Z > Fixed64.Zero);
        Assert.False(CollisionDetectionMixed.TryCollide(mover, far, out _));
        // The far hypotenuse is parallel to the selected normal but is not
        // its supporting feature; the real contact is the opposite vertex.
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { corner, far });
        var rows = Rows(mover, corner, far);
        Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.Equal(ordinary.PrimaryContact.Depth, sample.Row.Depth);
            Assert.True(sample.Row.Normal.X > Fixed64.Zero && sample.Row.Normal.Z > Fixed64.Zero);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(Fixed64.Zero, source.X); Assert.Equal(Fixed64.Zero, source.Z);
            Assert.InRange(source.Y.m_rawValue, (-Fixed64.Half).m_rawValue, Fixed64.Half.m_rawValue);
        }
        SetPosition(mover, new(-Fixed64.One, Fixed64.Zero, -Fixed64.One));
        Assert.False(CollisionDetectionMixed.TryBuildManifold(mover, corner, ordinary));
        Assert.False(CollisionDetectionMixed.TryBuildManifold(mover, far, ordinary));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void DeepContainment_ShouldExitTheCompleteJoinedSolidRatherThanItsCoveredSide(bool hull)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.Two),
            Rectangle(scene, 0, 4, -2, 0, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        Vector3d start = new(Fixed64.Zero, Fixed64.Zero, -Fixed64.Half);
        LSCollider mover = Mover(scene, hull, start, FixedQuaternion.Identity);
        var rows = Rows(mover, members);
        Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.Equal(Vector3d.Backward, sample.Row.Normal);
            Assert.Equal(Fixed64.One, sample.Row.Depth);
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(-Fixed64.One, moving.Z); Assert.Equal(Fixed64.Zero, source.Z);
        }
        AssertWholeShapeExit(mover, members, start + Vector3d.Forward, Vector3d.Forward);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void DeepWideUnion_ShouldRejectTheCoveredExitBeyondTheInitialNeighborQuery(bool hull)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        var members = new[] { Rectangle(scene, -10, 2, -10, 10, (Fixed64)10),
            Rectangle(scene, 2, 10, -10, 10, (Fixed64)10) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCollider mover = Mover(scene, hull, Vector3d.Zero, FixedQuaternion.Identity);
        var rows = Rows(mover, members);
        Assert.NotEmpty(rows);
        Fixed64 depth = 10 * Fixed64.One + Fixed64.Half;
        foreach (var sample in rows)
        {
            Assert.Equal(depth, sample.Row.Depth);
            Assert.False(sample.Row.DepthIsClamped);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(-(Fixed64)10, Vector3d.Dot(source, sample.Row.Normal));
            Assert.InRange(source.X.m_rawValue, ((Fixed64)(-10)).m_rawValue, ((Fixed64)10).m_rawValue);
            Assert.InRange(source.Y.m_rawValue, ((Fixed64)(-10)).m_rawValue, ((Fixed64)10).m_rawValue);
            Assert.InRange(source.Z.m_rawValue, ((Fixed64)(-10)).m_rawValue, ((Fixed64)10).m_rawValue);
        }
        // This rectangular union has an independent complete-solid exit:
        // touching its real outer plane, then one raw unit beyond it.
        Vector3d normal = rows[0].Row.Normal;
        AssertWholeShapeExit(mover, members, -normal * depth, -normal);
    }

    [Theory]
    [InlineData(false, false, false)]
    [InlineData(false, true, false)]
    [InlineData(true, false, false)]
    [InlineData(true, true, false)]
    [InlineData(false, false, true)]
    [InlineData(false, true, true)]
    [InlineData(true, false, true)]
    [InlineData(true, true, true)]
    public void ObliqueVerticalCorner_ShouldPreserveDisconnectedBandsAndAdmitAnOpenMiddle(bool hull, bool exposedMiddle, bool tall)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        LSCollider2D corner = Rectangle(scene, -2, 0, -2, 0, Fixed64.Two);
        LSCollider2D[] members = exposedMiddle
            ? new[] { corner, Rectangle(scene, 0, 2, -2, 0, 3 * Fixed64.Quarter, -5 * Fixed64.Quarter),
                Rectangle(scene, 0, 2, -2, 0, 3 * Fixed64.Quarter, 5 * Fixed64.Quarter) }
            : new[] { corner, Rectangle(scene, 0, 2, -2, 0, Fixed64.Half) };
        Fixed64 offset = Fixed64.FromFraction(3, 10);
        LSCollider mover = Mover(scene, hull, new(offset, Fixed64.Zero, offset),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)45, Fixed64.Zero), new(3, tall ? 5 : 3, 3));
        var ordinary = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, corner, ordinary));
        Assert.True(ordinary.PrimaryContact.Normal.X < Fixed64.Zero && ordinary.PrimaryContact.Normal.Z < Fixed64.Zero);
        Fixed64 expectedDepth = ordinary.PrimaryContact.Depth;
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var regions = new HashSet<int>();
        bool lower = false, upper = false, interior = false;
        foreach (var sample in Rows(mover, members))
        {
            if (sample.Row.Normal.X >= Fixed64.Zero || sample.Row.Normal.Z >= Fixed64.Zero) continue;
            Assert.Equal(Vector3d.Zero.Y, sample.Row.Normal.Y);
            Assert.Equal(expectedDepth, sample.Row.Depth);
            Assert.Equal(corner.Id + 1, sample.Supplier);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(Fixed64.Zero, source.X); Assert.Equal(Fixed64.Zero, source.Z);
            if (exposedMiddle)
            {
                // Exact admission precedes final anchor rounding, which may
                // materialize an interior witness on a closed band endpoint.
                Assert.InRange(source.Y.m_rawValue, (-Fixed64.Half).m_rawValue, Fixed64.Half.m_rawValue);
                interior |= source.Y > -Fixed64.Half && source.Y < Fixed64.Half;
            }
            else
            {
                Assert.True(source.Y < -Fixed64.Half || source.Y > Fixed64.Half);
                lower |= source.Y < -Fixed64.Half; upper |= source.Y > Fixed64.Half;
            }
            regions.Add(sample.Region);
        }
        Assert.Equal(exposedMiddle ? 1 : 2, regions.Count);
        if (exposedMiddle) Assert.True(interior);
        if (!exposedMiddle) Assert.True(lower && upper);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RotatedPartialJoin_ShouldRetainFiniteSupportsAndReplayFreeAngularMotion(bool hull)
    {
        var expected = Trace();
        Assert.Equal(expected, Trace());

        List<(Vector3d Position, Vector3d Velocity, Vector3d Angular, FixedQuaternion Rotation)> Trace()
        {
            using var scene = PhysicsScenarioBuilder.Create();
            Configure(scene.Context);
            FixedQuaternion placement = FixedQuaternion.FromAxisAngle(Vector3d.Up, -Fixed64.Pi / 7);
            var left = new LSPolygonCollider2D(new(-4, -2), new(0, -2), new(0, 0), new(-4, 0))
                { MixedHalfThicknessOverride = Fixed64.Two, Material = PhysicsMaterial.Frictionless };
            var right = new LSPolygonCollider2D(new(0, -1), new(4, -1), new(4, 0), new(0, 0))
                { MixedHalfThicknessOverride = Fixed64.Two, Material = PhysicsMaterial.Frictionless };
            foreach (LSCollider2D member in new[] { left, right })
                member.InitializeWithNoBody(new TestMatterAgent(scene.Context,
                    new FixedTransform(Vector3d.Zero, placement, Vector3d.One)));
            Assert.NotEqual(Fixed64.Zero, left.ConvexRotation);
            Assert.Equal(left.Center, right.Center); Assert.Equal(left.ConvexRotation, right.ConvexRotation);
            using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right });
            FixedQuaternion chart = FixedQuaternion.FromAxisAngle(Vector3d.Up, -left.ConvexRotation);
            Fixed64 cosine = FixedMath.Cos(left.ConvexRotation), sine = FixedMath.Sin(left.ConvexRotation);
            Vector3d outward = hull ? chart * Vector3d.Forward : new(-sine, Fixed64.Zero, cosine);
            FixedQuaternion initial = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)17, (Fixed64)31, (Fixed64)23);
            LSCollider mover = Mover(scene, hull, outward * Fixed64.Quarter, initial);
            if (hull)
            {
                // An odd raw scale times a half-coordinate retains a half-raw
                // residual, forcing the authored affine chart rather than compaction.
                mover.Body!.Agent.Transform.LocalScale = new(Fixed64.FromFraction(2, 3), 3 * Fixed64.Quarter, 5 * Fixed64.Quarter);
                mover.Simulate();
            }
            var rows = Rows(mover, left, right);
            Assert.NotEmpty(rows);
            Fixed64 tolerance = Fixed64.FromFraction(1, 100000);
            foreach (var sample in rows)
            {
                Assert.True((Vector3d.Dot(sample.Row.Normal, outward) + Fixed64.One).Abs() <= tolerance);
                Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
                Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
                Assert.True(Vector3d.Dot(source, outward).Abs() <= tolerance);
                Assert.InRange(source.Y.m_rawValue, (-Fixed64.Two).m_rawValue, Fixed64.Two.m_rawValue);
                Assert.True((Vector3d.Dot(moving - source, sample.Row.Normal) - sample.Row.Depth).Abs() <= tolerance);
            }
            SolidBody body = mover.Body!;
            Assert.Equal(BodyFreezeAxes3D.None, body.FreezeAxes);
            body.SleepEnabled = false; body.UseManualGrounding();
            body.AddAngularImpulse(new(Fixed64.FromFraction(1, 50), Fixed64.FromFraction(1, 100), Fixed64.FromFraction(3, 100)));
            var trace = new List<(Vector3d Position, Vector3d Velocity, Vector3d Angular, FixedQuaternion Rotation)>();
            for (int frame = 0; frame < 8; frame++)
            {
                body.AddLinearImpulse(-outward * Fixed64.FromFraction(1, 32));
                scene.Context.Simulate(); scene.Context.LateSimulate();
                Assert.True(Vector3d.Dot(body.Position3d, outward) > -Fixed64.Half);
                trace.Add((body.Position3d, body.LinearVelocity, body.AngularVelocity, mover.Rotation));
            }
            Assert.NotEqual(initial, mover.Rotation);
            Assert.NotEqual(Vector3d.Zero, body.AngularVelocity);
            return trace;
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void UnequalHeightJoin_ShouldKeepTheRealUpperStepSide(bool hull)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        LSCollider2D left = Rectangle(scene, -4, 0, -2, 0, Fixed64.Two);
        LSCollider2D right = Rectangle(scene, 0, 4, -2, 0, Fixed64.One);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right });
        LSCollider mover = Mover(scene, hull,
            new(Fixed64.Quarter, 3 * Fixed64.Half, -Fixed64.One), FixedQuaternion.Identity);
        int sideRows = 0;
        foreach (var sample in Rows(mover, left, right))
        {
            if (sample.Row.Normal != Vector3d.Left) continue;
            sideRows++;
            Assert.Equal(left.Id + 1, sample.Supplier);
            Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(Fixed64.Zero, source.X);
            Assert.InRange(source.Y.m_rawValue, Fixed64.One.m_rawValue, Fixed64.Two.m_rawValue);
        }
        Assert.True(sideRows > 0);
        Fixed64 beyond = Fixed64.FromRaw(1);
        SetPosition(mover, new(Fixed64.Half + beyond, 3 * Fixed64.Half + beyond, -Fixed64.One));
        var ordinary = new ContactManifold();
        Assert.False(CollisionDetectionMixed.TryBuildManifold(mover, left, ordinary));
        Assert.False(CollisionDetectionMixed.TryBuildManifold(mover, right, ordinary));
    }

    [Fact]
    public void RemoteUBridge_ShouldNotMergeDisconnectedCapSupportsOrFillTheGap()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        var members = new[] { Rectangle(scene, -3, -1, -2, 2, Fixed64.Two),
            Rectangle(scene, 1, 3, -2, 2, Fixed64.Two), Rectangle(scene, -1, 1, -2, -1, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var mover = new LSCuboidCollider { Size = new(5, 1, 1) };
        scene.CreateBody(mover, new(Fixed64.Zero, 7 * Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity);
        var regions = new HashSet<int>();
        foreach (var sample in Rows(mover, members))
        {
            Assert.Equal(Vector3d.Down, sample.Row.Normal);
            Assert.Equal(3 * Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.True(source.X <= -Fixed64.One || source.X >= Fixed64.One);
            Assert.Equal(Fixed64.Two, source.Y);
            regions.Add(sample.Region);
        }
        Assert.Equal(2, regions.Count);
        LSCollider gap = Mover(scene, false, new(Fixed64.Zero, 7 * Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity);
        var ordinary = new ContactManifold();
        foreach (LSCollider2D member in members)
            Assert.False(CollisionDetectionMixed.TryBuildManifold(gap, member, ordinary));
        Assert.Empty(Rows(gap, members));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ObliqueProjectedSupport_ShouldKeepItsInitiallySeparatedFiniteSupplier(bool hull)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        LSCollider2D left = Rectangle(scene, -8, 0, -2, 0, (Fixed64)4);
        LSCollider2D right = Rectangle(scene, 0, 8, -2, 0, (Fixed64)4);
        right.Material = PhysicsMaterial.Bouncy;
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)30, Fixed64.Zero);
        LSCollider mover = Mover(scene, hull,
            new(-Fixed64.FromFraction(19, 5), Fixed64.Zero, -Fixed64.Half), rotation, new(10, 1, 1));
        // The rod's largest X within the solid's Z=[-2,0] is approximately
        // -0.202, while its deepest-Z support projects to X=+0.280 on the right.
        var ordinary = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, left, ordinary));
        Assert.False(CollisionDetectionMixed.TryBuildManifold(mover, right, ordinary));
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right });
        var rows = Rows(mover, left, right);
        Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.Equal(right.Id + 1, sample.Supplier);
            Assert.Equal(right.Material, sample.Row.MaterialB);
            Assert.Equal(Vector3d.Backward, sample.Row.Normal);
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.True(source.X > Fixed64.Zero); Assert.Equal(Fixed64.Zero, source.Z);
            Assert.Equal(-moving.Z, sample.Row.Depth);
        }
        SetPosition(mover, new(-Fixed64.FromFraction(19, 5), Fixed64.Zero, (Fixed64)4));
        Assert.False(CollisionDetectionMixed.TryBuildManifold(mover, left, ordinary));
        Assert.False(CollisionDetectionMixed.TryBuildManifold(mover, right, ordinary));
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void VerticallyPartitionedNeighbors_ShouldCoverTheirCompleteSharedSide(bool hull, bool upper)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        var members = new[] { Rectangle(scene, -2, 0, -1, 1, Fixed64.Two),
            Rectangle(scene, 0, 2, -1, 1, Fixed64.One, centerY: -Fixed64.One),
            Rectangle(scene, 0, 2, -1, 1, Fixed64.One, centerY: Fixed64.One) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        Fixed64 y = (upper ? Fixed64.One : -Fixed64.One) * (3 * Fixed64.Quarter);
        LSCollider mover = Mover(scene, hull, new(-Fixed64.Quarter, y, 5 * Fixed64.Quarter), FixedQuaternion.Identity);
        var rows = Rows(mover, members);
        Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.Equal(Vector3d.Backward, sample.Row.Normal);
            Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(Fixed64.One, source.Z);
        }
        AssertWholeShapeExit(mover, members, new(-Fixed64.Quarter, y, 3 * Fixed64.Half), Vector3d.Forward);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void CapWitnessesNearEitherScalarLimit_ShouldUseTheBoundedPhysicalChart(bool hull, bool upper)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        Fixed64 y = upper ? Fixed64.MaxValue - 8 : Fixed64.MinValue + 8;
        Assert.True(scene.Context.World.TryAddGrid(new GridConfiguration(
            new Vector3d((Fixed64)(-16), y - 4, (Fixed64)(-16)),
            new Vector3d((Fixed64)16, y + 4, (Fixed64)16)), out _));
        var members = new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.Two, centerY: y),
            Rectangle(scene, 0, 4, -2, 0, Fixed64.Two, centerY: y) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCollider mover = Mover(scene, hull,
            new(Fixed64.Zero, y + 7 * Fixed64.Quarter, -Fixed64.One), FixedQuaternion.Identity);
        var rows = Rows(mover, members);
        Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.Equal(Vector3d.Down, sample.Row.Normal);
            Assert.Equal(3 * Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(y + Fixed64.Two, source.Y);
        }
        AssertWholeShapeExit(mover, members,
            new(Fixed64.Zero, y + 5 * Fixed64.Half, -Fixed64.One), Vector3d.Up);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void LocalMemberCountShrink_ShouldRetainOnlyCurrentFiniteSuppliers(bool hull)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Configure(scene.Context);
        LSCollider2D left = Rectangle(scene, -8, 0, -2, 0, Fixed64.Two);
        LSCollider2D right = Rectangle(scene, 0, 8, -2, 0, Fixed64.Two);
        right.Material = PhysicsMaterial.Bouncy;
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right });
        LSCollider mover = Mover(scene, hull, Vector3d.Forward * Fixed64.Quarter, FixedQuaternion.Identity);
        Assert.NotEmpty(Rows(mover, left, right));
        foreach (int x in new[] { -3, 3, -3 })
        {
            SetPosition(mover, new((Fixed64)x, Fixed64.Zero, Fixed64.Quarter));
            LSCollider2D supplier = x < 0 ? left : right;
            var rows = Rows(mover, left, right);
            Assert.NotEmpty(rows);
            foreach (var sample in rows)
            {
                Assert.Equal(supplier.Id + 1, sample.Supplier);
                if (sample.Row.HasMaterialOverride) Assert.Equal(supplier.Material, sample.Row.MaterialB);
                Assert.Equal(Vector3d.Backward, sample.Row.Normal);
                Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
            }
        }
    }

    [Theory]
    [InlineData(false, false, false)]
    [InlineData(false, false, true)]
    [InlineData(false, true, false)]
    [InlineData(false, true, true)]
    [InlineData(true, false, false)]
    [InlineData(true, false, true)]
    [InlineData(true, true, false)]
    [InlineData(true, true, true)]
    public void CrossingJoinedSolidWall_ShouldMatchContinuousResponseAndKeepMemberEvents(
        bool hull, bool polygon, bool unequalHeight)
    {
        List<(Vector3d Position, Vector3d Velocity)> expected = Trace(joined: false, reverse: false);
        Check(Trace(joined: true, reverse: false));
        Check(Trace(joined: true, reverse: true));

        void Check(List<(Vector3d Position, Vector3d Velocity)> actual)
        {
            Fixed64 tolerance = Fixed64.FromFraction(1, 1000);
            for (int frame = 0; frame < expected.Count; frame++)
            {
                Assert.Equal(expected[frame].Position.X, actual[frame].Position.X);
                Assert.True((actual[frame].Position.Z - expected[frame].Position.Z).Abs() <= tolerance,
                    $"Wall-normal position differs at frame {frame}: {actual[frame].Position.Z} versus {expected[frame].Position.Z}.");
                Assert.True((actual[frame].Velocity.Z - expected[frame].Velocity.Z).Abs() <= tolerance);
            }
        }

        List<(Vector3d Position, Vector3d Velocity)> Trace(bool joined, bool reverse)
        {
            using var scene = PhysicsScenarioBuilder.Create();
            GravitasWorldContext context = scene.Context;
            Configure(context);
            LSCollider2D[] members = joined
                ? new[] { Wall(-4, 0, Fixed64.Two), Wall(0, 4, unequalHeight ? Fixed64.One : Fixed64.Two) }
                : new[] { Wall(-4, 4, Fixed64.Two) };
            using var surface = joined ? context.Physics2D.CreateStaticSurface(reverse
                ? new[] { members[1], members[0] } : members) : null;
            int entered = 0, exited = 0;
            foreach (LSCollider2D member in members)
            {
                member.OnMixedContactEnter += _ => entered++;
                member.OnMixedContactExit += _ => exited++;
            }
            LSCollider collider = hull ? MeshTestFixtures.CreateConvexCube()
                : new LSCuboidCollider { Size = Vector3d.One };
            collider.Material = PhysicsMaterial.Frictionless;
            SolidBody body = scene.CreateBody(collider,
                new Vector3d(-Fixed64.Two, Fixed64.Zero, Fixed64.Half), FixedQuaternion.Identity).Body;
            body.FreezeAxes = BodyFreezeAxes3D.PositionY | BodyFreezeAxes3D.Rotation;
            body.SleepEnabled = false;
            body.UseManualGrounding();
            for (int frame = 0; frame < 32; frame++) Step();
            body.AddLinearImpulse(Vector3d.Right);
            var trace = new List<(Vector3d Position, Vector3d Velocity)>();
            for (int frame = 0; frame < 64; frame++)
            {
                Step();
                // Do not reapply tangential velocity: that would hide a seam impulse.
                Assert.True(body.LinearVelocity.X == Fixed64.One,
                    $"joined={joined}, frame={frame}, position={body.Position3d}, velocity={body.LinearVelocity}");
                Assert.Equal(Fixed64.Zero, body.Position3d.Y);
                Assert.Equal(Vector3d.Zero, body.AngularVelocity);
                Assert.True(body.Position3d.Z > Fixed64.Zero);
                trace.Add((body.Position3d, body.LinearVelocity));
            }
            Assert.True(body.Position3d.X > Fixed64.One);
            Assert.Equal(joined ? 2 : 1, entered);
            Assert.Equal(joined ? 1 : 0, exited);
            return trace;

            void Step()
            {
                body.AddLinearImpulse(Vector3d.Backward * Fixed64.FromFraction(1, 16));
                context.Simulate(); context.LateSimulate();
            }

            LSCollider2D Wall(int left, int right, Fixed64 halfThickness)
                => Rectangle(scene, left, right, -2, 0, halfThickness, polygon: polygon);
        }
    }

    private static void Configure(GravitasWorldContext context)
    {
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.SetFrameRate(16);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        context.Environment.MinSpeed = Fixed64.Zero;
    }

    private static LSCollider Mover(PhysicsScenarioBuilder scene, bool hull, Vector3d position, FixedQuaternion rotation,
        Vector3d? size = null)
    {
        Vector3d extent = size ?? Vector3d.One;
        LSCollider collider = hull ? MeshTestFixtures.CreateConvexCube() : new LSCuboidCollider { Size = extent };
        if (hull && extent != Vector3d.One)
        {
            var cube = (LSMeshCollider)collider;
            Vector3d[] vertices = cube.Mesh.LocalVertices.ToArray();
            for (int vertex = 0; vertex < vertices.Length; vertex++)
                vertices[vertex] = new(vertices[vertex].X * extent.X, vertices[vertex].Y * extent.Y, vertices[vertex].Z * extent.Z);
            collider = new LSMeshCollider(vertices, cube.Mesh.Triangles.ToArray(),
                MeshColliderMode.Convex, MeshInertiaPolicy.RequireClosedVolume);
        }
        collider.Material = PhysicsMaterial.Frictionless;
        scene.CreateBody(collider, position, rotation);
        return collider;
    }

    private static LSCollider2D Rectangle(PhysicsScenarioBuilder scene, int left, int right, int bottom, int top,
        Fixed64 halfThickness, Fixed64 centerY = default, bool polygon = false)
    {
        Fixed64 halfWidth = (Fixed64)(right - left) * Fixed64.Half, halfDepth = (Fixed64)(top - bottom) * Fixed64.Half;
        LSCollider2D collider = polygon ? new LSPolygonCollider2D(new(-halfWidth, -halfDepth),
            new(halfWidth, -halfDepth), new(halfWidth, halfDepth), new(-halfWidth, halfDepth))
            : new LSAABBoxCollider2D(new Vector2d(right - left, top - bottom));
        collider.Material = PhysicsMaterial.Frictionless; collider.MixedHalfThicknessOverride = halfThickness;
        collider.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(
            new Vector3d((Fixed64)(left + right) * Fixed64.Half, centerY, (Fixed64)(bottom + top) * Fixed64.Half),
            FixedQuaternion.Identity, Vector3d.One)));
        return collider;
    }

    private static List<(int Region, int Supplier, ManifoldContact Row)> Rows(LSCollider mover, params LSCollider2D[] members)
    {
        var rows = new List<(int Region, int Supplier, ManifoldContact Row)>();
        var manifold = new ContactManifold();
        foreach (LSCollider2D member in members)
        {
            CollisionDetectionMixed.TryBuildDiscreteManifold(mover, member, manifold);
            for (int group = 0; group < manifold.GroupCount; group++)
            {
                ref ContactGroup contacts = ref manifold.GetGroup(group);
                for (int row = 0; row < contacts.Count; row++)
                    // Zero uses the pair's own target; explicit IDs identify borrowed suppliers.
                    rows.Add((contacts.Key.Region, contacts.Key.SourceIdB == 0 ? member.Id + 1 : contacts.Key.SourceIdB, contacts[row]));
            }
        }
        return rows;
    }

    private static void AssertWholeShapeExit(LSCollider mover, LSCollider2D[] members, Vector3d touching, Vector3d outward)
    {
        var ordinary = new ContactManifold();
        SetPosition(mover, touching);
        bool hit = false;
        foreach (LSCollider2D member in members) hit |= CollisionDetectionMixed.TryBuildManifold(mover, member, ordinary);
        Assert.True(hit);
        SetPosition(mover, touching + outward * Fixed64.FromRaw(1));
        foreach (LSCollider2D member in members) Assert.False(CollisionDetectionMixed.TryBuildManifold(mover, member, ordinary));
    }

    private static void SetPosition(LSCollider mover, Vector3d position)
    {
        mover.Body!.SetPosition(position);
        mover.RebuildRuntimeShapeOnly();
    }
}
