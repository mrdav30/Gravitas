using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Support;
using Gravitas.Tests.Support;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using Xunit;

namespace Gravitas.Tests;

/// <content>Checks finite spherical supports before joined-solid response selection.</content>
public sealed partial class JoinedStaticSolidMixedTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void SphericalSeamOverlap_ShouldAdmitTheExposedWallBeforeSelectingItsExit(bool capsule, bool positiveX)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.Two),
            Rectangle(scene, 0, 4, -2, 0, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        Fixed64 x = positiveX ? Fixed64.Quarter : -Fixed64.Quarter;
        LSCollider mover = SphericalMover(scene, capsule, new(x, Fixed64.Zero, Fixed64.Half), Fixed64.One);

        // Public per-member geometry remains distinct: its corner normal is
        // still meaningful to queries, but must not become a joined solver row.
        var ordinary = new ContactManifold();
        foreach (LSCollider2D member in members)
            Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, member, ordinary));
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.Equal(Vector3d.Backward, sample.Row.Normal);
            Assert.Equal(Fixed64.Half, sample.Row.Depth);
            Assert.False(sample.Row.DepthIsClamped);
            Assert.Equal(members[positiveX ? 1 : 0].Id + 1, sample.Supplier);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(new Vector3d(x, Fixed64.Zero, Fixed64.Zero), source);
        }
        AssertWholeShapeExit(mover, members, new(x, Fixed64.Zero, Fixed64.One), Vector3d.Forward);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SphericalDeepContainment_ShouldRejectTheCoveredNearSide(bool capsule)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.Two),
            Rectangle(scene, 0, 4, -2, 0, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCollider mover = SphericalMover(scene, capsule, new(Fixed64.Zero, Fixed64.Zero, -Fixed64.Half), Fixed64.Half);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.Equal(Vector3d.Backward, sample.Row.Normal);
            Assert.Equal(Fixed64.One, sample.Row.Depth);
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(-Fixed64.One, moving.Z); Assert.Equal(Fixed64.Zero, source.Z);
        }
        AssertWholeShapeExit(mover, members, Vector3d.Forward * Fixed64.Half, Vector3d.Forward);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SphericalDeepWideUnion_ShouldExpandBeforeRejectingTheCoveredNearExit(bool capsule)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -10, 2, -10, 10, (Fixed64)10),
            Rectangle(scene, 2, 10, -10, 10, (Fixed64)10) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCollider mover = SphericalMover(scene, capsule, Vector3d.Zero, Fixed64.Half);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.Equal(10 * Fixed64.One + Fixed64.Half, sample.Row.Depth);
            Assert.False(sample.Row.DepthIsClamped);
            Vector3d normal = sample.Row.Normal;
            Assert.Equal(Fixed64.One, normal.X.Abs() + normal.Y.Abs() + normal.Z.Abs());
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(-(Fixed64)10, Vector3d.Dot(source, normal));
        }
        ManifoldContact first = rows[0].Row;
        AssertWholeShapeExit(mover, members, -first.Normal * first.Depth, -first.Normal);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void SphericalOuterCapOrCorner_ShouldKeepTheRealFiniteSupplier(bool capsule, bool cap)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.Two),
            Rectangle(scene, 0, 4, -2, 0, Fixed64.Two) };
        LSCollider mover = SphericalMover(scene, capsule, cap
            ? new(Fixed64.Zero, 2 * Fixed64.One + Fixed64.Quarter, -Fixed64.One)
            : new(4 * Fixed64.One + Fixed64.Quarter, Fixed64.Quarter, Fixed64.Quarter), Fixed64.Half);
        var ordinary = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, members[1], ordinary));
        ManifoldContact expected = ordinary.PrimaryContact;
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.Equal(expected.Normal, sample.Row.Normal); Assert.Equal(expected.Depth, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            if (cap)
            {
                Assert.Equal(Vector3d.Down, sample.Row.Normal); Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
                Assert.Equal(new Vector3d(0, 2, -1), source);
            }
            else
            {
                Assert.Equal(members[1].Id + 1, sample.Supplier);
                Assert.Equal(new Vector3d(4 * Fixed64.One, Fixed64.Quarter, Fixed64.Zero), source);
                Assert.True(sample.Row.Normal.X < Fixed64.Zero);
                Assert.Equal(sample.Row.Normal.X, sample.Row.Normal.Z);
            }
        }
        if (cap) AssertWholeShapeExit(mover, members, new(Fixed64.Zero, 2 * Fixed64.One + Fixed64.Half, -Fixed64.One), Vector3d.Up);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SphericalHeightStep_ShouldKeepTheExposedUpperSideAndLowerCap(bool capsule)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -2, 0, -2, 0, Fixed64.Two),
            Rectangle(scene, 0, 2, -2, 0, Fixed64.One) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCollider mover = SphericalMover(scene, capsule, new(Fixed64.Quarter, 3 * Fixed64.Half, -Fixed64.One), Fixed64.Half);
        var rows = Rows(mover, members);
        Assert.Contains(rows, sample => sample.Supplier == members[0].Id + 1
            && sample.Row.Normal == Vector3d.Left && sample.Row.Depth == Fixed64.Quarter);
        Assert.Contains(rows, sample => sample.Supplier == members[1].Id + 1
            && sample.Row.Normal == Vector3d.Down && sample.Row.Depth == Fixed64.Zero);
        foreach (var sample in rows)
        {
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(-Fixed64.One, source.Z);
            Assert.True(source.X == Fixed64.Zero && source.Y == 3 * Fixed64.Half
                || source.X == Fixed64.Quarter && source.Y == Fixed64.One);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SphericalPartialJoin_ShouldPreserveItsUncoveredSideSpan(bool capsule)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -2, 0, -2, 2, Fixed64.Two),
            Rectangle(scene, 0, 2, -2, 0, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCollider mover = SphericalMover(scene, capsule, new(Fixed64.Quarter, Fixed64.Zero, Fixed64.One), Fixed64.Half);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.Equal(members[0].Id + 1, sample.Supplier);
            Assert.Equal(Vector3d.Left, sample.Row.Normal); Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(new Vector3d(0, 0, 1), source);
        }
        AssertWholeShapeExit(mover, members, new(Fixed64.Half, Fixed64.Zero, Fixed64.One), Vector3d.Right);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void SphericalJoinedWallCrossing_ShouldPreserveContinuousResponseAndMemberEvents(bool capsule, bool polygon)
    {
        List<(Vector3d Position, Vector3d Velocity)> expected = Trace(joined: false, reverse: false);
        Check(Trace(joined: true, reverse: false)); Check(Trace(joined: true, reverse: true));

        void Check(List<(Vector3d Position, Vector3d Velocity)> actual)
        {
            Fixed64 tolerance = Fixed64.FromFraction(1, 1000);
            for (int frame = 0; frame < expected.Count; frame++)
            {
                Assert.Equal(expected[frame].Position.X, actual[frame].Position.X);
                Assert.True((actual[frame].Position.Z - expected[frame].Position.Z).Abs() <= tolerance);
                Assert.True((actual[frame].Velocity.Z - expected[frame].Velocity.Z).Abs() <= tolerance);
            }
        }

        List<(Vector3d Position, Vector3d Velocity)> Trace(bool joined, bool reverse)
        {
            using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
            var members = joined
                ? new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.Two, polygon: polygon),
                    Rectangle(scene, 0, 4, -2, 0, Fixed64.Two, polygon: polygon) }
                : new[] { Rectangle(scene, -4, 4, -2, 0, Fixed64.Two, polygon: polygon) };
            using var surface = joined ? scene.Context.Physics2D.CreateStaticSurface(reverse
                ? new[] { members[1], members[0] } : members) : null;
            int entered = 0, exited = 0;
            foreach (LSCollider2D member in members)
            {
                member.OnMixedContactEnter += _ => entered++;
                member.OnMixedContactExit += _ => exited++;
            }
            LSCollider collider = SphericalMover(scene, capsule, new(-Fixed64.Two, Fixed64.Zero, Fixed64.Half), Fixed64.Half);
            SolidBody body = collider.Body!;
            body.FreezeAxes = BodyFreezeAxes3D.PositionY | BodyFreezeAxes3D.Rotation;
            body.SleepEnabled = false; body.UseManualGrounding();
            for (int frame = 0; frame < 32; frame++) Step();
            body.AddLinearImpulse(Vector3d.Right);
            var trace = new List<(Vector3d Position, Vector3d Velocity)>();
            for (int frame = 0; frame < 64; frame++)
            {
                Step();
                Assert.True(body.LinearVelocity.X == Fixed64.One,
                    $"joined={joined}, frame={frame}, position={body.Position3d}, velocity={body.LinearVelocity}");
                Assert.Equal(Fixed64.Zero, body.Position3d.Y); Assert.Equal(Vector3d.Zero, body.AngularVelocity);
                Assert.True(body.Position3d.Z > Fixed64.Zero);
                trace.Add((body.Position3d, body.LinearVelocity));
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
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void SphericalScalarYawJoin_ShouldMatchTheSameContinuousNativePolygon(bool capsule, bool cap)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        Vector3d origin = new(3, 0, -2);
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)31, Fixed64.Zero);
        LSPolygonCollider2D left = Polygon(-4, 0), right = Polygon(0, 4), continuous = Polygon(-4, 4);
        Assert.NotEqual(Fixed64.Zero, left.ConvexRotation); Assert.Equal(left.ConvexRotation, right.ConvexRotation);
        Fixed64 yaw = left.ConvexRotation;
        Vector2d nativeOffset = new Vector2d(Fixed64.Quarter, Fixed64.Half).Rotated(FixedMath.Cos(yaw), FixedMath.Sin(yaw));
        Vector3d center = cap ? origin + Vector3d.Up * (7 * Fixed64.Quarter)
            : origin + new Vector3d(nativeOffset.X, Fixed64.Zero, nativeOffset.Y);
        LSCollider mover = SphericalMover(scene, capsule, center, cap ? Fixed64.Half : Fixed64.One);
        var ordinary = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, continuous, ordinary));
        ManifoldContact expected = ordinary.PrimaryContact;
        Assert.True(expected.AnchorB.TryGetWorldPoint(out Vector3d expectedSource));
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new LSCollider2D[] { left, right });
        var rows = Rows(mover, left, right); Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.Equal(expected.Normal, sample.Row.Normal); Assert.Equal(expected.Depth, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(expectedSource, source);
            if (cap) Assert.Equal(new Vector3d(3, 2, -2), source);
        }
        if (cap)
            AssertWholeShapeExit(mover, new LSCollider2D[] { left, right }, origin + Vector3d.Up * (5 * Fixed64.Half), Vector3d.Up);

        LSPolygonCollider2D Polygon(int first, int last)
        {
            int top = cap ? 2 : 0;
            var polygon = new LSPolygonCollider2D(new(first, -2), new(last, -2), new(last, top), new(first, top))
                { MixedHalfThicknessOverride = Fixed64.Two, Material = PhysicsMaterial.Frictionless };
            polygon.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(origin, rotation, Vector3d.One)));
            return polygon;
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SphericalDisconnectedJoinedComponents_ShouldKeepTheirFiniteSuppliersAndMaterials(bool capsule)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -3, -2, -2, 0, Fixed64.Two),
            Rectangle(scene, -2, -1, -2, 0, Fixed64.Two), Rectangle(scene, 1, 2, -2, 0, Fixed64.Two),
            Rectangle(scene, 2, 3, -2, 0, Fixed64.Two) };
        members[1].Material = PhysicsMaterial.Bouncy;
        members[2].Material = PhysicsMaterial.Default;
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCollider mover = SphericalMover(scene, capsule, Vector3d.Forward * Fixed64.Quarter, 3 * Fixed64.Half);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        var suppliers = new HashSet<int>(); var regions = new HashSet<int>();
        foreach (var sample in rows)
        {
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            int supplier = sample.Supplier == members[1].Id + 1 ? 1 : 2;
            Assert.Equal(members[supplier].Id + 1, sample.Supplier);
            Assert.Equal(new Vector3d(supplier == 1 ? -1 : 1, 0, 0), source);
            Assert.True(supplier == 1 ? sample.Row.Normal.X < Fixed64.Zero : sample.Row.Normal.X > Fixed64.Zero);
            Assert.True(sample.Row.Normal.Z < Fixed64.Zero); Assert.Equal(Fixed64.Zero, sample.Row.Normal.Y);
            Assert.True(sample.Row.HasMaterialOverride); Assert.Equal(members[supplier].Material, sample.Row.MaterialB);
            suppliers.Add(sample.Supplier); regions.Add(sample.Region);
        }
        Assert.Equal(2, suppliers.Count); Assert.Equal(2, regions.Count);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void SphericalNeighborEligibilityChange_ShouldExposeTheRimAndRestoreContinuity(bool capsule, bool trigger)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var left = Rectangle(scene, -4, 0, -2, 0, Fixed64.Two);
        var right = Rectangle(scene, 0, 4, -2, 0, Fixed64.Two);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right });
        LSCollider mover = SphericalMover(scene, capsule, new(Fixed64.Quarter, Fixed64.Zero, -Fixed64.One), Fixed64.Half);
        var joined = Rows(mover, left, right); Assert.NotEmpty(joined);
        foreach (var sample in joined) Assert.Equal(Fixed64.Zero, sample.Row.Normal.X);
        if (trigger) right.IsTrigger = true; else right.IgnoredCollisionLayers = PhysicsLayerMask.All;
        var exposed = Rows(mover, left); Assert.NotEmpty(exposed);
        foreach (var sample in exposed)
        {
            Assert.Equal(Vector3d.Left, sample.Row.Normal); Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(new Vector3d(0, 0, -1), source);
        }
        right.IsTrigger = false; right.IgnoredCollisionLayers = PhysicsLayerMask.None;
        var restored = Rows(mover, left, right); Assert.NotEmpty(restored);
        foreach (var sample in restored) Assert.Equal(Fixed64.Zero, sample.Row.Normal.X);
        right.Deactivate();
        Assert.Null(right.StaticSurface);
        var removed = Rows(mover, left); Assert.NotEmpty(removed);
        foreach (var sample in removed) Assert.Equal(Vector3d.Left, sample.Row.Normal);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SphericalContextReset_ShouldRetireTheOldDeclarationBeforeRegistryReuse(bool capsule)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.Two), Rectangle(scene, 0, 4, -2, 0, Fixed64.Two) };
        using var retired = scene.Context.Physics2D.CreateStaticSurface(members);
        int formerId = members[0].Id;
        Vector3d position = new(Fixed64.Quarter, Fixed64.Zero, -Fixed64.One);
        LSCollider mover = SphericalMover(scene, capsule, position, Fixed64.Half);
        Assert.NotEmpty(Rows(mover, members));
        scene.Context.Reset();
        foreach (LSCollider2D member in members) Assert.Null(member.StaticSurface);

        LSCollider2D replacement = Rectangle(scene, -4, 0, -2, 0, Fixed64.Two);
        Assert.Equal(formerId, replacement.Id);
        Assert.Throws<ObjectDisposedException>(() => retired.ReplaceMembers(new[] { replacement }));
        using var next = scene.Context.Physics2D.CreateStaticSurface(new[] { replacement });
        LSCollider reused = SphericalMover(scene, capsule, position, Fixed64.Half);
        var exposed = Rows(reused, replacement); Assert.NotEmpty(exposed);
        foreach (var sample in exposed)
        {
            Assert.Equal(replacement.Id + 1, sample.Supplier);
            Assert.Equal(Vector3d.Left, sample.Row.Normal); Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SphericalEndedDeclaration_ShouldReleaseItsCachedMoverAndMembers(bool capsule)
    {
        var retained = RetiredSphericalDeclaration(capsule);
        GC.Collect(); GC.WaitForPendingFinalizers(); GC.Collect();
        Assert.False(retained.Mover.IsAlive, "The ended spherical mover remains retained.");
        Assert.False(retained.Member.IsAlive, "The ended spherical supplier remains retained.");
        GC.KeepAlive(retained.Surface);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static (StaticSurface2D Surface, WeakReference Mover, WeakReference Member) RetiredSphericalDeclaration(bool capsule)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.Two), Rectangle(scene, 0, 4, -2, 0, Fixed64.Two) };
        var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCollider mover = SphericalMover(scene, capsule, Vector3d.Forward * Fixed64.Quarter, Fixed64.Half);
        mover.Body!.UseManualGrounding();
        Assert.NotEmpty(Rows(mover, members));
        mover.Body.Deactivate();
        foreach (LSCollider2D member in members) member.Deactivate();
        surface.Dispose();
        return (surface, new(mover), new(members[1]));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SphericalConcaveCut_ShouldRetainBothFiniteFacePeaks(bool capsule)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -2, 0, -2, 2, Fixed64.Two), Rectangle(scene, 0, 2, -2, 0, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCollider mover = SphericalMover(scene, capsule, new(-Fixed64.Quarter, Fixed64.Zero, -Fixed64.Quarter), Fixed64.Half);
        var rows = Rows(mover, members);
        Assert.Equal(2, rows.Count);
        Fixed64 expected = FixedMath.Sqrt(Fixed64.FromFraction(3, 16)) + Fixed64.Quarter;
        var normals = new HashSet<Vector3d>();
        foreach (var sample in rows)
        {
            normals.Add(sample.Row.Normal);
            Assert.InRange(sample.Row.Depth.m_rawValue, expected.m_rawValue - 1, expected.m_rawValue + 1);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d point)); Assert.Equal(Vector3d.Zero, point);
        }
        Assert.Contains(Vector3d.Left, normals); Assert.Contains(Vector3d.Backward, normals);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SphericalConcaveHeightStep_ShouldKeepFiniteSideAndCapPeaks(bool capsule)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -2, 0, -2, 0, Fixed64.Two), Rectangle(scene, 0, 2, -2, 0, Fixed64.One) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCollider mover = SphericalMover(scene, capsule,
            new(-Fixed64.FromFraction(2, 5), Fixed64.FromFraction(9, 10), -Fixed64.One), Fixed64.Half);
        var rows = Rows(mover, members); Assert.Equal(2, rows.Count);
        var normals = new HashSet<Vector3d>();
        foreach (var sample in rows)
        {
            normals.Add(sample.Row.Normal);
            Fixed64 expected = sample.Row.Normal == Vector3d.Down ? Fixed64.FromFraction(2, 5)
                : FixedMath.Sqrt(Fixed64.FromFraction(6, 25)) + Fixed64.FromFraction(2, 5);
            // The decimal fixture and independent scalar square root each round;
            // four raw units cover those input/expected-value conversions.
            Assert.InRange(sample.Row.Depth.m_rawValue, expected.m_rawValue - 4, expected.m_rawValue + 4);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d point));
            Assert.Equal(new Vector3d(0, 1, -1), point);
        }
        Assert.Contains(Vector3d.Left, normals); Assert.Contains(Vector3d.Down, normals);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SphericalConcaveCorner_ShouldRepeatItsCompleteHostResponseWithoutWarmAllocation(bool capsule)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -2, 0, -2, 2, Fixed64.Two), Rectangle(scene, 0, 2, -2, 0, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        Vector3d start = new(-Fixed64.Quarter, Fixed64.Zero, -Fixed64.Quarter);
        LSCollider mover = SphericalMover(scene, capsule, start, Fixed64.Half);
        SolidBody body = mover.Body!;
        body.FreezeAxes = BodyFreezeAxes3D.PositionY | BodyFreezeAxes3D.Rotation;
        body.SleepEnabled = false; body.UseManualGrounding();
        for (int frame = 0; frame < 32; frame++) Step();
        Vector3d expected = body.Position3d;
        Assert.True(expected.X > start.X && expected.Z > start.Z);
        Assert.True(expected.X < Fixed64.Half && expected.Z < Fixed64.Half);
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int frame = 0; frame < 64; frame++)
        {
            Step();
            if (body.Position3d != expected || body.LinearVelocity != Vector3d.Zero)
                throw new InvalidOperationException("The same concave placement must repeat its finite response.");
        }
        Assert.Equal(0, GC.GetAllocatedBytesForCurrentThread() - before);
        void Step() { body.SetPosition(start); scene.Context.Simulate(); scene.Context.LateSimulate(); }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SphericalOpposingWallsInOneComponent_ShouldKeepBothNormals(bool capsule)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -2, -1, -3, 1, Fixed64.Two),
            Rectangle(scene, 1, 2, -3, 1, Fixed64.Two), Rectangle(scene, -1, 1, -3, -2, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCollider mover = SphericalMover(scene, capsule, Vector3d.Zero, 3 * Fixed64.Half);
        var rows = Rows(mover, members);
        Assert.Contains(rows, sample => sample.Row.Normal == Vector3d.Left && sample.Row.Depth == Fixed64.Half);
        Assert.Contains(rows, sample => sample.Row.Normal == Vector3d.Right && sample.Row.Depth == Fixed64.Half);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SphericalParallelWallsConnectedElsewhere_ShouldKeepTheirDifferentPlanes(bool capsule)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -2, 2, -1, 0, (Fixed64)10),
            Rectangle(scene, -2, 2, -3, -2, (Fixed64)10), Rectangle(scene, 2, 3, -3, 0, (Fixed64)10) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCollider mover = SphericalMover(scene, capsule, Vector3d.Forward * Fixed64.Half, (Fixed64)3);
        var rows = Rows(mover, members);
        Assert.Contains(rows, sample => sample.Row.Normal == Vector3d.Backward && sample.Row.Depth == 5 * Fixed64.Half
            && sample.Supplier == members[0].Id + 1);
        Assert.Contains(rows, sample => sample.Row.Normal == Vector3d.Backward && sample.Row.Depth == Fixed64.Half
            && sample.Supplier == members[1].Id + 1);
    }

    [Theory]
    [InlineData("Cylinder")]
    [InlineData("Cone")]
    [InlineData("OpenHull")]
    [InlineData("OpenPatch")]
    public void DeferredMovingFamilies_ShouldPreserveTheirExistingPerMemberContact(string shape)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.Two), Rectangle(scene, 0, 4, -2, 0, Fixed64.Two) };
        LSCollider mover = shape switch
        {
            "Cylinder" => new LSCylinderCollider(),
            "Cone" => new LSConeCollider(),
            "OpenHull" => MeshTestFixtures.CreateConvexQuadFloor(),
            _ => MeshTestFixtures.CreateConvexQuadFloor(MeshColliderMode.Concave)
        };
        scene.CreateBody(mover, new(-Fixed64.Quarter, Fixed64.Zero, Fixed64.Quarter), FixedQuaternion.Identity);
        var ordinary = new ContactManifold(); var expected = new List<(bool Hit, ManifoldContact Row)>();
        foreach (LSCollider2D member in members)
        {
            bool hit = CollisionDetectionMixed.TryBuildManifold(mover, member, ordinary);
            expected.Add((hit, hit ? ordinary.PrimaryContact : default));
        }
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        for (int index = 0; index < members.Length; index++)
        {
            Assert.Equal(expected[index].Hit, CollisionDetectionMixed.TryBuildDiscreteManifold(mover, members[index], ordinary));
            if (!expected[index].Hit) continue;
            Assert.Equal(expected[index].Row.Normal, ordinary.PrimaryContact.Normal);
            Assert.Equal(expected[index].Row.Depth, ordinary.PrimaryContact.Depth);
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void SphericalContainedNearSeam_ShouldPreferTheActualBoundaryOverAFarRecovery(bool capsule, bool positiveX)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.Two), Rectangle(scene, 0, 4, -2, 0, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        Fixed64 x = positiveX ? Fixed64.Quarter : -Fixed64.Quarter;
        LSCollider mover = SphericalMover(scene, capsule, new(x, Fixed64.Zero, -Fixed64.Quarter), Fixed64.Half);
        var rows = Rows(mover, members); Assert.Single(rows);
        Assert.Equal(Vector3d.Backward, rows[0].Row.Normal); Assert.Equal(3 * Fixed64.Quarter, rows[0].Row.Depth);
        AssertWholeShapeExit(mover, members, new(x, Fixed64.Zero, Fixed64.Half), Vector3d.Forward);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SphericalPhysicalFace_ShouldNotHideASeparateConcavePeak(bool capsule)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, 0, 2, 0, 2, Fixed64.Two), Rectangle(scene, -2, 0, 0, 1, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCollider mover = SphericalMover(scene, capsule, new(Fixed64.FromFraction(1, 10), Fixed64.Zero, Fixed64.FromFraction(2, 5)), Fixed64.One);
        var rows = Rows(mover, members);
        Assert.Contains(rows, sample => sample.Row.Normal == Vector3d.Forward);
        Assert.Contains(rows, sample => sample.Row.Normal == Vector3d.Right
            && (sample.Row.Depth - Fixed64.FromFraction(9, 10)).Abs() <= Fixed64.FromRaw(4));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SphericalUnjoinedMembers_ShouldPreserveTheirOrdinaryWitnesses(bool capsule)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, -1, -2, 0, Fixed64.Two), Rectangle(scene, 1, 4, -2, 0, Fixed64.Two) };
        LSCollider mover = SphericalMover(scene, capsule, Vector3d.Forward * Fixed64.Quarter, 3 * Fixed64.Half);
        mover.Body!.SetRotation(FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)31, (Fixed64)17, (Fixed64)43));
        mover.RebuildRuntimeShapeOnly();
        var manifold = new ContactManifold(); var expected = new List<(Vector3d Normal, Fixed64 Depth, Vector3d A, Vector3d B)>();
        foreach (LSCollider2D member in members)
        {
            Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, member, manifold));
            ManifoldContact row = manifold.PrimaryContact;
            Assert.True(row.AnchorA.TryGetWorldPoint(out Vector3d a)); Assert.True(row.AnchorB.TryGetWorldPoint(out Vector3d b));
            expected.Add((row.Normal, row.Depth, a, b));
        }
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        for (int index = 0; index < members.Length; index++)
        {
            Assert.True(CollisionDetectionMixed.TryBuildDiscreteManifold(mover, members[index], manifold));
            ManifoldContact row = manifold.PrimaryContact;
            Assert.Equal(expected[index].Normal, row.Normal); Assert.Equal(expected[index].Depth, row.Depth);
            Assert.True(row.AnchorA.TryGetWorldPoint(out Vector3d a)); Assert.True(row.AnchorB.TryGetWorldPoint(out Vector3d b));
            Assert.Equal(expected[index].A, a); Assert.Equal(expected[index].B, b);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SphericalUnjoinedCornerBounds_ShouldNotTurnARealGapIntoAContact(bool capsule)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -1, 1, -1, 1, Fixed64.Two), Rectangle(scene, 4, 5, -1, 1, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCollider mover = SphericalMover(scene, capsule,
            new(Fixed64.FromFraction(7, 5), Fixed64.Zero, Fixed64.FromFraction(7, 5)), Fixed64.Half);
        Assert.True(mover.Bounds.Intersects(members[0].MixedBounds3D));
        Assert.Empty(Rows(mover, members));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SphericalConcaveResponse_ShouldIgnoreUnrelatedHeightBandCuts(bool capsule)
    {
        var expected = Evaluate(false);
        Assert.Equal(2, expected.Count);
        Assert.Equal(expected, Evaluate(true));

        (int Count, Vector3d Position, Vector3d Velocity) Evaluate(bool unrelatedBand)
        {
            using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
            var left = Rectangle(scene, -2, 0, -2, 2, Fixed64.Two);
            var right = Rectangle(scene, 0, 2, -2, 0, Fixed64.Two);
            LSCollider2D[] members = { left, right };
            if (unrelatedBand)
            {
                var separate = new LSAABBoxCollider2D(Vector2d.One * Fixed64.FromFraction(1, 10))
                    { MixedHalfThicknessOverride = Fixed64.Quarter };
                separate.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(
                    new Vector3d(3 * Fixed64.Quarter, -Fixed64.Quarter, 3 * Fixed64.Quarter), FixedQuaternion.Identity, Vector3d.One)));
                members = new[] { left, right, separate };
            }
            using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
            LSCollider mover = SphericalMover(scene, capsule, new(-Fixed64.Quarter, Fixed64.Zero, -Fixed64.Quarter), Fixed64.Half);
            SolidBody body = mover.Body!; body.UseManualGrounding(); body.SleepEnabled = false;
            body.FreezeAxes = BodyFreezeAxes3D.PositionY | BodyFreezeAxes3D.Rotation;
            int count = Rows(mover, left, right).Count;
            scene.Context.Simulate(); scene.Context.LateSimulate();
            return (count, body.Position3d, body.LinearVelocity);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SphericalConcaveCapVertex_ShouldKeepOnePeakPerPhysicalFace(bool capsule)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var lower = Rectangle(scene, -2, 0, -2, 0, Fixed64.One);
        var members = new[] { lower, Rectangle(scene, 0, 2, -2, 0, Fixed64.Two),
            Rectangle(scene, -2, 0, 0, 2, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCollider mover = SphericalMover(scene, capsule,
            new(Fixed64.Quarter, Fixed64.FromFraction(9, 10), Fixed64.Quarter), Fixed64.Half);
        var cap = Rows(mover, members).Where(sample => sample.Supplier == lower.Id + 1 && sample.Row.Normal == Vector3d.Down).ToArray();
        Assert.Single(cap);
        Assert.True(cap[0].Row.AnchorB.TryGetWorldPoint(out Vector3d point));
        Assert.Equal(Vector3d.Up, point);
    }

    private static LSCollider SphericalMover(PhysicsScenarioBuilder scene, bool capsule, Vector3d position, Fixed64 radius)
    {
        LSCollider collider = capsule ? new LSCapsuleCollider { Radius = radius, Size = Vector3d.One * (radius * Fixed64.Two) }
            : new LSSphereCollider { Radius = radius };
        collider.Material = PhysicsMaterial.Frictionless;
        scene.CreateBody(collider, position, FixedQuaternion.Identity);
        if (collider is LSCapsuleCollider limit) Assert.Equal(Fixed64.Zero, limit.AxisLength);
        return collider;
    }
}
