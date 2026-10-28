using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests;

/// <content>Checks finite capsule support intervals against exposed joined prisms.</content>
public sealed partial class JoinedStaticSolidMixedTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void CapsuleSeamSupport_ShouldMatchTheContinuousWallAndClearTheWholeShape(bool horizontal, bool positiveX)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.Two),
            Rectangle(scene, 0, 4, -2, 0, Fixed64.Two) };
        LSCollider2D continuous = Rectangle(scene, -4, 4, -2, 0, Fixed64.Two);
        Fixed64 x = positiveX ? Fixed64.Quarter : -Fixed64.Quarter;
        LSCapsuleCollider mover = CapsuleMover(scene, new(x, Fixed64.Zero, Fixed64.Quarter), horizontal, Fixed64.Two);
        var ordinary = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, continuous, ordinary));
        ManifoldContact expected = ordinary.PrimaryContact;
        Assert.Equal(Vector3d.Backward, expected.Normal); Assert.Equal(Fixed64.Quarter, expected.Depth);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        var regions = new HashSet<int>();
        foreach (var sample in rows)
        {
            regions.Add(sample.Region);
            Assert.Equal(expected.Normal, sample.Row.Normal); Assert.Equal(expected.Depth, sample.Row.Depth);
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(-Fixed64.Quarter, moving.Z); Assert.Equal(Fixed64.Zero, source.Z);
            Assert.Equal(moving.X, source.X); Assert.Equal(moving.Y, source.Y);
            if (horizontal)
            {
                Assert.Equal(Fixed64.Zero, source.Y);
                Assert.InRange(source.X.m_rawValue, (x - Fixed64.One).m_rawValue, (x + Fixed64.One).m_rawValue);
            }
            else
            {
                Assert.Equal(x, source.X);
                Assert.InRange(source.Y.m_rawValue, (-Fixed64.One).m_rawValue, Fixed64.One.m_rawValue);
            }
        }
        Assert.Single(regions);
        AssertWholeShapeExit(mover, members, new(x, Fixed64.Zero, Fixed64.Half), Vector3d.Forward);
    }

    [Fact]
    public void CapsuleCoveredSupportMidpoint_ShouldRetainBothExposedIntervals()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        LSCollider2D wall = Rectangle(scene, -4, 4, -2, 0, Fixed64.Two);
        LSCollider2D tab = Rectangle(scene, -1, 1, 0, 2, Fixed64.Two);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { wall, tab });
        LSCapsuleCollider mover = CapsuleMover(scene, Vector3d.Forward * Fixed64.Quarter, true, (Fixed64)4);
        var rows = Rows(mover, wall); Assert.NotEmpty(rows);
        var regions = new HashSet<int>(); bool left = false, right = false;
        foreach (var sample in rows)
        {
            if (sample.Row.Normal != Vector3d.Backward) continue;
            Assert.Equal(Fixed64.Quarter, sample.Row.Depth); Assert.Equal(wall.Id + 1, sample.Supplier);
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(Fixed64.Zero, source.Z); Assert.Equal(Fixed64.Zero, source.Y);
            Assert.Equal(source.X, moving.X); Assert.Equal(-Fixed64.Quarter, moving.Z);
            Assert.InRange(source.X.m_rawValue, (-Fixed64.Two).m_rawValue, Fixed64.Two.m_rawValue);
            Assert.True(source.X <= -Fixed64.One || source.X >= Fixed64.One,
                "The shared middle span is filled by the tab, so it cannot supply a wall response.");
            left |= source.X <= -Fixed64.One; right |= source.X >= Fixed64.One;
            regions.Add(sample.Region);
        }
        Assert.True(left && right); Assert.Equal(2, regions.Count);
        // The tab is a separate genuine constraint. These local wall rows are
        // not a promise that translating by their depth clears the whole union.
    }

    [Fact]
    public void CapsuleScalarYawJoin_ShouldMatchTheEquivalentContinuousNativePrism()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        Vector3d origin = new(3, 0, -2);
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)31, Fixed64.Zero);
        LSPolygonCollider2D left = Polygon(-4, 0), right = Polygon(0, 4), continuous = Polygon(-4, 4);
        Assert.NotEqual(Fixed64.Zero, left.ConvexRotation); Assert.Equal(left.ConvexRotation, right.ConvexRotation);
        Vector2d offset = new Vector2d(Fixed64.Quarter, Fixed64.Quarter)
            .Rotated(FixedMath.Cos(left.ConvexRotation), FixedMath.Sin(left.ConvexRotation));
        LSCapsuleCollider mover = CapsuleMover(scene, origin + new Vector3d(offset.X, Fixed64.Zero, offset.Y), false, Fixed64.Two);
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
            Assert.Equal(expectedSource.X, source.X); Assert.Equal(expectedSource.Z, source.Z);
            Assert.InRange(source.Y.m_rawValue, (-Fixed64.One).m_rawValue, Fixed64.One.m_rawValue);
        }

        LSPolygonCollider2D Polygon(int first, int last)
        {
            var polygon = new LSPolygonCollider2D(new(first, -2), new(last, -2), new(last, 0), new(first, 0))
                { MixedHalfThicknessOverride = Fixed64.Two, Material = PhysicsMaterial.Frictionless };
            polygon.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(origin, rotation, Vector3d.One)));
            return polygon;
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void CapsuleJoinedWallCrossing_ShouldKeepTangentialVelocityAndMemberEvents(bool horizontal, bool rotationFree)
    {
        Trace(joined: false);
        var expected = Trace(joined: true);
        Assert.Equal(expected, Trace(joined: true));

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
            LSCapsuleCollider collider = CapsuleMover(scene, new(-Fixed64.Two, Fixed64.Zero, Fixed64.Half), horizontal, Fixed64.Two);
            SolidBody body = collider.Body!;
            body.FreezeAxes = BodyFreezeAxes3D.PositionY | (rotationFree ? BodyFreezeAxes3D.None : BodyFreezeAxes3D.Rotation);
            body.SleepEnabled = false; body.UseManualGrounding();
            for (int frame = 0; frame < 32; frame++) Step();
            body.AddLinearImpulse(Vector3d.Right);
            var trace = new List<(Vector3d Position, Vector3d Linear, Vector3d Angular, FixedQuaternion Rotation)>();
            // A 1/16 impulse integrates over a 1/16 step. At 80% correction,
            // steady excess above slop is (1-.8)/.8 * 1/256 = 1/1024.
            Fixed64 clearance = CollisionResponseMixed.PenetrationSlop + Fixed64.FromFraction(1, 1024)
                + Fixed64.MinIncrement + Fixed64.MinIncrement;
            for (int frame = 0; frame < 64; frame++)
            {
                Vector3d beforePosition = body.Position3d;
                FixedBoundBox before = FixedBoundBox.FromCenteredCapsuleClippedToDomain(beforePosition, body.Rotation,
                    Vector3d.Up, collider.AxisLength, collider.ScaledRadius);
                Fixed64 incomingZ = body.LinearVelocity.Z - Fixed64.FromFraction(1, 16);
                Step();
                Vector3d p = body.Position3d, v = body.LinearVelocity, w = body.AngularVelocity;
                FixedQuaternion q = body.Rotation;
                string state = $"joined={joined}, frame={frame}, positionRaw=({p.X.m_rawValue},{p.Y.m_rawValue},{p.Z.m_rawValue}), "
                    + $"velocityRaw=({v.X.m_rawValue},{v.Y.m_rawValue},{v.Z.m_rawValue}), angularRaw=({w.X.m_rawValue},{w.Y.m_rawValue},{w.Z.m_rawValue}), "
                    + $"rotationRaw=({q.X.m_rawValue},{q.Y.m_rawValue},{q.Z.m_rawValue},{q.W.m_rawValue})";
                Assert.True(body.LinearVelocity.X == Fixed64.One,
                    state);
                Assert.Equal(Fixed64.Zero, body.Position3d.Y);
                if (!rotationFree) Assert.Equal(Vector3d.Zero, body.AngularVelocity);
                Assert.True(body.Position3d.Z > Fixed64.Zero);
                // Discrete correction publishes body pose before the next
                // collider refresh; evaluate the authoritative finite shape here.
                FixedBoundBox finalBounds = FixedBoundBox.FromCenteredCapsuleClippedToDomain(
                    p, q, Vector3d.Up, collider.AxisLength, collider.ScaledRadius);
                Fixed64 permitted = clearance;
                if (rotationFree)
                {
                    // Finite solver iterations can leave angular motion. Check
                    // actual whole-shape correction, including its new rotated reach.
                    Fixed64 radiusBefore = beforePosition.Z - before.Min.Z, radiusAfter = p.Z - finalBounds.Min.Z;
                    Fixed64 predicted = -before.Min.Z - incomingZ * scene.Context.DeltaTime + radiusAfter - radiusBefore;
                    Fixed64 slop = CollisionResponseMixed.PenetrationSlop;
                    permitted = slop + (Fixed64.One - CollisionResponseMixed.PenetrationCorrectionPercent)
                        * FixedMath.Max(Fixed64.Zero, predicted - slop) + Fixed64.FromRaw(4);
                }
                Assert.True(finalBounds.Min.Z >= -permitted,
                    $"{state}, minimumZRaw={finalBounds.Min.Z.m_rawValue}, allowedPenetrationRaw={permitted.m_rawValue}");
                trace.Add((body.Position3d, body.LinearVelocity, body.AngularVelocity, body.Rotation));
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
    public void CapsuleConcaveCut_ShouldKeepEachGenuineFaceConstraint(bool inside)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -2, 0, -2, 2, Fixed64.Two), Rectangle(scene, 0, 2, -2, 0, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        Fixed64 offset = inside ? -Fixed64.Quarter : Fixed64.Quarter;
        LSCapsuleCollider mover = CapsuleMover(scene, new(offset, Fixed64.Zero, offset), false, Fixed64.Two);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        var normals = new HashSet<Vector3d>();
        foreach (var sample in rows)
        {
            normals.Add(sample.Row.Normal);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d point));
            Assert.InRange(point.Y.m_rawValue, (-Fixed64.One).m_rawValue, Fixed64.One.m_rawValue);
            Fixed64 tangent = (sample.Row.Normal == Vector3d.Left ? point.Z : point.X) - offset;
            Assert.InRange(tangent.m_rawValue, (-Fixed64.Half).m_rawValue, Fixed64.Half.m_rawValue);
            Fixed64 expected = FixedMath.Sqrt(Fixed64.Quarter - tangent * tangent) - offset;
            // A normal ray through the final finite witness intersects the
            // circular section. Four raw units cover witness/sqrt rounding.
            Assert.InRange(sample.Row.Depth.m_rawValue, expected.m_rawValue - 4, expected.m_rawValue + 4);
            if (inside) { Assert.Equal(Fixed64.Zero, point.X); Assert.Equal(Fixed64.Zero, point.Z); }
            else if (sample.Row.Normal == Vector3d.Left)
            { Assert.Equal(Fixed64.Zero, point.X); Assert.InRange(point.Z.m_rawValue, 0, Fixed64.Quarter.m_rawValue); }
            else { Assert.InRange(point.X.m_rawValue, 0, Fixed64.Quarter.m_rawValue); Assert.Equal(Fixed64.Zero, point.Z); }
        }
        Assert.Equal(2, normals.Count);
        Assert.Contains(Vector3d.Left, normals); Assert.Contains(Vector3d.Backward, normals);
        if (!inside)
        {
            Assert.Contains(rows, sample => sample.Row.Normal == Vector3d.Left && sample.Row.Depth == Fixed64.Quarter);
            Assert.Contains(rows, sample => sample.Row.Normal == Vector3d.Backward && sample.Row.Depth == Fixed64.Quarter);
        }
    }

    [Fact]
    public void CapsuleHeightStep_ShouldClipTheParallelSupportToItsExposedUpperBand()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -2, 0, -2, 0, Fixed64.Two), Rectangle(scene, 0, 2, -2, 0, Fixed64.One) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCapsuleCollider mover = CapsuleMover(scene, new(Fixed64.Quarter, 3 * Fixed64.Half, -Fixed64.One), false, Fixed64.Two);
        var rows = Rows(mover, members);
        Assert.Contains(rows, sample => sample.Row.Normal == Vector3d.Left && sample.Row.Depth == Fixed64.Quarter
            && sample.Supplier == members[0].Id + 1);
        Assert.Contains(rows, sample => sample.Row.Normal == Vector3d.Down && sample.Row.Depth == Fixed64.One
            && sample.Supplier == members[1].Id + 1);
        foreach (var sample in rows)
        {
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d point)); Assert.Equal(-Fixed64.One, point.Z);
            if (sample.Row.Normal == Vector3d.Left)
            {
                Assert.Equal(Fixed64.Zero, point.X);
                Assert.InRange(point.Y.m_rawValue, Fixed64.One.m_rawValue, Fixed64.Two.m_rawValue);
            }
            if (sample.Row.Normal == Vector3d.Down) Assert.Equal(Fixed64.One, point.Y);
        }
    }

    [Fact]
    public void CapsuleObliqueQuaternion_ShouldPreserveItsContinuousWallSupport()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.Two), Rectangle(scene, 0, 4, -2, 0, Fixed64.Two) };
        LSCollider2D continuous = Rectangle(scene, -4, 4, -2, 0, Fixed64.Two);
        LSCapsuleCollider mover = CapsuleMover(scene, Vector3d.Forward * Fixed64.Quarter, false, Fixed64.Two);
        mover.Body!.SetRotation(FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)17, (Fixed64)31, (Fixed64)23));
        mover.RebuildRuntimeShapeOnly();
        var manifold = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, continuous, manifold));
        ManifoldContact expected = manifold.PrimaryContact;
        Assert.Equal(Vector3d.Backward, expected.Normal);
        Assert.True(expected.AnchorB.TryGetWorldPoint(out Vector3d expectedSource));
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.Equal(expected.Normal, sample.Row.Normal); Assert.Equal(expected.Depth, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source)); Assert.Equal(expectedSource, source);
        }
        SetPosition(mover, Vector3d.Forward * (Fixed64)3);
        foreach (LSCollider2D member in members) Assert.False(CollisionDetectionMixed.TryBuildManifold(mover, member, manifold));
    }

    [Fact]
    public void CapsuleUnjoinedMembers_ShouldPreserveTheirOrdinaryFiniteWitnesses()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, -1, -2, 0, Fixed64.Two), Rectangle(scene, 1, 4, -2, 0, Fixed64.Two) };
        LSCapsuleCollider mover = CapsuleMover(scene, Vector3d.Forward * Fixed64.Quarter, true, Fixed64.Two);
        var manifold = new ContactManifold(); var expected = new List<(Vector3d Normal, Fixed64 Depth, Vector3d A, Vector3d B)>();
        foreach (LSCollider2D member in members)
        {
            Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, member, manifold));
            ManifoldContact row = manifold.PrimaryContact;
            Assert.True(row.AnchorA.TryGetWorldPoint(out Vector3d a)); Assert.True(row.AnchorB.TryGetWorldPoint(out Vector3d b));
            expected.Add((row.Normal, row.Depth, a, b));
        }
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        for (int member = 0; member < members.Length; member++)
        {
            Assert.True(CollisionDetectionMixed.TryBuildDiscreteManifold(mover, members[member], manifold));
            ManifoldContact row = manifold.PrimaryContact;
            Assert.Equal(expected[member].Normal, row.Normal); Assert.Equal(expected[member].Depth, row.Depth);
            Assert.True(row.AnchorA.TryGetWorldPoint(out Vector3d a)); Assert.True(row.AnchorB.TryGetWorldPoint(out Vector3d b));
            Assert.Equal(expected[member].A, a); Assert.Equal(expected[member].B, b);
        }
    }

    [Fact]
    public void CapsuleLaterEligibilityCallback_ShouldRetireBorrowedMaterialRowsBeforeResponse()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var left = Rectangle(scene, -4, 0, -2, 0, Fixed64.Two);
        var right = Rectangle(scene, 0, 4, -2, 0, Fixed64.Two); right.Material = PhysicsMaterial.Bouncy;
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right });
        LSCapsuleCollider mover = CapsuleMover(scene, new(Fixed64.Quarter, Fixed64.Zero, Fixed64.Quarter), true, Fixed64.Two);
        var initial = Rows(mover, left, right); var suppliers = new HashSet<int>();
        foreach (var sample in initial)
        {
            suppliers.Add(sample.Supplier); Assert.True(sample.Row.HasMaterialOverride);
            Assert.Equal(sample.Supplier == left.Id + 1 ? left.Material : right.Material, sample.Row.MaterialB);
        }
        Assert.Contains(left.Id + 1, suppliers); Assert.Contains(right.Id + 1, suppliers);
        SolidBody body = mover.Body!;
        body.FreezeAxes = BodyFreezeAxes3D.PositionY | BodyFreezeAxes3D.Rotation;
        body.SleepEnabled = false; body.UseManualGrounding(); body.AddLinearImpulse(Vector3d.Backward);
        bool changed = false;
        left.OnMixedContactEnter += _ =>
        {
            right.IgnoredCollisionLayers = PhysicsLayerMask.All;
            changed = true;
        };
        scene.Context.Simulate(); scene.Context.LateSimulate(); Assert.True(changed);
        CollisionPairMixed response = NativePair(scene.Context, mover, left); Assert.NotEmpty(response.Manifold);
        for (int group = 0; group < response.Manifold.GroupCount; group++)
        {
            ref ContactGroup contacts = ref response.Manifold.GetGroup(group);
            Assert.NotEqual(right.Id + 1, contacts.Key.SourceIdB);
            for (int row = 0; row < contacts.Count; row++)
                if (contacts[row].HasMaterialOverride) Assert.Equal(left.Material, contacts[row].MaterialB);
        }
        Assert.Equal(Fixed64.Zero, body.LinearVelocity.Z); Assert.True(body.Position3d.Z > Fixed64.Quarter);
    }

    [Fact]
    public void CapsuleAmbiguousCapSeam_ShouldKeepCoincidentSupportRegionsIndependent()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        LSCollider2D left = Rectangle(scene, -2, 0, -2, 2, Fixed64.Two);
        LSCollider2D right = Rectangle(scene, 0, 2, -2, 2, Fixed64.Two);
        LSCollider2D middle = Rectangle(scene, -1, 1, -1, 1, Fixed64.Two);
        left.Material = PhysicsMaterial.Bouncy;
        right.Material = PhysicsMaterial.Default;
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right, middle });
        LSCapsuleCollider mover = CapsuleMover(scene, Vector3d.Up * (11 * Fixed64.Quarter), false, Fixed64.One);

        // The global component joins outside Z=[-1,1]. The capsule reaches
        // only the ambiguous middle span, so its coincident cap point cannot
        // authorize continuity between the left and right support regions.
        var rows = Rows(mover, left, right); Assert.NotEmpty(rows);
        var regions = new HashSet<int>(); var suppliers = new HashSet<int>();
        foreach (var sample in rows)
        {
            Assert.Equal(Vector3d.Down, sample.Row.Normal);
            Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(Vector3d.Up * (7 * Fixed64.Quarter), moving);
            Assert.Equal(Vector3d.Up * Fixed64.Two, source);
            Assert.True(sample.Row.HasMaterialOverride);
            Assert.Equal(sample.Supplier == left.Id + 1 ? left.Material : right.Material, sample.Row.MaterialB);
            regions.Add(sample.Region); suppliers.Add(sample.Supplier);
        }
        Assert.Equal(2, regions.Count);
        Assert.Equal(2, suppliers.Count);
        Assert.Contains(left.Id + 1, suppliers); Assert.Contains(right.Id + 1, suppliers);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CapsuleConcaveCapVertex_ShouldKeepOnePeakPerPhysicalFace(bool opposite)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        LSCollider2D lower = Rectangle(scene, -2, 0, -2, 0, Fixed64.One);
        var members = opposite
            ? new[] { lower, Rectangle(scene, -4, -2, -2, 0, Fixed64.Two), Rectangle(scene, -2, 0, -4, -2, Fixed64.Two) }
            : new[] { lower, Rectangle(scene, 0, 2, -2, 0, Fixed64.Two), Rectangle(scene, -2, 0, 0, 2, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        Fixed64 offset = opposite ? -9 * Fixed64.Quarter : Fixed64.Quarter;
        LSCapsuleCollider mover = CapsuleMover(scene,
            new(offset, Fixed64.FromFraction(9, 10), offset), false, Fixed64.Quarter);

        // Both joined cap edges meet at the same finite point. Its two
        // producers belong to one physical cap constraint, not two rows.
        int capCount = 0;
        foreach (var sample in Rows(mover, members))
        {
            if (sample.Supplier != lower.Id + 1 || sample.Row.Normal != Vector3d.Down) continue;
            capCount++;
            Assert.True(sample.Row.Depth > Fixed64.Zero);
            Assert.False(sample.Row.DepthIsClamped);
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out _));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(opposite ? new Vector3d(-2, 1, -2) : Vector3d.Up, source);
        }
        Assert.Equal(1, capCount);
    }

    [Fact]
    public void CapsuleDeepWideUnion_ShouldDiscoverTheRemoteJoinBeforePublishingRecovery()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -10, 4, -10, 10, (Fixed64)10), Rectangle(scene, 4, 10, -10, 10, (Fixed64)10) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCapsuleCollider mover = CapsuleMover(scene, Vector3d.Zero, false, Fixed64.Two);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.Equal(10 * Fixed64.One + Fixed64.Half, sample.Row.Depth);
            Assert.Equal(Fixed64.Zero, sample.Row.Normal.Y);
            Assert.Equal(Fixed64.One, sample.Row.Normal.X.Abs() + sample.Row.Normal.Z.Abs());
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(-(Fixed64)10, Vector3d.Dot(source, sample.Row.Normal));
            Assert.False(sample.Row.DepthIsClamped);
        }
        ManifoldContact first = rows[0].Row;
        AssertWholeShapeExit(mover, members, -first.Normal * first.Depth, -first.Normal);
    }

    [Fact]
    public void CapsuleUnjoinedCornerGap_ShouldRejectAnOverlappingBoundsCandidate()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.Two), Rectangle(scene, 0, 4, -2, 0, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        Fixed64 delta = Fixed64.FromFraction(2, 5);
        LSCapsuleCollider mover = CapsuleMover(scene, new(4 * Fixed64.One + delta, Fixed64.Zero, delta), false, Fixed64.Two);
        Assert.True(mover.Bounds.Intersects(members[1].MixedBounds3D));
        // The vertical core is parallel to the corner: its exact planar gap
        // has square 8/25, greater than radius squared 1/4.
        var ordinary = new ContactManifold();
        Assert.False(CollisionDetectionMixed.TryBuildManifold(mover, members[1], ordinary));
        Assert.Empty(Rows(mover, members));
    }

    [Fact]
    public void CapsuleRemoteConcaveAndIndependentPieces_ShouldNotChangeTheLocalOuterFace()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -2, 0, -2, 2, Fixed64.Two), Rectangle(scene, 0, 2, -2, 0, Fixed64.Two),
            Rectangle(scene, 1, 2, 1, 2, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCapsuleCollider mover = CapsuleMover(scene, Vector3d.Zero, false, Fixed64.Two);
        var ordinary = new ContactManifold();
        foreach (Fixed64 offset in new[] { 3 * Fixed64.Half, 5 * Fixed64.Quarter })
        {
            Vector3d position = new(-offset, Fixed64.Zero, offset);
            SetPosition(mover, position);
            Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, members[0], ordinary));
            ManifoldContact expected = ordinary.PrimaryContact;
            Assert.Equal(5 * Fixed64.Half - offset, expected.Depth);
            for (int remote = 1; remote < members.Length; remote++)
                Assert.False(CollisionDetectionMixed.TryBuildManifold(mover, members[remote], ordinary));
            var rows = Rows(mover, members); Assert.NotEmpty(rows);
            foreach (var sample in rows)
            {
                Assert.Equal(members[0].Id + 1, sample.Supplier);
                Assert.Equal(expected.Normal, sample.Row.Normal); Assert.Equal(expected.Depth, sample.Row.Depth);
                Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
                Assert.True(members[0].ContainsPoint(new(source.X, source.Z)));
                Assert.InRange(source.Y.m_rawValue, (-Fixed64.One).m_rawValue, Fixed64.One.m_rawValue);
            }
            AssertWholeShapeExit(mover, members, position - expected.Normal * expected.Depth, -expected.Normal);
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
    public void CapsuleRealOuterCapRim_ShouldPreserveItsEndpointDistanceAcrossRemoteHeightChanges(bool corner, bool stepped, bool lower)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        Fixed64 half = stepped ? Fixed64.Two : Fixed64.One;
        var members = new[] { Rectangle(scene, -4, -2, -2, 2, Fixed64.One), Rectangle(scene, -2, 0, -2, 2, half) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        Fixed64 sign = lower ? -Fixed64.One : Fixed64.One;
        Vector3d center = new(Fixed64.Quarter, sign * (half + Fixed64.One), corner ? 9 * Fixed64.Quarter : Fixed64.One);
        var mover = new LSCapsuleCollider { Radius = Fixed64.One, Size = new(2, 3, 2), Material = PhysicsMaterial.Frictionless };
        scene.CreateBody(mover, center, FixedQuaternion.Identity);
        Assert.Equal(Fixed64.One, mover.AxisLength);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        Fixed64 expected = Fixed64.One - FixedMath.Sqrt((Fixed64)(corner ? 6 : 5)) * Fixed64.Quarter;
        foreach (var sample in rows)
        {
            Assert.Equal(members[1].Id + 1, sample.Supplier);
            Assert.InRange(sample.Row.Depth.m_rawValue, expected.m_rawValue - 2, expected.m_rawValue + 2);
            Assert.False(sample.Row.DepthIsClamped);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d target));
            Assert.Equal(new Vector3d(Fixed64.Zero, sign * half, corner ? Fixed64.Two : Fixed64.One), target);
            Assert.True(sample.Row.Normal.X < Fixed64.Zero);
            Assert.True(lower ? sample.Row.Normal.Y > Fixed64.Zero : sample.Row.Normal.Y < Fixed64.Zero);
            if (corner) Assert.Equal(sample.Row.Normal.X, sample.Row.Normal.Z);
            else Assert.Equal(Fixed64.Zero, sample.Row.Normal.Z);
        }
        SetPosition(mover, center + Vector3d.Right * Fixed64.Two);
        var ordinary = new ContactManifold();
        foreach (LSCollider2D member in members) Assert.False(CollisionDetectionMixed.TryBuildManifold(mover, member, ordinary));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CapsuleConcaveFaceLines_ShouldConnectOnlyThroughAnAdmittedJunction(bool connected)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        LSCollider2D wall = Rectangle(scene, -2, 2, -2, 0, Fixed64.Two);
        LSCollider2D tab = Rectangle(scene, 0, 2, 0, 2, Fixed64.One);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { wall, tab });
        Fixed64 x = connected ? Fixed64.Quarter : Fixed64.FromFraction(2, 5);
        Fixed64 y = connected ? 5 * Fixed64.Quarter : Fixed64.FromFraction(3, 5);
        Vector3d center = new(x, y, Fixed64.Quarter);
        // Reuse committed topology across shape families, rather than assuming
        // the next mover owns a freshly prepared declaration.
        Assert.NotEmpty(Rows(SphericalMover(scene, false, center, Fixed64.Half), wall, tab));
        Fixed64 length = connected ? Fixed64.Two : Fixed64.FromFraction(1, 5);
        LSCapsuleCollider mover = CapsuleMover(scene, center, false, length);
        var rows = Rows(mover, wall); var regions = new HashSet<int>();
        foreach (var sample in rows)
        {
            if (sample.Row.Normal != Vector3d.Backward) continue;
            Assert.Equal(wall.Id + 1, sample.Supplier);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(Fixed64.Zero, source.Z);
            Assert.True(source.X <= Fixed64.Zero || source.Y >= Fixed64.One,
                "The lower-right part of the original face is filled by the tab.");
            Fixed64 dx = source.X - center.X;
            Fixed64 dy = FixedMath.Max(Fixed64.Zero, (source.Y - center.Y).Abs() - length * Fixed64.Half);
            Fixed64 radial = Fixed64.Quarter - dx * dx - dy * dy;
            Assert.True(radial >= Fixed64.Zero);
            Fixed64 expected = FixedMath.Sqrt(radial) - Fixed64.Quarter;
            Assert.InRange(sample.Row.Depth.m_rawValue, expected.m_rawValue - 4, expected.m_rawValue + 4);
            regions.Add(sample.Region);
        }
        // The disconnected fixture's junction gap has squared length
        // (2/5)^2+(3/10)^2+(1/4)^2=5/16, exceeding radius squared 1/4.
        // Both boundary lines still meet the capsule away from that junction.
        Assert.Equal(connected ? 1 : 2, regions.Count);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CapsuleSubdividedHeightStep_ShouldPreserveItsContinuousBoundaryPeak(bool lower)
    {
        Assert.Equal(Capture(split: false), Capture(split: true));

        List<(Vector3d Point, Fixed64 Depth)> Capture(bool split)
        {
            using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
            LSCollider2D wall = Rectangle(scene, -2, 2, -2, 0, Fixed64.Two);
            LSCollider2D[] members = split
                ? new[] { wall, Rectangle(scene, 0, 1, 0, 2, Fixed64.One), Rectangle(scene, 1, 2, 0, 2, Fixed64.One) }
                : new[] { wall, Rectangle(scene, 0, 2, 0, 2, Fixed64.One) };
            using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
            LSCapsuleCollider mover = CapsuleMover(scene, new(3 * Fixed64.Quarter,
                (lower ? -Fixed64.One : Fixed64.One) * (5 * Fixed64.Quarter), Fixed64.Quarter), false, Fixed64.Two);
            var result = new List<(Vector3d Point, Fixed64 Depth)>();
            foreach (var sample in Rows(mover, wall))
            {
                if (sample.Row.Normal != Vector3d.Backward) continue;
                Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d point));
                Assert.Equal(Fixed64.Zero, point.Z);
                Assert.True(point.X <= Fixed64.Zero || point.Y.Abs() >= Fixed64.One);
                result.Add((point, sample.Row.Depth));
            }
            Assert.NotEmpty(result);
            result.Sort(static (a, b) =>
            {
                int order = a.Point.X.CompareTo(b.Point.X);
                if (order == 0) order = a.Point.Y.CompareTo(b.Point.Y);
                if (order == 0) order = a.Point.Z.CompareTo(b.Point.Z);
                return order != 0 ? order : a.Depth.CompareTo(b.Depth);
            });
            return result;
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CapsuleContainedAcrossTheSeam_ShouldChooseTheLeastRealOuterRecovery(bool offCenter)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -10, 0, -10, 10, (Fixed64)10), Rectangle(scene, 0, 10, -10, 10, (Fixed64)10) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        Fixed64 x = offCenter ? Fixed64.Quarter : Fixed64.Zero;
        LSCapsuleCollider mover = CapsuleMover(scene, Vector3d.Right * x, false, Fixed64.Two);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        Fixed64 depth = 10 * Fixed64.One + Fixed64.Half - x;
        foreach (var sample in rows)
        {
            Assert.Equal(depth, sample.Row.Depth); Assert.False(sample.Row.DepthIsClamped);
            if (offCenter) Assert.Equal(Vector3d.Left, sample.Row.Normal);
            Assert.Equal(Fixed64.Zero, sample.Row.Normal.Y);
            Assert.Equal(Fixed64.One, sample.Row.Normal.X.Abs() + sample.Row.Normal.Z.Abs());
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(-(Fixed64)10, Vector3d.Dot(source, sample.Row.Normal));
        }
        ManifoldContact first = rows[0].Row;
        AssertWholeShapeExit(mover, members, Vector3d.Right * x - first.Normal * depth, -first.Normal);
    }

    [Fact]
    public void CapsuleOffsetHeightNeighbors_ShouldPreserveOnlyTheirActualExposedFaceWindows()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        LSCollider2D wall = Rectangle(scene, -2, 2, -2, 0, Fixed64.Two);
        var members = new[] { wall, Rectangle(scene, -1, 1, 0, 2, Fixed64.One, -Fixed64.One),
            Rectangle(scene, 1, 2, 0, 2, Fixed64.One, Fixed64.One) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCapsuleCollider mover = CapsuleMover(scene, new(Fixed64.Quarter, 5 * Fixed64.Quarter, Fixed64.Quarter), false, Fixed64.Two);
        var rows = Rows(mover, wall);
        Assert.Contains(rows, sample => sample.Row.Normal == Vector3d.Backward && sample.Row.Depth == Fixed64.Quarter);
        foreach (var sample in rows)
        {
            if (sample.Row.Normal != Vector3d.Backward) continue;
            Assert.Equal(wall.Id + 1, sample.Supplier);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d point));
            Assert.Equal(Fixed64.Zero, point.Z);
            Assert.True(point.Y >= Fixed64.Zero || point.X <= -Fixed64.One || point.X >= Fixed64.One);
            Assert.True(point.Y <= Fixed64.Zero || point.X <= Fixed64.One);
            Assert.InRange(point.X.m_rawValue, (-Fixed64.Two).m_rawValue, Fixed64.Two.m_rawValue);
            Assert.InRange(point.Y.m_rawValue, (-Fixed64.Two).m_rawValue, Fixed64.Two.m_rawValue);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CapsuleNearScalarBoundary_ShouldKeepItsFiniteContactWhenTheRemoteCapIsClipped(bool upper)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        Fixed64 y = upper ? Fixed64.MaxValue - Fixed64.One : Fixed64.MinValue + Fixed64.One;
        Fixed64 gridMin = upper ? y - 8 : y, gridMax = upper ? y : y + 8;
        Assert.True(scene.Context.World.TryAddGrid(new GridConfiguration(
            new Vector3d((Fixed64)(-16), gridMin, (Fixed64)(-16)), new Vector3d((Fixed64)16, gridMax, (Fixed64)16)), out _));
        var members = new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.Two, y), Rectangle(scene, 0, 4, -2, 0, Fixed64.Two, y) };
        foreach (LSCollider2D member in members)
            Assert.Equal(upper ? Fixed64.MaxValue : Fixed64.MinValue, upper ? member.MixedBounds3D.Max.Y : member.MixedBounds3D.Min.Y);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        Fixed64 sign = upper ? -Fixed64.One : Fixed64.One;
        Fixed64 cap = y + sign * Fixed64.Two;
        LSCapsuleCollider mover = CapsuleMover(scene, new(Fixed64.Zero, cap + sign * (5 * Fixed64.Quarter), -Fixed64.One), false, Fixed64.Two);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.Equal(upper ? Vector3d.Up : Vector3d.Down, sample.Row.Normal);
            Assert.Equal(Fixed64.Quarter, sample.Row.Depth); Assert.False(sample.Row.DepthIsClamped);
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(new Vector3d(Fixed64.Zero, cap, -Fixed64.One), source);
            Assert.Equal(cap - sign * Fixed64.Quarter, moving.Y);
        }
        AssertWholeShapeExit(mover, members,
            new(Fixed64.Zero, cap + sign * (3 * Fixed64.Half), -Fixed64.One), Vector3d.Up * sign);
    }

    [Fact]
    public void CapsuleClosedConcaveWindow_ShouldReduceItsConnectedBoundaryCycleOnce()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        LSCollider2D wall = Rectangle(scene, -2, 2, -4, 0, Fixed64.Two);
        var members = new[] { wall, Rectangle(scene, -2, -1, 0, 2, Fixed64.Two), Rectangle(scene, 1, 2, 0, 2, Fixed64.Two),
            Rectangle(scene, -1, 1, 0, 2, Fixed64.Half, -3 * Fixed64.Half),
            Rectangle(scene, -1, 1, 0, 2, Fixed64.Half, 3 * Fixed64.Half) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var mover = new LSCapsuleCollider { Radius = Fixed64.Two, Size = new(4, 6, 4), Material = PhysicsMaterial.Frictionless };
        scene.CreateBody(mover, Vector3d.Forward * Fixed64.Quarter, FixedQuaternion.Identity);
        var rows = Rows(mover, wall); Assert.NotEmpty(rows);
        Assert.True(rows.Count <= ContactManifold.MaxContactsPerGroup);
        var regions = new HashSet<int>();
        foreach (var sample in rows)
        {
            Assert.Equal(Vector3d.Backward, sample.Row.Normal); Assert.Equal(wall.Id + 1, sample.Supplier);
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(Fixed64.Zero, source.Z);
            Assert.InRange(source.X.m_rawValue, (-Fixed64.One).m_rawValue, Fixed64.One.m_rawValue);
            Assert.InRange(source.Y.m_rawValue, (-Fixed64.One).m_rawValue, Fixed64.One.m_rawValue);
            Fixed64 depth = FixedMath.Sqrt(4 * Fixed64.One - source.X * source.X) - Fixed64.Quarter;
            Assert.InRange(sample.Row.Depth.m_rawValue, depth.m_rawValue - 4, depth.m_rawValue + 4);
            Assert.Equal(source.X, moving.X); Assert.Equal(source.Y, moving.Y);
            Assert.True((moving.Z + sample.Row.Depth).Abs() <= Fixed64.MinIncrement);
            regions.Add(sample.Region);
        }
        Assert.Single(regions);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CapsuleSeparateCapCuts_ShouldKeepTheirAdmittedParentCap(bool oppositeCuts)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        int left = oppositeCuts ? -4 : -2;
        LSCollider2D lower = Rectangle(scene, left, 0, -2, 0, Fixed64.One);
        var members = oppositeCuts
            ? new[] { lower, Rectangle(scene, left, 0, 0, 2, Fixed64.Two), Rectangle(scene, left, 0, -4, -2, Fixed64.Two) }
            : new[] { lower, Rectangle(scene, 0, 2, -2, 0, Fixed64.Two), Rectangle(scene, -2, 0, 0, 2, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        Fixed64 x = oppositeCuts ? -Fixed64.Two : -Fixed64.FromFraction(2, 5);
        Fixed64 z = oppositeCuts ? -Fixed64.One : x;
        Fixed64 radius = oppositeCuts ? 5 * Fixed64.Quarter : Fixed64.Half;
        var mover = new LSCapsuleCollider { Radius = radius,
            Size = new Vector3d(radius * Fixed64.Two, radius * Fixed64.Two + Fixed64.Quarter, radius * Fixed64.Two),
            Material = PhysicsMaterial.Frictionless };
        scene.CreateBody(mover, new(x, Fixed64.FromFraction(9, 10), z), FixedQuaternion.Identity);
        Assert.Equal(Fixed64.Quarter, mover.AxisLength);
        var selected = Assert.Single(Rows(mover, lower));
        Assert.Equal(Vector3d.Down, selected.Row.Normal);
        Assert.Equal(lower.Id + 1, selected.Supplier);
        Assert.True(selected.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
        Assert.Equal(new Vector3d(x, Fixed64.One, z), source);
        Fixed64 depth = Fixed64.One - (Fixed64.FromFraction(9, 10) - Fixed64.FromFraction(1, 8)) + radius;
        Assert.Equal(depth, selected.Row.Depth);
        // Both finite rim cuts lie inside the same admitted full cap. Retain
        // its maximum once, rather than adding its shallower subset constraints.
        AssertWholeShapeExit(mover, new[] { lower }, new(x, Fixed64.One + Fixed64.FromFraction(1, 8) + radius, z), Vector3d.Up);
    }

    [Fact]
    public void CapsuleSeparatedHeightCuts_ShouldNotInventAConcaveLineAcrossTheirGap()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        LSCollider2D wall = Rectangle(scene, -4, 4, -2, 0, Fixed64.Two);
        var members = new[] { wall, Rectangle(scene, -3, -2, 0, 2, Fixed64.One), Rectangle(scene, 2, 3, 0, 2, Fixed64.One) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCapsuleCollider mover = CapsuleMover(scene, new(Fixed64.Zero, 5 * Fixed64.Quarter, Fixed64.Quarter), true, (Fixed64)6);
        var regions = new HashSet<int>(); bool left = false, right = false;
        foreach (var sample in Rows(mover, wall))
        {
            if (sample.Row.Normal != Vector3d.Backward) continue;
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(Fixed64.Zero, source.Z);
            if (source.Y != Fixed64.One) continue;
            Assert.True(source.X >= -(Fixed64)3 && source.X <= -Fixed64.Two
                || source.X >= Fixed64.Two && source.X <= (Fixed64)3);
            Fixed64 expected = FixedMath.Sqrt(Fixed64.FromFraction(3, 16)) - Fixed64.Quarter;
            Assert.InRange(sample.Row.Depth.m_rawValue, expected.m_rawValue - 4, expected.m_rawValue + 4);
            left |= source.X < Fixed64.Zero; right |= source.X > Fixed64.Zero;
            regions.Add(sample.Region);
        }
        Assert.True(left && right); Assert.Equal(2, regions.Count);
    }

    [Fact]
    public void CapsuleInitiallySpinning_ShouldContractItsActualPenetrationAndRepeatItsFullHostTrace()
    {
        Assert.Equal(Trace(), Trace());

        List<(Vector3d Position, Vector3d Linear, Vector3d Angular, FixedQuaternion Rotation)> Trace()
        {
            using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
            var members = new[] { Rectangle(scene, -8, 0, -4, 0, (Fixed64)4), Rectangle(scene, 0, 8, -4, 0, (Fixed64)4) };
            using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
            LSCapsuleCollider mover = CapsuleMover(scene, new(-Fixed64.Quarter, Fixed64.Zero, Fixed64.Half), true, Fixed64.Two);
            SolidBody body = mover.Body!;
            body.FreezeAxes = BodyFreezeAxes3D.PositionY;
            body.SleepEnabled = false; body.UseManualGrounding();
            body.AddLinearImpulse(Vector3d.Right);
            body.AddAngularImpulse(Vector3d.Up * Fixed64.FromFraction(1, 8));
            Assert.NotEqual(Vector3d.Zero, body.AngularVelocity);
            FixedQuaternion initialRotation = body.Rotation;
            var trace = new List<(Vector3d Position, Vector3d Linear, Vector3d Angular, FixedQuaternion Rotation)>();
            var probe = new ContactManifold(); int contactCount = 0;
            for (int frame = 0; frame < 32; frame++)
            {
                Vector3d beforePosition = body.Position3d;
                FixedBoundBox before = FixedBoundBox.FromCenteredCapsuleClippedToDomain(beforePosition, body.Rotation,
                    Vector3d.Up, mover.AxisLength, mover.ScaledRadius);
                body.AddLinearImpulse(Vector3d.Backward * Fixed64.FromFraction(1, 16));
                Fixed64 incomingZ = body.LinearVelocity.Z;
                scene.Context.Simulate(); scene.Context.LateSimulate();
                FixedBoundBox after = FixedBoundBox.FromCenteredCapsuleClippedToDomain(body.Position3d, body.Rotation,
                    Vector3d.Up, mover.AxisLength, mover.ScaledRadius);
                Fixed64 radiusBefore = beforePosition.Z - before.Min.Z, radiusAfter = body.Position3d.Z - after.Min.Z;
                Fixed64 predicted = -before.Min.Z - incomingZ * scene.Context.DeltaTime + radiusAfter - radiusBefore;
                Fixed64 slop = CollisionResponseMixed.PenetrationSlop;
                // Use the actual rotation's new finite reach, not a zero-spin
                // steady-state bound. Four raw units cover integration,
                // complementary correction products and outward bound rounding.
                Fixed64 permitted = slop + (Fixed64.One - CollisionResponseMixed.PenetrationCorrectionPercent)
                    * FixedMath.Max(Fixed64.Zero, predicted - slop) + Fixed64.FromRaw(4);
                Assert.True(-after.Min.Z <= permitted,
                    $"frame={frame}, predictedRaw={predicted.m_rawValue}, actualRaw={(-after.Min.Z).m_rawValue}, "
                    + $"permittedRaw={permitted.m_rawValue}, incomingZRaw={incomingZ.m_rawValue}, "
                    + $"radiusBeforeRaw={radiusBefore.m_rawValue}, radiusAfterRaw={radiusAfter.m_rawValue}");
                Assert.Equal(Fixed64.One, body.LinearVelocity.X); Assert.Equal(Fixed64.Zero, body.Position3d.Y);
                foreach (LSCollider2D member in members)
                {
                    if (!CollisionDetectionMixed.TryBuildManifold(mover, member, probe)) continue;
                    CollisionPairMixed pair = NativePair(scene.Context, mover, member);
                    for (int group = 0; group < pair.Manifold.GroupCount; group++)
                    {
                        ref ContactGroup contacts = ref pair.Manifold.GetGroup(group);
                        LSCollider2D supplier = contacts.Key.SourceIdB == 0 ? member
                            : contacts.Key.SourceIdB == members[0].Id + 1 ? members[0] : members[1];
                        if (contacts.Key.SourceIdB != 0) Assert.Equal(supplier.Id + 1, contacts.Key.SourceIdB);
                        for (int index = 0; index < contacts.Count; index++)
                        {
                            ManifoldContact row = contacts[index]; contactCount++;
                            Assert.Equal(Vector3d.Backward, row.Normal); Assert.False(row.DepthIsClamped);
                            Assert.True(row.AnchorA.TryGetWorldPoint(out Vector3d moving));
                            Assert.True(row.AnchorB.TryGetWorldPoint(out Vector3d source));
                            Assert.Equal(Fixed64.Zero, source.Z);
                            Assert.InRange(source.X.m_rawValue, supplier.Bounds.Min.X.m_rawValue, supplier.Bounds.Max.X.m_rawValue);
                            Assert.InRange(source.Y.m_rawValue, (-(Fixed64)4).m_rawValue, ((Fixed64)4).m_rawValue);
                            Assert.True((moving.Z + row.Depth).Abs() <= Fixed64.FromRaw(2));
                        }
                    }
                }
                trace.Add((body.Position3d, body.LinearVelocity, body.AngularVelocity, body.Rotation));
            }
            Assert.NotEqual(initialRotation, trace[0].Rotation);
            Assert.True(contactCount > 0);
            return trace;
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CapsuleBodylessOrOffcenterCompoundLeaf_ShouldKeepItsFiniteGeometryAndProvenance(bool compoundMover)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.Two), Rectangle(scene, 0, 4, -2, 0, Fixed64.Two) };
        LSCollider2D continuous = Rectangle(scene, -4, 4, -2, 0, Fixed64.Two);
        if (!compoundMover)
            foreach (LSCollider2D member in members)
            {
                Vector2d center = member.Center;
                member.Deactivate();
                var stationary = new SolidBody2D(new TestMatterAgent(scene.Context), member);
                stationary.Initialize(center, motionType: BodyMotionType.Static);
            }
        FixedQuaternion horizontal = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized;
        LSCollider mover; LSCapsuleCollider capsule;
        if (compoundMover)
        {
            var owner = new LSCompoundCollider(new CompoundColliderPart(
                ColliderShapeDefinition.Capsule(Fixed64.Half, (Fixed64)3, PhysicsMaterial.Frictionless),
                Vector3d.Right, horizontal, Vector3d.One), CompoundColliderPart.Sphere(Fixed64.Half, new(-1, 3, 3)));
            SolidBody body = scene.CreateBody(owner, new(-3 * Fixed64.Quarter, Fixed64.Zero, Fixed64.Quarter), FixedQuaternion.Identity).Body;
            capsule = (LSCapsuleCollider)owner.GetPartCollider(0); mover = owner;
            Assert.Same(owner, capsule.CompoundOwner); Assert.Same(body, owner.Body);
            Assert.NotEqual(capsule.Center, body.WorldCenterOfMass);
        }
        else
        {
            capsule = new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 3, 1), Material = PhysicsMaterial.Frictionless };
            capsule.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(
                new(Fixed64.Quarter, Fixed64.Zero, Fixed64.Quarter), horizontal, Vector3d.One)));
            mover = capsule;
        }
        Assert.Null(capsule.Body); Assert.Equal(Fixed64.Two, capsule.AxisLength);
        var ordinary = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, continuous, ordinary));
        ManifoldContact expected = ordinary.PrimaryContact;
        Assert.Equal(Vector3d.Backward, expected.Normal); Assert.Equal(Fixed64.Quarter, expected.Depth);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.Equal(expected.Normal, sample.Row.Normal); Assert.Equal(expected.Depth, sample.Row.Depth);
            Assert.Equal(compoundMover ? 1 : 0, sample.Row.FeatureNamespaceA);
            Assert.True(sample.Row.HasMaterialOverride); Assert.Equal(PhysicsMaterial.Frictionless, sample.Row.MaterialA);
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(new Vector3d(source.X, Fixed64.Zero, -Fixed64.Quarter), moving);
            Assert.Equal(Fixed64.Zero, source.Y); Assert.Equal(Fixed64.Zero, source.Z);
            Assert.InRange(source.X.m_rawValue, (-3 * Fixed64.Quarter).m_rawValue, (5 * Fixed64.Quarter).m_rawValue);
            LSCollider2D supplier = sample.Supplier == members[0].Id + 1 ? members[0] : members[1];
            Assert.Equal(supplier.Id + 1, sample.Supplier);
            Assert.InRange(source.X.m_rawValue, supplier.Bounds.Min.X.m_rawValue, supplier.Bounds.Max.X.m_rawValue);
        }
        if (compoundMover)
            AssertWholeShapeExit(mover, members, mover.Body!.Position3d + Vector3d.Forward * Fixed64.Quarter, Vector3d.Forward);
    }

    private static LSCapsuleCollider CapsuleMover(PhysicsScenarioBuilder scene, Vector3d position, bool horizontal, Fixed64 coreLength)
    {
        var collider = new LSCapsuleCollider { Radius = Fixed64.Half,
            Size = new Vector3d(Fixed64.One, coreLength + Fixed64.One, Fixed64.One), Material = PhysicsMaterial.Frictionless };
        FixedQuaternion rotation = horizontal
            ? new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized : FixedQuaternion.Identity;
        scene.CreateBody(collider, position, rotation);
        Assert.Equal(coreLength, collider.AxisLength);
        return collider;
    }
}
