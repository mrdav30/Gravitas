using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests;

public sealed class JoinedExtrusionSurfaceTests
{
    [Theory]
    [InlineData(false, 0)]
    [InlineData(false, 1)]
    [InlineData(false, 2)]
    [InlineData(true, 0)]
    [InlineData(true, 1)]
    [InlineData(true, 2)]
    public void SymmetricExit_ShouldRepeatItsExactNormalAcrossNativeAxisAndDeclarationOrder(bool cylinder, int axis)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var z = new[] { (Fixed64)(-4), (Fixed64)4 };
        var left = Patch(scene, new[] { (Fixed64)(-4), Fixed64.Zero }, z);
        var right = Patch(scene, new[] { Fixed64.Zero, (Fixed64)4 }, z);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { left, right });
        var rotation = axis == 0 ? FixedQuaternion.Identity
            : axis == 1 ? new FixedQuaternion(Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.One).Normalized
            : new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized;
        var shape = Extrusion(cylinder);
        scene.CreateBody(shape, Vector3d.Zero, rotation);
        Vector3d? normal = null;
        foreach (bool reverse in new[] { false, true })
        {
            surface.ReplaceMembers(reverse ? new[] { right, left } : new[] { left, right });
            var rows = Contacts(scene, shape, left, right);
            Assert.NotEmpty(rows);
            foreach (var row in rows)
            {
                Assert.Equal(Fixed64.Half, row.Depth);
                Assert.True(row.Normal == Vector3d.Up || row.Normal == Vector3d.Down);
                normal ??= row.Normal;
                Assert.Equal(normal.Value, row.Normal);
            }
        }
    }

    [Fact]
    public void JoinedFloorAndFoldedMember_ShouldRetainBothIndependentPlaneConstraints()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var left = Patch(scene, new[] { (Fixed64)(-4), Fixed64.Zero }, new[] { (Fixed64)(-4), (Fixed64)4 });
        var folded = new LSMeshCollider(new[] { new Vector3d(0, 0, -4), new(4, 0, -4), new(0, 0, 4), new(4, 0, 4),
            new(0, 4, -4), new(0, 4, 4) }, new[] { 0, 2, 1, 1, 2, 3, 0, 4, 2, 2, 4, 5 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scene.InitializeStaticCollider(folded, Vector3d.Zero);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { folded, left });
        var shape = Extrusion(false);
        scene.CreateBody(shape, new Vector3d(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity);
        var rows = Contacts(scene, shape, left, folded);
        Assert.Contains(rows, row => row.Normal == Vector3d.Up && row.Depth == Fixed64.Quarter);
        Assert.Contains(rows, row => row.Normal == Vector3d.Right && row.Depth == Fixed64.Quarter);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void AdmittedStripWithoutAnIntrinsicPole_ShouldConnectTheCompleteSupport(bool cylinder, bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Fixed64 near = Fixed64.FromFraction(1, 10), far = Fixed64.FromFraction(1, 5);
        var z = new[] { (Fixed64)(-4), (Fixed64)4 };
        var members = new[] { Patch(scene, new[] { (Fixed64)(-4), near }, z),
            Patch(scene, new[] { near, far }, z), Patch(scene, new[] { far, (Fixed64)4 }, z) };
        using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { members[2], members[1], members[0] } : members);
        var shape = Extrusion(cylinder);
        scene.CreateBody(shape, Vector3d.Up * Fixed64.Quarter, FixedQuaternion.Identity);
        var rows = Contacts(scene, shape, members);
        Assert.Equal(1, surface.SlabFaces.RegionCount);
        Assert.Equal(4, rows.Count);
        foreach (var row in rows)
        {
            Assert.Equal(Vector3d.Up, row.Normal);
            Assert.Equal(Fixed64.Quarter, row.Depth);
            Assert.True(row.AnchorA.TryGetWorldPoint(out var point));
            Assert.True(point.X < near || point.X > far);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void DuplicateFilledPieces_ShouldKeepTheirFiniteCapSupportsWithoutTrustedJoins(bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var extent = new[] { (Fixed64)(-2), (Fixed64)2 };
        var first = Patch(scene, extent, extent);
        var second = Patch(scene, extent, extent);
        first.Material = PhysicsMaterial.Bouncy;
        second.Material = PhysicsMaterial.Frictionless;
        var members = new[] { first, second };
        using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { second, first } : members);
        var cylinder = new LSCylinderCollider { Radius = Fixed64.Quarter, Size = Vector3d.One };
        var body = scene.CreateBody(cylinder, Vector3d.Up * Fixed64.Quarter, FixedQuaternion.Identity).Body;
        foreach (var member in members)
        {
            var pair = scene.CreatePair(member, cylinder);
            Assert.True(CollisionDetection.DoCollisionCheck(pair));
            Assert.NotEmpty(pair.Manifold);
            for (int index = 0; index < pair.Manifold.GroupCount; index++)
            {
                var group = pair.Manifold.GetGroup(index);
                bool fromFirst = group.Key.SourceIdA == first.Id + 1;
                Assert.True(fromFirst || group.Key.SourceIdA == second.Id + 1);
                for (int rowIndex = 0; rowIndex < group.Count; rowIndex++)
                {
                    var row = group[rowIndex];
                    Assert.Equal(Vector3d.Up, row.Normal);
                    Assert.Equal(Fixed64.Quarter, row.Depth);
                    Assert.Equal(fromFirst ? first.Material : second.Material, row.MaterialA);
                    Assert.True(row.AnchorA.TryGetWorldPoint(out Vector3d point));
                    Assert.Equal(Fixed64.Zero, point.Y);
                    Assert.True(point.X >= (Fixed64)(-2) && point.X <= (Fixed64)2);
                    Assert.True(point.Z >= (Fixed64)(-2) && point.Z <= (Fixed64)2);
                    Assert.True(row.AnchorB.TryGetWorldPoint(out Vector3d exit));
                    Assert.Equal(-Fixed64.Quarter, exit.Y);
                    Assert.Equal(point.X, exit.X);
                    Assert.Equal(point.Z, exit.Z);
                    Assert.True(exit.X * exit.X + exit.Z * exit.Z <= Fixed64.FromFraction(1, 16) + Fixed64.MinIncrement);
                }
            }
        }
        body.SetPosition(Vector3d.Up * (Fixed64.Half + Fixed64.MinIncrement));
        cylinder.RebuildRuntimeShapeOnly();
        Assert.True(cylinder.Center.Y - cylinder.Height / 2 > Fixed64.Zero);
        foreach (var member in members)
            Assert.False(CollisionDetection.DoCollisionCheck(scene.CreatePair(member, cylinder)));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ProjectedNeighborOutsideTheSection_ShouldNotSupplyAnUnclippedSupport(bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var z = new[] { -Fixed64.Half, Fixed64.Half };
        var left = Patch(scene, new[] { (Fixed64)(-2), Fixed64.Zero }, z);
        var right = Patch(scene, new[] { Fixed64.Zero, (Fixed64)2 }, z);
        left.Material = PhysicsMaterial.Bouncy;
        right.Material = PhysicsMaterial.Frictionless;
        using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { right, left } : new[] { left, right });
        var cylinder = new LSCylinderCollider { Radius = Fixed64.Quarter, Size = Vector3d.One };
        scene.CreateBody(cylinder, new(-Fixed64.Half, Fixed64.Quarter, Fixed64.FromFraction(2, 5)),
            FixedQuaternion.Identity);
        // Padding gathers the right piece, but the complete cylinder remains
        // left of its fill; the real Z rim prevents whole-component promotion.
        Assert.True(cylinder.BoundsMax.X < Fixed64.Zero);
        Assert.True(cylinder.BoundsMax.Z > Fixed64.Half);
        var pair = scene.CreatePair(left, cylinder);
        Assert.True(CollisionDetection.DoCollisionCheck(pair));
        Assert.NotEmpty(pair.Manifold);
        for (int group = 0; group < pair.Manifold.GroupCount; group++)
            Assert.Equal(left.Id + 1, pair.Manifold.GetGroup(group).Key.SourceIdA);
        foreach (var row in pair.Manifold)
        {
            Assert.Equal(left.Material, row.MaterialA);
            Assert.True(row.AnchorA.TryGetWorldPoint(out Vector3d point));
            Assert.Equal(Fixed64.Zero, point.Y);
            Assert.True(point.X >= (Fixed64)(-2) && point.X <= Fixed64.Zero);
            Assert.True(point.Z >= -Fixed64.Half && point.Z <= Fixed64.Half);
        }
        var outside = scene.CreatePair(right, cylinder);
        Assert.False(CollisionDetection.DoCollisionCheck(outside));
        Assert.Empty(outside.Manifold);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void TiltedFaceExit_ShouldClearTheWholeShapeAgainstEveryFiniteMember(bool cylinder)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var z = new[] { (Fixed64)(-4), (Fixed64)4 };
        var members = new[] { Patch(scene, new[] { (Fixed64)(-8), Fixed64.Zero }, z),
            Patch(scene, new[] { Fixed64.Zero, (Fixed64)8 }, z) };
        using var surface = scene.Context.Physics.CreateStaticSurface(members);
        var shape = Extrusion(cylinder);
        if (!cylinder) shape.Size = new Vector3d(4, 1, 2);
        Vector3d start = Vector3d.Up * Fixed64.Quarter;
        var body = scene.CreateBody(shape, start,
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)30)).Body;
        var rows = Contacts(scene, shape, members);
        Assert.NotEmpty(rows);
        Fixed64 depth = Fixed64.Zero;
        foreach (var row in rows)
        {
            Assert.Equal(Vector3d.Up, row.Normal);
            Assert.False(row.DepthIsClamped);
            depth = FixedMath.Max(depth, row.Depth);
        }
        // Check the complete translated shapes through the independent native
        // member path, allowing only final nearest-even depth rounding.
        surface.Dispose();
        body.SetPosition(start + Vector3d.Up * depth);
        shape.RebuildRuntimeShapeOnly();
        foreach (var member in members)
        {
            var pair = scene.CreatePair(member, shape);
            if (CollisionDetection.DoCollisionCheck(pair))
                foreach (var row in pair.Manifold) Assert.True(row.Depth.m_rawValue <= 1);
        }
        body.SetPosition(start + Vector3d.Up * (depth + Fixed64.FromRaw(2)));
        shape.RebuildRuntimeShapeOnly();
        foreach (var member in members) Assert.False(CollisionDetection.DoCollisionCheck(scene.CreatePair(member, shape)));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FootprintInsideJoinedHole_ShouldRemainSeparated(bool cylinder)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var z = new[] { (Fixed64)(-2), -Fixed64.Half, Fixed64.Half, (Fixed64)2 };
        var members = new[] { Patch(scene, new[] { (Fixed64)(-2), -Fixed64.Half, Fixed64.Zero }, z, hole: true),
            Patch(scene, new[] { Fixed64.Zero, Fixed64.Half, (Fixed64)2 }, z, hole: true) };
        using var surface = scene.Context.Physics.CreateStaticSurface(members);
        var shape = Extrusion(cylinder);
        if (cylinder) shape.Radius = Fixed64.Quarter;
        else shape.Size = Vector3d.One * Fixed64.Half;
        scene.CreateBody(shape, Vector3d.Up * Fixed64.Quarter, FixedQuaternion.Identity);
        foreach (var member in members)
        {
            var pair = scene.CreatePair(member, shape);
            Assert.False(CollisionDetection.DoCollisionCheck(pair));
            Assert.Empty(pair.Manifold);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FreelyRotatingExtrusion_ShouldPreserveFrictionlessCrossingVelocity(bool cylinder)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.SetFrameRate(32);
        scene.Context.Environment.Gravity = Fixed64.One;
        scene.Context.Environment.AirDensity = scene.Context.Environment.DampingFactor = scene.Context.Environment.MinSpeed = Fixed64.Zero;
        var z = new[] { (Fixed64)(-4), (Fixed64)4 };
        var left = Patch(scene, new[] { (Fixed64)(-8), Fixed64.Zero }, z);
        var right = Patch(scene, new[] { Fixed64.Zero, (Fixed64)8 }, z);
        left.Material = right.Material = PhysicsMaterial.Frictionless;
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { right, left });
        var body = scene.CreateBody(Extrusion(cylinder), new Vector3d(-Fixed64.One, Fixed64.Half, Fixed64.Zero), FixedQuaternion.Identity).Body;
        body.UseManualGrounding(); body.SleepEnabled = false;
        body.AddLinearImpulse(Vector3d.Right);
        body.AddAngularImpulse(Vector3d.Forward * Fixed64.FromFraction(1, 64));
        bool rotated = false;
        for (int frame = 0; frame < 128; frame++)
        {
            scene.Context.Simulate(); scene.Context.LateSimulate();
            rotated |= body.Rotation != FixedQuaternion.Identity;
            Assert.Equal(Fixed64.One, body.LinearVelocity.X);
            Assert.Equal(Fixed64.Zero, body.LinearVelocity.Z);
            Assert.True(body.Position3d.Y >= Fixed64.Zero);
        }
        Assert.True(rotated);
        Assert.True(body.Position3d.X > Fixed64.Two);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void SingleAdmittedDeclaredPiece_ShouldKeepItsWholeSurfaceAcrossInternalDiagonal(bool cylinder, bool singleDeclaration)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var z = new[] { (Fixed64)(-4), (Fixed64)4 };
        var left = Patch(scene, new[] { (Fixed64)(-8), Fixed64.Zero }, z);
        var right = Patch(scene, new[] { Fixed64.Zero, (Fixed64)8 }, z);
        using var surface = scene.Context.Physics.CreateStaticSurface(singleDeclaration ? new[] { right } : new[] { left, right });
        var shape = Extrusion(cylinder);
        // Exact pose captured as the freely rotating body reached X+Z=4,
        // the right piece's artificial triangle diagonal, far from its join.
        var rotation = new FixedQuaternion(Fixed64.FromRaw(-5146721), Fixed64.FromRaw(567776),
            Fixed64.FromRaw(4181090), Fixed64.FromRaw(4294962140));
        Vector3d center = new((Fixed64)3, Fixed64.FromRaw(2110383359), Fixed64.Zero);
        var body = scene.CreateBody(shape, center, rotation).Body;
        var pair = scene.CreatePair(right, shape);
        Assert.True(CollisionDetection.DoCollisionCheck(pair));
        Fixed64 maximum = Fixed64.Zero;
        foreach (var row in pair.Manifold)
        {
            Assert.Equal(Vector3d.Up, row.Normal);
            Assert.True(row.AnchorA.TryGetWorldPoint(out Vector3d source));
            Assert.InRange(source.Y.m_rawValue, -1L, 1L);
            Assert.InRange(source.X.m_rawValue, 0L, ((Fixed64)8).m_rawValue);
            Assert.InRange(source.Z.m_rawValue, ((Fixed64)(-4)).m_rawValue, ((Fixed64)4).m_rawValue);
            maximum = FixedMath.Max(maximum, row.Depth);
        }
        Assert.True(maximum > Fixed64.Zero);
        // The retained whole-owner exit clears both authored triangles, not
        // merely the triangle whose diagonal selected the old false normal.
        body.SetPosition(center + Vector3d.Up * (maximum + Fixed64.FromRaw(32)));
        shape.RebuildRuntimeShapeOnly();
        Assert.False(CollisionDetection.DoCollisionCheck(scene.CreatePair(right, shape)));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void SupplierMutation_ShouldImmediatelyClearItsRegisteredCarriersWarmStart(int mutation)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Fixed64 join = -Fixed64.FromFraction(36, 25);
        var z = new[] { (Fixed64)(-4), (Fixed64)4 };
        var supplier = Patch(scene, new[] { -Fixed64.FromFraction(3, 2), join }, z);
        var carrier = Patch(scene, new[] { join, (Fixed64)2 }, z);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { supplier, carrier });
        LSCollider shape = mutation == 1
            ? new LSCompoundCollider(CompoundColliderPart.Cuboid(new Vector3d(4, 1, 2), Vector3d.Zero))
            : new LSCuboidCollider { Size = new Vector3d(4, 1, 2) };
        scene.CreateBody(shape, Vector3d.Up * Fixed64.Quarter,
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)30));
        var pair = scene.Context.Physics.GetCollisionPair(carrier.Id, shape.Id)!;
        Assert.True(CollisionDetection.DoCollisionCheck(pair));
        ref var group = ref pair.Manifold.GetGroup(0);
        var key = group.Key;
        Assert.True(key.SourceIdA == supplier.Id + 1 || key.SourceIdB == supplier.Id + 1);
        var row = group[0];
        pair.StoreWarmStartImpulse(key, row.ContactId, row.Normal, Fixed64.One, Fixed64.Half, Fixed64.Quarter);
        Assert.True(pair.TryGetWarmStartImpulse(key, row.ContactId, out _));

        switch (mutation)
        {
            case 0: supplier.Deactivate(); break;
            case 1:
                var replacement = Patch(scene, new[] { -Fixed64.FromFraction(3, 2), join }, z);
                surface.ReplaceMembers(new[] { replacement, carrier });
                break;
            case 2: supplier.Material = PhysicsMaterial.Bouncy; break;
            default: supplier.IgnoredCollisionLayers = Gravitas.Support.PhysicsLayerMask.FromLayer(shape.Layer.Index); break;
        }

        Assert.True(pair.Active);
        // Invalidation must retire a remote supplier's load before another
        // narrow-phase pass can reuse the carrier's otherwise unchanged rows.
        Assert.False(pair.TryGetWarmStartImpulse(key, row.ContactId, out _));
    }

    [Fact]
    public void JoinedFaceAndExposedBoundary_ShouldMatchContinuousFloorPositionCorrection()
    {
        Assert.Equal(Step(false), Step(true));

        static Vector3d Step(bool joined)
        {
            using var scene = PhysicsScenarioBuilder.Create();
            scene.Context.Environment.Gravity = Fixed64.Zero;
            scene.Context.Environment.AirDensity = Fixed64.Zero;
            scene.Context.Environment.DampingFactor = Fixed64.Zero;
            scene.Context.Environment.MinSpeed = Fixed64.Zero;
            var z = new[] { (Fixed64)(-4), (Fixed64)4 };
            var left = Patch(scene, new[] { (Fixed64)(-8), joined ? Fixed64.Zero : Fixed64.Quarter }, z);
            left.Material = PhysicsMaterial.Frictionless;
            using var surface = joined ? scene.Context.Physics.CreateStaticSurface(new[] { left,
                Patch(scene, new[] { Fixed64.Zero, Fixed64.Quarter }, z) }) : null;
            var box = new LSCuboidCollider { Material = PhysicsMaterial.Frictionless };
            var body = scene.CreateBody(box, new Vector3d(Fixed64.FromFraction(1, 10), Fixed64.Quarter, Fixed64.Zero),
                FixedQuaternion.Identity, preventAngularForces: true).Body;
            body.UseManualGrounding(); body.SleepEnabled = false;
            scene.Context.Simulate(); scene.Context.LateSimulate();
            Assert.Equal(Vector3d.Zero, body.LinearVelocity);
            return body.Position3d;
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void TiltedBoxSupportOutsideInitialSection_ShouldStillJoinItsCompleteSupportLine(bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var first = Strips(scene, -4, 0, 32);
        var second = Strips(scene, 0, 4, 32);
        using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { second, first } : new[] { first, second });
        var box = new LSCuboidCollider { Size = new Vector3d(4, 1, 2) };
        scene.CreateBody(box, Vector3d.Up * Fixed64.Quarter,
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)30));
        var rows = Contacts(scene, box, first, second);
        Assert.Equal(1, surface.SlabFaces.RegionCount);
        Assert.Equal(2, rows.Count);
        foreach (var row in rows)
        {
            Assert.Equal(Vector3d.Up, row.Normal);
            Assert.True(row.AnchorA.TryGetWorldPoint(out var source));
            // The lower support edge projects beyond the original plane section.
            Assert.True(source.X < -Fixed64.One);
            Assert.Equal(Fixed64.Zero, source.Y);
            Assert.Equal(Fixed64.One, FixedMath.Abs(source.Z));
            Assert.True(row.Depth > Fixed64.One);
        }
    }

    [Fact]
    public void BoxEnclosingBothFiniteMembers_ShouldRetainPhysicalRecoveryContacts()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var first = Patch(scene, new[] { -Fixed64.Quarter, Fixed64.Zero }, new[] { -Fixed64.Quarter, Fixed64.Quarter });
        var second = Patch(scene, new[] { Fixed64.Zero, Fixed64.Quarter }, new[] { -Fixed64.Quarter, Fixed64.Quarter });
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { first, second });
        var box = new LSCuboidCollider { Size = new Vector3d(4, 2, 4) };
        scene.CreateBody(box, Vector3d.Up * Fixed64.Quarter, FixedQuaternion.Identity);
        var rows = Contacts(scene, box, first, second);
        Assert.NotEmpty(rows);
        foreach (var row in rows)
        {
            Assert.True(row.Depth > Fixed64.Zero);
            Assert.True(row.AnchorA.TryGetWorldPoint(out var source));
            Assert.Equal(Fixed64.Zero, source.Y);
            Assert.True(FixedMath.Abs(source.X) <= Fixed64.Quarter && FixedMath.Abs(source.Z) <= Fixed64.Quarter);
        }
    }

    [Fact]
    public void ProjectedSupportOwnedByInitiallySeparatedNeighbor_ShouldStillSupplyJoinedResponse()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Fixed64 join = -Fixed64.FromFraction(36, 25);
        var z = new[] { (Fixed64)(-4), (Fixed64)4 };
        var first = Patch(scene, new[] { -Fixed64.FromFraction(3, 2), join }, z);
        var second = Patch(scene, new[] { join, (Fixed64)2 }, z);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { first, second });
        var box = new LSCuboidCollider { Size = new Vector3d(4, 1, 2) };
        scene.CreateBody(box, Vector3d.Up * Fixed64.Quarter,
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)30));
        var sourcePair = scene.CreatePair(first, box);
        Assert.False(CollisionDetection.DoCollisionCheck(sourcePair));
        Assert.Empty(sourcePair.Manifold);
        var carrier = scene.CreatePair(second, box);
        Assert.True(CollisionDetection.DoCollisionCheck(carrier));
        // Initial overlap occurs only on the right. Its joined left neighbor
        // owns the support pole needed to keep the complete floor blocking.
        Assert.Equal(2, carrier.Manifold.Count);
        foreach (var row in carrier.Manifold)
        {
            Assert.Equal(Vector3d.Up, row.Normal);
            Assert.True(row.Depth > Fixed64.One);
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ProjectedSupplier_ShouldBlockThroughTheOverlappingMemberWithoutFalseEvents(bool obliqueFloor, bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Environment.Gravity = Fixed64.Zero;
        scene.Context.Environment.AirDensity = Fixed64.Zero;
        scene.Context.Environment.DampingFactor = Fixed64.Zero;
        Vector3d floorNormal = new Vector3d(-1, 1, 0).Normalized;
        var z = new[] { (Fixed64)(-4), (Fixed64)4 };
        var supplier = obliqueFloor
            ? Patch(scene, new[] { Fixed64.FromFraction(13, 20), (Fixed64)4 }, z, sloped: true)
            : Patch(scene, new[] { -Fixed64.FromFraction(3, 2), -Fixed64.FromFraction(36, 25) }, z);
        var carrier = obliqueFloor
            ? Patch(scene, new[] { (Fixed64)(-4), Fixed64.FromFraction(13, 20) }, z, sloped: true)
            : Patch(scene, new[] { -Fixed64.FromFraction(36, 25), (Fixed64)2 }, z);
        supplier.Material = PhysicsMaterial.Frictionless;
        using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { carrier, supplier } : new[] { supplier, carrier });
        int supplierEnters = 0, carrierEnters = 0;
        supplier.OnContactEnter += _ => supplierEnters++;
        carrier.OnContactEnter += _ => carrierEnters++;
        var box = new LSCuboidCollider { Size = new Vector3d(4, 1, 2), Material = PhysicsMaterial.Frictionless };
        Vector3d start = obliqueFloor ? floorNormal * Fixed64.FromFraction(1, 10) : Vector3d.Up * Fixed64.Quarter;
        var moving = scene.CreateBody(box, start, obliqueFloor ? FixedQuaternion.Identity
            : FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)30)).Body;
        moving.UseManualGrounding();
        if (obliqueFloor) Assert.True(supplier.BoundsMin.Y > box.BoundsMax.Y, $"Supplier min {supplier.BoundsMin}, mover max {box.BoundsMax}");
        scene.Context.Simulate(); scene.Context.LateSimulate();
        Assert.Equal(0, supplierEnters);
        Assert.Equal(1, carrierEnters);
        Vector3d normal = obliqueFloor ? floorNormal : Vector3d.Up;
        var pair = scene.Context.Physics.GetCollisionPair(carrier.Id, box.Id);
        Assert.NotNull(pair);
        Assert.Equal(2, pair.Manifold.Count);
        Assert.True(Vector3d.Dot(moving.Position3d - start, normal) > Fixed64.Half, $"Start {start}, final {moving.Position3d}, normal {normal}");
        foreach (var row in pair.Manifold)
        {
            Assert.Equal(PhysicsMaterial.Frictionless, row.MaterialA);
            Assert.True(row.AnchorA.TryGetWorldPoint(out var source));
            Assert.True(obliqueFloor ? source.Y > Fixed64.Half : source.X < -Fixed64.FromFraction(36, 25));
        }
    }

    [Fact]
    public void BoxEnclosingJoinedRing_ShouldRetainContactsWithoutFillingTheHole()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var z = new[] { (Fixed64)(-2), -Fixed64.Half, Fixed64.Half, (Fixed64)2 };
        var first = Patch(scene, new[] { (Fixed64)(-2), -Fixed64.Half, Fixed64.Zero }, z, hole: true);
        var second = Patch(scene, new[] { Fixed64.Zero, Fixed64.Half, (Fixed64)2 }, z, hole: true);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { second, first });
        var box = new LSCuboidCollider { Size = new Vector3d(6, 2, 6) };
        scene.CreateBody(box, Vector3d.Up * Fixed64.Quarter, FixedQuaternion.Identity);
        var rows = Contacts(scene, box, first, second);
        Assert.NotEmpty(rows);
        foreach (var row in rows)
        {
            Assert.True(row.Depth > Fixed64.Zero);
            Assert.True(row.AnchorA.TryGetWorldPoint(out var source));
            Assert.Equal(Fixed64.Zero, source.Y);
            Assert.True(FixedMath.Abs(source.X) >= Fixed64.Half || FixedMath.Abs(source.Z) >= Fixed64.Half);
        }
    }

    private static LSCollider Extrusion(bool cylinder) => cylinder
        ? new LSCylinderCollider { Material = PhysicsMaterial.Frictionless }
        : new LSCuboidCollider { Material = PhysicsMaterial.Frictionless };

    private static List<ManifoldContact> Contacts(PhysicsScenarioBuilder scene, LSCollider shape, params LSMeshCollider[] members)
    {
        var rows = new List<ManifoldContact>();
        foreach (var member in members)
        {
            var pair = scene.CreatePair(member, shape);
            Assert.True(CollisionDetection.DoCollisionCheck(pair));
            foreach (var row in pair.Manifold) rows.Add(row);
        }
        return rows;
    }

    private static LSMeshCollider Strips(PhysicsScenarioBuilder scene, int near, int far, int count)
    {
        var x = new Fixed64[count + 1];
        for (int index = 0; index <= count; index++) x[index] = -4 + (Fixed64)8 * index / count;
        return Patch(scene, x, new[] { (Fixed64)near, (Fixed64)far });
    }

    private static LSMeshCollider Patch(PhysicsScenarioBuilder scene, Fixed64[] x, Fixed64[] z, bool hole = false,
        bool sloped = false)
    {
        var vertices = new Vector3d[x.Length * z.Length];
        var triangles = new List<int>();
        for (int row = 0; row < z.Length; row++)
            for (int column = 0; column < x.Length; column++)
            {
                int vertex = row * x.Length + column;
                vertices[vertex] = new(x[column], sloped ? x[column] : Fixed64.Zero, z[row]);
                if (row == z.Length - 1 || column == x.Length - 1) continue;
                if (hole && x[column] >= -Fixed64.Half && x[column + 1] <= Fixed64.Half
                    && z[row] >= -Fixed64.Half && z[row + 1] <= Fixed64.Half) continue;
                triangles.AddRange(new[] { vertex, vertex + x.Length, vertex + 1,
                    vertex + 1, vertex + x.Length, vertex + x.Length + 1 });
            }
        var mesh = new LSMeshCollider(vertices, triangles.ToArray(), MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scene.InitializeStaticCollider(mesh, Vector3d.Zero);
        return mesh;
    }
}
