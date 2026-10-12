using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class CoplanarPatchInitialSweepTests
{
    private static readonly PhysicsLayerMask LayerZero = PhysicsLayerMask.FromLayer(0);
    private static readonly Vector3d CrossingCenter = new(
        Fixed64.FromFraction(3, 10), -Fixed64.Quarter, Fixed64.FromFraction(3, 10));

    [Theory]
    [InlineData(0)] // Original diagonal.
    [InlineData(1)] // Reversed authored winding.
    [InlineData(2)] // Reversed triangle order.
    [InlineData(3)] // Alternate diagonal.
    [InlineData(4)] // Fan subdivision.
    [InlineData(5)] // Mixed winding.
    public void CrossingCapsule_ShouldUseFilledPatchNormalAcrossEquivalentTriangulations(int variant)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider target = Quad(variant);
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        var source = Capsule();
        scenario.CreateBody(source, CrossingCenter, FixedQuaternion.Identity);
        var hits = new SwiftList<Physics3DHit>();

        foreach (Vector3d travel in new[] { Vector3d.Down, Vector3d.Up, Vector3d.Right })
        {
            scenario.Context.Query3D.SweepCapsule(source, travel, LayerZero, out Physics3DHit hit)
                .Should().BeTrue();
            scenario.Context.Query3D.SweepCapsuleAll(source, travel, LayerZero, hits).Should().Be(1);
            hits[0].Should().Be(hit);
            AssertInitialFace(hit, target);
            // The vertical core crosses the filled interior at this X/Z;
            // final exact barycentric materialization permits one raw unit.
            (hit.Point.X - CrossingCenter.X).Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
            (hit.Point.Z - CrossingCenter.Z).Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        }
    }

    [Theory]
    [InlineData(ColliderType.Sphere)]
    [InlineData(ColliderType.Capsule)] // Zero-length core equals a sphere.
    [InlineData(ColliderType.OBBox)]
    [InlineData(ColliderType.Cylinder)]
    [InlineData(ColliderType.Cone)]
    [InlineData(ColliderType.Mesh)]
    public void OtherConvexSources_ShouldRetainTheSameInitialFaceSide(ColliderType shape)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider target = Quad(5);
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        LSCollider source = shape switch
        {
            ColliderType.Sphere => new LSSphereCollider(),
            ColliderType.Capsule => new LSCapsuleCollider { Radius = Fixed64.Half, Size = Vector3d.One },
            ColliderType.OBBox => new LSCuboidCollider(),
            ColliderType.Cylinder => new LSCylinderCollider(),
            ColliderType.Mesh => MeshTestFixtures.CreateConvexCube(),
            _ => new LSConeCollider()
        };
        scenario.CreateBody(source, CrossingCenter, FixedQuaternion.Identity);
        var worker = new ConvexSweepQueryWorker();
        foreach (Vector3d travel in new[] { Vector3d.Down, Vector3d.Up, Vector3d.Right })
        {
            if (source is LSMeshCollider mesh)
                worker.PrepareConvexMeshSource(mesh, travel);
            else
                worker.PreparePrimitiveSource(source, travel);
            worker.TrySweepPreparedSource(target, out Physics3DHit hit).Should().BeTrue();
            AssertInitialFace(hit, target);
        }
    }

    [Fact]
    public void EmbeddedCircleSource_ShouldUseTheSameFilledPatchCertificate()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider target = Quad(5);
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        var worker = new ConvexSweepQueryWorker();
        foreach (Vector3d travel in new[] { Vector3d.Down, Vector3d.Up, Vector3d.Right })
        {
            worker.PrepareCircleSlabSource(CrossingCenter, Fixed64.Half, Fixed64.Half, travel);
            worker.TrySweepPreparedSource(target, out Physics3DHit hit).Should().BeTrue();
            AssertInitialFace(hit, target);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void TrustedNonconvexDomain_ShouldNotFillItsHoleOrNotch(bool hole)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider target = hole ? Ring() : LShape();
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        var source = new LSCapsuleCollider
        {
            Radius = Fixed64.Quarter,
            Size = new Vector3d(Fixed64.Half, (Fixed64)2, Fixed64.Half)
        };
        Vector3d center = hole ? Vector3d.Down * Fixed64.Quarter
            : new Vector3d(Fixed64.FromFraction(7, 4), -Fixed64.Quarter, Fixed64.FromFraction(7, 4));
        scenario.CreateBody(source, center, FixedQuaternion.Identity);
        // The core crosses the plane, but the radius stays strictly inside the
        // empty region. Small vertical travel cannot touch a finite patch edge.
        scenario.Context.Query3D.SweepCapsule(source, Vector3d.Up * Fixed64.Quarter, LayerZero, out _)
            .Should().BeFalse();
    }

    [Fact]
    public void TrustedRingInterior_ShouldSuppressCoveredSeamsWithoutFillingTheHole()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider target = Ring();
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        var source = Capsule();
        Vector3d center = new(-Fixed64.FromFraction(3, 2), -Fixed64.Quarter, -Fixed64.FromFraction(13, 10));
        scenario.CreateBody(source, center, FixedQuaternion.Identity);
        // Downward exit is 3/4; the nearer real ring boundary requires at
        // least one unit for the whole radius to clear it.
        foreach (Vector3d travel in new[] { Vector3d.Down, Vector3d.Up, Vector3d.Right })
        {
            scenario.Context.Query3D.SweepCapsule(source, travel, LayerZero, out Physics3DHit hit)
                .Should().BeTrue();
            AssertInitialFace(hit, target);
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void RealOuterAndHoleEdges_ShouldKeepTheirRadialNormal(bool hole, bool overlap)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider target = hole ? Ring() : Quad(5);
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        var source = Capsule();
        Fixed64 centerX = hole
            ? overlap ? Fixed64.FromFraction(3, 4) : Fixed64.Half
            : overlap ? Fixed64.FromFraction(9, 4) : Fixed64.FromFraction(5, 2);
        scenario.CreateBody(source, new Vector3d(centerX, -Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity);
        Vector3d expected = hole ? Vector3d.Left : Vector3d.Right;
        foreach (Vector3d travel in new[] { expected, -expected, Vector3d.Up })
        {
            scenario.Context.Query3D.SweepCapsule(source, travel, LayerZero, out Physics3DHit hit)
                .Should().BeTrue();
            hit.Collider.Should().BeSameAs(target);
            hit.Distance.Should().Be(Fixed64.Zero);
            hit.Normal.Should().Be(expected);
            (hit.Point.X - (hole ? Fixed64.One : (Fixed64)2)).Abs()
                .Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        }
    }

    [Fact]
    public void RotatedZeroCoreCapsule_ShouldMatchPublicSphereSweepGeometry()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider target = Quad(5);
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        var source = new LSCapsuleCollider { Radius = Fixed64.Half, Size = Vector3d.One };
        FixedQuaternion rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized;
        scenario.CreateBody(source, CrossingCenter, rotation);
        foreach (Vector3d travel in new[] { Vector3d.Down, Vector3d.Up, Vector3d.Right })
        {
            scenario.Context.Query3D.SweepCapsule(source, travel, LayerZero, out Physics3DHit capsuleHit)
                .Should().BeTrue();
            scenario.Context.Query3D.SweepSphere(CrossingCenter, Fixed64.Half, travel, Fixed64.One,
                out Physics3DHit sphereHit, LayerZero, excludedCollider: source).Should().BeTrue();
            AssertInitialFace(capsuleHit, target);
            sphereHit.Collider.Should().BeSameAs(target);
            sphereHit.Distance.Should().Be(Fixed64.Zero);
            sphereHit.Point.Y.Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
            (sphereHit.Normal.Y + Fixed64.One).Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
            // The sphere normal is reconstructed from its barycentric point:
            // at most two raw units of coordinate error divided by the 1/4 gap.
            sphereHit.Normal.X.Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon * (Fixed64)8);
            sphereHit.Normal.Z.Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon * (Fixed64)8);
            capsuleHit.Anchor.Origin.Should().Be(sphereHit.Anchor.Origin);
            capsuleHit.Anchor.Rotation.Should().Be(sphereHit.Anchor.Rotation);
            // Compare the authoritative anchors after materialization: each
            // independent barycentric path may round by one raw unit.
            (capsuleHit.Point.X - sphereHit.Point.X).Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon * (Fixed64)2);
            (capsuleHit.Point.Z - sphereHit.Point.Z).Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon * (Fixed64)2);
        }
    }

    [Fact]
    public void DisconnectedCoplanarFace_ShouldNotFillAnotherOwnersHole()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        PhysicsMesh ring = Ring().Mesh;
        var vertices = new Vector3d[11];
        ring.LocalVertices.CopyTo(vertices);
        vertices[8] = new Vector3d(Fixed64.Half, Fixed64.Zero, -Fixed64.Quarter);
        vertices[9] = new Vector3d(Fixed64.One, Fixed64.Zero, -Fixed64.Quarter);
        vertices[10] = new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Quarter);
        var indices = new int[ring.Triangles.Length + 3];
        ring.Triangles.CopyTo(indices);
        indices[indices.Length - 3] = 8; indices[indices.Length - 2] = 10; indices[indices.Length - 1] = 9;
        var target = new LSMeshCollider(vertices, indices, MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        var source = Capsule();
        scenario.CreateBody(source, new Vector3d(Fixed64.FromFraction(3, 4), -Fixed64.Quarter, Fixed64.Zero),
            FixedQuaternion.Identity);
        // The authored ring boundary is the first zero-distance constraint;
        // a separate filled face inside its hole cannot change that owner's exit.
        scenario.Context.Query3D.SweepCapsule(source, Vector3d.Up, LayerZero, out Physics3DHit hit)
            .Should().BeTrue();
        hit.Distance.Should().Be(Fixed64.Zero);
        hit.Normal.Should().Be(Vector3d.Left);
    }

    [Fact]
    public void InitialCapsuleExit_ShouldClearTheWholeShapeAtTheReportedFaceSide()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider target = Quad(0);
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        var source = Capsule();
        scenario.InitializeStaticCollider(source, CrossingCenter);
        scenario.Context.Query3D.SweepCapsule(source, Vector3d.Right, LayerZero, out Physics3DHit hit)
            .Should().BeTrue();
        AssertInitialFace(hit, target);

        // The vertical capsule's total half-height is one. Moving its center
        // to -1 leaves the top hemisphere touching, then one contact-tolerance
        // gap clears the complete capsule rather than only its core witness.
        source.Position = new Vector3d(CrossingCenter.X, -Fixed64.One, CrossingCenter.Z);
        CollisionDetection.DoCollisionCheck(scenario.CreatePair(target, source)).Should().BeTrue();
        source.Position -= Vector3d.Up * Fixed64.FromFraction(1, 4096);
        CollisionDetection.DoCollisionCheck(scenario.CreatePair(target, source)).Should().BeFalse();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CompoundSourceAndTarget_ShouldUseLeafGeometryAndReturnTargetOwner(bool compoundTarget)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        // Convex-only authored compound leaves require coherent winding. The
        // equivalent concave standalone target retains the mixed-winding case.
        LSCollider target = compoundTarget
            ? new LSCompoundCollider(new CompoundColliderPart(
                MeshTestFixtures.CreateDefinition(MeshTestFixtures.CreateConvexQuadFloor()), Vector3d.Zero))
            : Quad(5);
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        var source = new LSCompoundCollider(CompoundColliderPart.Capsule(Fixed64.Half, (Fixed64)2,
            Vector3d.Down * (Fixed64)2));
        scenario.CreateBody(source, CrossingCenter + Vector3d.Up * (Fixed64)2, FixedQuaternion.Identity);
        foreach (Vector3d travel in new[] { Vector3d.Down, Vector3d.Up, Vector3d.Right })
        {
            scenario.Context.Query3D.SweepCompound(source, travel, LayerZero, out Physics3DHit hit)
                .Should().BeTrue();
            AssertInitialFace(hit, target);
        }
    }

    [Fact]
    public void ClosingAdmission_ShouldUseFilledPatchNormalBeforeTriangleReduction()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider target = Quad(5);
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        var source = Capsule();
        scenario.CreateBody(source, CrossingCenter, FixedQuaternion.Identity);
        var worker = new ConvexSweepQueryWorker();
        foreach (Vector3d travel in new[] { Vector3d.Down, Vector3d.Right, Vector3d.Left })
        {
            worker.PreparePrimitiveSource(source, travel);
            worker.TrySweepPreparedSource(target, out _, requireClosingInitialContact: true).Should().BeFalse();
        }
        worker.PreparePrimitiveSource(source, Vector3d.Up);
        worker.TrySweepPreparedSource(target, out Physics3DHit hit, requireClosingInitialContact: true)
            .Should().BeTrue();
        AssertInitialFace(hit, target);
    }

    [Theory]
    [InlineData(false, -1)]
    [InlineData(false, 0)]
    [InlineData(false, 1)]
    [InlineData(false, 2)]
    [InlineData(true, -1)]
    [InlineData(true, 0)]
    [InlineData(true, 1)]
    [InlineData(true, 2)]
    public void FullLifecycleCcd_ShouldAdmitExitAndTangentButStopClosing(bool kinematic, int motion)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.SetFrameRate(1);
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        scenario.Context.Environment.MaxSpeed = (Fixed64)16;
        scenario.Context.Environment.MaxFallSpeed = (Fixed64)16;
        LSMeshCollider target = Quad(5);
        target.Material = PhysicsMaterial.Frictionless;
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        SolidBody source = scenario.CreateBody(Capsule(), CrossingCenter, FixedQuaternion.Identity,
            preventAngularForces: true, isKinematic: kinematic).Body;
        source.Collider.Material = PhysicsMaterial.Frictionless;
        source.UseManualGrounding();
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Vector3d displacement = motion == 0 ? Vector3d.Right : motion == 2 ? Vector3d.Left
            : Vector3d.Up * (Fixed64)motion;
        if (kinematic)
            source.Agent.Transform.LocalPosition = CrossingCenter + displacement;
        else
            source.AddLinearImpulse(displacement * source.Mass);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        if (motion == 1)
        {
            source.Position3d.Y.Should().BeLessThanOrEqualTo(CrossingCenter.Y + Fixed64.Epsilon);
            source.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
            if (!kinematic)
                source.LinearVelocity.Y.Should().Be(Fixed64.Zero);
        }
        else
        {
            // Existing penetration may be corrected along Y by the discrete
            // solver; CCD must preserve the requested tangential coordinates.
            source.Position3d.X.Should().Be(CrossingCenter.X + displacement.X);
            source.Position3d.Z.Should().Be(CrossingCenter.Z);
            source.Position3d.Y.Should().BeLessThanOrEqualTo(CrossingCenter.Y + displacement.Y + Fixed64.Epsilon);
            source.LastContinuousCollisionToiIterationCount.Should().Be(0);
            if (!kinematic)
                source.LinearVelocity.Should().Be(displacement);
        }
    }

    private static LSCapsuleCollider Capsule() =>
        new() { Radius = Fixed64.Half, Size = new Vector3d(1, 2, 1) };

    private static LSMeshCollider Quad(int variant)
    {
        Vector3d[] vertices = variant == 4
            ? new[] { new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2), new Vector3d(-2, 0, 2), new Vector3d(2, 0, 2), Vector3d.Zero }
            : new[] { new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2), new Vector3d(-2, 0, 2), new Vector3d(2, 0, 2) };
        int[] triangles = variant switch
        {
            1 => new[] { 1, 2, 0, 3, 2, 1 },
            2 => new[] { 1, 2, 3, 0, 2, 1 },
            3 => new[] { 0, 3, 1, 0, 2, 3 },
            4 => new[] { 4, 1, 0, 4, 3, 1, 4, 2, 3, 4, 0, 2 },
            5 => new[] { 0, 2, 1, 3, 2, 1 },
            _ => new[] { 0, 2, 1, 1, 2, 3 }
        };
        return new LSMeshCollider(vertices, triangles, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
    }

    private static LSMeshCollider Ring() => new(
        new[] { new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2), new Vector3d(2, 0, 2), new Vector3d(-2, 0, 2),
            new Vector3d(-1, 0, -1), new Vector3d(1, 0, -1), new Vector3d(1, 0, 1), new Vector3d(-1, 0, 1) },
        new[] { 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7, 6, 3, 0, 4, 3, 4, 7 },
        MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);

    private static LSMeshCollider LShape() => new(
        new[] { new Vector3d(0, 0, 0), new Vector3d(2, 0, 0), new Vector3d(2, 0, 1),
            new Vector3d(1, 0, 1), new Vector3d(1, 0, 2), new Vector3d(0, 0, 2) },
        new[] { 0, 1, 3, 1, 2, 3, 0, 3, 5, 3, 4, 5 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);

    private static void AssertInitialFace(Physics3DHit hit, LSCollider target)
    {
        hit.Collider.Should().BeSameAs(target);
        hit.Distance.Should().Be(Fixed64.Zero);
        hit.Normal.Should().Be(Vector3d.Down);
        hit.Point.Y.Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
    }
}
