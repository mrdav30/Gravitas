using FixedMathSharp;
using FixedMathSharp.Geometry;
using FixedMathSharp.Assertions;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests;

public sealed class MeshConePatchContactTests
{
    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void InteriorContact_ShouldIgnoreDiagonalFanAndWeldedSeams(int triangulation)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var vertices = new[] { new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2),
            new Vector3d(2, 0, 2), new Vector3d(-2, 0, 2), Vector3d.Zero };
        int[] triangles = triangulation switch
        {
            0 => new[] { 0, 2, 1, 0, 3, 2 },
            1 => new[] { 0, 3, 1, 1, 3, 2 },
            2 => new[] { 0, 4, 1, 1, 4, 2, 2, 4, 3, 3, 4, 0 },
            _ => new[] { 0, 2, 1, 4, 3, 5 }
        };
        if (triangulation < 2)
            Array.Resize(ref vertices, 4);
        if (triangulation == 3)
            vertices = new[] { vertices[0], vertices[1], vertices[2], vertices[3],
                vertices[0], vertices[2] };
        CollisionPair pair = CreatePair(scenario, vertices, triangles, Vector3d.Down * Fixed64.Quarter);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.Count.Should().Be(1);
        pair.Manifold.PrimaryContact.Normal.Should().Be(Vector3d.Down);
        pair.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.Quarter);
        pair.Manifold.PrimaryContact.PointA.Should().Be(Vector3d.Zero);
        pair.Manifold.PrimaryContact.PointB.Should().Be(Vector3d.Up * Fixed64.Quarter);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(0)]
    [InlineData(1)]
    public void PatchApexTangency_ShouldRetainExactAdmissionBeforeDepthRounding(int rawOffset)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var mesh = MeshTestFixtures.CreateConvexQuadFloor(MeshColliderMode.Concave);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        var cone = new LSConeCollider();
        scenario.InitializeStaticCollider(cone,
            Vector3d.Up * (-Fixed64.Half + Fixed64.FromRaw(rawOffset)));
        CollisionPair pair = scenario.CreatePair(mesh, cone);

        CollisionDetection.DoCollisionCheck(pair).Should().Be(rawOffset >= 0);
        if (rawOffset >= 0)
        {
            pair.Manifold.PrimaryContact.Normal.Should().Be(Vector3d.Down);
            pair.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.FromRaw(rawOffset));
        }
        else
            pair.Manifold.Count.Should().Be(0);
    }

    [Theory]
    [InlineData(1, 5)]
    [InlineData(1, 4)]
    public void SmallPatch_ShouldNotSubstituteCoveredDiagonalForWholeSurfaceExit(int numerator, int denominator)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 extent = Fixed64.FromFraction(numerator, denominator);
        var vertices = new[] { new Vector3d(-extent, Fixed64.Zero, -extent),
            new Vector3d(extent, Fixed64.Zero, -extent), new Vector3d(-extent, Fixed64.Zero, extent),
            new Vector3d(extent, Fixed64.Zero, extent) };
        CollisionPair pair = CreatePair(scenario, vertices, new[] { 0, 2, 1, 1, 2, 3 },
            Vector3d.Down * Fixed64.Quarter);
        // The cone's plane slice shrinks under downward translation. A
        // lateral square-edge exit remains longer than the 1/4 apex exit;
        // the covered diagonal cannot provide its triangle-only shortcut.
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.PrimaryContact.Normal.Should().Be(Vector3d.Down);
        pair.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.Quarter);
    }

    [Fact]
    public void NonintersectingPatchSeed_ShouldNotPreventLaterRealCornerContact()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 extent = Fixed64.FromFraction(1, 5);
        var vertices = new[] { new Vector3d(-extent, Fixed64.Zero, -extent),
            new Vector3d(extent, Fixed64.Zero, -extent), new Vector3d(-extent, Fixed64.Zero, extent),
            new Vector3d(extent, Fixed64.Zero, extent) };
        Vector3d center = new(Fixed64.FromFraction(6, 25), -Fixed64.Quarter, Fixed64.FromFraction(6, 25));
        // Both triangle bounds reach the cone, but its plane slice only
        // intersects the second triangle near the patch's exposed corner.
        new FixedTriangle(vertices[0], vertices[2], vertices[1])
            .TryGetCenteredFiniteConeContact(Vector3d.Zero, FixedQuaternion.Identity,
                center, FixedQuaternion.Identity, Fixed64.One, Fixed64.Half, out _).Should().BeFalse();
        int[] triangles = { 0, 2, 1, 1, 2, 3 };
        CollisionPair pair = CreatePair(scenario, vertices, triangles, center);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        contact.Depth.Should().BeGreaterThan(Fixed64.Zero);
        contact.Depth.Should().BeLessThan(Fixed64.Quarter);
        contact.PointA.Should().Be(new Vector3d(extent, Fixed64.Zero, extent));
        Fixed64 margin = Fixed64.FromRaw(64);
        CollisionDetection.DoCollisionCheck(CreatePair(scenario, vertices, triangles,
            center + contact.Normal * (contact.Depth - margin))).Should().BeTrue();
        CollisionDetection.DoCollisionCheck(CreatePair(scenario, vertices, triangles,
            center + contact.Normal * (contact.Depth + margin))).Should().BeFalse();
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void SmallPatchExposedPerimeter_ShouldRetainShorterRealEdgeExit(bool alternateDiagonal, bool reverseWinding)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 extent = Fixed64.FromFraction(3, 20);
        var vertices = new[] { new Vector3d(-extent, Fixed64.Zero, -extent),
            new Vector3d(extent, Fixed64.Zero, -extent), new Vector3d(-extent, Fixed64.Zero, extent),
            new Vector3d(extent, Fixed64.Zero, extent) };
        int[] triangles = alternateDiagonal ? new[] { 0, 2, 3, 0, 3, 1 } : new[] { 0, 2, 1, 1, 2, 3 };
        if (reverseWinding)
            for (int i = 0; i < triangles.Length; i += 3)
                (triangles[i + 1], triangles[i + 2]) = (triangles[i + 2], triangles[i + 1]);
        CollisionPair pair = CreatePair(scenario, vertices, triangles, Vector3d.Down * Fixed64.Quarter);

        // The real perimeter has a shorter exit than the 1/4 face depth:
        // (3/20 + 1/8) / sqrt(1 + (1/2)^2). The diagonal's 0.1118
        // exit is covered; always substituting the face would also be wrong.
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        contact.Depth.Should().BeApproximately(Fixed64.FromFraction(11, 40)
            / FixedMath.Sqrt(Fixed64.FromFraction(5, 4)), Fixed64.FromRaw(6));
        contact.Normal.Y.Should().BeLessThan(Fixed64.Zero);
        (contact.Normal.X == Fixed64.Zero ^ contact.Normal.Z == Fixed64.Zero).Should().BeTrue();
        bool onXEdge = FixedMath.Abs(contact.PointA.X) == extent;
        bool onZEdge = FixedMath.Abs(contact.PointA.Z) == extent;
        (onXEdge || onZEdge).Should().BeTrue();
        contact.PointA.Y.Should().Be(Fixed64.Zero);

        Vector3d center = Vector3d.Down * Fixed64.Quarter;
        Fixed64 margin = Fixed64.FromRaw(64);
        CollisionPair stillIntersecting = CreatePair(scenario, vertices, triangles,
            center + contact.Normal * (contact.Depth - margin));
        CollisionPair cleared = CreatePair(scenario, vertices, triangles,
            center + contact.Normal * (contact.Depth + margin));
        CollisionDetection.DoCollisionCheck(stillIntersecting).Should().BeTrue();
        CollisionDetection.DoCollisionCheck(cleared).Should().BeFalse();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FrictionlessFullLoop_ShouldPreserveTangentialMotionAcrossCoveredSeam(bool reverseWinding)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        var mesh = MeshTestFixtures.CreateConvexQuadFloor(MeshColliderMode.Concave);
        mesh.Material = PhysicsMaterial.Frictionless;
        var rotation = reverseWinding
            ? new FixedQuaternion(Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero)
            : FixedQuaternion.Identity;
        mesh.InitializeWithNoBody(new TestMatterAgent(scenario.Context,
            new FixedTransform(Vector3d.Zero, rotation, Vector3d.One)));
        SolidBody body = scenario.CreateCone(Vector3d.Down * Fixed64.Quarter,
            preventAngularForces: true).Body;
        body.Collider.Material = PhysicsMaterial.Frictionless;
        body.UseManualGrounding();
        body.AddLinearImpulse(Vector3d.Right * body.Mass);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        body.LinearVelocity.X.Should().Be(Fixed64.One);
        body.LinearVelocity.Z.Should().Be(Fixed64.Zero);
        body.Position3d.X.Should().Be(scenario.Context.DeltaTime);
        body.Position3d.Z.Should().Be(Fixed64.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SubdividedSurface_ShouldRetainOneWholePatchContact(bool tilted)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        const int subdivision = 8, width = subdivision + 1;
        var vertices = new Vector3d[width * width];
        var triangles = new int[subdivision * subdivision * 6];
        for (int z = 0; z < width; z++)
            for (int x = 0; x < width; x++)
                vertices[z * width + x] = new Vector3d(
                    Fixed64.FromFraction(4 * x, subdivision) - Fixed64.Two,
                    Fixed64.Zero, Fixed64.FromFraction(4 * z, subdivision) - Fixed64.Two);
        int offset = 0;
        for (int z = 0; z < subdivision; z++)
            for (int x = 0; x < subdivision; x++)
            {
                int first = z * width + x;
                triangles[offset++] = first; triangles[offset++] = first + width; triangles[offset++] = first + 1;
                triangles[offset++] = first + 1; triangles[offset++] = first + width; triangles[offset++] = first + width + 1;
            }
        var rotation = tilted ? new FixedQuaternion(Fixed64.Zero, Fixed64.Zero,
            Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5)) : FixedQuaternion.Identity;
        // For the tilted cone, its projected X extent is 31/50 around -28/25.
        // Enlarging the whole footprint by the 9/100 face exit still fits
        // strictly inside the quad. A cell seam therefore cannot clear it.
        Vector3d center = new(tilted ? -Fixed64.FromFraction(28, 25) : Fixed64.Zero,
            -Fixed64.Quarter, Fixed64.Zero);
        CollisionPair pair = CreatePair(scenario, vertices, triangles, center, rotation);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.Count.Should().Be(1);
        pair.Manifold.PrimaryContact.Normal.Should().Be(Vector3d.Down);
        pair.Manifold.PrimaryContact.Depth.Should().BeApproximately(
            tilted ? Fixed64.FromFraction(9, 100) : Fixed64.Quarter, Fixed64.FromRaw(4));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ExposedEdge_ShouldRetainShorterObliqueExit(bool singleTriangle)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var vertices = new[] { new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2),
            new Vector3d(-2, 0, 2), new Vector3d(2, 0, 2) };
        if (singleTriangle)
            Array.Resize(ref vertices, 3);
        CollisionPair pair = CreatePair(scenario, vertices,
            singleTriangle ? new[] { 0, 2, 1 } : new[] { 0, 2, 1, 1, 2, 3 },
            new Vector3d(singleTriangle ? Fixed64.Zero : Fixed64.Two, -Fixed64.Quarter, Fixed64.Zero));

        // Sliding toward the uncovered side clears this edge before moving
        // the apex below the plane; it remains a real curved-feature contact.
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        contact.Depth.Should().BeGreaterThan(Fixed64.Zero).And.BeLessThan(Fixed64.Quarter);
        contact.Normal.Should().NotBe(Vector3d.Down);
        contact.PointA.Y.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void LShapeInteriorOutsideConvexKernel_ShouldUseTheRealSurfaceExit()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var vertices = new[] { new Vector3d(0, 0, 0), new Vector3d(4, 0, 0),
            new Vector3d(4, 0, 1), new Vector3d(1, 0, 1), new Vector3d(1, 0, 4), new Vector3d(0, 0, 4) };
        CollisionPair pair = CreatePair(scenario, vertices,
            new[] { 0, 3, 1, 1, 3, 2, 0, 5, 3, 3, 5, 4 },
            new Vector3d((Fixed64)3, -Fixed64.Quarter, Fixed64.Half));

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.PrimaryContact.Normal.Should().Be(Vector3d.Down);
        pair.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.Quarter);
    }

    [Fact]
    public void NonconvexPatchExposedEdge_ShouldRetainCompleteTriangleContact()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var vertices = new[] { new Vector3d(0, 0, 0), new Vector3d(4, 0, 0),
            new Vector3d(4, 0, 1), new Vector3d(1, 0, 1), new Vector3d(1, 0, 4), new Vector3d(0, 0, 4) };
        int[] triangles = { 0, 3, 1, 1, 3, 2, 0, 5, 3, 3, 5, 4 };
        Vector3d center = new((Fixed64)3, -Fixed64.Quarter, Fixed64.One);
        CollisionPair pair = CreatePair(scenario, vertices, triangles, center);

        // The notch makes this patch nonconvex. At the distant exposed edge,
        // its real triangle exit still clears the complete L-shaped surface.
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        contact.Depth.Should().BeGreaterThan(Fixed64.Zero).And.BeLessThan(Fixed64.Quarter);
        contact.PointA.Z.Should().Be(Fixed64.One);
        contact.PointA.Y.Should().Be(Fixed64.Zero);
        Fixed64 margin = Fixed64.FromRaw(64);
        CollisionDetection.DoCollisionCheck(CreatePair(scenario, vertices, triangles,
            center + contact.Normal * (contact.Depth - margin))).Should().BeTrue();
        CollisionDetection.DoCollisionCheck(CreatePair(scenario, vertices, triangles,
            center + contact.Normal * (contact.Depth + margin))).Should().BeFalse();
    }

    [Fact]
    public void Hole_ShouldRemainUncovered()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var vertices = new[] { new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2),
            new Vector3d(2, 0, 2), new Vector3d(-2, 0, 2), new Vector3d(-1, 0, -1),
            new Vector3d(1, 0, -1), new Vector3d(1, 0, 1), new Vector3d(-1, 0, 1) };
        CollisionPair pair = CreatePair(scenario, vertices,
            new[] { 0, 5, 1, 0, 4, 5, 1, 6, 2, 1, 5, 6,
                2, 7, 3, 2, 6, 7, 3, 4, 0, 3, 7, 4 }, Vector3d.Down * Fixed64.Quarter);

        CollisionDetection.DoCollisionCheck(pair).Should().BeFalse();
        pair.Manifold.Count.Should().Be(0);
    }

    [Fact]
    public void TiltedBaseRimInterior_ShouldUseFaceExitAcrossInternalSeam()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var mesh = MeshTestFixtures.CreateConvexQuadFloor(MeshColliderMode.Concave);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        // sin(half-angle)=3/5, cos(half-angle)=4/5: axis Y=7/25 and
        // radial Y extent=12/25. Base rim reaches Y=-1/4-7/50+12/25=9/100.
        var rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero,
            Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5));
        var cone = new LSConeCollider();
        cone.InitializeWithNoBody(new TestMatterAgent(scenario.Context,
            new FixedTransform(new Vector3d(-Fixed64.FromFraction(3, 5), -Fixed64.Quarter, Fixed64.Zero),
                rotation, Vector3d.One)));
        CollisionPair pair = scenario.CreatePair(mesh, cone);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        contact.Normal.Should().Be(Vector3d.Down);
        contact.Depth.Should().BeApproximately(Fixed64.FromFraction(9, 100), Fixed64.FromRaw(4));
        contact.PointA.Y.Should().Be(Fixed64.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SharedRigidPose_ShouldPreservePatchContactAnchorsAndDirection(bool smallPatch)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        // This exact unit quaternion cycles X->Y->Z->X without rounding.
        var rotation = new FixedQuaternion(Fixed64.Half, Fixed64.Half, Fixed64.Half, Fixed64.Half);
        Vector3d origin = new(7, -3, 5);
        var mesh = MeshTestFixtures.CreateConvexQuadFloor(MeshColliderMode.Concave);
        if (smallPatch)
        {
            Fixed64 extent = Fixed64.FromFraction(1, 5);
            mesh = new LSMeshCollider(new[] { new Vector3d(-extent, Fixed64.Zero, -extent),
                new Vector3d(extent, Fixed64.Zero, -extent), new Vector3d(-extent, Fixed64.Zero, extent),
                new Vector3d(extent, Fixed64.Zero, extent) }, new[] { 0, 2, 1, 1, 2, 3 },
                MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        }
        mesh.InitializeWithNoBody(new TestMatterAgent(scenario.Context,
            new FixedTransform(origin, rotation, Vector3d.One)));
        var cone = new LSConeCollider();
        cone.InitializeWithNoBody(new TestMatterAgent(scenario.Context,
            new FixedTransform(origin + rotation * (Vector3d.Down * Fixed64.Quarter), rotation, Vector3d.One)));
        CollisionPair pair = scenario.CreatePair(mesh, cone);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        contact.Normal.Should().Be(rotation * Vector3d.Down);
        contact.Depth.Should().Be(Fixed64.Quarter);
        contact.PointA.Should().Be(origin);
        contact.PointB.Should().Be(origin + rotation * (Vector3d.Up * Fixed64.Quarter));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RepeatedWarmedPatchDispatch_ShouldBeStableAndAllocationFree(bool exposedEdge)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var mesh = MeshTestFixtures.CreateConvexQuadFloor(MeshColliderMode.Concave);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        var cone = new LSConeCollider();
        scenario.InitializeStaticCollider(cone,
            new Vector3d(exposedEdge ? Fixed64.Two : Fixed64.Zero, -Fixed64.Quarter, Fixed64.Zero));
        CollisionPair pair = scenario.CreatePair(mesh, cone);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        ManifoldContact expected = pair.Manifold.PrimaryContact;
        long allocated = AllocationTestHelper.MeasureSteadyState(() =>
        {
            if (!CollisionDetection.DoCollisionCheck(pair))
                throw new InvalidOperationException("Repeated patch contact changed.");
            ManifoldContact actual = pair.Manifold.PrimaryContact;
            if (actual.ContactId != expected.ContactId || actual.Depth != expected.Depth
                || actual.Normal != expected.Normal || actual.DepthIsClamped != expected.DepthIsClamped)
                throw new InvalidOperationException("Repeated patch contact changed.");
        }, warmupIterations: 8, stabilizationIterations: 2, measurementIterations: 4);
        allocated.Should().Be(0);
    }

    private static CollisionPair CreatePair(PhysicsScenarioBuilder scenario,
        Vector3d[] vertices, int[] triangles, Vector3d center, FixedQuaternion? rotation = null)
    {
        var mesh = new LSMeshCollider(vertices, triangles,
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        var cone = new LSConeCollider();
        cone.InitializeWithNoBody(new TestMatterAgent(scenario.Context,
            new FixedTransform(center, rotation ?? FixedQuaternion.Identity, Vector3d.One)));
        return scenario.CreatePair(mesh, cone);
    }
}
