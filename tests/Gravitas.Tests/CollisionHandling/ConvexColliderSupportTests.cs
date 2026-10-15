using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ConvexColliderSupportTests
{
    [Fact]
    public void CylinderSupport_InteriorDistanceSupport_ShouldPreserveContactFeatureSelection()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var cylinder = new LSCylinderCollider(ColliderShapeDefinition.Cylinder(Fixed64.One, (Fixed64)20));
        cylinder.InitializeWithNoBody(new TestMatterAgent(context, new FixedTransform(
            new Vector3d(Fixed64.FromFraction(19, 10), (Fixed64)10, Fixed64.Zero),
            FixedQuaternion.Identity, Vector3d.One)));

        // A response witness must retain its cap feature; only distance searches
        // can choose the interior of the exactly tied axial support face.
        FixedPointAnchor contact = ConvexColliderSupport.GetSupportAnchor(
            cylinder, Vector3d.Left, Vector3d.Zero);
        FixedPointAnchor distance = ConvexColliderSupport.GetSupportAnchor(
            cylinder, Vector3d.Left, Vector3d.Zero, preferAxialCenter: true);
        contact.TryGetPoint(out Vector3d contactPoint).Should().BeTrue();
        distance.TryGetPoint(out Vector3d distancePoint).Should().BeTrue();
        contactPoint.Y.Should().Be(Fixed64.Zero);
        distancePoint.Y.Should().Be((Fixed64)10);
        contactPoint.X.Should().Be(distancePoint.X);
        contactPoint.Z.Should().Be(distancePoint.Z);
    }

    [Fact]
    public void Intersects_WhenLaterRemoteSupportLeavesTheScalarDomain_ShouldRejectSeparation()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        FixedQuaternion tilt = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero,
            -Fixed64.FromRaw(2147), Fixed64.One).Normalized;
        var size = new Vector3d(Fixed64.FromRaw(4295), (Fixed64)4000, Fixed64.One);
        var first = new LSCuboidCollider { Size = size };
        var second = new LSCuboidCollider { Size = size };
        first.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, tilt, Vector3d.One)));
        second.InitializeWithNoBody(new TestMatterAgent(context, new FixedTransform(
            new Vector3d(Fixed64.MaxValue - Fixed64.MinIncrement, Fixed64.Zero, Fixed64.Zero),
            tilt, Vector3d.One)));

        // A small change in search direction switches the ends of ordinary thin
        // rods; their remote world-space support difference still needs a guard.
        FixedPointAnchor a = ConvexColliderSupport.GetSupportAnchor(first, Vector3d.Right, Vector3d.Zero);
        FixedPointAnchor b = ConvexColliderSupport.GetSupportAnchor(second, Vector3d.Left, Vector3d.Zero);
        a.TryGetOffsetFrom(b, out Vector3d difference).Should().BeTrue();
        a = ConvexColliderSupport.GetSupportAnchor(first, -difference, Vector3d.Zero);
        b = ConvexColliderSupport.GetSupportAnchor(second, difference, Vector3d.Zero);
        a.TryGetOffsetFrom(b, out _).Should().BeFalse();
        ConvexColliderSupport.Intersects(first, second).Should().BeFalse();
    }

    [Theory]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(256, false)]
    [InlineData(256, true)]
    [InlineData(1024, false)]
    [InlineData(1024, true)]
    public void ConeConvexContacts_ShouldPreserveClassificationAcrossScaleAndRigidRotation(
        int scale, bool oblique)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        FixedQuaternion rotation = oblique
            ? FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)19, (Fixed64)31, (Fixed64)7)
            : FixedQuaternion.Identity;
        var cone = new LSConeCollider(ColliderShapeDefinition.Cone((Fixed64)(2 * scale), (Fixed64)(4 * scale)));
        // Collider axes run base-to-apex, opposite the apex-origin query axis.
        cone.InitializeWithNoBody(new TestMatterAgent(context, new FixedTransform(
            rotation * new Vector3d(0, 2 * scale, 0),
            rotation * new FixedQuaternion(Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
            Vector3d.One)));

        foreach ((Vector3d center, bool expected) in new[]
        {
            (new Vector3d(2, 2, 0), false),
            (new Vector3d(1, 3, 1), true),
            (new Vector3d(2, 3, 2), false),
            (new Vector3d(0, 2, 0), true)
        })
        {
            Vector3d position = rotation * (center * (Fixed64)scale);
            var box = new LSCuboidCollider { Size = Vector3d.One * (Fixed64)scale * Fixed64.Half };
            box.InitializeWithNoBody(new TestMatterAgent(context,
                new FixedTransform(position, rotation, Vector3d.One)));
            LSMeshCollider sourceMesh = MeshTestFixtures.CreateConvexCube();
            Vector3d[] vertices = sourceMesh.Mesh.LocalVertices.ToArray();
            for (int i = 0; i < vertices.Length; i++)
                vertices[i] *= (Fixed64)scale * Fixed64.Half;
            var mesh = new LSMeshCollider(vertices, sourceMesh.Mesh.Triangles.ToArray(), MeshColliderMode.Convex);
            mesh.InitializeWithNoBody(new TestMatterAgent(context, new FixedTransform(
                position, rotation, Vector3d.One)));

            var sphere = new LSSphereCollider { Radius = Fixed64.Quarter * scale };
            var capsule = new LSCapsuleCollider(ColliderShapeDefinition.Capsule(
                Fixed64.FromFraction(1, 8) * scale, Fixed64.Half * scale));
            var cylinder = new LSCylinderCollider(ColliderShapeDefinition.Cylinder(
                Fixed64.Quarter * scale, Fixed64.Half * scale));
            var smallCone = new LSConeCollider(ColliderShapeDefinition.Cone(
                Fixed64.Quarter * scale, Fixed64.Half * scale));
            foreach (LSCollider target in new LSCollider[] { sphere, capsule, cylinder, smallCone })
                target.InitializeWithNoBody(new TestMatterAgent(context, new FixedTransform(position, rotation, Vector3d.One)));

            foreach (LSCollider target in new LSCollider[] { box, mesh, sphere, capsule, cylinder, smallCone })
            {
                ConvexColliderSupport.Intersects(cone, target).Should().Be(expected,
                    "the placement {0} has an analytical containment or separation margin", center);
                ConvexColliderSupport.Intersects(target, cone).Should().Be(expected);
                foreach (CollisionPair pair in new[] { new CollisionPair(cone, target), new CollisionPair(target, cone) })
                {
                    CollisionDetection.DoCollisionCheck(pair).Should().Be(expected);
                    pair.Manifold.HasContact.Should().Be(expected);
                    if (expected)
                    {
                        Vector3d normal = pair.ColliderA == cone
                            ? pair.Manifold.PrimaryContact.Normal
                            : -pair.Manifold.PrimaryContact.Normal;
                        Vector3d originalPosition = target.Position;
                        target.Position += normal * (pair.Manifold.PrimaryContact.Depth + Fixed64.FromFraction(1, 64));
                        ConvexColliderSupport.Intersects(cone, target).Should().BeFalse(
                            "the complete target should leave the cone after its reported exit depth");
                        CollisionDetection.DoCollisionCheck(new CollisionPair(cone, target)).Should().BeFalse();
                        target.Position = originalPosition;
                    }
                }
            }
        }
    }

    [Fact]
    public void IsSupported_ShouldRejectUnsupportedColliderTypes()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(Vector3d.Zero);
        var unsupported = new UnsupportedTestCollider3D();
        scenario.InitializeStaticCollider(unsupported, Vector3d.Zero);

        ConvexColliderSupport.IsSupported(sphere.Collider).Should().BeTrue();
        ConvexColliderSupport.IsSupported(unsupported).Should().BeFalse();
        ConvexColliderSupport.Intersects(sphere.Collider, unsupported).Should().BeFalse();
        ConvexColliderSupport.Intersects(unsupported, sphere.Collider).Should().BeFalse();
    }

    [Fact]
    public void Support_WithZeroDirection_ShouldUseRightAxisFallback()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(Vector3d.Zero);

        GetSupport(sphere.Collider, Vector3d.Zero)
            .Should().Be(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
    }

    [Fact]
    public void Support_WithCylinderAxisDirection_ShouldUseStableCapCenterTie()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCylinderCollider> cylinder = scenario.CreateCylinder(Vector3d.Zero);

        GetSupport(cylinder.Collider, Vector3d.Up)
            .Should().Be(new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Zero));
    }

    [Fact]
    public void Support_WithOddRawCylinderHeight_ShouldRoundThePositiveEndpointOutward()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var cylinder = new LSCylinderCollider
        {
            Radius = Fixed64.FromRaw(1),
            Size = new Vector3d(
                Fixed64.FromRaw(2),
                Fixed64.FromRaw(1),
                Fixed64.FromRaw(2))
        };
        scenario.InitializeStaticCollider(
            cylinder,
            new Vector3d(
                Fixed64.Zero,
                Fixed64.FromRaw(1),
                Fixed64.Zero));

        GetSupport(cylinder, Vector3d.Up)
            .Should().Be(new Vector3d(
                Fixed64.Zero,
                Fixed64.FromRaw(2),
                Fixed64.Zero));
    }

    [Fact]
    public void Support_WithExtremeTranslatedCuboid_ShouldCompareCenteredFeatures()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 offset = new(2_000_000_000);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(
            new Vector3d(offset, offset, Fixed64.Zero));
        Vector3d direction = new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero).Normalized;

        Vector3d support = GetSupport(cuboid.Collider, direction);

        support.Should().Be(new Vector3d(offset + Fixed64.Half, offset + Fixed64.Half, -Fixed64.Half));
    }

    [Fact]
    public void Support_WithExtremeTranslatedConvexMesh_ShouldCompareLocalFeatures()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 offset = new(2_000_000_000);
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            MeshTestFixtures.CreateConvexCube(inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            new Vector3d(offset, offset, Fixed64.Zero),
            FixedQuaternion.Identity);
        Vector3d direction = new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero).Normalized;

        Vector3d support = GetSupport(mesh.Collider, direction);

        support.X.Should().Be(offset + Fixed64.Half);
        support.Y.Should().Be(offset + Fixed64.Half);
    }

    [Fact]
    public void Support_WithExtremeTranslatedRotatedCapsule_ShouldSelectForwardSegmentEndpoint()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 offset = new(2_000_000_000);
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateBody(
            new LSCapsuleCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) },
            new Vector3d(offset, offset, Fixed64.Zero),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)(-45)));
        Vector3d direction = capsule.Collider.WorldAxis;

        Vector3d support = GetSupport(capsule.Collider, direction);

        FixedPointAnchor expectedAnchor =
            FixedSegment.GetCenteredCapsuleSupportAnchor(
            capsule.Collider.Center,
            capsule.Collider.Rotation,
            capsule.Collider.AxisLength,
            capsule.Collider.ScaledRadius,
            direction);
        expectedAnchor.TryGetPoint(out Vector3d expected).Should().BeTrue();
        support.Should().Be(expected);
    }

    [Theory]
    [InlineData(FiniteAxisShape.Capsule)]
    [InlineData(FiniteAxisShape.Cylinder)]
    [InlineData(FiniteAxisShape.Cone)]
    public void OffsetSupportAnchor_ShouldPreserveCanonicalFeatureAndTranslation(
        FiniteAxisShape shape)
    {
        using PhysicsScenarioBuilder scenario =
            PhysicsScenarioBuilder.Create();
        FixedQuaternion rotation =
            PhysicsScenarioBuilder.Yaw(35);
        LSCollider collider = CreateFiniteAxisCollider(
            scenario,
            shape,
            Vector3d.Zero,
            rotation);
        Vector3d direction = rotation.Rotate(
            new Vector3d(1, -1, 1)).Normalized;
        Vector3d worldTranslation = new(3, -2, 1);
        rotation.Inverse().TryRotate(
                worldTranslation,
                out Vector3d localTranslation)
            .Should()
            .BeTrue();
        FixedPointAnchor canonical =
            GetCanonicalSupportAnchor(collider, direction);

        FixedPointAnchor translated =
            ConvexColliderSupport.GetSupportAnchor(
                collider,
                direction,
                worldTranslation);

        translated.CompareLocalFeature(canonical).Should().Be(0);
        translated.LocalTranslation.Should().Be(localTranslation);
        translated.TryGetOffsetFrom(
                canonical,
                out Vector3d resolvedTranslation)
            .Should()
            .BeTrue();
        resolvedTranslation.Should().Be(worldTranslation);
    }

    [Theory]
    [InlineData(FiniteAxisShape.Capsule, true)]
    [InlineData(FiniteAxisShape.Capsule, false)]
    [InlineData(FiniteAxisShape.Cylinder, true)]
    [InlineData(FiniteAxisShape.Cylinder, false)]
    [InlineData(FiniteAxisShape.Cone, true)]
    [InlineData(FiniteAxisShape.Cone, false)]
    public void OffsetSupportAnchor_AtScalarFaceShouldRetainExactFeature(
        FiniteAxisShape shape,
        bool positiveFace)
    {
        using PhysicsScenarioBuilder scenario =
            PhysicsScenarioBuilder.Create();
        Fixed64 face = positiveFace
            ? Fixed64.MaxValue
            : Fixed64.MinValue;
        Fixed64 inward = positiveFace
            ? -Fixed64.One
            : Fixed64.One;
        Vector3d center = new(
            face + inward,
            Fixed64.Zero,
            Fixed64.Zero);
        LSCollider collider = CreateFiniteAxisCollider(
            scenario,
            shape,
            center,
            FixedQuaternion.Identity);
        Vector3d direction = new Vector3d(
            inward,
            -Fixed64.One,
            Fixed64.One).Normalized;
        Vector3d translation = new(
            -inward,
            Fixed64.Zero,
            Fixed64.Zero);
        FixedPointAnchor canonical =
            GetCanonicalSupportAnchor(collider, direction);

        FixedPointAnchor translated =
            ConvexColliderSupport.GetSupportAnchor(
                collider,
                direction,
                translation);

        translated.CompareLocalFeature(canonical).Should().Be(0);
        translated.TryGetOffsetFrom(
                canonical,
                out Vector3d resolvedTranslation)
            .Should()
            .BeTrue();
        resolvedTranslation.Should().Be(translation);
        translated.TryGetPoint(out Vector3d point).Should().BeTrue();
        canonical.TryGetPoint(out Vector3d canonicalPoint)
            .Should()
            .BeTrue();
        Vector3d.TryAdd(
                canonicalPoint,
                translation,
                out Vector3d expectedPoint)
            .Should()
            .BeTrue();
        point.Should().Be(expectedPoint);
    }

    [Fact]
    public void Support_ShouldUseConvexMeshVerticesAndRejectUnsupportedColliders()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            MeshTestFixtures.CreateConvexCube(inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity);
        var unsupported = new UnsupportedTestCollider3D();
        scenario.InitializeStaticCollider(unsupported, Vector3d.Zero);

        Vector3d support = GetSupport(mesh.Collider, Vector3d.Right);
        Action unsupportedSupport = () => ConvexColliderSupport.GetSupportAnchor(
            unsupported,
            Vector3d.Right,
            Vector3d.Zero);

        support.X.Should().Be(Fixed64.FromFraction(5, 2));
        unsupportedSupport.Should().Throw<NotSupportedException>();
    }

    [Fact]
    public void Intersects_WithSameCenterSpheres_ShouldReturnTrueImmediately()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Zero);

        ConvexColliderSupport.Intersects(first.Collider, second.Collider).Should().BeTrue();
    }

    [Fact]
    public void Intersects_WhenIterationBudgetIsExhaustedWithoutSeparation_ShouldRemainConservative()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)4);

        ConvexColliderSupport.Intersects(first.Collider, second.Collider, maxIterations: 0).Should().BeTrue();
    }

    [Fact]
    public void Intersects_WithSeparatedConvexShapes_ShouldReturnFalse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));

        ConvexColliderSupport.Intersects(sphere.Collider, cuboid.Collider).Should().BeFalse();
    }

    [Fact]
    public void Intersects_WithUnrepresentableSupportDifference_ShouldRejectCandidate()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> first =
            scenario.CreateCuboid(Vector3d.Zero);
        ScenarioBody<LSCuboidCollider> second =
            scenario.CreateCuboid(Vector3d.Zero);
        first.Body.ResetPosition(new Vector3d(Fixed64.MinValue + Fixed64.One, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity);
        second.Body.ResetPosition(new Vector3d(Fixed64.MaxValue - Fixed64.One, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity);
        first.Collider.RebuildRuntimeShapeOnly();
        second.Collider.RebuildRuntimeShapeOnly();

        ConvexColliderSupport.Intersects(first.Collider, second.Collider)
            .Should().BeFalse();
    }

    [Fact]
    public void Intersects_WithUnsupportedSphere_ShouldRejectBeforeNarrowPhase()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var huge = new LSSphereCollider { Radius = Fixed64.MaxValue };
        Action initialize = () => scenario.InitializeStaticCollider(huge, Vector3d.Zero);
        initialize.Should().Throw<ArgumentException>().WithMessage("*geometry reach*");
        huge.Id.Should().Be(-1);
    }

    [Fact]
    public void IntersectsConeVolume_WhenSupportDifferenceLeavesDomain_ShouldRejectCandidate()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider sphere = scenario.CreateStaticSphere(
            Vector3d.Zero);

        ConvexColliderSupport.IntersectsConeVolume(
            sphere,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.MaxValue).Should().BeFalse();
    }

    [Fact]
    public void IntersectsConeVolume_WhenLaterSupportDifferenceLeavesDomain_ShouldRejectCandidate()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider sphere = scenario.CreateStaticSphere(
            new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));

        ConvexColliderSupport.IntersectsConeVolume(
            sphere,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.MaxValue).Should().BeFalse();
    }

    [Fact]
    public void OffsetSupportAnchor_WithUnrepresentableLocalTranslation_ShouldReject()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateBody(
            new LSSphereCollider(),
            Vector3d.Zero,
            FixedQuaternion.FromAxisAngle(
                Vector3d.Forward,
                Fixed64.PiOver4));
        Vector3d displacement = new(
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Fixed64.Zero);

        Action getSupport = () => ConvexColliderSupport.GetSupportAnchor(
            sphere.Collider,
            Vector3d.Right,
            displacement);

        getSupport.Should().Throw<InvalidOperationException>();
    }

    [Fact]
    public void Intersects_WithOverlappingRotatedConvexShapes_ShouldReturnTrue()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSConeCollider> cone = scenario.CreateBody(
            new LSConeCollider
            {
                Radius = Fixed64.Half,
                Size = new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One)
            },
            Vector3d.Zero,
            PhysicsScenarioBuilder.Yaw(35));
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(
            new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Zero),
            PhysicsScenarioBuilder.Yaw(-20));

        ConvexColliderSupport.Intersects(cone.Collider, cuboid.Collider).Should().BeTrue();
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void Intersects_WithRotatedCuboidAtScalarFace_ShouldUseRelativeSupportDifferences(
        bool positiveFace)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Vector3d cuboidCenter = positiveFace
            ? new Vector3d(
                Fixed64.MaxValue - Fixed64.One,
                Fixed64.Zero,
                Fixed64.Zero)
            : new Vector3d(
                Fixed64.MinValue + Fixed64.One,
                Fixed64.Zero,
                Fixed64.Zero);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)4, Fixed64.One, Fixed64.One)
            },
            cuboidCenter,
            PhysicsScenarioBuilder.Yaw(45));
        Vector3d sphereCenter = positiveFace
            ? new Vector3d(
                Fixed64.MaxValue - Fixed64.FromFraction(1, 200),
                Fixed64.Zero,
                -Fixed64.One)
            : new Vector3d(
                Fixed64.MinValue + Fixed64.FromFraction(1, 200),
                Fixed64.Zero,
                Fixed64.One);
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateBody(
            new LSSphereCollider
            {
                Radius = Fixed64.FromFraction(1, 100)
            },
            sphereCenter,
            FixedQuaternion.Identity);

        cuboid.Collider.OrientedBox.Contains(sphereCenter).Should().BeTrue();
        ConvexColliderSupport.Intersects(cuboid.Collider, sphere.Collider).Should().BeTrue();
        ConvexColliderSupport.Intersects(sphere.Collider, cuboid.Collider).Should().BeTrue();
    }

    [Fact]
    public void Intersects_WithTouchingAxisAlignedSpheres_ShouldReturnTrue()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right);

        ConvexColliderSupport.Intersects(first.Collider, second.Collider).Should().BeTrue();
    }

    [Fact]
    public void Intersects_WithVerticallyTouchingSpheres_ShouldUseStablePerpendicularFallback()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Up);

        ConvexColliderSupport.Intersects(first.Collider, second.Collider).Should().BeTrue();
    }

    [Fact]
    public void Intersects_WithOffsetCapsuleAgainstCuboid_ShouldReduceTriangleSimplex()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateBody(
            new LSCapsuleCollider
            {
                Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One)
            },
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            PhysicsScenarioBuilder.Yaw(90));
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(Vector3d.Zero);

        ConvexColliderSupport.Intersects(capsule.Collider, cuboid.Collider).Should().BeTrue();
    }

    [Fact]
    public void Intersects_WithSphereInsideCuboid_ShouldReduceTetrahedronSimplex()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)3, (Fixed64)3, (Fixed64)3)
            },
            Vector3d.Zero,
            PhysicsScenarioBuilder.Yaw(15));

        ConvexColliderSupport.Intersects(sphere.Collider, cuboid.Collider).Should().BeTrue();
    }

    [Fact]
    public void Intersects_WithConeRimTouchingCuboidFace_ShouldAcceptEpsilonDirection()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSConeCollider> cone = scenario.CreateBody(
            new LSConeCollider
            {
                Radius = Fixed64.One,
                Size = new Vector3d((Fixed64)2, (Fixed64)2, (Fixed64)2)
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(
            new Vector3d(-Fixed64.FromFraction(3, 2), -Fixed64.One, Fixed64.FromFraction(1, 8)));
        Vector3d touchingPoint = new(-Fixed64.One, -Fixed64.One, Fixed64.Zero);
        CollisionPair pair = scenario.CreatePair(cone.Collider, cuboid.Collider);

        ConvexColliderSupport.Intersects(cone.Collider, cuboid.Collider).Should().BeTrue();
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.HasContact.Should().BeTrue();
        FixedSegment.TryGetCenteredAxisEndpoint(
            cone.Collider.Center,
            cone.Collider.WorldAxis,
            cone.Collider.Height,
            positive: false,
            out Vector3d baseCenter).Should().BeTrue();
        baseCenter.Y.Should().Be(touchingPoint.Y);
        cuboid.Collider.BoundsMax.X.Should().Be(touchingPoint.X);
    }

    [Fact]
    public void Intersects_WithConeRimTouchingCuboidCorner_ShouldAcceptIterationBudget()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSConeCollider> cone = scenario.CreateBody(
            new LSConeCollider
            {
                Radius = Fixed64.One,
                Size = new Vector3d((Fixed64)2, (Fixed64)2, (Fixed64)2)
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(
            new Vector3d(-Fixed64.FromFraction(3, 2), -Fixed64.Half, -Fixed64.Half));
        ScenarioBody<LSCuboidCollider> separated = scenario.CreateCuboid(
            new Vector3d(-Fixed64.FromFraction(13, 8), -Fixed64.Half, -Fixed64.Half));
        Vector3d touchingPoint = new(-Fixed64.One, -Fixed64.One, Fixed64.Zero);
        CollisionPair pair = scenario.CreatePair(cone.Collider, cuboid.Collider);

        ConvexColliderSupport.Intersects(cone.Collider, cuboid.Collider).Should().BeTrue();
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.HasContact.Should().BeTrue();
        ConvexColliderSupport.Intersects(cone.Collider, separated.Collider).Should().BeFalse();
        FixedSegment.TryGetCenteredAxisEndpoint(
            cone.Collider.Center,
            cone.Collider.WorldAxis,
            cone.Collider.Height,
            positive: false,
            out Vector3d baseCenter).Should().BeTrue();
        baseCenter.Y.Should().Be(touchingPoint.Y);
        cuboid.Collider.BoundsMax.X.Should().Be(touchingPoint.X);
        cuboid.Collider.BoundsMin.Y.Should().Be(touchingPoint.Y);
        cuboid.Collider.BoundsMax.Z.Should().Be(touchingPoint.Z);
    }

    [Fact]
    public void IntersectsConeVolume_ShouldDetectConvexHitsAndMisses()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> hit = scenario.CreateSphere(new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> miss = scenario.CreateSphere(new Vector3d((Fixed64)4, Fixed64.One, Fixed64.Zero));
        Vector3d apex = Vector3d.Zero;
        Vector3d baseCenter = new(Fixed64.Zero, (Fixed64)2, Fixed64.Zero);
        Vector3d axis = Vector3d.Up;
        Fixed64 radius = Fixed64.One;

        ConvexColliderSupport.IntersectsConeVolume(hit.Collider, apex, baseCenter, axis, radius).Should().BeTrue();
        ConvexColliderSupport.IntersectsConeVolume(miss.Collider, apex, baseCenter, axis, radius).Should().BeFalse();
    }

    [Fact]
    public void IntersectsConeVolume_WithSphereTouchingApex_ShouldReturnTrueImmediately()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(
            new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Zero));

        ConvexColliderSupport.IntersectsConeVolume(
            sphere.Collider,
            Vector3d.Zero,
            new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero),
            Vector3d.Up,
            Fixed64.One).Should().BeTrue();
        sphere.Collider.BoundsMax.Y.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void IntersectsConeVolume_WithCuboidTouchingBaseRim_ShouldAcceptEpsilonDirection()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(
            new Vector3d(-Fixed64.FromFraction(3, 2), (Fixed64)2, Fixed64.FromFraction(1, 4)));
        Vector3d touchingPoint = new(-Fixed64.One, (Fixed64)2, Fixed64.Zero);

        ConvexColliderSupport.IntersectsConeVolume(
            cuboid.Collider,
            Vector3d.Zero,
            new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero),
            Vector3d.Up,
            Fixed64.One).Should().BeTrue();
        cuboid.Collider.BoundsMax.X.Should().Be(touchingPoint.X);
        cuboid.Collider.BoundsMin.Y.Should().BeLessThanOrEqualTo(touchingPoint.Y);
        cuboid.Collider.BoundsMax.Y.Should().BeGreaterThanOrEqualTo(touchingPoint.Y);
    }

    [Fact]
    public void IntersectsConeVolume_ShouldRejectUnsupportedColliderTypes()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var unsupported = new UnsupportedTestCollider3D();
        scenario.InitializeStaticCollider(unsupported, Vector3d.Zero);

        ConvexColliderSupport.IntersectsConeVolume(
            unsupported,
            Vector3d.Zero,
            Vector3d.Up,
            Vector3d.Up,
            Fixed64.Half).Should().BeFalse();
    }

    [Fact]
    public void IntersectsConeVolume_WithExhaustedIterationBudget_ShouldPreserveCandidate()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));

        ConvexColliderSupport.IntersectsConeVolume(
            sphere.Collider,
            Vector3d.Zero,
            new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero),
            Vector3d.Up,
            Fixed64.One,
            maxIterations: 0).Should().BeTrue();
    }

    private static LSCollider CreateFiniteAxisCollider(
        PhysicsScenarioBuilder scenario,
        FiniteAxisShape shape,
        Vector3d center,
        FixedQuaternion rotation) =>
        shape switch
        {
            FiniteAxisShape.Capsule => scenario.CreateBody(
                new LSCapsuleCollider
                {
                    Size = new Vector3d(2, 4, 2)
                },
                center,
                rotation).Collider,
            FiniteAxisShape.Cylinder => scenario.CreateBody(
                new LSCylinderCollider
                {
                    Size = new Vector3d(2, 4, 2)
                },
                center,
                rotation).Collider,
            _ => scenario.CreateBody(
                new LSConeCollider
                {
                    Size = new Vector3d(2, 4, 2)
                },
                center,
                rotation).Collider,
        };

    private static FixedPointAnchor GetCanonicalSupportAnchor(
        LSCollider collider,
        Vector3d direction) =>
        collider switch
        {
            LSCapsuleCollider capsule =>
                FixedSegment.GetCenteredCapsuleSupportAnchor(
                    capsule.Center,
                    capsule.Rotation,
                    capsule.AxisLength,
                    capsule.ScaledRadius,
                    direction),
            LSCylinderCollider cylinder =>
                FixedSegment.GetCenteredFiniteCylinderSupportAnchor(
                    cylinder.Center,
                    cylinder.Rotation,
                    cylinder.Height,
                    cylinder.ScaledRadius,
                    direction),
            LSConeCollider cone =>
                FixedSegment.GetCenteredFiniteConeSupportAnchor(
                    cone.Center,
                    cone.Rotation,
                    cone.Height,
                    cone.ScaledRadius,
                    direction),
            _ => throw new InvalidOperationException(),
        };

    private static Vector3d GetSupport(
        LSCollider collider,
        Vector3d direction)
    {
        FixedPointAnchor anchor = ConvexColliderSupport.GetSupportAnchor(
            collider,
            direction,
            Vector3d.Zero);
        anchor.TryGetPoint(out Vector3d point).Should().BeTrue();
        return point;
    }

    public enum FiniteAxisShape
    {
        Capsule,
        Cylinder,
        Cone,
    }
}
