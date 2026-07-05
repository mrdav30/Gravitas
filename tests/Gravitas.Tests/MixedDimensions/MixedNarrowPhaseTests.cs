using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using System;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed class MixedNarrowPhaseTests
{
    public static TheoryData<string, Func<GravitasWorldContext, LSCollider>, Func<GravitasWorldContext, LSCollider2D>> RemainingPrimitivePairs =>
        new()
        {
            {
                "Cuboid_Circle",
                context => CreateCuboid3D(context, new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero).Collider
            },
            {
                "Cuboid_AABox",
                context => CreateCuboid3D(context, new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)), Vector2d.Zero).Collider
            },
            {
                "Cuboid_Capsule",
                context => CreateCuboid3D(context, new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3), Vector2d.Zero).Collider
            },
            {
                "Cuboid_ConvexPolygon",
                context => CreateCuboid3D(
                    context,
                    new Vector3d(Fixed64.FromFraction(13, 10), Fixed64.Zero, Fixed64.Zero),
                    FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)45, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, CreateSquarePolygon(), Vector2d.Zero, FixedMath.DegToRad((Fixed64)45)).Collider
            },
            {
                "Capsule_Circle",
                context => CreateCapsule3D(context, new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero).Collider
            },
            {
                "Capsule_AABox",
                context => CreateCapsule3D(
                    context,
                    new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.Zero),
                    FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90)).Collider,
                context => CreateBody2D(context, new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)), Vector2d.Zero).Collider
            },
            {
                "Capsule_Capsule2D",
                context => CreateCapsule3D(context, new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3), Vector2d.Zero).Collider
            },
            {
                "Capsule_ConvexPolygon",
                context => CreateCapsule3D(context, new Vector3d(Fixed64.FromFraction(13, 10), Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, CreateSquarePolygon(), Vector2d.Zero, FixedMath.DegToRad((Fixed64)45)).Collider
            },
            {
                "Cylinder_Circle",
                context => CreateCylinder3D(context, new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero).Collider
            },
            {
                "Cylinder_AABox",
                context => CreateCylinder3D(
                    context,
                    new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.Zero),
                    FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90)).Collider,
                context => CreateBody2D(context, new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)), Vector2d.Zero).Collider
            },
            {
                "Cylinder_Capsule2D",
                context => CreateCylinder3D(context, new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3), Vector2d.Zero).Collider
            },
            {
                "Cylinder_ConvexPolygon",
                context => CreateCylinder3D(context, new Vector3d(Fixed64.FromFraction(13, 10), Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, CreateSquarePolygon(), Vector2d.Zero, FixedMath.DegToRad((Fixed64)45)).Collider
            },
            {
                "Cone_Circle",
                context => CreateCone3D(context, new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero).Collider
            },
            {
                "Cone_AABox",
                context => CreateCone3D(context, new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)), Vector2d.Zero).Collider
            },
            {
                "Cone_Capsule2D",
                context => CreateCone3D(context, new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3), Vector2d.Zero).Collider
            },
            {
                "Cone_ConvexPolygon",
                context => CreateCone3D(context, new Vector3d(Fixed64.FromFraction(13, 10), Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, CreateSquarePolygon(), Vector2d.Zero, FixedMath.DegToRad((Fixed64)45)).Collider
            }
        };

    [Fact]
    public void SphereCircleSlab_WithPlanarOverlap_ShouldReportDeterministicContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(
            context,
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D circle = CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(sphere.Collider, circle.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.HasContact.Should().BeTrue();
        contact.Normal3DTo2D.Should().Be(-Vector3d.Right);
        contact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        contact.Point3D.Should().Be(new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero));
        contact.Point2D.Should().Be(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
    }

    [Fact]
    public void SphereCircleSlab_WithSeparatedYSlab_ShouldNotCollide()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero));
        SolidBody2D circle = CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero);

        CollisionDetectionMixed.TryCollide(sphere.Collider, circle.Collider, out MixedContact contact).Should().BeFalse();
        contact.HasContact.Should().BeFalse();
    }

    [Fact]
    public void SphereCapsuleSlab_WithPlanarSideOverlap_ShouldReportContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(
            context,
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D capsule = CreateBody2D(context, new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3), Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(sphere.Collider, capsule.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.HasContact.Should().BeTrue();
        contact.Normal3DTo2D.Should().Be(-Vector3d.Right);
        contact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        contact.Point3D.X.Should().Be(Fixed64.FromFraction(1, 4));
        contact.Point2D.X.Should().Be(Fixed64.Half);
    }

    [Fact]
    public void SphereAABoxSlab_WithTouchingFace_ShouldReportZeroDepthContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(
            context,
            new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D box = CreateBody2D(context, new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)), Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(sphere.Collider, box.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.Zero);
        contact.Normal3DTo2D.Should().Be(-Vector3d.Right);
        contact.Point3D.Should().Be(new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero));
        contact.Point2D.Should().Be(new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero));
    }

    [Fact]
    public void SphereConvexPolygonSlab_WithCornerOverlap_ShouldReportContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(
            context,
            new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)2));
        var polygon = new LSPolygonCollider2D(
            new Vector2d(-Fixed64.One, -Fixed64.One),
            new Vector2d(Fixed64.One, -Fixed64.One),
            new Vector2d(Fixed64.One, Fixed64.One),
            new Vector2d(-Fixed64.One, Fixed64.One));
        SolidBody2D polygonBody = CreateBody2D(context, polygon, Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(sphere.Collider, polygonBody.Collider, out MixedContact contact);

        collided.Should().BeFalse();

        ScenarioBody<LSSphereCollider> overlappingSphere = CreateSphere3D(
            context,
            new Vector3d(Fixed64.FromFraction(13, 10), Fixed64.Zero, Fixed64.FromFraction(13, 10)));

        CollisionDetectionMixed.TryCollide(overlappingSphere.Collider, polygonBody.Collider, out contact).Should().BeTrue();
        contact.Depth.Should().BeGreaterThan(Fixed64.Zero);
        contact.Point2D.X.Should().Be(Fixed64.One);
        contact.Point2D.Z.Should().Be(Fixed64.One);
    }

    [Theory]
    [MemberData(nameof(RemainingPrimitivePairs))]
    public void RemainingPrimitivePairs_WithRepresentativeOverlap_ShouldReportContact(
        string caseName,
        Func<GravitasWorldContext, LSCollider> create3D,
        Func<GravitasWorldContext, LSCollider2D> create2D)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider collider3D = create3D(context);
        LSCollider2D collider2D = create2D(context);

        bool collided = CollisionDetectionMixed.TryCollide(collider3D, collider2D, out MixedContact contact);

        collided.Should().BeTrue(caseName);
        contact.HasContact.Should().BeTrue();
        contact.Depth.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        contact.Normal3DTo2D.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void CuboidAABoxSlab_WithTouchingFace_ShouldReportZeroDepthContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCuboidCollider> cuboid = CreateCuboid3D(
            context,
            new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D box = CreateBody2D(context, new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)), Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(cuboid.Collider, box.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.Zero);
        contact.Normal3DTo2D.Should().Be(-Vector3d.Right);
        contact.Point3D.X.Should().Be(Fixed64.One);
        contact.Point2D.X.Should().Be(Fixed64.One);
    }

    [Fact]
    public void CylinderAABoxSlab_WithStartingOverlap_ShouldReturnStableContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCylinderCollider> cylinder = CreateCylinder3D(context, Vector3d.Zero);
        SolidBody2D box = CreateBody2D(context, new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)), Vector2d.Zero);

        bool firstCollided = CollisionDetectionMixed.TryCollide(cylinder.Collider, box.Collider, out MixedContact first);
        bool secondCollided = CollisionDetectionMixed.TryCollide(cylinder.Collider, box.Collider, out MixedContact second);

        firstCollided.Should().BeTrue();
        secondCollided.Should().BeTrue();
        first.HasContact.Should().BeTrue();
        first.Depth.Should().BeGreaterThan(Fixed64.Zero);
        first.Normal3DTo2D.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
        second.Point3D.Should().Be(first.Point3D);
        second.Point2D.Should().Be(first.Point2D);
        second.Normal3DTo2D.Should().Be(first.Normal3DTo2D);
        second.Depth.Should().Be(first.Depth);
    }

    [Fact]
    public void CylinderCircleSlab_WithPlanarRimOverlap_ShouldReportFiniteCylinderContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCylinderCollider> cylinder = CreateCylinder3D(
            context,
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D circle = CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(cylinder.Collider, circle.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.HasContact.Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        contact.Normal3DTo2D.Should().Be(-Vector3d.Right);
        contact.Point3D.X.Should().Be(Fixed64.FromFraction(1, 4));
        contact.Point2D.X.Should().Be(Fixed64.Half);
    }

    [Fact]
    public void ConeCircleSlab_WithBaseRimOverlap_ShouldReportFiniteConeContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSConeCollider> cone = CreateCone3D(
            context,
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D circle = CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(cone.Collider, circle.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.HasContact.Should().BeTrue();
        contact.Depth.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        contact.Normal3DTo2D.Should().Be(-Vector3d.Right);
        contact.Point3D.X.Should().BeLessThan(cone.Collider.Center.X);
        contact.Point2D.X.Should().Be(Fixed64.Half);
    }

    [Fact]
    public void ConeCircleSlab_WithSeparatedYSlab_ShouldNotCollide()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSConeCollider> cone = CreateCone3D(
            context,
            new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero));
        SolidBody2D circle = CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero);

        CollisionDetectionMixed.TryCollide(cone.Collider, circle.Collider, out MixedContact contact).Should().BeFalse();
        contact.HasContact.Should().BeFalse();
    }

    [Fact]
    public void CapsuleCircleSlab_WithPlanarSideOverlap_ShouldReportContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCapsuleCollider> capsule = CreateCapsule3D(
            context,
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D circle = CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(capsule.Collider, circle.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        contact.Normal3DTo2D.Should().Be(-Vector3d.Right);
        contact.Point3D.X.Should().Be(Fixed64.FromFraction(1, 4));
        contact.Point2D.X.Should().Be(Fixed64.Half);
    }

    [Fact]
    public void CylinderConvexPolygonSlab_WithSeparatedYSlab_ShouldNotCollide()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCylinderCollider> cylinder = CreateCylinder3D(
            context,
            new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero));
        SolidBody2D polygon = CreateBody2D(context, CreateSquarePolygon(), Vector2d.Zero, FixedMath.DegToRad((Fixed64)45));

        CollisionDetectionMixed.TryCollide(cylinder.Collider, polygon.Collider, out MixedContact contact).Should().BeFalse();
        contact.HasContact.Should().BeFalse();
    }

    [Fact]
    public void CompoundCircleSlab_WithSeparatedFirstPart_ShouldUseLaterPartContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCompoundCollider> compound = CreateCompound3D(context, Vector3d.Zero);
        SolidBody2D circle = CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(compound.Collider, circle.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.HasContact.Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        contact.Normal3DTo2D.Should().Be(-Vector3d.Right);
        contact.Point3D.X.Should().Be(Fixed64.FromFraction(1, 4));
        contact.Point2D.X.Should().Be(Fixed64.Half);
    }

    [Fact]
    public void SphereCompound2DSlab_WithSeparatedFirstPart_ShouldUseLaterPartContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(
            context,
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        var compound2D = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)(-4), Fixed64.Zero)),
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero));
        SolidBody2D body2D = CreateBody2D(context, compound2D, Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(sphere.Collider, body2D.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.HasContact.Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        contact.Normal3DTo2D.Should().Be(-Vector3d.Right);
        body2D.Collider.Should().BeSameAs(compound2D);
    }

    [Fact]
    public void TryGetPlanarBoundaryPoint_WithCompoundContainingPoint_ShouldUseContainingPartBoundary()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)(-3), Fixed64.Zero)),
            CompoundColliderPart2D.AABBox(new Vector2d((Fixed64)2, (Fixed64)2), Vector2d.Zero));
        SolidBody2D body = CreateBody2D(context, compound, Vector2d.Zero);

        bool found = MixedEmbedded2DGeometry.TryGetPlanarBoundaryPoint(
            body.Collider,
            new Vector2d(Fixed64.FromFraction(1, 4), Fixed64.Zero),
            out Vector2d boundary,
            out Fixed64 distance);

        found.Should().BeTrue();
        boundary.Should().Be(new Vector2d(Fixed64.One, Fixed64.Zero));
        distance.Should().Be(Fixed64.FromFraction(3, 4));
    }

    [Fact]
    public void TryGetPlanarBoundaryPoint_WithCompoundSeparatedPoint_ShouldFallbackToNearestPartBoundary()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)(-3), Fixed64.Zero)),
            CompoundColliderPart2D.AABBox(new Vector2d((Fixed64)2, (Fixed64)2), new Vector2d((Fixed64)3, Fixed64.Zero)));
        SolidBody2D body = CreateBody2D(context, compound, Vector2d.Zero);

        bool found = MixedEmbedded2DGeometry.TryGetPlanarBoundaryPoint(
            body.Collider,
            new Vector2d(Fixed64.FromFraction(3, 2), Fixed64.Zero),
            out Vector2d boundary,
            out Fixed64 distance);

        found.Should().BeTrue();
        boundary.Should().Be(new Vector2d((Fixed64)2, Fixed64.Zero));
        distance.Should().Be(Fixed64.Half);
    }

    [Fact]
    public void ConvexMeshCircleSlab_WithTriangleCandidateOverlap_ShouldReportContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSMeshCollider> mesh = CreateMesh3D(
            context,
            MeshTestFixtures.CreateConvexCube(),
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D circle = CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(mesh.Collider, circle.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.HasContact.Should().BeTrue();
        contact.Depth.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        contact.Normal3DTo2D.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void ConvexMeshCapsuleSlab_WithTriangleCandidateOverlap_ShouldReportContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSMeshCollider> mesh = CreateMesh3D(
            context,
            MeshTestFixtures.CreateConvexCube(),
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D capsule = CreateBody2D(context, new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3), Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(mesh.Collider, capsule.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.HasContact.Should().BeTrue();
        contact.Depth.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        contact.Normal3DTo2D.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void ConcaveMeshAABoxSlab_WithInsideCornerFeature_ShouldReportStableContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSMeshCollider> mesh = CreateMesh3D(
            context,
            MeshTestFixtures.CreateInsideCorner(inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero);
        SolidBody2D box = CreateBody2D(
            context,
            new LSAABBoxCollider2D(new Vector2d(Fixed64.Half, Fixed64.Half)),
            new Vector2d(Fixed64.FromFraction(1, 4), Fixed64.FromFraction(1, 4)));

        bool firstCollided = CollisionDetectionMixed.TryCollide(mesh.Collider, box.Collider, out MixedContact first);
        bool secondCollided = CollisionDetectionMixed.TryCollide(mesh.Collider, box.Collider, out MixedContact second);

        firstCollided.Should().BeTrue();
        secondCollided.Should().BeTrue();
        first.HasContact.Should().BeTrue();
        first.Depth.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        first.Normal3DTo2D.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
        second.Point3D.Should().Be(first.Point3D);
        second.Point2D.Should().Be(first.Point2D);
        second.Normal3DTo2D.Should().Be(first.Normal3DTo2D);
        second.Depth.Should().Be(first.Depth);
    }

    [Fact]
    public void ConcaveMeshConvexPolygonSlab_WithRotatedPolygonFeature_ShouldReportContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSMeshCollider> mesh = CreateMesh3D(
            context,
            MeshTestFixtures.CreateInsideCorner(inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero);
        SolidBody2D polygon = CreateBody2D(
            context,
            CreateSquarePolygon(),
            new Vector2d(Fixed64.FromFraction(1, 4), Fixed64.FromFraction(1, 4)),
            FixedMath.DegToRad((Fixed64)45));

        bool collided = CollisionDetectionMixed.TryCollide(mesh.Collider, polygon.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.HasContact.Should().BeTrue();
        contact.Depth.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        contact.Normal3DTo2D.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void ConcaveMeshCircleSlab_InOpenChannelGap_ShouldNotCollideWithHullOnly()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSMeshCollider> mesh = CreateMesh3D(
            context,
            MeshTestFixtures.CreateUChannel(inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero);
        SolidBody2D circle = CreateBody2D(context, new LSCircleCollider2D(Fixed64.FromFraction(1, 4)), new Vector2d(Fixed64.Zero, (Fixed64)2));

        CollisionDetectionMixed.TryCollide(mesh.Collider, circle.Collider, out MixedContact contact).Should().BeFalse();
        contact.HasContact.Should().BeFalse();
    }

    private static GravitasWorldContext CreateMixedContext()
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-8), (Fixed64)(-4), (Fixed64)(-8)),
                new Vector3d((Fixed64)8, (Fixed64)4, (Fixed64)8)),
            out _).Should().BeTrue();
        return context;
    }

    private static ScenarioBody<LSSphereCollider> CreateSphere3D(GravitasWorldContext context, Vector3d position)
    {
        return CreateBody3D(context, new LSSphereCollider(), position, FixedQuaternion.Identity);
    }

    private static ScenarioBody<LSCuboidCollider> CreateCuboid3D(
        GravitasWorldContext context,
        Vector3d position,
        FixedQuaternion? rotation = null)
    {
        return CreateBody3D(context, new LSCuboidCollider(), position, rotation ?? FixedQuaternion.Identity);
    }

    private static ScenarioBody<LSCapsuleCollider> CreateCapsule3D(
        GravitasWorldContext context,
        Vector3d position,
        FixedQuaternion? rotation = null)
    {
        var collider = new LSCapsuleCollider
        {
            Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One)
        };
        return CreateBody3D(context, collider, position, rotation ?? FixedQuaternion.Identity);
    }

    private static ScenarioBody<LSCylinderCollider> CreateCylinder3D(
        GravitasWorldContext context,
        Vector3d position,
        FixedQuaternion? rotation = null)
    {
        var collider = new LSCylinderCollider
        {
            Size = new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One)
        };
        return CreateBody3D(context, collider, position, rotation ?? FixedQuaternion.Identity);
    }

    private static ScenarioBody<LSConeCollider> CreateCone3D(
        GravitasWorldContext context,
        Vector3d position,
        FixedQuaternion? rotation = null)
    {
        var collider = new LSConeCollider
        {
            Size = new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One)
        };
        return CreateBody3D(context, collider, position, rotation ?? FixedQuaternion.Identity);
    }

    private static ScenarioBody<LSCompoundCollider> CreateCompound3D(GravitasWorldContext context, Vector3d position)
    {
        var collider = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero)),
            CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero)));
        return CreateBody3D(context, collider, position, FixedQuaternion.Identity);
    }

    private static ScenarioBody<LSMeshCollider> CreateMesh3D(
        GravitasWorldContext context,
        LSMeshCollider collider,
        Vector3d position,
        FixedQuaternion? rotation = null)
    {
        return CreateBody3D(context, collider, position, rotation ?? FixedQuaternion.Identity);
    }

    private static ScenarioBody<TCollider> CreateBody3D<TCollider>(
        GravitasWorldContext context,
        TCollider collider,
        Vector3d position,
        FixedQuaternion rotation)
        where TCollider : LSCollider
    {
        var agent = new TestMatterAgent(context, new FixedTransform(position, rotation, Vector3d.One));
        var body = new SolidBody(agent, collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(position, rotation);
        return new ScenarioBody<TCollider>(body, collider);
    }

    private static SolidBody2D CreateBody2D(
        GravitasWorldContext context,
        LSCollider2D collider,
        Vector2d position,
        Fixed64 rotation = default)
    {
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One));
        var body = new SolidBody2D(agent, collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(position, rotation);
        return body;
    }

    private static LSPolygonCollider2D CreateSquarePolygon() =>
        new(
            new Vector2d(-Fixed64.One, -Fixed64.One),
            new Vector2d(Fixed64.One, -Fixed64.One),
            new Vector2d(Fixed64.One, Fixed64.One),
            new Vector2d(-Fixed64.One, Fixed64.One));
}
