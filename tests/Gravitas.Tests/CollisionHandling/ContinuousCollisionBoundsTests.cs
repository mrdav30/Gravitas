using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ContinuousCollisionBoundsTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void MeshInterval_WithUnrepresentableCapsuleLocalExtent_ShouldRemainUncertified(
        bool capsuleIsSource, bool positiveMotionBound)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var capsule = new LSCapsuleCollider { Radius = Fixed64.One, Size = new Vector3d(2, 3, 2) };
        Initialize(context, capsule, new Vector3d(0, 0, -1_000_000_000),
            FixedQuaternion.FromAxisAngle(Vector3d.Right, Fixed64.Pi * Fixed64.Half),
            Vector3d.One * (Fixed64)2_000_000_000);
        LSMeshCollider mesh = CreateSmallTriangleAtZ(context, (Fixed64)1_500_000_000);

        capsule.ScaledRadius.Should().Be((Fixed64)2_000_000_000);
        capsule.AxisLength.Should().Be((Fixed64)2_000_000_000);
        capsule.Bounds.Intersects(mesh.Bounds).Should().BeTrue();
        // The upper core endpoint is near Z=0; the triangle at Z=1.5 billion
        // is inside its radius-two-billion cap, despite the clipped local extent.
        ContinuousCollisionBounds.AreSeparated(capsuleIsSource ? capsule : mesh,
            capsuleIsSource ? mesh : capsule,
            positiveMotionBound ? Fixed64.Half : Fixed64.Zero).Should().BeFalse();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MixedMeshInterval_WithUnrepresentableCapsuleLocalExtent_ShouldRemainUncertified(
        bool positiveMotionBound)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        var capsule = new LSCapsuleCollider2D(Fixed64.One, (Fixed64)3)
        {
            MixedHalfThicknessOverride = Fixed64.Half
        };
        capsule.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(new Vector3d(0, 0, -1_000_000_000), FixedQuaternion.Identity,
                Vector3d.One * (Fixed64)2_000_000_000)));
        LSMeshCollider mesh = CreateSmallTriangleAtZ(context, (Fixed64)1_500_000_000);

        capsule.ScaledRadius.Should().Be((Fixed64)2_000_000_000);
        capsule.AxisLength.Should().Be((Fixed64)2_000_000_000);
        mesh.Bounds.Intersects(capsule.MixedBounds3D).Should().BeTrue();
        ContinuousCollisionBounds.AreSeparated(mesh, capsule,
            positiveMotionBound ? Fixed64.Half : Fixed64.Zero).Should().BeFalse();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ClosedConvexMesh_WithContainedCollider_ShouldNotCertifyAnEmptyInterval(bool mixed)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        LSMeshCollider mesh = MeshTestFixtures.CreateConvexCube();
        Initialize(context, mesh, Vector3d.Zero);
        mesh.IsClosedSurface.Should().BeTrue();
        if (mixed)
        {
            var circle = new LSCircleCollider2D(Fixed64.FromFraction(1, 10))
            {
                MixedHalfThicknessOverride = Fixed64.FromFraction(1, 10)
            };
            circle.InitializeWithNoBody(new TestMatterAgent(context));
            ContinuousCollisionBounds.AreSeparated(mesh, circle, Fixed64.Zero).Should().BeFalse();
        }
        else
        {
            var box = new LSCuboidCollider { Size = Vector3d.One * Fixed64.FromFraction(1, 5) };
            Initialize(context, box, Vector3d.Zero);
            ContinuousCollisionBounds.AreSeparated(box, mesh, Fixed64.Zero).Should().BeFalse();
            ContinuousCollisionBounds.AreSeparated(mesh, box, Fixed64.Zero).Should().BeFalse();
        }
    }

    [Theory]
    [InlineData(ColliderType2D.Circle)]
    [InlineData(ColliderType2D.Capsule)]
    [InlineData(ColliderType2D.AABox)]
    [InlineData(ColliderType2D.ConvexPolygon)]
    public void MixedMeshInterval_EmptyFiniteTriangleCorner_ShouldCertifySeparation(ColliderType2D shape)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        var mesh = new LSMeshCollider(new[] { Vector3d.Zero, new Vector3d(10, 0, 0),
            new Vector3d(0, 0, 10) }, new[] { 0, 1, 2 }, MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);
        Initialize(context, mesh, Vector3d.Zero);
        Fixed64 tenth = Fixed64.FromFraction(1, 10);
        LSCollider2D embedded = shape switch
        {
            ColliderType2D.Circle => new LSCircleCollider2D(tenth),
            ColliderType2D.Capsule => new LSCapsuleCollider2D(tenth, tenth * (Fixed64)4),
            ColliderType2D.AABox => new LSAABBoxCollider2D(Vector2d.One * tenth * Fixed64.Two),
            _ => new LSPolygonCollider2D(new Vector2d(-tenth, -tenth), new Vector2d(tenth, -tenth),
                new Vector2d(tenth, tenth), new Vector2d(-tenth, tenth))
        };
        embedded.MixedHalfThicknessOverride = tenth;
        embedded.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(new Vector3d(9, 0, 9), FixedQuaternion.Identity, Vector3d.One)));

        mesh.Bounds.Intersects(embedded.MixedBounds3D).Should().BeTrue();
        // Every triangle point has X+Z <= 10; this slab stays beyond X+Z=17.
        ContinuousCollisionBounds.AreSeparated(mesh, embedded, tenth).Should().BeTrue();
        CollisionDetectionMixed.IntersectsForContinuousCollision(mesh, embedded).Should().BeFalse();
        mesh = new LSMeshCollider(new[] { Vector3d.Zero, Vector3d.Right * (Fixed64)10,
            Vector3d.Forward * (Fixed64)10 }, new[] { 0, 1, 2 }, MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);
        Initialize(context, mesh, new Vector3d(8, 0, 8));
        // The same four slabs now intersect the finite face; Boolean admission
        // must agree with the full retained surface manifold.
        CollisionDetectionMixed.IntersectsForContinuousCollision(mesh, embedded).Should().BeTrue();
        CollisionDetectionMixed.TryCollide(mesh, embedded, out _).Should().BeTrue();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MixedMeshAdmission_CompoundSlab_ShouldRetainOrdinaryManifoldDecision(bool overlaps)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        LSMeshCollider mesh = CreateSmallTriangleAtZ(context, Fixed64.Zero);
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.AABBox(Vector2d.One, Vector2d.Zero));
        compound.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(overlaps ? Vector3d.Zero : Vector3d.Right * (Fixed64)10,
                FixedQuaternion.Identity, Vector3d.One)));

        CollisionDetectionMixed.IntersectsForContinuousCollision(mesh, compound).Should().Be(overlaps);
        CollisionDetectionMixed.TryCollide(mesh, compound, out _).Should().Be(overlaps);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MeshInterval_WithUnavailableWorldVertex_ShouldUseFiniteLocalGeometry(bool negativeLimit)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 sign = negativeLimit ? -Fixed64.One : Fixed64.One;
        Fixed64 originX = negativeLimit ? Fixed64.MinValue + Fixed64.Half : Fixed64.MaxValue - Fixed64.Half;
        var mesh = new LSMeshCollider(new[] { Vector3d.Zero, Vector3d.Right * sign,
            Vector3d.Forward }, new[] { 0, 1, 2 }, MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);
        Initialize(context, mesh, new Vector3d(originX, Fixed64.Zero, Fixed64.Zero));
        var box = new LSCuboidCollider { Size = Vector3d.One * Fixed64.FromFraction(1, 10) };
        Initialize(context, box, new Vector3d(originX + sign * Fixed64.FromFraction(2, 5),
            Fixed64.Zero, Fixed64.FromFraction(9, 10)));

        mesh.Mesh.TryGetVertexWorld(1, out _).Should().BeFalse();
        mesh.Bounds.Intersects(box.Bounds).Should().BeTrue();
        ContinuousCollisionBounds.AreSeparated(box, mesh, Fixed64.FromFraction(1, 100)).Should().BeTrue();
        ContinuousCollisionBounds.AreSeparated(mesh, box, Fixed64.FromFraction(1, 100)).Should().BeTrue();
    }

    [Fact]
    public void MeshInterval_WithAdmittedTriangleBelowPublicDegeneracyThreshold_ShouldRemainUncertified()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 height = Fixed64.Epsilon * Fixed64.Two + Fixed64.FromRaw(3);
        var mesh = new LSMeshCollider(new[] { Vector3d.Zero, Vector3d.Right, Vector3d.Up * height },
            new[] { 0, 1, 2 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        Initialize(context, mesh, Vector3d.Zero);
        var sphere = new LSSphereCollider { Radius = Fixed64.FromFraction(1, 10) };
        Initialize(context, sphere, new Vector3d(Fixed64.FromFraction(1, 4), height / (Fixed64)4, Fixed64.Zero));
        mesh.Mesh.GetLocalTriangleVertices(0, out Vector3d a, out Vector3d b, out Vector3d c);

        mesh.Area.Should().BeGreaterThan(Fixed64.Epsilon);
        new FixedTriangle(a, b, c).IsDegenerate.Should().BeTrue();
        ContinuousCollisionBounds.AreSeparated(sphere, mesh, Fixed64.Zero).Should().BeFalse();
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void MeshInterval_WithZeroWidthPlanarEnclosure_ShouldRemainUncertified(int zeroAxis)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Vector3d first = zeroAxis == 0 ? Vector3d.Up : Vector3d.Right;
        Vector3d second = zeroAxis == 2 ? Vector3d.Up : Vector3d.Forward;
        var source = new LSMeshCollider(new[] { Vector3d.Zero, first, second }, new[] { 0, 1, 2 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        var target = new LSMeshCollider(new[] { Vector3d.Zero, first, second }, new[] { 0, 1, 2 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        Initialize(context, source, Vector3d.Zero);
        Initialize(context, target, Vector3d.Zero);

        ContinuousCollisionBounds.AreSeparated(source, target, Fixed64.Zero).Should().BeFalse();
    }

    [Fact]
    public void MeshInterval_WithUnrepresentableExpandedHalfExtent_ShouldRemainUncertified()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var source = new LSSphereCollider { Radius = Fixed64.MaxValue };
        Initialize(context, source, Vector3d.Zero);
        LSMeshCollider target = CreateSmallTriangleAtZ(context, Fixed64.Zero);

        // Expanding the admitted radius by one spans the full scalar domain;
        // no representable centered half-extent encloses both clipped endpoints.
        ContinuousCollisionBounds.AreSeparated(source, target, Fixed64.One).Should().BeFalse();
    }

    [Fact]
    public void CuboidMeshInterval_WithUnrepresentableExpandedRigidExtent_ShouldRemainUncertified()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var box = new LSCuboidCollider();
        Initialize(context, box, Vector3d.Zero);
        LSMeshCollider mesh = CreateSmallTriangleAtZ(context, Fixed64.Zero);

        ContinuousCollisionBounds.AreSeparated(box, mesh, Fixed64.MaxValue).Should().BeFalse();
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
    public void CompoundMeshInterval_ShouldRequireSeparationFromEveryFinitePart(
        bool compoundIsSource, bool otherIsCompound, bool overlapsMesh)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 small = Fixed64.FromFraction(1, 10);
        // Mesh parts replace their default offset; restore the authored bounds center.
        var meshCompound = new LSCompoundCollider(
            CompoundColliderPart.ConvexMesh(new[] { Vector3d.Zero, new Vector3d(10, 0, 0),
                new Vector3d(0, 10, 0) }, new[] { 0, 1, 2 }, new Vector3d(5, 5, 0),
                MeshInertiaPolicy.SurfaceApproximation),
            CompoundColliderPart.Cuboid(Vector3d.One * small, new Vector3d(-10, -10, 0)));
        Initialize(context, meshCompound, Vector3d.Zero);
        Vector3d position = overlapsMesh ? new Vector3d(2, 2, 0) : new Vector3d(9, 9, 0);
        LSCollider other;
        if (otherIsCompound)
        {
            other = new LSCompoundCollider(
                CompoundColliderPart.Cuboid(Vector3d.One * small, Vector3d.Zero),
                CompoundColliderPart.Cuboid(Vector3d.One * small, new Vector3d(20, 20, 0)));
        }
        else
        {
            other = new LSCuboidCollider { Size = Vector3d.One * small };
        }
        Initialize(context, other, position);

        meshCompound.Bounds.Intersects(other.Bounds).Should().BeTrue();
        ContinuousCollisionBounds.AreSeparated(compoundIsSource ? meshCompound : other,
            compoundIsSource ? other : meshCompound, small).Should().Be(!overlapsMesh);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MixedMeshInterval_WithUnrepresentableOriginCenteredPolygonExtent_ShouldRemainUncertified(
        bool minimumOnZ)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        Vector2d[] vertices = minimumOnZ
            ? new[] { new Vector2d(-Fixed64.Half, Fixed64.MinValue),
                new Vector2d(Fixed64.Half, Fixed64.MinValue), new Vector2d(Fixed64.Half, Fixed64.Zero),
                new Vector2d(-Fixed64.Half, Fixed64.Zero) }
            : new[] { new Vector2d(Fixed64.MinValue, -Fixed64.Half),
                new Vector2d(Fixed64.Zero, -Fixed64.Half), new Vector2d(Fixed64.Zero, Fixed64.Half),
                new Vector2d(Fixed64.MinValue, Fixed64.Half) };
        var polygon = new LSPolygonCollider2D(vertices) { MixedHalfThicknessOverride = Fixed64.Half };
        polygon.InitializeWithNoBody(new TestMatterAgent(context));
        LSMeshCollider mesh = CreateSmallTriangleAtZ(context,
            minimumOnZ ? Fixed64.FromFraction(-1, 10) : Fixed64.Zero);

        polygon.ContainsPoint(new Vector2d(Fixed64.FromFraction(-1, 10),
            Fixed64.FromFraction(-1, 10))).Should().BeTrue();
        mesh.Bounds.Intersects(polygon.MixedBounds3D).Should().BeTrue();
        // -MinValue is outside the scalar domain: a saturated symmetric box
        // cannot replace the admitted polygon's conceptual local extent.
        ContinuousCollisionBounds.AreSeparated(mesh, polygon, Fixed64.Zero).Should().BeFalse();
    }

    private static LSMeshCollider CreateSmallTriangleAtZ(GravitasWorldContext context, Fixed64 z)
    {
        var mesh = new LSMeshCollider(new[] { new Vector3d(-Fixed64.Half, -Fixed64.FromFraction(1, 4), z),
            new Vector3d(Fixed64.Half, -Fixed64.FromFraction(1, 4), z),
            new Vector3d(Fixed64.Zero, Fixed64.FromFraction(1, 4), z) }, new[] { 0, 1, 2 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        Initialize(context, mesh, Vector3d.Zero);
        return mesh;
    }

    private static void Initialize(GravitasWorldContext context, LSCollider collider, Vector3d position,
        FixedQuaternion? rotation = null, Vector3d? scale = null) =>
        collider.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(position, rotation ?? FixedQuaternion.Identity, scale ?? Vector3d.One)));
}
