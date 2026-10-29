using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Gravitas.Queries;
using Gravitas.Support;
using System.Numerics;
using Xunit;

namespace Gravitas.Tests;

public sealed class StaticSurfaceNormalAdmissionTests
{
    [Fact]
    public void DeclaredCircleContacts_ShouldRetainClosingFacesBeyondTheQueryRepresentative()
    {
        using var scene = PhysicsScenarioBuilder.Create(); scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        var square = new LSAABBoxCollider2D(Vector2d.One);
        square.InitializeWithNoBody(new TestMatterAgent(scene.Context,
            new FixedTransform(new(Fixed64.Half, Fixed64.Zero, Fixed64.Half), FixedQuaternion.Identity, Vector3d.One)));
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { square });
        var circle = new LSCircleCollider2D(Fixed64.FromFraction(3, 5));
        var body = new SolidBody2D(new TestMatterAgent(scene.Context), circle) { Mass = Fixed64.One };
        body.Initialize(new(-Fixed64.FromFraction(1, 10), Fixed64.Half));
        Assert.True(QueryDetection2D.TrySweepMoverShape(circle, Vector2d.Backward, square, out var hit));
        Assert.Equal(Fixed64.Zero, Vector2d.Dot(hit.Normal, Vector2d.Backward));
        var native = new ContactManifold2D();
        Assert.True(CollisionDetection2D.TryCollide(new CollisionWorkItem2D(circle, square, CollisionType2D.Circle_Convex),
            native, scene.Context.FrameCount));
        bool additionalClosingFace = false;
        foreach (var row in native) additionalClosingFace |= Vector2d.Dot(-row.Normal, Vector2d.Backward) < -Fixed64.Epsilon;
        Assert.True(additionalClosingFace);
    }

    [Theory]
    [InlineData(-1)] [InlineData(0)] [InlineData(1)]
    public void SweptSphereAdmission_ShouldOrientItsGeometricPlaneNormalAgainstApproach(int side)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var mesh = MeshTestFixtures.CreateConvexQuadFloor(MeshColliderMode.Concave);
        scene.InitializeStaticCollider(mesh, Vector3d.Zero);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { mesh });
        var worker = new SweptSphereQueryWorker();
        worker.Prepare(Vector3d.Up * (Fixed64)side,
            Vector3d.Up * (Fixed64)(side == 0 ? 2 : -side), Fixed64.Half);
        ContinuousSurfaceTriangleAdmission admission = (triangle, ref hit, axis) => true;
        Assert.True(worker.TrySweepMeshForContinuousCollision(mesh, admission, out Physics3DHit hit));
        Assert.Same(mesh, hit.Collider);
        Assert.Equal(side <= 0 ? Vector3d.Down : Vector3d.Up, hit.Normal);
        Assert.Equal(side == 0 ? Fixed64.Zero : Fixed64.Half, hit.Distance);
    }

    [Fact]
    public void CapsuleSweepAxis_ShouldRequireItsWitnessOnTheOriginalFiniteTriangle()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var mesh = MeshTestFixtures.CreateConvexQuadFloor(MeshColliderMode.Concave);
        scene.InitializeStaticCollider(mesh, Vector3d.Zero);
        var capsule = scene.CreateBody(new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) },
            Vector3d.Up, FixedQuaternion.Identity).Collider;
        mesh.Mesh.GetGeometryTriangleVertices(0, out var a, out var b, out var c);
        var source = new ConvexShape(capsule, Vector3d.Zero);
        var target = new ConvexShape(mesh, 0, a, b, c);
        Assert.False(source.TryGetCapsuleTriangleNormalAxis(target,
            new FixedPointAnchor(Vector3d.One * (Fixed64)16, FixedQuaternion.Identity, Vector3d.Zero), out _));
    }

    [Theory]
    [InlineData(-1)] [InlineData(1)]
    public void CircleSlabWitnessOutsideItsCap_ShouldRemainConservative(int side)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        LSMeshCollider left = Sheet(-2, 0), right = Sheet(0, 2);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { left, right });
        var circle = new LSCircleCollider2D(Fixed64.Quarter) { MixedHalfThicknessOverride = Fixed64.Half };
        var body = new SolidBody2D(new TestMatterAgent(scene.Context), circle) { Mass = Fixed64.One };
        body.Initialize(new(Fixed64.Zero, Fixed64.Quarter));
        Vector3d center = new(Fixed64.Zero, Fixed64.Zero, Fixed64.Quarter);
        var cap = ContactAnchor.FromWorldPoint(new(Fixed64.Zero,
            (Fixed64)side * (Fixed64.Half + Fixed64.FromRaw(1)), Fixed64.Zero));
        Assert.True(surface.AdmitContinuousWitness(null, circle, center, left, 1, cap,
            circleSlabHalfThickness: Fixed64.Half));
        body.SetPosition(new((Fixed64)10, Fixed64.Quarter));
        Assert.True(surface.AdmitContinuousWitness(null, circle, center + Vector3d.Right * (Fixed64)10,
            left, 1, cap));

        LSMeshCollider Sheet(int first, int last)
        {
            var mesh = new LSMeshCollider(new[] { new Vector3d(first, -2, 0), new(first, 2, 0),
                new(last, -2, 0), new(last, 2, 0) }, new[] { 0, 1, 2, 2, 1, 3 }, MeshColliderMode.Concave,
                MeshInertiaPolicy.SurfaceApproximation);
            scene.InitializeStaticCollider(mesh, Vector3d.Zero);
            return mesh;
        }
    }

    [Theory]
    [InlineData(0)] [InlineData(1)] [InlineData(2)]
    public void PlanarBoundaryAdmission_ShouldKeepExposedAndUnresolvedWitnesses(int shape)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        LSAABBoxCollider2D left = Floor(-2), right = Floor(2), remote = Floor(10);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right, remote });
        LSCollider2D mover = shape == 0 ? new LSCircleCollider2D(Fixed64.Half)
            : shape == 1 ? new LSCapsuleCollider2D(Fixed64.Half, Fixed64.One)
            : new LSAABBoxCollider2D(Vector2d.One);
        var body = new SolidBody2D(new TestMatterAgent(scene.Context), mover) { Mass = Fixed64.One };
        body.Initialize(new(Fixed64.Quarter, shape == 1 ? Fixed64.One : Fixed64.Half));
        var exposed = new ContactAnchor2D(new Vector2d(Fixed64.Quarter, Fixed64.Zero), Vector2d.Zero);
        Assert.True(surface.AdmitContinuousWitness(mover, right, exposed));
        Assert.True(surface.AdmitContinuousWitness(mover, right, exposed));
        // An unresolved future supplier or interior witness cannot certify exclusion.
        Assert.True(surface.AdmitContinuousWitness(mover, remote, new(new Vector2d(10, 0), Vector2d.Zero)));
        Assert.True(surface.AdmitContinuousWitness(mover, right, new(new Vector2d(1, -1), Vector2d.Zero)));
        body.SetPosition(new(Fixed64.FromRaw(1), shape == 1 ? Fixed64.One : Fixed64.Half));
        Assert.True(surface.AdmitContinuousWitness(mover, right, new(Vector2d.Zero, Vector2d.Zero)),
            "A materialized seam point with a tangential error cannot exclude its own face.");

        LSAABBoxCollider2D Floor(int x)
        {
            var floor = new LSAABBoxCollider2D(new Vector2d(4, 2));
            floor.InitializeWithNoBody(new TestMatterAgent(scene.Context,
                new FixedTransform(new(x, 0, -1), FixedQuaternion.Identity, Vector3d.One)));
            return floor;
        }
    }

    [Theory]
    [InlineData(false, 0)] [InlineData(true, 0)] [InlineData(false, -1)] [InlineData(false, 1)]
    public void MixedBoundaryAdmission_ShouldKeepTheExposedFaceAcrossAnIncidentJoin(bool elevatedNeighbor, int heightBand)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        LSAABBoxCollider2D left = Floor(-2, 0), right = Floor(2, elevatedNeighbor ? 2 : 0), remote = Floor(10, 0);
        if (heightBand != 0) left.MixedHalfThicknessOverride = Fixed64.Two;
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right, remote });
        Fixed64 y = (Fixed64)heightBand * (Fixed64.One + Fixed64.Half);
        var actor = scene.CreateBody(new LSSphereCollider { Radius = Fixed64.Half },
            new(Fixed64.Zero, y, Fixed64.Half), FixedQuaternion.Identity);
        var sphere = actor.Collider;
        Assert.True(scene.Context.QueryMixed.SweepSphereAgainst2D(sphere.Center, sphere.Center + Vector3d.Backward,
            Fixed64.Half, PhysicsLayerMask.All, out PhysicsMixedHit hit));
        Assert.True(surface.AdmitMixedContinuousWitness(sphere, sphere.Center, left, hit.Anchor2D));
        actor.Body.SetPosition(new(-Fixed64.One, y, Fixed64.Half));
        Assert.True(surface.AdmitMixedContinuousWitness(sphere, sphere.Center, left,
            ContactAnchor.FromWorldPoint(new(-Fixed64.One, y, Fixed64.Zero))));
        Assert.True(surface.AdmitMixedContinuousWitness(sphere, sphere.Center, remote,
            ContactAnchor.FromWorldPoint(new(10, 0, 0))));
        Assert.True(surface.AdmitMixedContinuousWitness(sphere, sphere.Center, left,
            ContactAnchor.FromWorldPoint(new(-1, 1, -1))));
        actor.Body.SetPosition(new(Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Half));
        Assert.True(surface.AdmitMixedContinuousWitness(sphere, sphere.Center, right,
            ContactAnchor.FromWorldPoint(Vector3d.Zero)));
        Assert.True(surface.AdmitMixedContinuousWitness(sphere, sphere.Center, left,
            ContactAnchor.FromWorldPoint(new(0, -2, 0))));
        Assert.True(surface.AdmitMixedContinuousWitness(sphere, sphere.Center, left,
            ContactAnchor.FromWorldPoint(new(0, 2, 0))));

        LSAABBoxCollider2D Floor(int x, int height)
        {
            var floor = new LSAABBoxCollider2D(new Vector2d(4, 2)) { MixedHalfThicknessOverride = Fixed64.One };
            floor.InitializeWithNoBody(new TestMatterAgent(scene.Context,
                new FixedTransform(new(x, height, -1), FixedQuaternion.Identity, Vector3d.One)));
            return floor;
        }
    }

    [Fact]
    public void UncertifiedRoundedNativeFaceNormal_ShouldConservativelyRetainTheContinuousCandidate()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        const long a = 2354819978, b = 844583451, c = 3242482676;
        Vector3d edge = Raw(-b, a, 0), interior = Raw(-c, 0, a);
        long k = (1L << 32) / 5;
        var rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(3 * k), Fixed64.FromRaw(4 * k));
        LSMeshCollider left = Triangle(edge, interior), right = Triangle(-interior, edge);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { left, right });
        var mover = new LSSphereCollider { Radius = Fixed64.Quarter };
        scene.CreateBody(mover, Vector3d.Zero, FixedQuaternion.Identity);
        Vector3d normal = left.Mesh.GetFaceNormalWorld(0);

        // The exact rotation is (7 -24;24 7)/25. The exact local face
        // normal is parallel (a,b,c), so its dot with this rotated edge is zero.
        // Cached local normalization followed by rounded rotation can exceed a
        // half-raw WORLD component certificate even for this valid face normal.
        BigInteger tangentX = -7 * (BigInteger)b - 24 * (BigInteger)a;
        BigInteger tangentY = -24 * (BigInteger)b + 7 * (BigInteger)a;
        BigInteger projection = tangentX * normal.X.m_rawValue + tangentY * normal.Y.m_rawValue;
        Assert.True(2 * projection > BigInteger.Abs(tangentX) + BigInteger.Abs(tangentY),
            "The fixture must exercise the accumulated local-normal and world-rotation rounding.");
        Assert.True(surface.AdmitContinuousWitness(mover, null, Vector3d.Zero, left, 0,
            ContactAnchor.FromWorldPoint(Vector3d.Zero)));

        LSMeshCollider Triangle(Vector3d second, Vector3d third)
        {
            var mesh = new LSMeshCollider(new[] { Vector3d.Zero, second, third }, new[] { 0, 1, 2 },
                MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
            mesh.InitializeWithNoBody(new TestMatterAgent(scene.Context,
                new FixedTransform(Vector3d.Zero, rotation, Vector3d.One)));
            return mesh;
        }
        static Vector3d Raw(long x, long y, long z) =>
            new(Fixed64.FromRaw(x), Fixed64.FromRaw(y), Fixed64.FromRaw(z));
    }
}
