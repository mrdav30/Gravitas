using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests;

public sealed class MixedContinuousSectionAdmissionTests
{
    [Theory]
    [InlineData("Sphere", false)] [InlineData("Sphere", true)]
    [InlineData("Capsule", false)] [InlineData("Capsule", true)]
    [InlineData("Cone", false)] [InlineData("Cone", true)]
    [InlineData("Cylinder", false)] [InlineData("Cylinder", true)]
    [InlineData("Box", false)] [InlineData("Box", true)]
    [InlineData("Mesh", false)] [InlineData("Mesh", true)]
    public void RemoteFold_ShouldKeepItsRealEdgeWithoutRevokingTheCertifiedFloorJoin(string shape, bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.ThreeD;
        var (left, right) = FoldedFloor(scene, horizontal: true);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { left, right });
        LSCollider mover = NativeMover(shape);
        Fixed64 height = shape is "Capsule" or "Cone" or "Cylinder" ? Fixed64.FromFraction(3, 4) : Fixed64.Quarter;
        scene.CreateBody(mover, new(Fixed64.Zero, height, Fixed64.Zero), FixedQuaternion.Identity);
        int carriers = 0;
        Fixed64 maximumDepth = Fixed64.Zero;
        foreach (LSMeshCollider member in new[] { left, right })
        {
            CollisionPair pair = reverse ? scene.CreatePair(mover, member) : scene.CreatePair(member, mover);
            Assert.True(CollisionDetection.DoCollisionCheck(pair));
            if (pair.Manifold.HasContact) carriers++;
            foreach (ManifoldContact row in pair.Manifold)
            {
                bool memberFirst = pair.ColliderA == member;
                Assert.Equal(Vector3d.Up, memberFirst ? row.Normal : -row.Normal);
                // Finite curved samples include exact rim tangencies alongside
                // the deepest support. Their anchors still lie on this floor.
                Assert.InRange(row.Depth.m_rawValue, Fixed64.Zero.m_rawValue, Fixed64.Quarter.m_rawValue);
                maximumDepth = FixedMath.Max(maximumDepth, row.Depth);
                Assert.True((memberFirst ? row.AnchorA : row.AnchorB).TryGetWorldPoint(out Vector3d point));
                Assert.Equal(Fixed64.Zero, point.Y);
            }
        }
        Assert.Equal(Fixed64.Quarter, maximumDepth);
        Assert.Equal(1, carriers);
    }

    [Fact]
    public void JoinedConeFace_ShouldRecoverOnceLikeTheSameMonolithicSurface()
    {
        var states = new (Vector3d Position, Vector3d Velocity)[2, 3];
        foreach (bool joined in new[] { false, true })
        {
            using var scene = PhysicsScenarioBuilder.Create();
            scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.ThreeD;
            scene.Context.SetFrameRate(60);
            scene.Context.Environment.MaxSpeed = scene.Context.Environment.MaxFallSpeed = (Fixed64)1024;
            scene.Context.Environment.Gravity = scene.Context.Environment.AirDensity =
                scene.Context.Environment.DampingFactor = scene.Context.Environment.MinSpeed = Fixed64.Zero;
            LSMeshCollider[] members;
            if (joined)
            {
                var pieces = FoldedFloor(scene, horizontal: true);
                members = new[] { pieces.Left, pieces.Right };
            }
            else
            {
                Fixed64 x = Fixed64.FromFraction(3, 4);
                var monolithic = new LSMeshCollider(new[] { new Vector3d(-8, 0, 2), new(-8, 0, -2), new(0, 0, 2), new(0, 0, -2),
                    new(0, 0, 2), new(0, 0, -2), new(x, Fixed64.Zero, (Fixed64)2), new(x, Fixed64.Zero, -(Fixed64)2),
                    new(x, (Fixed64)4, (Fixed64)2), new(x, (Fixed64)4, -(Fixed64)2) },
                    new[] { 0, 1, 2, 2, 1, 3, 4, 5, 6, 6, 5, 7, 6, 7, 8, 8, 7, 9 },
                    MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation) { Material = PhysicsMaterial.Frictionless };
                scene.InitializeStaticCollider(monolithic, Vector3d.Zero);
                members = new[] { monolithic };
            }
            using var surface = scene.Context.Physics.CreateStaticSurface(members);
            SolidBody body = scene.CreateBody(NativeMover("Cone"), new(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero),
                FixedQuaternion.Identity, preventAngularForces: true).Body;
            body.UseManualGrounding();
            body.AddLinearImpulse(Vector3d.Down);
            for (int frame = 0; frame < 3; frame++)
            {
                scene.Context.Simulate(); scene.Context.LateSimulate();
                states[joined ? 1 : 0, frame] = (body.Position3d, body.LinearVelocity);
                Assert.True(body.Position3d.Y < Fixed64.One, "Shared floor response must not overrecover past the cone's physical support.");
            }
        }
        for (int frame = 0; frame < 3; frame++) Assert.Equal(states[0, frame], states[1, frame]);
    }

    [Fact]
    public void JoinedConeFace_ShouldKeepTheActualFiniteSupplierWhenOnePairOwnsResponse()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var (left, right) = FoldedFloor(scene, horizontal: true);
        right.Material = PhysicsMaterial.Bouncy;
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { left, right });
        LSCollider cone = NativeMover("Cone");
        scene.CreateBody(cone, Vector3d.Up * Fixed64.FromFraction(3, 4), FixedQuaternion.Identity);
        CollisionPair carrier = scene.CreatePair(left, cone);
        Assert.True(CollisionDetection.DoCollisionCheck(carrier));
        bool borrowed = false;
        for (int index = 0; index < carrier.Manifold.GroupCount; index++)
        {
            ref ContactGroup group = ref carrier.Manifold.GetGroup(index);
            bool fromRight = group.Key.SourceIdA == right.Id + 1;
            borrowed |= fromRight;
            Assert.True(fromRight || group.Key.SourceIdA == left.Id + 1);
            for (int sample = 0; sample < group.Count; sample++)
            {
                ManifoldContact row = group[sample];
                Assert.Equal(fromRight ? right.Material : left.Material, row.MaterialA);
                Assert.True(row.AnchorA.TryGetWorldPoint(out Vector3d point));
                LSMeshCollider supplier = fromRight ? right : left;
                Assert.InRange(point.X.m_rawValue, supplier.BoundsMin.X.m_rawValue - 1, supplier.BoundsMax.X.m_rawValue + 1);
            }
        }
        Assert.True(borrowed);
        CollisionPair other = scene.CreatePair(right, cone);
        Assert.True(CollisionDetection.DoCollisionCheck(other));
        Assert.False(other.Manifold.HasContact);
    }

    [Theory]
    [InlineData(false)] [InlineData(true)]
    public void JoinedCylinderOuterEndpoint_ShouldKeepTheTrueRimAcrossAuthoredVertexOrders(bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var left = Floor(-8, 0); var right = Floor(0, 8);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { left, right });
        var cylinder = new LSCylinderCollider { Radius = Fixed64.Half, Size = Vector3d.One };
        scene.CreateBody(cylinder, new(Fixed64.Zero, Fixed64.Quarter, -(Fixed64)2 - Fixed64.Quarter), FixedQuaternion.Identity);
        bool rim = false;
        Fixed64 maximumDepth = Fixed64.Zero;
        foreach (LSMeshCollider member in new[] { left, right })
        {
            CollisionPair pair = scene.CreatePair(member, cylinder);
            Assert.True(CollisionDetection.DoCollisionCheck(pair));
            for (int groupIndex = 0; groupIndex < pair.Manifold.GroupCount; groupIndex++)
            {
                ref ContactGroup group = ref pair.Manifold.GetGroup(groupIndex);
                for (int index = 0; index < group.Count; index++)
                {
                    ManifoldContact row = group[index];
                    maximumDepth = FixedMath.Max(maximumDepth, row.Depth);
                    if (group.Key.Region >= 0) continue;
                    rim = true;
                    Assert.Equal(Fixed64.Zero, row.Normal.X);
                    Assert.InRange(row.Depth.m_rawValue, 0L, Fixed64.Quarter.m_rawValue);
                    Assert.True(row.AnchorA.TryGetWorldPoint(out Vector3d point));
                    Assert.Equal(Fixed64.Zero, point.Y);
                    Assert.Equal(-(Fixed64)2, point.Z);
                }
            }
        }
        Assert.True(rim, "The shared outer endpoint is a physical rim, not an internal seam.");
        Assert.Equal(Fixed64.Quarter, maximumDepth);

        LSMeshCollider Floor(int minX, int maxX)
        {
            var vertices = new[] { new Vector3d(minX, 0, -2), new(minX, 0, 2), new(maxX, 0, -2), new(maxX, 0, 2) };
            if (reverse) System.Array.Reverse(vertices);
            var mesh = new LSMeshCollider(vertices, new[] { 0, 1, 2, 2, 1, 3 },
                MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
            scene.InitializeStaticCollider(mesh, Vector3d.Zero);
            return mesh;
        }
    }

    [Fact]
    public void ContinuousFamilies_ShouldCaptureTheClosingWallWithoutSeedingTheTouchingFloor()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        var (left, right) = FoldedFloor(scene);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { left, right });
        var capsule = new LSCapsuleCollider2D(Fixed64.Half, Fixed64.Two) { MixedHalfThicknessOverride = Fixed64.Half };
        var body = new SolidBody2D(new TestMatterAgent(scene.Context), capsule) { Mass = Fixed64.One };
        body.Initialize(new(Fixed64.Half, Fixed64.One));
        body.AddLinearImpulse(Vector2d.Right);
        body.AddAngularImpulse(Fixed64.FromFraction(1, 64));
        var manifold = new ContactManifold(); manifold.BeginUpdate(scene.Context.FrameCount);
        surface.AppendMixedContinuousContacts(capsule, manifold);
        Assert.Contains(manifold, row => row.Normal == Vector3d.Forward && row.Depth == Fixed64.Zero);
        Assert.Contains(manifold, row => row.Normal == Vector3d.Left && row.Depth > Fixed64.Zero);
        Vector3d center = new(body.Position.X, capsule.MixedSlabCenterY, body.Position.Y);
        var impact = new ContactResponseSnapshot(left.Center, center, ContactAnchor.FromWorldPoint(left.Center),
            ContactAnchor.FromWorldPoint(center), Vector3d.Zero, Vector3d.Zero,
            new(body.LinearVelocity.X, Fixed64.Zero, body.LinearVelocity.Y),
            new(Fixed64.Zero, -body.AngularVelocity, Fixed64.Zero), body.AngularVelocity);
        var families = new SwiftCollections.SwiftList<(LSMeshCollider Supplier, int Owner)>();
        surface.CaptureMixedContinuousFamilies(manifold, impact, capsule, families);
        Assert.Single(families);
        Assert.Same(right, families[0].Supplier);
        Assert.Equal(right.Mesh.GetManifoldSurfaceOwner(2), families[0].Owner);
    }

    [Theory]
    [InlineData(false)] [InlineData(true)]
    public void DisconnectedDomainMembers_ShouldKeepTheirOwnMaterialsAndGroupsWhenCcdAggregatesOrdinaryContacts(bool compound)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        var left = Rectangle(-Fixed64.FromFraction(3, 2), PhysicsMaterial.Frictionless);
        var right = Rectangle(Fixed64.FromFraction(3, 2), PhysicsMaterial.Bouncy);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right });
        LSCollider mover = compound
            ? new LSCompoundCollider(CompoundColliderPart.Sphere(Fixed64.Two, Vector3d.Zero, PhysicsMaterial.Frictionless))
                { Material = PhysicsMaterial.Bouncy }
            : new LSSphereCollider { Radius = Fixed64.Two, Material = PhysicsMaterial.Frictionless };
        scene.CreateBody(mover, new(Fixed64.Zero, Fixed64.Zero, Fixed64.Half), FixedQuaternion.Identity);
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, left, new ContactManifold()));
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, right, new ContactManifold()));
        var aggregated = new ContactManifold();
        aggregated.BeginUpdate(scene.Context.FrameCount);
        surface.AppendMixedContinuousContacts(mover, aggregated);
        Assert.Equal(2, aggregated.GroupCount);
        bool seenLeft = false, seenRight = false;
        for (int groupIndex = 0; groupIndex < aggregated.GroupCount; groupIndex++)
        {
            ref ContactGroup group = ref aggregated.GetGroup(groupIndex);
            Assert.True(group.Key.SourceIdB == left.Id + 1 || group.Key.SourceIdB == right.Id + 1);
            bool fromLeft = group.Key.SourceIdB == left.Id + 1;
            seenLeft |= fromLeft; seenRight |= !fromLeft;
            for (int point = 0; point < group.Count; point++)
            {
                ManifoldContact row = group[point];
                Assert.True(row.HasMaterialOverride);
                Assert.Equal(PhysicsMaterial.Frictionless, row.MaterialA);
                Assert.Equal(fromLeft ? left.Material : right.Material, row.MaterialB);
                Assert.Equal(compound ? 1 : 0, row.FeatureNamespaceA);
            }
        }
        Assert.True(seenLeft); Assert.True(seenRight);

        LSCollider2D Rectangle(Fixed64 x, PhysicsMaterial material)
        {
            var member = new LSAABBoxCollider2D(new Vector2d(2, 2)) { Material = material };
            Initialize(member, scene.Context, new(x, Fixed64.Zero, -Fixed64.One));
            return member;
        }
    }

    [Theory]
    [InlineData(false)] [InlineData(true)]
    public void DeclaredDomain_ShouldNotCertifyAnEnclosurePastEitherScalarFace(bool upper)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        LSMeshCollider mesh = RightSheet(scene.Context, false);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { mesh });
        var circle = new LSCircleCollider2D(Fixed64.Half);
        Initialize(circle, scene.Context, new(upper ? Fixed64.MaxValue - Fixed64.Half : Fixed64.MinValue + Fixed64.Half,
            Fixed64.Zero, Fixed64.Zero));
        Assert.True(surface.IntersectsMixedContinuous(circle, Fixed64.One));
    }

    [Fact]
    public void RotatingCapsule_ShouldReachAndHitALaterWallInTheDeclaredFloorDomain()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        scene.Context.SetFrameRate(16);
        scene.Context.Environment.MaxSpeed = scene.Context.Environment.MaxFallSpeed = (Fixed64)1024;
        scene.Context.Environment.Gravity = scene.Context.Environment.AirDensity =
            scene.Context.Environment.DampingFactor = scene.Context.Environment.MinSpeed = Fixed64.Zero;
        Fixed64 wallX = Fixed64.FromFraction(3, 4);
        var (left, right) = FoldedFloor(scene);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { left, right });
        var capsule = new LSCapsuleCollider2D(Fixed64.Half, Fixed64.Two)
        { Material = PhysicsMaterial.Frictionless, MixedHalfThicknessOverride = Fixed64.Half };
        var body = new SolidBody2D(new TestMatterAgent(scene.Context), capsule)
        { Mass = Fixed64.One, SleepEnabled = false, ContinuousCollisionMode = ContinuousCollisionMode.Continuous };
        body.Initialize(new(-Fixed64.Quarter, Fixed64.One)); body.UseManualGrounding();
        body.AddLinearImpulse(Vector2d.Right * (Fixed64)8);
        body.AddAngularImpulse(Fixed64.FromFraction(1, 64));
        for (int frame = 0; frame < 16; frame++)
        {
            body.AddLinearImpulse(new(Fixed64.Zero, -Fixed64.FromFraction(1, 16)));
            scene.Context.Simulate(); scene.Context.LateSimulate();
            if (body.LinearVelocity.X <= (Fixed64)4) break;
        }
        Assert.True(body.Position.X > Fixed64.FromFraction(1, 8), $"The capsule did not reach the wall: {body.Position}.");
        Assert.True(body.Position.X < wallX, $"The capsule crossed the wall: {body.Position}.");
        Assert.True(body.LinearVelocity.X <= (Fixed64)4, "An unresolved stationary tail must not count as wall response.");
    }

    [Theory]
    [InlineData(false, false)] [InlineData(false, true)]
    [InlineData(true, false)] [InlineData(true, true)]
    public void CurvedEmptyCorner_ShouldExcludeOnlyAnEnclosureThatStillMissesTheActualTriangle(bool capsule, bool affine)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        LSMeshCollider mesh = RightSheet(scene.Context, affine);
        LSCollider2D curved = capsule ? new LSCapsuleCollider2D(Fixed64.Half, Fixed64.Two)
            : new LSCircleCollider2D(Fixed64.Half);
        curved.MixedHalfThicknessOverride = Fixed64.Half;
        Initialize(curved, scene.Context, new(-Fixed64.Quarter, Fixed64.Zero, capsule ? Fixed64.One : Fixed64.Half));
        Assert.True(mesh.Bounds.Intersects(curved.MixedBounds3D));
        Assert.False(CollisionDetectionMixed.IntersectsForContinuousCollision(mesh, curved));
        Assert.False(CollisionDetectionMixed.IntersectsForContinuousCollision(mesh, curved, Fixed64.FromFraction(1, 100)));
        Assert.True(CollisionDetectionMixed.IntersectsForContinuousCollision(mesh, curved, Fixed64.FromFraction(1, 10)));
        // Search padding must not mutate the physical shape or its admission.
        Assert.False(CollisionDetectionMixed.IntersectsForContinuousCollision(mesh, curved));
    }

    [Fact]
    public void ClosedHullContainment_ShouldKeepNativeSolidAdmissionAndDeclineBoundaryOnlyCertificates()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        LSMeshCollider mesh = MeshTestFixtures.CreateConvexCube();
        scene.InitializeStaticCollider(mesh, Vector3d.Zero);
        var embedded = new LSAABBoxCollider2D(new Vector2d(Fixed64.Quarter, Fixed64.Quarter))
        { MixedHalfThicknessOverride = Fixed64.FromFraction(1, 8) };
        Initialize(embedded, scene.Context, Vector3d.Zero);
        var native = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mesh, embedded, native));
        Assert.NotEmpty(native);
        Assert.True(CollisionDetectionMixed.IntersectsForContinuousCollision(mesh, embedded));
        Assert.True(CollisionDetectionMixed.IntersectsForContinuousCollision(mesh, embedded, Fixed64.Quarter));
    }

    [Theory]
    [InlineData(false)] [InlineData(true)]
    public void UnsupportedPaddedFootprints_ShouldStayUnresolved(bool compound)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        LSMeshCollider mesh = RightSheet(scene.Context, false);
        LSCollider2D embedded = compound
            ? new LSCompoundCollider2D(CompoundColliderPart2D.Circle(Fixed64.Quarter, Vector2d.Zero))
            : new LSAABBoxCollider2D(Vector2d.One);
        Initialize(embedded, scene.Context, new(-4, 0, 4));
        Assert.False(CollisionDetectionMixed.IntersectsForContinuousCollision(mesh, embedded));
        Assert.True(CollisionDetectionMixed.IntersectsForContinuousCollision(mesh, embedded, Fixed64.Quarter));
    }

    [Theory]
    [InlineData("RadiusOverflow")]
    [InlineData("ThicknessOverflow")]
    [InlineData("ReachOverflow")]
    [InlineData("AffineRadiusWidth")]
    [InlineData("AffineCapWidth")]
    [InlineData("MinimumEndpoint")]
    [InlineData("MaximumEndpoint")]
    public void UnrepresentableSearchEnclosures_ShouldRemainConservative(string boundary)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        bool affine = boundary.StartsWith("Affine", System.StringComparison.Ordinal);
        LSMeshCollider mesh = RightSheet(scene.Context, affine);
        Fixed64 radius = boundary is "ThicknessOverflow" or "AffineCapWidth" ? Fixed64.Quarter : Fixed64.Half;
        LSCollider2D embedded = boundary == "ReachOverflow"
            ? new LSCapsuleCollider2D(radius, Fixed64.Two) : new LSCircleCollider2D(radius);
        embedded.MixedHalfThicknessOverride = boundary is "ThicknessOverflow" or "AffineCapWidth"
            ? Fixed64.Half : Fixed64.Quarter;
        Vector3d center = boundary == "MinimumEndpoint" ? new(Fixed64.MinValue + Fixed64.Half, Fixed64.Zero, Fixed64.Zero)
            : boundary == "MaximumEndpoint" ? new(Fixed64.MaxValue - Fixed64.Half, Fixed64.Zero, Fixed64.Zero)
            : Vector3d.Zero;
        Initialize(embedded, scene.Context, center);
        Fixed64 padding = boundary == "RadiusOverflow" ? Fixed64.MaxValue
            : boundary == "ThicknessOverflow" ? Fixed64.MaxValue - radius
            : boundary == "ReachOverflow" ? Fixed64.MaxValue - Fixed64.Half
            : affine ? (Fixed64)4096 - Fixed64.Quarter : Fixed64.One;
        Assert.True(CollisionDetectionMixed.IntersectsForContinuousCollision(mesh, embedded, padding));
    }

    private static LSCollider NativeMover(string shape) => shape switch
    {
        "Sphere" => new LSSphereCollider(),
        "Capsule" => new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) },
        "Cone" => new LSConeCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) },
        "Cylinder" => new LSCylinderCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) },
        "Mesh" => MeshTestFixtures.CreateConvexCube(),
        _ => new LSCuboidCollider()
    };

    private static (LSMeshCollider Left, LSMeshCollider Right) FoldedFloor(PhysicsScenarioBuilder scene, bool horizontal = false)
    {
        Fixed64 wallX = Fixed64.FromFraction(3, 4);
        Vector3d[] leftPoints = { new(-8, -2, 0), new(-8, 2, 0), new(0, -2, 0), new(0, 2, 0) };
        Vector3d[] rightPoints = { new(0, -2, 0), new(0, 2, 0),
            new(wallX, -(Fixed64)2, Fixed64.Zero), new(wallX, (Fixed64)2, Fixed64.Zero),
            new(wallX, -(Fixed64)2, (Fixed64)4), new(wallX, (Fixed64)2, (Fixed64)4) };
        if (horizontal)
        {
            for (int index = 0; index < leftPoints.Length; index++)
                leftPoints[index] = new(leftPoints[index].X, leftPoints[index].Z, -leftPoints[index].Y);
            for (int index = 0; index < rightPoints.Length; index++)
                rightPoints[index] = new(rightPoints[index].X, rightPoints[index].Z, -rightPoints[index].Y);
        }
        var left = new LSMeshCollider(leftPoints, new[] { 0, 1, 2, 2, 1, 3 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation) { Material = PhysicsMaterial.Frictionless };
        var right = new LSMeshCollider(rightPoints, new[] { 0, 1, 2, 2, 1, 3, 2, 3, 4, 4, 3, 5 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation) { Material = PhysicsMaterial.Frictionless };
        scene.InitializeStaticCollider(left, Vector3d.Zero); scene.InitializeStaticCollider(right, Vector3d.Zero);
        return (left, right);
    }

    private static LSMeshCollider RightSheet(GravitasWorldContext context, bool affine)
    {
        // The half-raw scaled translation cannot be baked into a rigid Q32 chart.
        Fixed64 epsilon = affine ? Fixed64.FromRaw(1) : Fixed64.Zero;
        Fixed64 right = (Fixed64)8 + epsilon;
        var mesh = new LSMeshCollider(new[] { new Vector3d(epsilon, -(Fixed64)2, Fixed64.Zero),
            new(epsilon, (Fixed64)2, Fixed64.Zero), new(right, -(Fixed64)2, Fixed64.Zero), new(right, (Fixed64)2, Fixed64.Zero) },
            new[] { 0, 1, 2, 2, 1, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        mesh.InitializeWithNoBody(new TestMatterAgent(context, new FixedTransform(Vector3d.Zero,
            FixedQuaternion.Identity, new(affine ? Fixed64.FromFraction(3, 2) : Fixed64.One, Fixed64.One, Fixed64.One))));
        return mesh;
    }

    private static void Initialize(LSCollider2D collider, GravitasWorldContext context, Vector3d position) =>
        collider.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One)));
}
