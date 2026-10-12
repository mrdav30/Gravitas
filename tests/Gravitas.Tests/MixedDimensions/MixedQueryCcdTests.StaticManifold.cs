using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

/// <content>Checks static mixed CCD before discrete response can conceal a discarded closing constraint.</content>
public sealed partial class MixedQueryCcdTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void StaticMeshCircleSweep_TangentToTheClippedFiniteCorner_ShouldNotBlockTravel(bool reversedWinding)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        var mesh = new LSMeshCollider(new[]
        {
            new Vector3d(0, -1, 0), new Vector3d(2, 2, 0), new Vector3d(2, 2, 2)
        }, reversedWinding ? new[] { 0, 2, 1 } : new[] { 0, 1, 2 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        CreateBodyless3D(context, mesh, Vector3d.Zero);
        Vector2d start = new(-Fixed64.One, -Fixed64.Half), end = new(Fixed64.One, -Fixed64.Half);
        var hits = new SwiftList<PhysicsMixedHit>();

        context.QueryMixed.SweepCircleAgainst3D(start, end, Fixed64.Half,
            Fixed64.Zero, Fixed64.Half, IncludeLayerZero, out PhysicsMixedHit geometric).Should().BeTrue();
        geometric.Distance.Should().Be(Fixed64.FromFraction(4, 3));
        int count = context.QueryMixed.SweepCircleAgainstStatic3DAll(start, end, Fixed64.Half,
            Fixed64.Zero, Fixed64.Half, IncludeLayerZero, hits, requireClosingContact: true);

        // Slab clipping creates the exact corner (1/3,-1/2,0). Its
        // rounded entry center lies just before touching, but its radial
        // normal remains perpendicular to travel and cannot impede CCD.
        count.Should().Be(0);
    }

    [Theory]
    [InlineData(-3)]
    [InlineData(-2)]
    [InlineData(-1)]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void StaticObliqueMeshCircleSweep_ShouldRetainRoundedFirstImpact(int rawShift)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        Fixed64 shift = Fixed64.FromRaw(rawShift);
        var mesh = new LSMeshCollider(new[]
        {
            new Vector3d((Fixed64)(-4) + shift, (Fixed64)(-2), (Fixed64)4),
            new Vector3d((Fixed64)(-4) + shift, (Fixed64)2, (Fixed64)4),
            new Vector3d((Fixed64)4 + shift, (Fixed64)(-2), (Fixed64)(-4)),
            new Vector3d((Fixed64)4 + shift, (Fixed64)2, (Fixed64)(-4))
        }, new[] { 0, 1, 2, 1, 3, 2 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        CreateBodyless3D(context, mesh, Vector3d.Zero);
        var hits = new SwiftList<PhysicsMixedHit>();

        int count = context.QueryMixed.SweepCircleAgainstStatic3DAll(Vector2d.Left, Vector2d.Right,
            Fixed64.Half, Fixed64.Zero, Fixed64.Half, IncludeLayerZero, hits, requireClosingContact: true);

        // Rounding an irrational entry time may put the sample just before
        // exact overlap. The certified sweep witness must still stop travel.
        count.Should().Be(1);
        hits[0].Collider3D.Should().BeSameAs(mesh);
        hits[0].Distance.Should().BeGreaterThan(Fixed64.Zero).And.BeLessThan(Fixed64.One);
        hits[0].Normal3DTo2D.X.Should().BeLessThan(Fixed64.Zero);
        hits[0].Normal3DTo2D.Z.Should().BeLessThan(Fixed64.Zero);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void MixedStaticClosingQuery_ShouldKeepFullRangeDiagonalInitialContact(bool source3D, bool outward)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        // This diagonal's exact magnitude fits Q32.32, but its projection onto
        // the rounded unit normal exceeds MaxValue by about 73 million raw units.
        // Overflow in materializing speed must not erase its exact closing sign.
        Fixed64 component = Fixed64.FromRaw(6_521_908_912_666_391_105);
        Vector3d displacement = new(component, Fixed64.Zero, component);
        if (outward) displacement = -displacement;
        Vector3d.TryGetMagnitude(displacement, out _).Should().BeTrue();
        Vector3d start = new(-Fixed64.Half, Fixed64.Zero, -Fixed64.Half);
        Vector3d end = start + displacement;
        var hits = new SwiftList<PhysicsMixedHit>();
        PhysicsMixedHit geometric;
        int count;
        if (source3D)
        {
            CreateBodylessPolygon2D(context, Vector2d.Zero, new[]
            {
                new Vector2d(-1, 0), new Vector2d(0, -1),
                new Vector2d(1, 0), new Vector2d(0, 1)
            });
            context.QueryMixed.SweepSphereAgainst2D(start, end, Fixed64.Half, IncludeLayerZero, out geometric)
                .Should().BeTrue();
            count = context.QueryMixed.SweepSphereAgainstStatic2DAll(start, end, Fixed64.Half,
                IncludeLayerZero, hits, requireClosingContact: true);
        }
        else
        {
            CreateBodyless3D(context, new LSSphereCollider { Radius = Fixed64.Half }, Vector3d.Zero);
            context.QueryMixed.SweepCircleAgainst3D(start.ToVector2d(), end.ToVector2d(), Fixed64.Half,
                Fixed64.Zero, Fixed64.Half, IncludeLayerZero, out geometric).Should().BeTrue();
            count = context.QueryMixed.SweepCircleAgainstStatic3DAll(start.ToVector2d(), end.ToVector2d(),
                Fixed64.Half, Fixed64.Zero, Fixed64.Half, IncludeLayerZero, hits, requireClosingContact: true);
        }
        Vector3d normal = source3D ? geometric.NormalFor3DSource : geometric.Normal3DTo2D;
        normal.X.Should().BeLessThan(Fixed64.Zero);
        normal.Z.Should().BeLessThan(Fixed64.Zero);
        Vector3d.TryDot(displacement, normal, out _).Should().BeFalse();
        geometric.Distance.Should().Be(Fixed64.Zero);
        count.Should().Be(outward ? 0 : 1);
        if (!outward)
        {
            hits[0].Distance.Should().Be(Fixed64.Zero);
            hits[0].Normal3DTo2D.Should().Be(geometric.Normal3DTo2D);
        }
    }

    [Fact]
    public void StaticMixedBoxProxyCcd_ShouldStopConservativelyWithoutFabricatingMeshContact()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        var mesh = new LSMeshCollider(new[]
        {
            new Vector3d(0, -2, -2), new Vector3d(0, 2, -2),
            new Vector3d(0, -2, 2), new Vector3d(0, 2, 2)
        }, new[] { 0, 1, 2, 1, 3, 2 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        CreateBodyless3D(context, mesh, Vector3d.Zero);
        var box = new LSAABBoxCollider2D(new Vector2d(Fixed64.FromFraction(1, 5), Fixed64.FromFraction(1, 5)))
        {
            Material = PhysicsMaterial.Frictionless,
            MixedHalfThicknessOverride = Fixed64.Half
        };
        Vector2d start = Vector2d.Left * Fixed64.FromFraction(1, 8);
        var source = new SolidBody2D(new TestMatterAgent(context, new FixedTransform(
            new Vector3d(start.X, Fixed64.Zero, start.Y), FixedQuaternion.Identity, Vector3d.One)), box)
        {
            Mass = Fixed64.One,
            FreezeAxes = BodyFreezeAxes2D.Rotation,
            ContinuousCollisionMode = ContinuousCollisionMode.Continuous
        };
        source.Initialize(start);
        source.UseManualGrounding();
        var manifold = new ContactManifold();
        CollisionDetectionMixed.TryBuildManifold(mesh, box, manifold).Should().BeFalse();
        source.ApplyCollisionLinearVelocityDelta(Vector2d.Right);
        PrepareMixedContinuousCollisionFrame(context);

        source.LateSimulate(updateSleepState: false, updateColliderState: false);

        // The enclosing circle reaches X=0 while the real box still has a 1/40
        // gap. CCD keeps its conservative stop without inventing a solver row.
        source.Position.Should().Be(start);
        source.LinearVelocity.Should().Be(Vector2d.Zero);
        source.LastContinuousCollisionToiIterationCount.Should().Be(1);
        CollisionDetectionMixed.TryBuildManifold(mesh, box, manifold).Should().BeFalse();
    }

    [Theory]
    [InlineData(false, 0)]
    [InlineData(true, 0)]
    [InlineData(false, 1)]
    [InlineData(true, 1)]
    [InlineData(false, 2)]
    [InlineData(true, 2)]
    [InlineData(false, 3)]
    [InlineData(true, 3)]
    public void MixedComplexTranslationalResponse_ShouldPublishOnlyAnAdmissibleImpact(bool source3D, int rejection)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        context.Settings.ContinuousCollisionMaxToiIterations = 1;
        SolidBody body3D;
        SolidBody2D body2D;
        Vector2d impact2D;
        if (source3D)
        {
            body3D = CreateSphere3D(context, Vector3d.Zero).Body;
            var walls = new LSCompoundCollider2D(
                CompoundColliderPart2D.AABBox(new Vector2d(Fixed64.FromFraction(1, 5), (Fixed64)4),
                    Vector2d.Right * Fixed64.FromFraction(2, 5), PhysicsMaterial.Frictionless),
                CompoundColliderPart2D.AABBox(new Vector2d((Fixed64)4, Fixed64.FromFraction(1, 5)),
                    Vector2d.Forward * Fixed64.FromFraction(2, 5), PhysicsMaterial.Frictionless));
            body2D = new SolidBody2D(new TestMatterAgent(context,
                new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)), walls) { Mass = Fixed64.One };
            body2D.Initialize(Vector2d.Zero);
            body3D.ApplyCollisionLinearVelocityDelta((Vector3d.Right + Vector3d.Forward)
                * (rejection == 2 ? -Fixed64.One : Fixed64.One));
            impact2D = Vector2d.Zero;
        }
        else
        {
            var wall = new LSMeshCollider(new[]
            {
                new Vector3d(1, -2, -2), new Vector3d(1, 2, -2),
                new Vector3d(1, -2, 2), new Vector3d(1, 2, 2)
            }, new[] { 0, 1, 2, 1, 3, 2 }, MeshColliderMode.Convex, MeshInertiaPolicy.SurfaceApproximation);
            body3D = CreateBody3D(context, wall, Vector3d.Zero).Body;
            body2D = CreateCircle2D(context, Vector2d.Zero);
            body2D.ApplyCollisionLinearVelocityDelta(Vector2d.Right
                * (rejection == 2 ? -Fixed64.One : Fixed64.One));
            impact2D = Vector2d.Right * Fixed64.Half;
        }
        body3D.FreezeAxes = BodyFreezeAxes3D.Rotation;
        body2D.FreezeAxes = BodyFreezeAxes2D.Rotation;
        body3D.Collider.Material = PhysicsMaterial.Frictionless;
        body2D.Collider.Material = PhysicsMaterial.Frictionless;
        body3D.UseManualGrounding();
        body2D.UseManualGrounding();
        PrepareMixedContinuousCollisionFrame(context);
        if (rejection == 3)
        {
            Fixed64 remaining = Fixed64.FromFraction(3, 4);
            if (source3D)
                body2D.ApplyContinuousCollisionHandoffState(Vector2d.Zero, Fixed64.Zero,
                    Vector2d.Zero, Fixed64.Zero, remaining).Should().BeTrue();
            else
                body3D.ApplyContinuousCollisionHandoff(Vector3d.Zero, FixedQuaternion.Identity,
                    Vector3d.Zero, Vector3d.Zero, remaining).Should().BeTrue();
        }
        var before = context.ComputeReplayHash(GravitasReplayHashMode.Authoritative);
        Vector3d boundsMin3D = body3D.Collider.BoundsMin;
        Vector3d boundsMax3D = body3D.Collider.BoundsMax;
        Fixed64 minX2D = body2D.Collider.MinX;
        Fixed64 maxX2D = body2D.Collider.MaxX;
        Fixed64 minZ2D = body2D.Collider.MinY;
        Fixed64 maxZ2D = body2D.Collider.MaxY;
        Vector3d impact3D = rejection == 1 && source3D ? Vector3d.Right * (Fixed64)6 : Vector3d.Zero;
        if (rejection == 1 && !source3D) impact2D = Vector2d.Right * (Fixed64)6;

        bool applied = source3D
            ? body3D.TryApplyContinuousCollisionMixed2DResponse(body2D, Vector3d.Left, impact3D,
                Vector2d.Zero, Fixed64.Half, Fixed64.Half)
            : body2D.TryApplyContinuousCollisionMixed3DResponse(body3D, Vector2d.Left, impact2D,
                Vector3d.Zero, Fixed64.Half, Fixed64.Half);

        applied.Should().Be(rejection == 0);
        if (rejection != 0)
        {
            // Sampling invalidates geometry/query version caches, but must restore
            // real shapes and leave authoritative poses, velocity and sleep intact.
            context.ComputeReplayHash(GravitasReplayHashMode.Authoritative).Should().Be(before);
            body3D.Position3d.Should().Be(Vector3d.Zero);
            body2D.Position.Should().Be(Vector2d.Zero);
            body3D.Collider.BoundsMin.Should().Be(boundsMin3D);
            body3D.Collider.BoundsMax.Should().Be(boundsMax3D);
            body2D.Collider.MinX.Should().Be(minX2D);
            body2D.Collider.MaxX.Should().Be(maxX2D);
            body2D.Collider.MinY.Should().Be(minZ2D);
            body2D.Collider.MaxY.Should().Be(maxZ2D);
        }
        else
        {
            // Equal masses and frozen rotation split each admitted incoming axis
            // equally. Response queues the target's remaining half frame while
            // the source pose is still owned by its outer CCD loop.
            Vector3d incoming = source3D ? Vector3d.Right + Vector3d.Forward : Vector3d.Right;
            body3D.LinearVelocity.Should().Be(incoming * Fixed64.Half);
            body2D.LinearVelocity.Should().Be(incoming.ToVector2d() * Fixed64.Half);
            if (source3D)
            {
                body3D.Position3d.Should().Be(Vector3d.Zero);
                body2D.Position.Should().Be(Vector2d.Zero);
                body2D.TryConsumeContinuousCollisionHandoff(updateSleepState: false, updateColliderState: false)
                    .Should().BeTrue();
                body2D.Position.Should().Be(incoming.ToVector2d() * Fixed64.Quarter);
            }
            else
            {
                body2D.Position.Should().Be(Vector2d.Zero);
                body3D.Position3d.Should().Be(Vector3d.Zero);
                body3D.TryConsumeContinuousCollisionHandoff(updateSleepState: false, updateColliderState: false)
                    .Should().BeTrue();
                body3D.Position3d.Should().Be(Vector3d.Right * Fixed64.Quarter);
            }
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void StaticMixedComplex3DSourceCcd_ShouldSeparateActualAndProxyWitnesses(bool meshSource)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        Fixed64 faceX = Fixed64.FromFraction(2, 5);
        LSCollider sourceCollider = meshSource
            ? new LSMeshCollider(new[]
            {
                new Vector3d(faceX, (Fixed64)(-2), (Fixed64)(-2)), new Vector3d(faceX, (Fixed64)2, (Fixed64)(-2)),
                new Vector3d(faceX, (Fixed64)(-2), (Fixed64)2), new Vector3d(faceX, (Fixed64)2, (Fixed64)2)
            }, new[] { 0, 1, 2, 1, 3, 2 }, MeshColliderMode.Convex, MeshInertiaPolicy.SurfaceApproximation)
            : new LSCompoundCollider(
                CompoundColliderPart.Sphere(Fixed64.FromFraction(1, 10), Vector3d.Left, PhysicsMaterial.Frictionless),
                CompoundColliderPart.Sphere(Fixed64.FromFraction(1, 10), Vector3d.Right, PhysicsMaterial.Frictionless));
        SolidBody source = CreateBody3D(context, sourceCollider, Vector3d.Zero).Body;
        LSCollider2D target = CreateBodylessCircle2D(context, meshSource ? Vector2d.Zero : Vector2d.Forward);
        sourceCollider.Material = PhysicsMaterial.Frictionless;
        source.FreezeAxes = BodyFreezeAxes3D.Rotation;
        source.UseManualGrounding();
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        var manifold = new ContactManifold();
        CollisionDetectionMixed.TryBuildManifold(sourceCollider, target, manifold).Should().Be(meshSource);
        source.ApplyCollisionLinearVelocityDelta(meshSource ? Vector3d.Left : Vector3d.Forward);
        PrepareMixedContinuousCollisionFrame(context);

        source.LateSimulate(updateSleepState: false, updateColliderState: false);

        // The mesh really intersects the circle's side; the two remote sphere
        // leaves miss while their enclosing sphere intersects. Both stop safely,
        // but only the first has real finite manifold rows available for solving.
        source.Position3d.Should().Be(Vector3d.Zero);
        source.LinearVelocity.Should().Be(Vector3d.Zero);
        source.LastContinuousCollisionToiIterationCount.Should().Be(1);
        CollisionDetectionMixed.TryBuildManifold(sourceCollider, target, manifold).Should().Be(meshSource);
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(1, false)]
    [InlineData(2, false)]
    [InlineData(0, true)]
    [InlineData(1, true)]
    [InlineData(2, true)]
    public void StaticMixedCcd_ShouldFindLaterWallDespiteInitialTangentFloor(int targetTopology, bool kinematic)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        LSCollider target = targetTopology != 0
            ? CreateStaticMixedCcdWallMesh(subdivided: targetTopology == 2)
            : new LSCompoundCollider(
                CompoundColliderPart.Cuboid(new Vector3d((Fixed64)4, Fixed64.Half, (Fixed64)4),
                    Vector3d.Down * Fixed64.Quarter, PhysicsMaterial.Frictionless),
                CompoundColliderPart.Cuboid(new Vector3d(Fixed64.Half, (Fixed64)4, (Fixed64)4),
                    Vector3d.Right * Fixed64.FromFraction(5, 4), PhysicsMaterial.Frictionless));
        CreateBodyless3D(context, target, Vector3d.Zero);
        var circle = new LSCircleCollider2D(Fixed64.Half)
        {
            Material = PhysicsMaterial.Frictionless,
            MixedHalfThicknessOverride = Fixed64.Half
        };
        var source = new SolidBody2D(new TestMatterAgent(context,
            new FixedTransform(new Vector3d(Fixed64.Zero, -Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity, Vector3d.One)), circle)
        {
            Mass = Fixed64.One,
            FreezeAxes = BodyFreezeAxes2D.Rotation,
            ContinuousCollisionMode = ContinuousCollisionMode.Continuous
        };
        source.Initialize(Vector2d.Zero, motionType: kinematic ? BodyMotionType.Kinematic : BodyMotionType.Dynamic);
        source.UseManualGrounding();
        if (kinematic) source.Agent.Transform.LocalPosition = new Vector3d(Fixed64.One, -Fixed64.Quarter, Fixed64.Zero);
        else source.ApplyCollisionLinearVelocityDelta(Vector2d.Right);
        PrepareMixedContinuousCollisionFrame(context);

        source.LateSimulate(updateSleepState: false, updateColliderState: false);

        // The existing floor overlaps at t=0 but cannot oppose planar travel.
        // The real wall is X=1: radius 1/2 first touches at center X=1/2.
        source.Position.Should().Be(Vector2d.Right * Fixed64.Half);
        if (!kinematic) source.LinearVelocity.Should().Be(Vector2d.Zero);
        source.LastContinuousCollisionToiIterationCount.Should().Be(1);
        source.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
    }

    private static LSMeshCollider CreateStaticMixedCcdWallMesh(bool subdivided)
    {
        if (!subdivided)
            return new LSMeshCollider(new[]
            {
                new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2),
                new Vector3d(-2, 0, 2), new Vector3d(2, 0, 2),
                new Vector3d(1, -2, -2), new Vector3d(1, 2, -2),
                new Vector3d(1, -2, 2), new Vector3d(1, 2, 2)
            }, new[] { 0, 2, 1, 1, 2, 3, 4, 5, 6, 5, 7, 6 },
                MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        // The covered strip edge X=3/4 is reached at distance1/4, before
        // the real wall at distance1/2. Triangle-only selection would hide it.
        Fixed64 seam = Fixed64.FromFraction(3, 4);
        return new LSMeshCollider(new[]
        {
            new Vector3d(-2, 0, -2), new Vector3d(-2, 0, 2),
            new Vector3d(0, 0, -2), new Vector3d(0, 0, 2),
            new Vector3d(seam, Fixed64.Zero, (Fixed64)(-2)), new Vector3d(seam, Fixed64.Zero, (Fixed64)2),
            new Vector3d(2, 0, -2), new Vector3d(2, 0, 2),
            new Vector3d(1, -2, -2), new Vector3d(1, 2, -2),
            new Vector3d(1, -2, 2), new Vector3d(1, 2, 2)
        }, new[] { 0, 1, 2, 2, 1, 3, 2, 3, 4, 4, 3, 5, 4, 5, 6, 6, 5, 7, 8, 9, 10, 9, 11, 10 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
    }

    [Theory]
    [InlineData(false, 0)]
    [InlineData(true, 0)]
    [InlineData(false, 1)]
    [InlineData(true, 1)]
    [InlineData(false, 2)]
    [InlineData(true, 2)]
    public void StaticMeshInitialRimCcd_ShouldAdmitClosingSiblingOfVerticalPrimary(bool kinematic, int motion)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        SolidBody2D source = CreateStaticMeshRimCcdSource(context, kinematic);
        Vector2d start = source.Position;
        Vector2d travel = (motion == 0 ? Vector2d.Left : motion == 1 ? Vector2d.Right : Vector2d.Forward)
            * Fixed64.FromFraction(1, 8);
        if (kinematic) source.Agent.Transform.LocalPosition = new Vector3d(start.X + travel.X, -Fixed64.Quarter, travel.Y);
        else source.ApplyCollisionLinearVelocityDelta(travel);
        PrepareMixedContinuousCollisionFrame(context);

        source.LateSimulate(updateSleepState: false, updateColliderState: false);

        // The circle overlaps the finite right edge by 1/8, while the vertical
        // face's 1/4 depth makes it the deepest summary. Only inward X closes.
        source.Position.Should().Be(motion == 0 ? start : start + travel);
        if (!kinematic) source.LinearVelocity.Should().Be(motion == 0 ? Vector2d.Zero : travel);
        source.LastContinuousCollisionToiIterationCount.Should().Be(motion == 0 ? 1 : 0);
        source.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
        source.Agent.Transform.LocalPosition.Y.Should().Be(-Fixed64.Quarter);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void StaticCompoundMixedCcd_ShouldSolveBothWallsAlongsideTangentFloor(bool source3D, bool kinematic)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        LSCompoundCollider compound = CreateMixedCcdFloorAndWalls(twoWalls: true);
        SolidBody? mover3D = null;
        SolidBody2D? mover2D = null;
        if (source3D)
        {
            mover3D = CreateBody3D(context, compound, Vector3d.Zero, isKinematic: kinematic).Body;
            var circle = new LSCircleCollider2D(Fixed64.Half) { Material = PhysicsMaterial.Frictionless };
            circle.InitializeWithNoBody(new TestMatterAgent(context,
                new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
            mover3D.FreezeAxes = BodyFreezeAxes3D.Rotation;
            mover3D.UseManualGrounding();
            mover3D.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            if (kinematic) mover3D.Agent.Transform.LocalPosition = Vector3d.Left - Vector3d.Forward;
            else mover3D.ApplyCollisionLinearVelocityDelta(Vector3d.Left - Vector3d.Forward);
        }
        else
        {
            CreateBodyless3D(context, compound, Vector3d.Zero);
            mover2D = CreateCircle2D(context, Vector2d.Zero, isKinematic: kinematic);
            mover2D.Collider.Material = PhysicsMaterial.Frictionless;
            mover2D.FreezeAxes = BodyFreezeAxes2D.Rotation;
            mover2D.UseManualGrounding();
            mover2D.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            if (kinematic) mover2D.Agent.Transform.LocalPosition = Vector3d.Right + Vector3d.Forward;
            else mover2D.ApplyCollisionLinearVelocityDelta(Vector2d.Right + Vector2d.Forward);
        }
        PrepareMixedContinuousCollisionFrame(context);

        if (source3D) mover3D!.LateSimulate(updateSleepState: false, updateColliderState: false);
        else mover2D!.LateSimulate(updateSleepState: false, updateColliderState: false);

        if (source3D)
        {
            if (!kinematic) mover3D!.LinearVelocity.Should().Be(Vector3d.Zero);
            mover3D!.Position3d.Should().Be(Vector3d.Zero);
            mover3D.LastContinuousCollisionToiIterationCount.Should().Be(1);
        }
        else
        {
            if (!kinematic) mover2D!.LinearVelocity.Should().Be(Vector2d.Zero);
            mover2D!.Position.Should().Be(Vector2d.Zero);
            mover2D.LastContinuousCollisionToiIterationCount.Should().Be(1);
        }
    }

    [Fact]
    public void StaticCompound2DMixedCcd_ShouldRetainBothClosingLeafWalls()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        var target = new LSCompoundCollider2D(
            CompoundColliderPart2D.AABBox(new Vector2d(Fixed64.FromFraction(1, 5), (Fixed64)4),
                Vector2d.Right * Fixed64.FromFraction(2, 5)),
            CompoundColliderPart2D.AABBox(new Vector2d((Fixed64)4, Fixed64.FromFraction(1, 5)),
                Vector2d.Forward * Fixed64.FromFraction(2, 5))) { Material = PhysicsMaterial.Frictionless };
        target.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(context, Vector3d.Zero);
        source.Collider.Material = PhysicsMaterial.Frictionless;
        source.Body.UseManualGrounding();
        source.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right + Vector3d.Forward);
        PrepareMixedContinuousCollisionFrame(context);

        source.Body.LateSimulate(updateSleepState: false, updateColliderState: false);

        source.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        source.Body.Position3d.Should().Be(Vector3d.Zero);
        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(1);
        source.Body.LastContinuousCollisionToiIterationLimitReached.Should().BeFalse();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void StaticMixedRimCcd_ShouldReuseAcceptedManifoldWithoutAllocating(bool laterWall)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        SolidBody2D source = CreateStaticMeshRimCcdSource(context, kinematic: false, laterWall);
        Vector2d start = source.Position;
        long allocated = AllocationTestHelper.MeasureSteadyState(() =>
        {
            source.ResetPosition(start);
            source.UseManualGrounding();
            source.ApplyCollisionLinearVelocityDelta(laterWall ? Vector2d.Right
                : Vector2d.Left * Fixed64.FromFraction(1, 8));
            PrepareMixedContinuousCollisionFrame(context);
            source.LateSimulate(updateSleepState: false, updateColliderState: false);
        }, warmupIterations: 16, stabilizationIterations: 4, measurementIterations: 16);

        allocated.Should().Be(0);
        source.Position.Should().Be(laterWall ? Vector2d.Right * Fixed64.Half : start);
        source.LinearVelocity.Should().Be(Vector2d.Zero);
        source.LastContinuousCollisionToiIterationCount.Should().Be(1);
    }

    private static SolidBody2D CreateStaticMeshRimCcdSource(GravitasWorldContext context, bool kinematic,
        bool laterWall = false)
    {
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        var mesh = laterWall ? CreateStaticMixedCcdWallMesh(subdivided: true) : new LSMeshCollider(
            new[] { new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2), new Vector3d(-2, 0, 2), new Vector3d(2, 0, 2) },
            new[] { 0, 2, 1, 1, 2, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        CreateBodyless3D(context, mesh, Vector3d.Zero);
        Vector2d start = laterWall ? Vector2d.Zero : new(Fixed64.FromFraction(19, 8), Fixed64.Zero);
        var circle = new LSCircleCollider2D(Fixed64.Half)
        {
            Material = PhysicsMaterial.Frictionless,
            MixedHalfThicknessOverride = Fixed64.Half
        };
        var source = new SolidBody2D(new TestMatterAgent(context,
            new FixedTransform(new Vector3d(start.X, -Fixed64.Quarter, start.Y), FixedQuaternion.Identity, Vector3d.One)), circle)
        {
            Mass = Fixed64.One,
            FreezeAxes = BodyFreezeAxes2D.Rotation,
            ContinuousCollisionMode = ContinuousCollisionMode.Continuous
        };
        source.Initialize(start, motionType: kinematic ? BodyMotionType.Kinematic : BodyMotionType.Dynamic);
        source.UseManualGrounding();
        var manifold = new ContactManifold();
        CollisionDetectionMixed.TryBuildManifold(mesh, circle, manifold).Should().BeTrue();
        manifold.PrimaryContact.Normal.Should().Be(Vector3d.Down);
        manifold.PrimaryContact.Depth.Should().Be(Fixed64.Quarter);
        return source;
    }
}
