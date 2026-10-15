using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class ColliderGeometryRangeTests
{
    private static readonly Fixed64 Limit = (Fixed64)4096;
    private static readonly Fixed64 RawUnit = Fixed64.FromRaw(1);

    [Fact]
    public void OriginalExtremeCapsuleSweepSource_ShouldRejectBeforeRuntimePublication()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 maximum = Fixed64.MaxValue;
        var capsule = new LSCapsuleCollider
        {
            Radius = Fixed64.FromRaw(long.MaxValue / 2),
            Size = new Vector3d(Fixed64.One, maximum, Fixed64.One)
        };
        var agent = new TestMatterAgent(context, new FixedTransform(
            new Vector3d(maximum - (Fixed64)4, maximum - (Fixed64)4, Fixed64.Zero),
            FixedQuaternion.Identity, Vector3d.One));
        uint version = capsule.RuntimeShapeVersion;

        Action initialize = () => capsule.InitializeWithNoBody(agent);

        initialize.Should().Throw<ArgumentException>().WithMessage("*4,096*");
        capsule.RuntimeShapeVersion.Should().Be(version);
        context.Physics.ColliderCount.Should().Be(0);
        capsule.IsPartitioned.Should().BeFalse();
    }

    [Fact]
    public void OriginalExtremeCapsuleTarget_ShouldRetainCompactRecenteredMeshAdmission()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 maximum = Fixed64.MaxValue;
        Fixed64 radius = Fixed64.FromRaw(long.MaxValue / 2);
        var mesh = new LSMeshCollider(new[]
        {
            new Vector3d(maximum, Fixed64.Zero, -Fixed64.One),
            new Vector3d(maximum, Fixed64.Zero, Fixed64.One),
            new Vector3d(maximum, (Fixed64)5, Fixed64.Zero)
        }, new[] { 0, 1, 2 }, MeshColliderMode.Concave);

        mesh.InitializeWithNoBody(new TestMatterAgent(context, new FixedTransform(
            new Vector3d(-radius - (Fixed64)3, maximum - (Fixed64)4, Fixed64.Zero),
            FixedQuaternion.Identity, Vector3d.One)));

        mesh.CanonicalCenteredProxyRadius.Should().BeLessThan((Fixed64)4);
        context.Physics.ColliderCount.Should().Be(1);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CompoundProxy_ShouldEncloseRoundedLeafCentersWithoutChangingExactAdmission(bool planar)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.One * Fixed64.FromFraction(5, 8));
        Vector2d center;
        Fixed64 admitted, proxy, ccdProxy;
        if (planar)
        {
            context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
            var compound = new LSCompoundCollider2D(CompoundColliderPart2D.Circle((Fixed64)400,
                new Vector2d(RawUnit, RawUnit)));
            var body = new SolidBody2D(new TestMatterAgent(context, transform), compound);
            body.Initialize(Vector2d.Zero);
            ccdProxy = body.ResolveContinuousCollisionProxyRadius();
            center = compound.GetPartCollider(0).Center;
            admitted = compound.PreparedGeometryReach;
            proxy = compound.ScaledRadius;
        }
        else
        {
            var compound = new LSCompoundCollider(CompoundColliderPart.Sphere((Fixed64)400,
                new Vector3d(RawUnit, Fixed64.Zero, RawUnit)));
            var body = new SolidBody(new TestMatterAgent(context, transform), compound);
            body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
            ccdProxy = body.ResolveContinuousCollisionProxyRadius();
            center = compound.GetPartCollider(0).Center.ToVector2d();
            admitted = compound.PreparedGeometryReach;
            proxy = compound.ScaledRadius;
        }
        center.Should().Be(new Vector2d(RawUnit, RawUnit));
        admitted.Should().Be((Fixed64)250 + RawUnit);
        // The 3-4-5 radial witness is on the materialized radius-250 leaf.
        (center + new Vector2d(150, 200)).TryGetMagnitudeCeiling(out Fixed64 required).Should().BeTrue();
        required.Should().Be((Fixed64)250 + RawUnit + RawUnit);
        proxy.Should().BeGreaterThanOrEqualTo(required);
        ccdProxy.Should().BeGreaterThanOrEqualTo(required);
    }

    [Fact]
    public void PublishedGeometryLimits_ShouldShareTheAdmissionBoundary()
    {
        ColliderSettings.MaximumGeometryReach.Should().Be(Limit);
        ColliderSettings2D.MaximumGeometryReach.Should().Be(Limit);
    }

    [Fact]
    public void AcceptedPlanarBoxResize_RecomputesThePreparedEnvelope()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var box = new LSAABBoxCollider2D(Vector2d.One);
        box.InitializeWithNoBody(new TestMatterAgent(context));
        uint version = box.RuntimeShapeVersion;
        box.Size = new Vector2d(6, 8);
        box.RebuildRuntimeShapeOnly().Should().BeTrue();
        box.CanonicalCenteredProxyRadius.Should().Be((Fixed64)5);
        box.Bounds.Min.Should().Be(new Vector2d(-3, -4));
        box.Bounds.Max.Should().Be(new Vector2d(3, 4));
        box.RuntimeShapeVersion.Should().BeGreaterThan(version);
    }

    [Fact]
    public void NegativePreparedReach_IsRejectedAsAnInvalidEnvelope()
    {
        Action validate = () => ColliderGeometryPolicy.ValidateReach(-RawUnit, "reach");
        validate.Should().Throw<ArgumentException>().WithParameterName("reach");
    }

    [Fact]
    public void FailedSpatialPreparation_RetainsThePublishedPoseAndGeometry()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var collider = new UnsupportedTestCollider3D();
        collider.InitializeWithNoBody(new TestMatterAgent(context));
        var bounds = collider.Bounds;
        uint version = collider.RuntimeShapeVersion;
        collider.ThrowOnShapePreparation = true;
        collider.TryPrepareBodyPose(Vector3d.Right, FixedQuaternion.Identity).Should().BeFalse();
        collider.Center.Should().Be(Vector3d.Zero);
        collider.Bounds.Should().Be(bounds);
        collider.RuntimeShapeVersion.Should().Be(version);
        collider.ThrowOnShapePreparation = false;
        collider.TryPrepareBodyPose(Vector3d.Right, FixedQuaternion.Identity).Should().BeTrue();
        collider.PublishPreparedBodyPose();
        collider.Center.Should().Be(Vector3d.Right);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void UnknownShapeMutation_CannotBypassAnUndeclaredCandidateEnvelope(bool planar)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        if (planar)
        {
            var collider = new UnsupportedTestCollider2D();
            collider.InitializeWithNoBody(new TestMatterAgent(context));
            Action mutate = () => collider.LocalOffset = Vector2d.Right;
            mutate.Should().Throw<ArgumentException>();
            collider.LocalOffset.Should().Be(Vector2d.Zero);
        }
        else
        {
            var collider = new UnsupportedTestCollider3D();
            collider.InitializeWithNoBody(new TestMatterAgent(context));
            Action mutate = () => collider.LocalOffset = Vector3d.Right;
            mutate.Should().Throw<ArgumentException>();
            collider.LocalOffset.Should().Be(Vector3d.Zero);
        }
    }

    [Fact]
    public void MixedThicknessMutation_UsesTheOwnerOverrideAndSupportsReturningToDefault()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        var detached = new LSCircleCollider2D(Fixed64.One) { MixedHalfThicknessOverride = Fixed64.One };
        detached.MixedHalfThicknessOverride = null;
        detached.MixedHalfThicknessOverride.Should().BeNull();
        var root = new LSCompoundCollider2D(CompoundColliderPart2D.Circle(Fixed64.One, Vector2d.Zero))
        {
            MixedHalfThicknessOverride = Fixed64.Two
        };
        root.InitializeWithNoBody(new TestMatterAgent(context));
        var part = (LSCircleCollider2D)root.GetPartCollider(0);
        part.MixedHalfThicknessOverride = (Fixed64)3;
        part.Radius = Fixed64.Two;
        root.LocalOffset = Vector2d.Right;
        root.RebuildRuntimeShapeOnly().Should().BeTrue();
        part.MixedHalfThickness.Should().Be(Fixed64.Two);
        root.MixedHalfThicknessOverride = null;
        root.RebuildRuntimeShapeOnly().Should().BeTrue();
        part.MixedHalfThickness.Should().Be(context.Settings.Mixed2DHalfThickness);
        part.Radius.Should().Be(Fixed64.Two);
    }

    [Fact]
    public void DetachedCompoundProxy_PreservesLargeAuthoringUntilEffectiveScaleIsKnown()
    {
        var stadium = new LSCompoundCollider2D(CompoundColliderPart2D.Capsule(
            Fixed64.One, (Fixed64)12000, Vector2d.Zero));
        stadium.ScaledRadius.Should().Be((Fixed64)6000);
        var ordinaryBox = new LSCompoundCollider2D(CompoundColliderPart2D.AABBox(
            Vector2d.One * Fixed64.Two, Vector2d.Zero));
        Vector2d.One.TryGetMagnitudeCeiling(out Fixed64 diagonal).Should().BeTrue();
        ordinaryBox.ScaledRadius.Should().Be(diagonal);
        var box = new LSCompoundCollider2D(CompoundColliderPart2D.AABBox(
            Vector2d.One * Fixed64.MaxValue, Vector2d.Zero,
            Fixed64.Zero, Vector2d.One * Fixed64.Two));
        box.ScaledRadius.Should().Be(Fixed64.MaxValue);
    }

    [Fact]
    public void PendingCompensatedSpatialAuthoring_ShouldPublishBeforeCandidateIndexing()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.SetFrameRate(1);
        var target = new LSSphereCollider { Radius = Fixed64.Half };
        target.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(new Vector3d(100000, 0, 0), FixedQuaternion.Identity, Vector3d.One)));
        SolidBody source = CreatePendingCompensatedSphere(context);
        var collider = (LSSphereCollider)source.Collider;
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        Fixed64 pendingProxy = source.ResolveContinuousCollisionProxyRadius();
        pendingProxy.Should().Be(Fixed64.FromFraction(3, 4) + RawUnit);
        collider.ScaledRadius.Should().Be(Fixed64.Quarter);
        source.GatherRotationalContinuousCollisionCandidates(Vector3d.Zero, Vector3d.Zero,
            Vector3d.Zero, pendingProxy).Should().Be(0); // Only the source's own indexed entry remains.
        var farBounds = DynamicCcdCandidateIndex.CreateBoundsBetween(
            new Vector3d(100000, 0, 0), new Vector3d(100000, 0, 0), Vector3d.One);
        context.Physics.QueryContinuousCollisionCandidates(farBounds).Should().NotContain(source.DynamicId);

        collider.RebuildRuntimeShapeOnly().Should().BeFalse();
        collider.ScaledRadius.Should().Be(Fixed64.Quarter);
        source.ResolveContinuousCollisionProxyRadius().Should().Be(pendingProxy);
        context.AdvanceLateSimulateToken();
        context.Physics.QueryContinuousCollisionCandidates(farBounds).Should().NotContain(source.DynamicId);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CompensatedSpatialAuthoring_MixedPublication_ShouldRetainStaticAndDynamicTargets(bool pending)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.SetFrameRate(1);
        context.Environment.Gravity = Fixed64.Zero;
        var staticCollider = new LSCircleCollider2D((Fixed64)10) { MixedHalfThicknessOverride = (Fixed64)10 };
        staticCollider.InitializeWithNoBody(new TestMatterAgent(context));
        var dynamicTarget = new SolidBody2D(new TestMatterAgent(context),
            new LSCircleCollider2D((Fixed64)10) { MixedHalfThicknessOverride = (Fixed64)10 });
        dynamicTarget.Initialize(Vector2d.Zero);
        var staticTarget = new SolidBody2D(new TestMatterAgent(context),
            new LSCircleCollider2D((Fixed64)10) { MixedHalfThicknessOverride = (Fixed64)10 });
        staticTarget.Initialize(Vector2d.Zero, motionType: BodyMotionType.Static);
        SolidBody source = CreatePendingCompensatedSphere(context, pending);
        source.Agent.Transform.LocalRotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi);
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        source.ResolveContinuousCollisionProxyRadius().Should().Be(Fixed64.FromFraction(3, 4) + RawUnit);
        FixedQuaternion requestedRotation = source.ContinuousCollisionFrameTargetRotation;

        source.LateSimulate(updateSleepState: false, updateColliderState: false);

        // Contained contacts may conservatively shorten the requested arc.
        // Pending and coherent authoring must both retain a valid swept pose.
        FixedQuaternion.Angle(FixedQuaternion.Identity, source.Rotation)
            .Should().BeInRange(Fixed64.Zero, FixedQuaternion.Angle(FixedQuaternion.Identity, requestedRotation));
        source.Position3d.Should().Be(Vector3d.Zero);
        ((LSSphereCollider)source.Collider).ScaledRadius.Should().Be(Fixed64.Quarter);
        dynamicTarget.Position.Should().Be(Vector2d.Zero);
        dynamicTarget.LinearVelocity.Should().Be(Vector2d.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void UndeclaredGeometryReach_RejectsBeforeRegistration(bool planar)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        if (planar)
        {
            var collider = new UnsupportedTestCollider2D { OmitGeometryReach = true };
            Action initialize = () => collider.InitializeWithNoBody(new TestMatterAgent(context));
            initialize.Should().Throw<ArgumentException>();
            collider.Id.Should().Be(-1);
        }
        else
        {
            var collider = new UnsupportedTestCollider3D { OmitGeometryReach = true };
            Action initialize = () => collider.InitializeWithNoBody(new TestMatterAgent(context));
            initialize.Should().Throw<ArgumentException>();
            collider.Id.Should().Be(-1);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RadiusLimit_IsInclusiveAndRejectsNextRawUnitBeforeRegistration(bool planar)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var agent = new TestMatterAgent(context);
        if (planar)
        {
            var accepted = new LSCircleCollider2D(Limit);
            accepted.InitializeWithNoBody(agent);
            var rejected = new LSCircleCollider2D(Limit + RawUnit);
            Action initialize = () => rejected.InitializeWithNoBody(new TestMatterAgent(context));
            initialize.Should().Throw<ArgumentException>();
            rejected.Id.Should().Be(-1);
            context.Physics2D.ColliderCount.Should().Be(1);
        }
        else
        {
            var accepted = new LSSphereCollider { Radius = Limit };
            accepted.InitializeWithNoBody(agent);
            var rejected = new LSSphereCollider { Radius = Limit + RawUnit };
            Action initialize = () => rejected.InitializeWithNoBody(new TestMatterAgent(context));
            initialize.Should().Throw<ArgumentException>();
            rejected.Id.Should().Be(-1);
            context.Physics.ColliderCount.Should().Be(1);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void BoundRadiusMutation_RejectsBeforeAuthoredStateOrSleepChanges(bool planar)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var agent = new TestMatterAgent(context);
        if (planar)
        {
            var collider = new LSCircleCollider2D(Fixed64.One);
            var body = new SolidBody2D(agent, collider);
            body.Initialize(Vector2d.Zero);
            body.Mass = Fixed64.One;
            body.Sleep();
            body.IsSleeping.Should().BeTrue();
            var bounds = collider.Bounds;
            uint version = collider.RuntimeShapeVersion;
            Action mutate = () => collider.Radius = Limit + RawUnit;
            mutate.Should().Throw<ArgumentException>();
            collider.Radius.Should().Be(Fixed64.One);
            collider.Bounds.Should().Be(bounds);
            collider.RuntimeShapeVersion.Should().Be(version);
            body.IsSleeping.Should().BeTrue();
        }
        else
        {
            var collider = new LSSphereCollider { Radius = Fixed64.One };
            var body = new SolidBody(agent, collider);
            body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
            body.Mass = Fixed64.One;
            body.Sleep();
            body.IsSleeping.Should().BeTrue();
            var bounds = collider.Bounds;
            uint version = collider.RuntimeShapeVersion;
            Action mutate = () => collider.Radius = Limit + RawUnit;
            mutate.Should().Throw<ArgumentException>();
            collider.Radius.Should().Be(Fixed64.One);
            collider.Bounds.Should().Be(bounds);
            collider.RuntimeShapeVersion.Should().Be(version);
            body.IsSleeping.Should().BeTrue();
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void BodyAttachment_RetainsFractionalRawExcess(bool planar)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var agent = new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One * Fixed64.Half));
        if (planar)
        {
            var collider = new LSCircleCollider2D(Limit * Fixed64.Two)
            {
                LocalOffset = new Vector2d(RawUnit, Fixed64.Zero)
            };
            var body = new SolidBody2D(agent, collider);
            Action initialize = () => body.Initialize(Vector2d.Zero);
            initialize.Should().Throw<ArgumentException>();
            collider.Id.Should().Be(-1);
        }
        else
        {
            var collider = new LSSphereCollider
            {
                Radius = Limit * Fixed64.Two,
                LocalOffset = new Vector3d(RawUnit, Fixed64.Zero, Fixed64.Zero)
            };
            var body = new SolidBody(agent, collider);
            Action initialize = () => body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
            initialize.Should().Throw<ArgumentException>();
            collider.Id.Should().Be(-1);
        }
    }

    [Fact]
    public void BodyAndPartOffsets_DoNotCancelAdmissionBudget()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var collider = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.One, new Vector3d(-3000, 0, 0)))
        {
            LocalOffset = new Vector3d(3000, 0, 0)
        };
        var body = new SolidBody(new TestMatterAgent(context), collider);
        Action initialize = () => body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
        initialize.Should().Throw<ArgumentException>();
        collider.Id.Should().Be(-1);
    }

    [Fact]
    public void StandaloneMesh_RejectsOversizedTransformWithoutPublishing()
    {
        var mesh = new PhysicsMesh(new[]
        {
            new Vector3d(-1, 0, -1), new Vector3d(1, 0, -1), new Vector3d(0, 0, 1)
        }, new[] { 0, 1, 2 }, Vector3d.Zero, FixedQuaternion.Identity);
        var bounds = mesh.Bounds;
        int builds = mesh.TriangleBvhBuildCount;
        Action transform = () => mesh.UpdateTransform(Vector3d.Right, FixedQuaternion.Identity,
            Vector3d.One * Limit);
        transform.Should().Throw<ArgumentException>();
        mesh.Bounds.Should().Be(bounds);
        mesh.OwnerScale.Should().Be(Vector3d.One);
        mesh.TriangleBvhBuildCount.Should().Be(builds);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void BodylessAttachment_IsPositionRatherThanGeometryReach(bool planar)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        if (planar)
        {
            var collider = new LSCircleCollider2D(Limit)
            {
                LocalOffset = new Vector2d(100000, 0)
            };
            collider.InitializeWithNoBody(new TestMatterAgent(context));
            collider.Center.Should().Be(new Vector2d(100000, 0));
        }
        else
        {
            var collider = new LSSphereCollider
            {
                Radius = Limit,
                LocalOffset = new Vector3d(100000, 0, 0)
            };
            collider.InitializeWithNoBody(new TestMatterAgent(context));
            collider.Center.Should().Be(new Vector3d(100000, 0, 0));
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CapsuleOddRawCore_RoundsReachOutward(bool planar)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 radius = Limit - RawUnit;
        Fixed64 height = Limit * Fixed64.Two + RawUnit;
        if (planar)
        {
            var collider = new LSCapsuleCollider2D(radius, height);
            Action initialize = () => collider.InitializeWithNoBody(new TestMatterAgent(context));
            initialize.Should().Throw<ArgumentException>();
        }
        else
        {
            var collider = new LSCapsuleCollider { Radius = radius, Size = Vector3d.One * height };
            Action initialize = () => collider.InitializeWithNoBody(new TestMatterAgent(context));
            initialize.Should().Throw<ArgumentException>();
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MixedSlab_UsesCompletePlanarReach(bool reject)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        var collider = new LSCircleCollider2D(Fixed64.One)
        {
            LocalOffset = new Vector2d(reject ? 4094 : 4000, 0),
            MixedHalfThicknessOverride = (Fixed64)128
        };
        var body = new SolidBody2D(new TestMatterAgent(context), collider);
        Action initialize = () => body.Initialize(Vector2d.Zero);
        if (reject)
            initialize.Should().Throw<ArgumentException>();
        else
        {
            initialize.Should().NotThrow();
            collider.Id.Should().BeGreaterThanOrEqualTo(0);
        }
    }

    [Fact]
    public void ColliderOwnedMesh_CannotBeTransformedOutsideColliderTransaction()
    {
        var collider = new LSMeshCollider(new[]
        {
            new Vector3d(-1, 0, -1), new Vector3d(1, 0, -1), new Vector3d(0, 0, 1)
        }, new[] { 0, 1, 2 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        Action transform = () => collider.Mesh.UpdateTransform(Vector3d.Zero,
            FixedQuaternion.Identity, Vector3d.One * Fixed64.Two);
        transform.Should().Throw<InvalidOperationException>();
        Action position = () => collider.Mesh.UpdatePosition(Vector3d.Right, FixedQuaternion.Identity);
        position.Should().Throw<InvalidOperationException>();
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    [InlineData(5)]
    public void LargeAuthoredShapes_AcceptEffectiveShrinkingHostScale(int family)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var agent = new TestMatterAgent(context, new FixedTransform(Vector3d.Zero,
            FixedQuaternion.Identity, Vector3d.One * Fixed64.FromFraction(1, 4)));
        LSCollider collider = family switch
        {
            0 => new LSSphereCollider { Radius = (Fixed64)10000 },
            1 => new LSCapsuleCollider { Radius = (Fixed64)1000, Size = Vector3d.One * (Fixed64)10000 },
            2 => new LSCuboidCollider { Size = Vector3d.One * (Fixed64)10000 },
            3 => new LSConeCollider { Radius = (Fixed64)10000, Size = Vector3d.One * (Fixed64)10000 },
            4 => new LSCylinderCollider { Radius = (Fixed64)10000, Size = Vector3d.One * (Fixed64)10000 },
            _ => new LSMeshCollider(new[]
            {
                new Vector3d(-10000, 0, -10000), new Vector3d(10000, 0, -10000), new Vector3d(0, 0, 10000)
            }, new[] { 0, 1, 2 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)
        };
        collider.InitializeWithNoBody(agent);
        collider.CanonicalCenteredProxyRadius.Should().BeLessThanOrEqualTo(Limit);
        collider.Id.Should().BeGreaterThanOrEqualTo(0);
    }

    [Fact]
    public void LargeAuthoredPolygon_AcceptsShrinkingScaleAppliedAfterConstruction()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var collider = new LSPolygonCollider2D(new Vector2d(-10000, -10000),
            new Vector2d(10000, -10000), new Vector2d(0, 10000));
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.One * Fixed64.FromFraction(1, 4));
        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));
        collider.CanonicalCenteredProxyRadius.Should().BeLessThanOrEqualTo(Limit);
    }

    [Fact]
    public void CompensatedCompoundScale_DoesNotMaterializeOverflowingFactorProduct()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Vector3d factor = Vector3d.One * (Fixed64)(1 << 20);
        var collider = new LSCompoundCollider(CompoundColliderPart.Sphere(RawUnit,
            Vector3d.Zero, FixedQuaternion.Identity, factor));
        collider.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, factor)));
        collider.ScaledRadius.Should().Be((Fixed64)256);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ExtremeWorldPosition_CannotHideOversizedRadius(bool maximum)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Vector3d position = new(maximum ? Fixed64.MaxValue : Fixed64.MinValue,
            Fixed64.Zero, Fixed64.Zero);
        var agent = new TestMatterAgent(context,
            new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One));
        var accepted = new LSSphereCollider { Radius = Fixed64.One };
        accepted.InitializeWithNoBody(agent);
        accepted.Center.Should().Be(position);
        var rejected = new LSSphereCollider { Radius = Limit + RawUnit };
        Action initialize = () => rejected.InitializeWithNoBody(new TestMatterAgent(context, agent.Transform));
        initialize.Should().Throw<ArgumentException>();
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void FiniteAxisOddRawHeight_UsesOutwardBoxDiagonal(bool cone, bool reject)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCollider collider = cone ? new LSConeCollider() : new LSCylinderCollider();
        collider.Radius = RawUnit;
        collider.Size = new Vector3d(Fixed64.One,
            Limit * Fixed64.Two - RawUnit * (reject ? 1 : 3), Fixed64.One);
        Action initialize = () => collider.InitializeWithNoBody(new TestMatterAgent(context));
        if (reject)
            initialize.Should().Throw<ArgumentException>();
        else
        {
            initialize.Should().NotThrow();
            collider.CanonicalCenteredProxyRadius.Should().Be(Limit);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ReservedLeafMutation_ValidatesWholeOwnerBeforeWaking(bool planar)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        if (planar)
        {
            var collider = new LSCompoundCollider2D(
                CompoundColliderPart2D.Circle(Fixed64.One, new Vector2d(4094, 0)));
            var body = new SolidBody2D(new TestMatterAgent(context), collider);
            body.Initialize(Vector2d.Zero);
            body.Mass = Fixed64.One;
            body.Sleep();
            body.IsSleeping.Should().BeTrue();
            var part = (LSCircleCollider2D)collider.GetPartCollider(0);
            Action mutate = () => part.Radius = (Fixed64)3;
            mutate.Should().Throw<ArgumentException>();
            part.Radius.Should().Be(Fixed64.One);
            body.IsSleeping.Should().BeTrue();
        }
        else
        {
            var collider = new LSCompoundCollider(
                CompoundColliderPart.Sphere(Fixed64.One, new Vector3d(4094, 0, 0)));
            var body = new SolidBody(new TestMatterAgent(context), collider);
            body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
            body.Mass = Fixed64.One;
            body.Sleep();
            body.IsSleeping.Should().BeTrue();
            var part = (LSSphereCollider)collider.GetPartCollider(0);
            Action mutate = () => part.Radius = (Fixed64)3;
            mutate.Should().Throw<ArgumentException>();
            part.Radius.Should().Be(Fixed64.One);
            body.IsSleeping.Should().BeTrue();
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    [InlineData(5)]
    [InlineData(6)]
    public void SpatialMutation_RejectsWithoutPublishingOrWaking(int family)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCollider collider = family switch
        {
            0 => new LSSphereCollider(),
            1 => new LSCapsuleCollider(),
            2 => new LSCuboidCollider(),
            3 => new LSConeCollider(),
            4 => new LSCylinderCollider(),
            5 => MeshTestFixtures.CreateConvexCube(),
            _ => new LSCompoundCollider(CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero))
        };
        var body = new SolidBody(new TestMatterAgent(context), collider) { Mass = Fixed64.One };
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
        body.Sleep();
        var bounds = collider.Bounds;
        var inertia = body.InverseInertiaTensor;
        int id = collider.Id;
        uint version = collider.RuntimeShapeVersion;
        Action mutate = family switch
        {
            0 => () => collider.Radius = Limit + RawUnit,
            3 => () => collider.Radius = Limit,
            1 or 4 => () => collider.Size = new Vector3d(Fixed64.One, (Fixed64)8193, Fixed64.One),
            2 => () => collider.Size = Vector3d.One * (Fixed64)8192,
            _ => () => collider.LocalOffset = new Vector3d(Limit, Fixed64.Zero, Fixed64.Zero)
        };
        mutate.Should().Throw<ArgumentException>();
        collider.Bounds.Should().Be(bounds);
        collider.Id.Should().Be(id);
        collider.RuntimeShapeVersion.Should().Be(version);
        body.InverseInertiaTensor.Should().Be(inertia);
        body.IsSleeping.Should().BeTrue();
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    public void PlanarMutation_RejectsWithoutPublishingOrWaking(int family)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCollider2D collider = family switch
        {
            0 => new LSCircleCollider2D(Fixed64.One),
            1 => new LSCapsuleCollider2D(Fixed64.Half, Fixed64.Two),
            2 => new LSAABBoxCollider2D(Vector2d.One),
            3 => new LSPolygonCollider2D(Vector2d.Zero, Vector2d.Right, Vector2d.Forward),
            _ => new LSCompoundCollider2D(CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero))
        };
        var body = new SolidBody2D(new TestMatterAgent(context), collider) { Mass = Fixed64.One };
        body.Initialize(Vector2d.Zero);
        body.Sleep();
        var bounds = collider.Bounds;
        Fixed64 inertia = body.MomentOfInertia;
        uint version = collider.RuntimeShapeVersion;
        Action mutate = family switch
        {
            0 => () => ((LSCircleCollider2D)collider).Radius = Limit + RawUnit,
            1 => () => ((LSCapsuleCollider2D)collider).Height = (Fixed64)8193,
            2 => () => ((LSAABBoxCollider2D)collider).Size = Vector2d.One * (Fixed64)8192,
            _ => () => collider.LocalOffset = new Vector2d(Limit, Fixed64.Zero)
        };
        mutate.Should().Throw<ArgumentException>();
        collider.Bounds.Should().Be(bounds);
        collider.RuntimeShapeVersion.Should().Be(version);
        body.MomentOfInertia.Should().Be(inertia);
        body.IsSleeping.Should().BeTrue();
    }

    private static SolidBody CreatePendingCompensatedSphere(GravitasWorldContext context, bool pending = true)
    {
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One * (pending ? Limit : Fixed64.One));
        var collider = new LSSphereCollider { Radius = pending ? Fixed64.Half : Fixed64.Quarter,
            LocalOffset = pending ? Vector3d.Zero : Vector3d.Right * Fixed64.Half };
        var source = new SolidBody(new TestMatterAgent(context, transform), collider);
        source.Initialize(Vector3d.Zero, FixedQuaternion.Identity, BodyMotionType.Kinematic);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        if (!pending) return source;
        transform.LocalScale = Vector3d.One * RawUnit;
        collider.Radius = (Fixed64)(1 << 30);
        collider.LocalOffset = new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero);
        return source;
    }
}
