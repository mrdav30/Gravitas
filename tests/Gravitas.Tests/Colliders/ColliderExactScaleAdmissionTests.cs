using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class ColliderExactScaleAdmissionTests
{
    [Fact]
    public void CustomColliderCanonicalBounds_ShouldRemainConservative()
    {
        var collider3D = new UnsupportedTestCollider3D();
        var collider2D = new UnsupportedTestCollider2D();

        collider3D.TryGetLocalScale(out Vector3d detached3DScale)
            .Should().BeTrue();
        detached3DScale.Should().Be(Vector3d.One);

        ColliderCanonicalBounds.GetCurrentCenteredProxyRadius(collider3D)
            .Should().Be(Fixed64.MaxValue);
        ColliderCanonicalBounds.GetCenteredProxyRadius(collider3D)
            .Should().Be(Fixed64.MaxValue);
        FixedBoundBox bounds =
            ColliderCanonicalBounds.GetRelativeBounds(
                collider3D,
                Vector3d.Zero);
        bounds.Min.Should().Be(new Vector3d(
            Fixed64.MinValue,
            Fixed64.MinValue,
            Fixed64.MinValue));
        bounds.Max.Should().Be(new Vector3d(
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Fixed64.MaxValue));
        ColliderCanonicalBounds2D.GetCurrentCenteredProxyRadius(collider2D)
            .Should().Be(Fixed64.MaxValue);
        ColliderCanonicalBounds2D.GetCenteredProxyRadius(collider2D)
            .Should().Be(Fixed64.MaxValue);
        ColliderCanonicalBounds2D.GetGroundProbeRadius(collider2D)
            .Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void DetachedPlanarMixedThicknessOverride_ShouldPublishIntoCanonicalSlab()
    {
        var collider = new LSCircleCollider2D(Fixed64.One)
        {
            MixedHalfThicknessOverride = Fixed64.Two
        };

        collider.RebuildRuntimeShapeOnly().Should().BeTrue();

        collider.MixedHalfThickness.Should().Be(Fixed64.Two);
        collider.MixedBounds3D.Min.Y.Should().Be(-Fixed64.Two);
        collider.MixedBounds3D.Max.Y.Should().Be(Fixed64.Two);
    }

    [Fact]
    public void PlanarNearZeroClamp_ShouldTreatComponentsIndependently()
    {
        PublishObservingCollider2D.ClampForTest(
                new Vector2d(Fixed64.Epsilon, Fixed64.Two))
            .Should().Be(new Vector2d(Fixed64.Zero, Fixed64.Two));
        PublishObservingCollider2D.ClampForTest(
                new Vector2d(Fixed64.Two, -Fixed64.Epsilon))
            .Should().Be(new Vector2d(Fixed64.Two, Fixed64.Zero));
    }

    [Fact]
    public void UncommittedProxyRadii_ShouldCoverFiniteAndCompoundFullDomainShapes()
    {
        var cylinder = new LSCylinderCollider
        {
            Radius = Fixed64.MaxValue,
            Size = Vector3d.One * Fixed64.MaxValue
        };
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero),
            CompoundColliderPart.Cuboid(
                Vector3d.One * Fixed64.MaxValue,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.One * Fixed64.Two));

        ColliderCanonicalBounds.GetCurrentCenteredProxyRadius(cylinder)
            .Should().Be(Fixed64.MaxValue);
        compound.ScaledRadius.Should().Be(Fixed64.MaxValue);
    }

    [Fact]
    public void DetachedCollider_ExposesAuthoredScaleWithoutSynthesizingRuntimeGeometry()
    {
        var collider = new UnsupportedTestCollider3D
        {
            Radius = Fixed64.Two,
            LocalOffset = new Vector3d(
                Fixed64.One,
                Fixed64.Two,
                (Fixed64)3)
        };

        collider.TryGetLocalScale(out Vector3d localScale).Should().BeTrue();
        localScale.Should().Be(Vector3d.One);
        collider.LocalScale.Should().Be(localScale);
        collider.ScaledRadius.Should().Be(Fixed64.Two);
        collider.ScaledOffset.Should().Be(collider.LocalOffset);
        Action readCenter = () => _ = collider.Center;
        readCenter.Should().Throw<InvalidOperationException>();
    }

    [Fact]
    public void CommittedCompoundCuboidProxyRadius_ShouldClampAnUnrepresentableDiagonal()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Cuboid(
                Vector3d.One * Fixed64.MaxValue,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.One * Fixed64.Two));
        compound.InitializeWithNoBody(new TestMatterAgent(context));
        var cuboid = (LSCuboidCollider)compound.GetPartCollider(0);

        ColliderCanonicalBounds.GetCenteredProxyRadius(cuboid)
            .Should()
            .Be(Fixed64.MaxValue);
    }

    [Fact]
    public void CompoundCircleGroundProbe_ShouldClampUnrepresentableRelativeBounds()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(
                Fixed64.MaxValue,
                Vector2d.Right));
        compound.InitializeWithNoBody(new TestMatterAgent(context));

        ColliderCanonicalBounds2D.GetGroundProbeRadius(compound)
            .Should()
            .Be(Fixed64.MaxValue);
    }

    [Fact]
    public void StrictHierarchyOverflow_ShouldRejectBeforeEitherDimensionalRegistration()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var parent = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.MaxValue, Fixed64.One, Fixed64.MaxValue));
        var child = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Two, Fixed64.One, Fixed64.Two),
            parent);
        var collider3D = new LSSphereCollider { Radius = Fixed64.FromRaw(1) };
        var collider2D = new LSCircleCollider2D(Fixed64.FromRaw(1));

        Action initialize3D = () =>
            collider3D.InitializeWithNoBody(new TestMatterAgent(context, child));
        Action initialize2D = () =>
            collider2D.InitializeWithNoBody(new TestMatterAgent(context, child));

        initialize3D.Should().Throw<ArgumentException>().WithParameterName("transform");
        initialize2D.Should().Throw<ArgumentException>().WithParameterName("transform");
        collider3D.Id.Should().Be(-1);
        collider2D.Id.Should().Be(-1);
        collider3D.HasHostBinding.Should().BeFalse();
        collider2D.HasHostBinding.Should().BeFalse();
        context.Physics.ColliderCount.Should().Be(0);
        context.Physics2D.ColliderCount.Should().Be(0);
    }

    [Fact]
    public void ShearedHierarchy_ShouldRejectBeforeEitherDimensionalRegistration()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var parent = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Two, Fixed64.One, Fixed64.One));
        var child = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4),
            Vector3d.One,
            parent);
        var collider3D = new LSSphereCollider();
        var collider2D = new LSCircleCollider2D(Fixed64.Half);

        Action initialize3D = () =>
            collider3D.InitializeWithNoBody(new TestMatterAgent(context, child));
        Action initialize2D = () =>
            collider2D.InitializeWithNoBody(new TestMatterAgent(context, child));

        initialize3D.Should().Throw<ArgumentException>().WithParameterName("transform");
        initialize2D.Should().Throw<ArgumentException>().WithParameterName("transform");
        collider3D.HasHostBinding.Should().BeFalse();
        collider2D.HasHostBinding.Should().BeFalse();
        context.Physics.ColliderCount.Should().Be(0);
        context.Physics2D.ColliderCount.Should().Be(0);
    }

    [Fact]
    public void NonPlanar2DHierarchy_ShouldRejectBeforeRegistration()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.FromAxisAngle(Vector3d.Right, Fixed64.PiOver4),
            Vector3d.One);
        var collider = new LSCircleCollider2D(Fixed64.Half);

        Action initialize = () =>
            collider.InitializeWithNoBody(new TestMatterAgent(context, transform));

        initialize.Should().Throw<ArgumentException>().WithParameterName("transform");
        collider.HasHostBinding.Should().BeFalse();
        context.Physics2D.ColliderCount.Should().Be(0);
    }

    [Fact]
    public void TinyNonPlanar2DTilt_ShouldRejectAndRollBackRegistration()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.FromAxisAngle(
                Vector3d.Right,
                Fixed64.FromRaw(2L)),
            Vector3d.One);
        transform.TryGetLocalToWorldMatrix(out Fixed4x4 matrix).Should().BeTrue();
        FixedMath.Max(FixedMath.Abs(matrix.M12), FixedMath.Abs(matrix.M32))
            .Should()
            .BeGreaterThan(Fixed64.Zero)
            .And
            .BeLessThanOrEqualTo(Fixed64.Epsilon);
        var collider = new LSCircleCollider2D(Fixed64.Half);

        Action initialize = () =>
            collider.InitializeWithNoBody(new TestMatterAgent(context, transform));

        initialize.Should().Throw<ArgumentException>().WithParameterName("transform");
        collider.Id.Should().Be(-1);
        collider.HasHostBinding.Should().BeFalse();
        context.Physics2D.ColliderCount.Should().Be(0);
    }

    [Fact]
    public void ReflectedPlanarProjection_ShouldRejectBeforeRegistration()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.FromAxisAngle(Vector3d.Right, Fixed64.Pi),
            Vector3d.One);
        var collider = new LSCircleCollider2D(Fixed64.Half);

        Action initialize = () =>
            collider.InitializeWithNoBody(new TestMatterAgent(context, transform));

        initialize.Should().Throw<ArgumentException>().WithParameterName("transform");
        collider.HasHostBinding.Should().BeFalse();
        context.Physics2D.ColliderCount.Should().Be(0);
    }

    [Fact]
    public void CompoundCuboidHalfExtents_ShouldNotRequireRepresentableCombinedScale()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 factor = (Fixed64)65536;
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One * factor);
        var collider = new LSCompoundCollider(
            CompoundColliderPart.Cuboid(
                Vector3d.One * Fixed64.FromRaw(2),
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.One * factor));

        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));

        LSCuboidCollider cuboid = (LSCuboidCollider)collider.GetPartCollider(0);
        cuboid.Bounds.Scope.Should().Be(Vector3d.One);
        cuboid.Bounds.Min.Should().Be(-Vector3d.One);
        cuboid.Bounds.Max.Should().Be(Vector3d.One);
    }

    [Fact]
    public void CompoundRadiusAndOffset_ShouldPreserveSeparateScaleFactors()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 factor = (Fixed64)65536;
        Fixed64 authoredRadius = Fixed64.FromRaw(1);
        Fixed64 authoredOffset = Fixed64.FromFraction(1, 65536);
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One * factor);
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(
                authoredRadius,
                new Vector3d(authoredOffset, Fixed64.Zero, Fixed64.Zero),
                FixedQuaternion.Identity,
                Vector3d.One * factor));

        compound.InitializeWithNoBody(new TestMatterAgent(context, transform));

        var sphere = (LSSphereCollider)compound.GetPartCollider(0);
        sphere.ScaledRadius.Should().Be(Fixed64.One);
        sphere.ScaledOffset.Should().Be(Vector3d.Right);
        sphere.Center.Should().Be(Vector3d.Right);
        sphere.TryGetLocalScale(out Vector3d unavailableScale).Should().BeFalse();
        unavailableScale.Should().Be(Vector3d.Zero);
        Action readUnrepresentableCombinedScale = () => _ = sphere.LocalScale;
        readUnrepresentableCombinedScale
            .Should()
            .Throw<InvalidOperationException>();
        context.ComputeReplayHash()
            .Should()
            .Be(context.ComputeReplayHash());
    }

    [Fact]
    public void CompoundPlanarRadiusAndOffset_ShouldPreserveSeparateScaleFactors()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 factor = (Fixed64)65536;
        Fixed64 authoredRadius = Fixed64.FromRaw(1);
        Fixed64 authoredOffset = Fixed64.FromFraction(1, 65536);
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One * factor);
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(
                authoredRadius,
                new Vector2d(authoredOffset, Fixed64.Zero),
                Fixed64.Zero,
                Vector2d.One * factor));

        compound.InitializeWithNoBody(new TestMatterAgent(context, transform));

        var circle = (LSCircleCollider2D)compound.GetPartCollider(0);
        circle.ScaledRadius.Should().Be(Fixed64.One);
        circle.Center.Should().Be(Vector2d.Right);
        circle.TryGetLocalScale(out Vector2d unavailableScale).Should().BeFalse();
        unavailableScale.Should().Be(Vector2d.Zero);
        Action readUnrepresentableCombinedScale = () => _ = circle.LocalScale;
        readUnrepresentableCombinedScale.Should().Throw<InvalidOperationException>();
    }

    [Fact]
    public void PlanarPublishBoundary_ShouldExposeStrictScaledOffsetBeforeCommit()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d((Fixed64)3, Fixed64.One, (Fixed64)5));
        var collider = new PublishObservingCollider2D
        {
            LocalOffset = new Vector2d(Fixed64.Two, Fixed64.One)
        };

        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));

        collider.ScaleObservedDuringPublish.Should().Be(new Vector2d(
            (Fixed64)3,
            (Fixed64)5));
        collider.OffsetObservedDuringPublish.Should().Be(new Vector2d(
            (Fixed64)6,
            (Fixed64)5));
    }

    [Fact]
    public void BodylessPlanarCenter_ShouldAdmitFinalScaleOffsetCancellation()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var transform = new FixedTransform(
            new Vector3d(
                Fixed64.MinValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.One,
                Fixed64.One));
        var collider = new LSCircleCollider2D(Fixed64.FromRaw(1L))
        {
            LocalOffset = new Vector2d(Fixed64.Two, Fixed64.Zero)
        };
        Vector2d origin = new(Fixed64.MinValue, Fixed64.Zero);
        Vector2d scale = new(Fixed64.MaxValue, Fixed64.One);
        Vector2d localOffset = new(Fixed64.Two, Fixed64.Zero);
        Vector2d.TryTransformScaledPoint(
                origin,
                localOffset,
                scale,
                Fixed64.Zero,
                out Vector2d expectedCenter)
            .Should()
            .BeTrue();

        collider.InitializeWithNoBody(
            new TestMatterAgent(context, transform));

        collider.Center.Should().Be(expectedCenter);
        collider.TryGetScaledLocalOffset(out Vector2d scaledOffset)
            .Should()
            .BeFalse();
        scaledOffset.Should().Be(Vector2d.Zero);
        Action readScaledOffset = () => _ = collider.ScaledLocalOffset;
        readScaledOffset.Should().Throw<InvalidOperationException>();
        Action readBodyLocalCenterOfMass =
            () => collider.CalculateLocalCenterOfMassOffset();
        readBodyLocalCenterOfMass
            .Should()
            .Throw<InvalidOperationException>();
    }

    [Fact]
    public void BodyPosePreparation_ShouldReturnFalseWhenOnlyWorldCenterCancelsIntoRange()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        var collider = new LSSphereCollider
        {
            Radius = Fixed64.FromRaw(1L),
            LocalOffset = new Vector3d(
                Fixed64.FromRaw((long.MaxValue / 2L) + 1L),
                Fixed64.Zero,
                Fixed64.Zero)
        };
        var body = new SolidBody(
            new TestMatterAgent(context, transform),
            collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            BodyMotionType.Dynamic);
        transform.LocalScale = new Vector3d(
            Fixed64.Two,
            Fixed64.One,
            Fixed64.One);

        collider.TryPrepareBodyPose(
                new Vector3d(
                    Fixed64.MinValue,
                    Fixed64.Zero,
                    Fixed64.Zero),
                FixedQuaternion.Identity)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void BodyPlanarCenter_ShouldRejectAtTrueBodyLocalLeverBoundary()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.One,
                Fixed64.One));
        var collider = new LSCircleCollider2D(Fixed64.FromRaw(1L))
        {
            LocalOffset = new Vector2d(Fixed64.Two, Fixed64.Zero)
        };
        var body = new SolidBody2D(
            new TestMatterAgent(context, transform),
            collider);

        Action initialize = () => body.Initialize(
            new Vector2d(Fixed64.MinValue, Fixed64.Zero));

        initialize.Should().Throw<InvalidOperationException>();
        body.Active.Should().BeFalse();
        collider.Id.Should().Be(-1);
        collider.HasHostBinding.Should().BeFalse();
        context.Physics2D.ColliderCount.Should().Be(0);
    }

    [Fact]
    public void BodySpatialCenter_ShouldRejectAtTrueBodyLocalLeverBoundary()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.One,
                Fixed64.One));
        var collider = new LSSphereCollider
        {
            Radius = Fixed64.FromRaw(1L),
            LocalOffset = new Vector3d(
                Fixed64.Two,
                Fixed64.Zero,
                Fixed64.Zero)
        };
        var body = new SolidBody(
            new TestMatterAgent(context, transform),
            collider);

        Action initialize = () => body.Initialize(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity);

        initialize.Should().Throw<InvalidOperationException>();
        body.Active.Should().BeFalse();
        collider.Id.Should().Be(-1);
        collider.HasHostBinding.Should().BeFalse();
        context.Physics.ColliderCount.Should().Be(0);
    }

    [Fact]
    public void CompoundPlanarOffsets_ShouldAdmitFinalBodyLocalCancellation()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var transform = new FixedTransform(
            new Vector3d(
                Fixed64.MinValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.One,
                Fixed64.One));
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(
                Fixed64.FromRaw(1L),
                new Vector2d(-Fixed64.Two, Fixed64.Zero),
                Fixed64.Zero,
                Vector2d.One))
        {
            LocalOffset = new Vector2d(Fixed64.Two, Fixed64.Zero)
        };

        compound.InitializeWithNoBody(
            new TestMatterAgent(context, transform));

        var part = (LSCircleCollider2D)compound.GetPartCollider(0);
        part.CalculateLocalCenterOfMassOffset()
            .Should()
            .Be(Vector2d.Zero);
        compound.CalculateLocalCenterOfMassOffset()
            .Should()
            .Be(Vector2d.Zero);
        compound.TryGetScaledLocalOffset(out _).Should().BeFalse();
        part.TryGetScaledLocalOffset(out _).Should().BeFalse();
    }

    [Fact]
    public void CompoundSpatialOffsets_ShouldAdmitFinalBodyLocalCancellation()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var transform = new FixedTransform(
            new Vector3d(
                Fixed64.MinValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.One,
                Fixed64.One));
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(
                Fixed64.FromRaw(1L),
                new Vector3d(-Fixed64.Two, Fixed64.Zero, Fixed64.Zero),
                FixedQuaternion.Identity,
                Vector3d.One))
        {
            LocalOffset = new Vector3d(
                Fixed64.Two,
                Fixed64.Zero,
                Fixed64.Zero)
        };

        compound.InitializeWithNoBody(
            new TestMatterAgent(context, transform));

        var part = (LSSphereCollider)compound.GetPartCollider(0);
        part.Center.Should().Be(new Vector3d(
            Fixed64.MinValue,
            Fixed64.Zero,
            Fixed64.Zero));
        part.CalculateLocalCenterOfMassOffset()
            .Should()
            .Be(Vector3d.Zero);
        compound.CalculateLocalCenterOfMassOffset()
            .Should()
            .Be(Vector3d.Zero);
        compound.TryGetScaledOffset(out _).Should().BeFalse();
        part.TryGetScaledOffset(out _).Should().BeFalse();
    }

    private sealed class PublishObservingCollider2D : LSCollider2D
    {
        internal Vector2d ScaleObservedDuringPublish { get; private set; }

        internal Vector2d OffsetObservedDuringPublish { get; private set; }

        public override ColliderType2D Shape => (ColliderType2D)byte.MaxValue;

        public override bool ContainsPoint(Vector2d point) => false;

        public override Vector2d GetClosestPoint(Vector2d point) => Center;

        public override Vector2d GetSupportPoint(Vector2d direction) => Center;

        internal override Fixed64 CalculateCenterOfMassMoment(
            Fixed64 mass) =>
            Fixed64.Zero;

        internal static Vector2d ClampForTest(Vector2d value) =>
            ClampNearZero(value);

        internal override ExactMassWeight CalculateAreaForMassProperties() =>
            ExactMassWeight.Zero;

        internal override ExactMassWeight CalculatePreparedAreaForMassProperties() =>
            ExactMassWeight.Zero;

        private protected override void PrepareShape(
            in ColliderShapeSnapshot2D snapshot) =>
            SetPreparedBounds(
                FixedBoundArea.FromCenterAndScopeClippedToDomain(
                    snapshot.Center,
                    Vector2d.One));

        private protected override void PublishShape()
        {
            TryGetLocalScale(out Vector2d scale).Should().BeTrue();
            ScaleObservedDuringPublish = scale;
            OffsetObservedDuringPublish = ScaledLocalOffset;
        }
    }
}
