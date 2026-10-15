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

public sealed partial class MixedQueryCcdTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void CircleSlabWitness_WithWideSourceOffset_ShouldResolveVerticalProjectionIndependently(
        bool horizontalOverflow, bool reflect)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 sign = reflect ? -Fixed64.One : Fixed64.One;
        Vector3d targetCenter = (horizontalOverflow ? new Vector3d(-10, -450, 0)
            : new Vector3d(-450, -10, 0)) * sign;
        FixedQuaternion rotation = horizontalOverflow
            ? new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, (Fixed64)(-5), (Fixed64)6).Normalized
            : new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, -Fixed64.One, (Fixed64)10).Normalized;
        var target = new LSCuboidCollider { Size = new Vector3d(1122, 2, 2) };
        target.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(targetCenter, rotation, Vector3d.One)));
        Fixed64 edge = reflect ? Fixed64.MaxValue : Fixed64.MinValue;
        var start = new Vector2d(edge, Fixed64.Zero);
        var center = new Vector3d(edge, edge, Fixed64.Zero);

        // A wide source can intersect one part of a finite solid while its
        // nearest Euclidean boundary lies beyond one source-coordinate span.
        context.QueryMixed.TrySweepCircleAgainst3DCollider(target, start,
            start + Vector2d.Right * sign, Vector2d.Right * sign, Fixed64.One,
            Fixed64.MaxValue, edge, Fixed64.MaxValue, Vector3d.Right * sign,
            null, out PhysicsMixedHit hit).Should().BeTrue();

        hit.Distance.Should().Be(Fixed64.Zero);
        hit.Anchor3D.TryGetOffsetFrom(center, out _).Should().BeFalse();
        hit.TryGetPoint2D(out Vector3d planarWitness).Should().BeTrue();
        if (horizontalOverflow)
            planarWitness.Y.Should().Be(hit.Point3D.Y);
        else
            planarWitness.Y.Should().Be(edge + Fixed64.MaxValue * sign);
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(2, false)]
    [InlineData(2, true)]
    [InlineData(3, false)]
    [InlineData(3, true)]
    [InlineData(4, false)]
    [InlineData(4, true)]
    public void ContainedCircleSweep_ShouldKeepSolidOutwardNormalRegardlessOfTravel(int shape, bool escaping)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider target = CreateBodyless3D(context, CreateContainedSweepTarget(shape), Vector3d.Zero);
        Vector2d start = new(Fixed64.FromFraction(1, 10), Fixed64.Zero);
        Vector2d travel = escaping ? Vector2d.Right : Vector2d.Left;
        Fixed64 radius = Fixed64.FromFraction(1, 20), thickness = Fixed64.FromFraction(1, 10);

        context.QueryMixed.SweepCircleAgainst3D(start, start + travel, radius,
            Fixed64.Zero, thickness, IncludeLayerZero, out PhysicsMixedHit hit).Should().BeTrue();

        hit.Collider3D.Should().BeSameAs(target);
        hit.Distance.Should().Be(Fixed64.Zero);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
        hit.Normal3DTo2D.X.Should().BeGreaterThan(Fixed64.Zero);
        hit.NormalFor2DSource.Should().Be(Vector2d.Right);
        hit.Point3D.X.Should().BeGreaterThan(start.X);
        hit.Point2D.X.Should().Be(start.X - radius);
        var hits = new SwiftList<PhysicsMixedHit>();
        context.QueryMixed.SweepCircleAgainst3DAll(start, start + travel, radius,
            Fixed64.Zero, thickness, IncludeLayerZero, hits).Should().Be(1);
        hits[0].Normal3DTo2D.Should().Be(hit.Normal3DTo2D);
        context.QueryMixed.SweepCircleAgainstStatic3DAll(start, start + travel, radius,
            Fixed64.Zero, thickness, IncludeLayerZero, hits, requireClosingContact: true)
            .Should().Be(escaping ? 0 : 1);

        for (int i = 0; i < 4; i++)
            context.QueryMixed.SweepCircleAgainst3DAll(start, start + travel, radius,
                Fixed64.Zero, thickness, IncludeLayerZero, hits);
        long before = System.GC.GetAllocatedBytesForCurrentThread();
        int total = 0;
        for (int i = 0; i < 8; i++)
            total += context.QueryMixed.SweepCircleAgainst3DAll(start, start + travel, radius,
                Fixed64.Zero, thickness, IncludeLayerZero, hits);
        long allocated = System.GC.GetAllocatedBytesForCurrentThread() - before;
        total.Should().Be(8);
        allocated.Should().Be(0);
    }

    private static LSCollider CreateContainedSweepTarget(int shape) => shape switch
    {
        0 => new LSSphereCollider { Radius = Fixed64.Half },
        1 => new LSCapsuleCollider { Radius = Fixed64.Half, Size = new Vector3d(1, 2, 1) },
        2 => new LSCylinderCollider { Radius = Fixed64.Half, Size = Vector3d.One },
        3 => new LSConeCollider { Radius = Fixed64.Half, Size = Vector3d.One },
        _ => new LSCuboidCollider { Size = Vector3d.One }
    };

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContainedSphereCuboidSweep_ShouldKeepTheSameOutwardNormal(bool escaping)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCuboidCollider target = CreateBodyless3D(context,
            new LSCuboidCollider { Size = Vector3d.One }, Vector3d.Zero);
        Vector3d start = Vector3d.Right * Fixed64.FromFraction(1, 10);
        Vector3d travel = escaping ? Vector3d.Right : Vector3d.Left;

        context.Query3D.SweepSphere(start, Fixed64.FromFraction(1, 20), travel, Fixed64.One,
            out Physics3DHit hit, IncludeLayerZero).Should().BeTrue();

        hit.Distance.Should().Be(Fixed64.Zero);
        hit.Normal.Should().Be(Vector3d.Right);
        hit.Point.Should().Be(Vector3d.Right * Fixed64.Half);
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(2, false)]
    [InlineData(2, true)]
    [InlineData(3, false)]
    [InlineData(3, true)]
    [InlineData(4, false)]
    [InlineData(4, true)]
    public void ContainedCircleCcd_FullHostLoop_ShouldPermitEscapeAndBlockDeeperTravel(int shape, bool escaping)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = context.Environment.AirDensity = context.Environment.DampingFactor = Fixed64.Zero;
        LSCollider target = CreateContainedSweepTarget(shape);
        target.Material = PhysicsMaterial.Frictionless;
        CreateBodyless3D(context, target, Vector3d.Zero);
        Vector2d start = new(Fixed64.FromFraction(1, 10), Fixed64.Zero);
        var circle = new LSCircleCollider2D(Fixed64.FromFraction(1, 20))
        {
            MixedHalfThicknessOverride = Fixed64.FromFraction(1, 10),
            Material = PhysicsMaterial.Frictionless
        };
        var body = new SolidBody2D(new TestMatterAgent(context, new FixedTransform(
            new Vector3d(start.X, Fixed64.Zero, start.Y), FixedQuaternion.Identity, Vector3d.One)), circle)
        {
            Mass = Fixed64.One,
            FreezeAxes = BodyFreezeAxes2D.Rotation,
            ContinuousCollisionMode = ContinuousCollisionMode.Continuous
        };
        body.Initialize(start);
        body.UseManualGrounding();
        body.ApplyCollisionLinearVelocityDelta(escaping ? Vector2d.Right : Vector2d.Left);

        context.Simulate();
        context.LateSimulate();

        body.LinearVelocity.Should().Be(escaping ? Vector2d.Right : Vector2d.Zero);
        if (escaping) body.Position.Should().Be(start + Vector2d.Right);
        body.LastContinuousCollisionToiIterationCount.Should().Be(escaping ? 0 : 1);
        body.Agent.Transform.WorldPosition.Y.Should().Be(Fixed64.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContainedCircleCompoundSweep_ShouldPreserveSelectedLeafAndClosingPolicy(bool escaping)
    {
        using GravitasWorldContext context = CreateMixedContext();
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Cylinder(Fixed64.Half, Fixed64.One, Vector3d.Zero),
            CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Right * 3));
        CreateBodyless3D(context, compound, Vector3d.Zero);
        Vector2d start = new(Fixed64.FromFraction(1, 10), Fixed64.Zero);
        Vector2d end = start + (escaping ? Vector2d.Right : Vector2d.Left);
        var hits = new SwiftList<PhysicsMixedHit>();

        context.QueryMixed.SweepCircleAgainst3D(start, end, Fixed64.FromFraction(1, 20),
            Fixed64.Zero, Fixed64.FromFraction(1, 10), IncludeLayerZero, out PhysicsMixedHit hit).Should().BeTrue();

        hit.Collider3D.Should().BeSameAs(compound);
        hit.Normal3DTo2D.Should().Be(Vector3d.Right);
        hit.Point3D.Should().Be(Vector3d.Right * Fixed64.Half);
        context.QueryMixed.SweepCircleAgainstStatic3DAll(start, end, Fixed64.FromFraction(1, 20),
            Fixed64.Zero, Fixed64.FromFraction(1, 10), IncludeLayerZero, hits, requireClosingContact: true)
            .Should().Be(escaping ? 0 : 1);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(0)]
    [InlineData(1)]
    public void CuboidSurfaceNormal_OneRawBoundary_ShouldKeepContainingAndExteriorSides(int rawOffset)
    {
        using GravitasWorldContext context = CreateMixedContext();
        CreateBodyless3D(context, new LSCuboidCollider { Size = Vector3d.One }, Vector3d.Zero);
        Vector2d start = new(Fixed64.Half + Fixed64.FromRaw(rawOffset), Fixed64.Zero);

        context.QueryMixed.SweepCircleAgainst3D(start, start + Vector2d.Right, Fixed64.FromFraction(1, 20),
            Fixed64.Zero, Fixed64.FromFraction(1, 10), IncludeLayerZero, out PhysicsMixedHit hit).Should().BeTrue();

        hit.Normal3DTo2D.Should().Be(Vector3d.Right);
        hit.Point3D.Should().Be(Vector3d.Right * Fixed64.Half);
    }

    [Fact]
    public void ExteriorCuboidCornerSweep_ShouldPreserveDiagonalSeparationNormal()
    {
        using GravitasWorldContext context = CreateMixedContext();
        CreateBodyless3D(context, new LSCuboidCollider { Size = Vector3d.One }, Vector3d.Zero);
        Vector2d start = new(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(3, 5));

        context.QueryMixed.SweepCircleAgainst3D(start, start + Vector2d.One, Fixed64.FromFraction(1, 5),
            Fixed64.Zero, Fixed64.FromFraction(1, 10), IncludeLayerZero, out PhysicsMixedHit hit).Should().BeTrue();

        hit.Normal3DTo2D.Should().Be(new Vector3d(1, 0, 1).Normalized);
        hit.Point3D.Should().Be(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Half));
    }

    [Fact]
    public void StandaloneAndCompoundCuboidSweeps_ShouldKeepTheSameExteriorCornerConstraint()
    {
        for (int shape = 0; shape < 2; shape++)
        {
            using GravitasWorldContext context = CreateMixedContext();
            LSCollider target = shape == 0 ? new LSCuboidCollider { Size = Vector3d.One }
                : new LSCompoundCollider(CompoundColliderPart.Cuboid(Vector3d.One, Vector3d.Zero));
            CreateBodyless3D(context, target, Vector3d.Zero);
            Vector3d start = new(Fixed64.FromFraction(3, 5), Fixed64.Zero, Fixed64.FromFraction(3, 5));
            context.Query3D.SweepSphere(start, Fixed64.FromFraction(1, 5), -Vector3d.Forward,
                Fixed64.One, out Physics3DHit hit, IncludeLayerZero).Should().BeTrue();
            hit.Collider.Should().BeSameAs(target);
            hit.Distance.Should().Be(Fixed64.Zero);
            hit.Normal.Should().Be(new Vector3d(1, 0, 1).Normalized);
            hit.Point.Should().Be(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Half));
            Vector3d.Dot(hit.Normal, -Vector3d.Forward).Should().BeLessThan(Fixed64.Zero);
        }
    }

    [Fact]
    public void ExteriorCylinderRimSweep_ShouldKeepItsDiagonalSeparationNormal()
    {
        using GravitasWorldContext context = CreateMixedContext();
        CreateBodyless3D(context, new LSCylinderCollider { Size = Vector3d.One, Radius = Fixed64.Half }, Vector3d.Zero);
        Vector2d start = new(Fixed64.FromFraction(3, 5), Fixed64.Zero);
        context.QueryMixed.SweepCircleAgainst3D(start, start + Vector2d.Right, Fixed64.FromFraction(1, 5),
            Fixed64.FromFraction(3, 5), Fixed64.FromFraction(1, 5), IncludeLayerZero, out PhysicsMixedHit hit)
            .Should().BeTrue();
        hit.Normal3DTo2D.Should().Be(new Vector3d(1, 1, 0).Normalized);
        hit.Point3D.Should().Be(new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Zero));
    }

    [Fact]
    public void ExteriorConeApexSweep_ShouldKeepVerticalNormalAndPermitPlanarTravel()
    {
        using GravitasWorldContext context = CreateMixedContext();
        CreateBodyless3D(context, new LSConeCollider { Size = Vector3d.One, Radius = Fixed64.Half }, Vector3d.Zero);
        var hits = new SwiftList<PhysicsMixedHit>();

        context.QueryMixed.SweepCircleAgainst3D(Vector2d.Zero, Vector2d.Right, Fixed64.FromFraction(1, 20),
            Fixed64.FromFraction(3, 5), Fixed64.FromFraction(1, 10), IncludeLayerZero, out PhysicsMixedHit hit)
            .Should().BeTrue();

        hit.Normal3DTo2D.Should().Be(Vector3d.Up);
        hit.Point3D.Should().Be(Vector3d.Up * Fixed64.Half);
        context.QueryMixed.SweepCircleAgainstStatic3DAll(Vector2d.Zero, Vector2d.Right,
            Fixed64.FromFraction(1, 20), Fixed64.FromFraction(3, 5), Fixed64.FromFraction(1, 10),
            IncludeLayerZero, hits, requireClosingContact: true).Should().Be(0);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContainedSphereSlabWitness_NearScalarLimit_ShouldRetainConceptualSurface(bool upperLimit)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 sign = upperLimit ? Fixed64.One : -Fixed64.One;
        Fixed64 x = upperLimit ? Fixed64.MaxValue - Fixed64.FromFraction(1, 8)
            : Fixed64.MinValue + Fixed64.FromFraction(1, 8);
        LSSphereCollider sphere = CreateBodyless3D(context, new LSSphereCollider { Radius = Fixed64.Half },
            new Vector3d(x, Fixed64.Zero, Fixed64.Zero));
        Vector2d start = new(x + sign * Fixed64.FromFraction(1, 10), Fixed64.Zero);
        Vector2d end = start - new Vector2d(sign * Fixed64.One, Fixed64.Zero);

        GravitasQueryMixedService.TrySweepCircleAgainstSphere(start, end, Fixed64.One,
            Fixed64.FromFraction(1, 20), Fixed64.Zero, Fixed64.FromFraction(1, 10),
            -Vector3d.Right * sign, sphere, null, out PhysicsMixedHit hit).Should().BeTrue();

        hit.Normal3DTo2D.Should().Be(Vector3d.Right * sign);
        hit.TryGetPoint3D(out _).Should().BeFalse();
        hit.Anchor3D.TryGetOffsetFrom(new Vector3d(x, Fixed64.Zero, Fixed64.Zero), out Vector3d offset)
            .Should().BeTrue();
        offset.Should().Be(Vector3d.Right * sign * Fixed64.Half);
        hit.TryGetPoint2D(out _).Should().BeTrue();
    }

    [Fact]
    public void ExteriorSphereWitness_RoundingToTheSourceCenter_ShouldKeepTheGeometricNormal()
    {
        using GravitasWorldContext context = CreateMixedContext();
        Fixed64 raw = Fixed64.FromRaw(1);
        CreateBodyless3D(context, new LSSphereCollider { Radius = raw * 10 }, Vector3d.Zero);
        Vector2d start = new(raw, Fixed64.Zero);

        context.QueryMixed.SweepCircleAgainst3D(start, start + Vector2d.Right, raw * 2,
            raw * 10, raw * 3, IncludeLayerZero, out PhysicsMixedHit hit).Should().BeTrue();

        hit.Point3D.Should().Be(new Vector3d(raw, raw * 10, Fixed64.Zero));
        hit.Normal3DTo2D.X.Should().BeGreaterThan(Fixed64.Zero);
        hit.Normal3DTo2D.Y.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RotatedSphereAndZeroCoreCapsuleSweeps_ShouldKeepMatchingAnchorsAndNormals(bool mixed)
    {
        FixedQuaternion rotation = new FixedQuaternion(Fixed64.Zero, -Fixed64.One, Fixed64.Zero, (Fixed64)2).Normalized;
        Vector2d start = Vector2d.Right * Fixed64.FromFraction(1, 10);
        Vector3d expectedNormal = Vector3d.Right;
        Vector3d expectedPoint = Vector3d.Right * Fixed64.Half;
        for (int shape = 0; shape < 2; shape++)
        {
            using GravitasWorldContext context = CreateMixedContext();
            LSCollider target = shape == 0 ? new LSSphereCollider { Radius = Fixed64.Half }
                : new LSCapsuleCollider { Radius = Fixed64.Half, Size = Vector3d.One };
            target.InitializeWithNoBody(new TestMatterAgent(context, new FixedTransform(Vector3d.Zero, rotation, Vector3d.One)));
            if (mixed)
            {
                context.QueryMixed.SweepCircleAgainst3D(start, start + Vector2d.Right, Fixed64.FromFraction(1, 20),
                    Fixed64.Zero, Fixed64.FromFraction(1, 10), IncludeLayerZero, out PhysicsMixedHit hit).Should().BeTrue();
                hit.Normal3DTo2D.Should().Be(expectedNormal);
                hit.Point3D.Should().Be(expectedPoint);
            }
            else
            {
                context.Query3D.SweepSphere(new Vector3d(start.X, Fixed64.Zero, Fixed64.Zero), Fixed64.FromFraction(1, 20), Vector3d.Right,
                    Fixed64.One, out Physics3DHit hit, IncludeLayerZero).Should().BeTrue();
                hit.Normal.Should().Be(expectedNormal);
                hit.Point.Should().Be(expectedPoint);
            }
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CenteredSphereAndZeroCoreCapsuleSweeps_ShouldSelectTheSameCanonicalSurface(bool mixed)
    {
        Physics3DHit spatial = default;
        PhysicsMixedHit planar = default;
        for (int shape = 0; shape < 2; shape++)
        {
            using GravitasWorldContext context = CreateMixedContext();
            LSCollider target = shape == 0 ? new LSSphereCollider { Radius = Fixed64.Half }
                : new LSCapsuleCollider { Radius = Fixed64.Half, Size = Vector3d.One };
            CreateBodyless3D(context, target, Vector3d.Zero);
            if (mixed)
            {
                context.QueryMixed.SweepCircleAgainst3D(Vector2d.Zero, Vector2d.Right, Fixed64.FromFraction(1, 20),
                    Fixed64.Zero, Fixed64.FromFraction(1, 10), IncludeLayerZero, out planar).Should().BeTrue();
                planar.Normal3DTo2D.Should().Be(Vector3d.Right);
                planar.Point3D.Should().Be(Vector3d.Right * Fixed64.Half);
            }
            else
            {
                context.Query3D.SweepSphere(Vector3d.Zero, Fixed64.FromFraction(1, 20), Vector3d.Right,
                    Fixed64.One, out spatial, IncludeLayerZero).Should().BeTrue();
                spatial.Normal.Should().Be(Vector3d.Right);
                spatial.Point.Should().Be(Vector3d.Right * Fixed64.Half);
            }
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void SupportedRotatedCuboidFaceWitness_ShouldProjectAndClampItsComponentsIndependently(bool swapAxes, bool negative)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 extent = (Fixed64)2048;
        FixedQuaternion rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, -Fixed64.One,
            swapAxes ? (Fixed64)3 : (Fixed64)2).Normalized;
        Vector3d axis = rotation.Rotate(Vector3d.Up);
        Vector3d capCenter = new(extent * Fixed64.FromFraction(9, 10), extent * Fixed64.FromFraction(9, 10), Fixed64.Zero);
        Vector3d boxCenter = capCenter + axis * (extent * Fixed64.FromFraction(1, 20));
        if (negative) boxCenter = -boxCenter;
        var box = new LSCuboidCollider
        {
            Size = new Vector3d(extent * Fixed64.Half, extent * Fixed64.FromFraction(1, 10), extent * Fixed64.Half)
        };
        box.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(boxCenter, rotation, Vector3d.One)));

        context.QueryMixed.TrySweepCircleAgainst3DCollider(box, Vector2d.Zero, Vector2d.Right,
            Vector2d.Right, Fixed64.One, extent * Fixed64.FromFraction(19, 20), Fixed64.Zero,
            extent * Fixed64.FromFraction(19, 20), Vector3d.Right, null, out PhysicsMixedHit hit).Should().BeTrue();

        hit.Anchor3D.TryGetOffsetFrom(Vector3d.Zero, out _).Should().BeTrue();
        if (swapAxes)
            hit.Point2D.Y.Should().Be((negative ? -extent : extent) * Fixed64.FromFraction(19, 20));
        else
            hit.Point2D.Y.Should().BeGreaterThan(extent * Fixed64.FromFraction(3, 4))
                .And.BeLessThan(extent * Fixed64.FromFraction(4, 5));
        (hit.Normal3DTo2D.X * (negative ? -Fixed64.One : Fixed64.One)).Should().BeLessThan(Fixed64.Zero);
        (hit.Normal3DTo2D.Y * (negative ? -Fixed64.One : Fixed64.One)).Should().BeLessThan(Fixed64.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MeshSurfaceSweep_ShouldKeepItsGeometricSideWithoutSolidContainment(bool below)
    {
        using GravitasWorldContext context = CreateMixedContext();
        var mesh = new LSMeshCollider(new[]
        {
            new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2),
            new Vector3d(-2, 0, 2), new Vector3d(2, 0, 2)
        }, new[] { 0, 2, 1, 1, 2, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        CreateBodyless3D(context, mesh, Vector3d.Zero);
        Fixed64 y = below ? -Fixed64.FromFraction(1, 10) : Fixed64.FromFraction(1, 10);
        mesh.GetClosestSurfaceAnchor(new Vector3d(Fixed64.Zero, y, Fixed64.Zero), out _, out bool contained);
        contained.Should().BeFalse();

        context.QueryMixed.SweepCircleAgainst3D(Vector2d.Zero, Vector2d.Right, Fixed64.FromFraction(1, 20),
            y, Fixed64.FromFraction(1, 5), IncludeLayerZero, out PhysicsMixedHit hit).Should().BeTrue();

        hit.Normal3DTo2D.Should().Be(below ? -Vector3d.Up : Vector3d.Up);
    }
}
