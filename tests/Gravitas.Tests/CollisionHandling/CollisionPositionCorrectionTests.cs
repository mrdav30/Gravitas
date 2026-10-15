using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using System.Numerics;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class CollisionPositionCorrectionTests
{
    [Fact]
    public void Resolve2D_WithTinyMass_ShouldRetainPenetrationSlop()
    {
        using GravitasWorldContext context = CreateContext();
        Fixed64 radius = (Fixed64)4095;
        SolidBody2D first = CreateCircle(context, radius, Fixed64.Zero);
        SolidBody2D second = CreateCircle(context, radius, Fixed64.FromRaw(3));
        var pair = new CollisionPair2D(first.Collider, second.Collider);
        pair.Manifold.SetContact(Vector2d.Right * radius, Vector2d.Left * radius,
            radius * Fixed64.Two, Vector2d.Right);

        CollisionResponse2D.Resolve(pair);

        Fixed64 expected = radius * Fixed64.Two - CollisionResponse2D.PenetrationSlop;
        second.Position.Should().Be(Vector2d.Right * expected);
        first.Position.Should().Be(Vector2d.Zero);
        second.LinearVelocity.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void Resolve3D_WithTinyMass_ShouldRetainWeightedPenetrationSlop()
    {
        using GravitasWorldContext context = CreateContext();
        Fixed64 radius = (Fixed64)4095;
        SolidBody first = CreateSphere(context, radius, Fixed64.Zero);
        SolidBody second = CreateSphere(context, radius, Fixed64.FromRaw(3));
        var pair = new CollisionPair(first.Collider, second.Collider);
        pair.Manifold.SetContact(Vector3d.Right * radius, Vector3d.Left * radius,
            radius * Fixed64.Two, Vector3d.Right);

        CollisionResponse.CalculateImpulse(pair);

        Fixed64 expected = (radius * Fixed64.Two - CollisionResponse.PenetrationSlop)
            * CollisionResponse.PenetrationCorrectionPercent;
        second.Position3d.Should().Be(Vector3d.Right * expected);
        first.Position3d.Should().Be(Vector3d.Zero);
        second.LinearVelocity.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void ResolveMixed_WithTinyMass_ShouldRetainWeightedPenetrationSlop()
    {
        using GravitasWorldContext context = CreateContext();
        Fixed64 radius = (Fixed64)4095;
        SolidBody first = CreateSphere(context, radius, Fixed64.Zero);
        SolidBody2D second = CreateCircle(context, radius, Fixed64.FromRaw(3));
        var pair = new CollisionPairMixed(first.Collider, second.Collider);
        var contact = new MixedContact(Vector3d.Right * radius, Vector3d.Left * radius,
            Vector3d.Right, radius * Fixed64.Two);
        pair.Manifold.SetContact(contact.Anchor3D, contact.Anchor2D, contact.Depth, contact.Normal3DTo2D);

        CollisionResponseMixed.Resolve(pair, contact, 0, 1,
            applyPositionCorrection: true, authoritativeNormal: true);

        Fixed64 expected = (radius * Fixed64.Two - CollisionResponseMixed.PenetrationSlop)
            * CollisionResponseMixed.PenetrationCorrectionPercent;
        second.Position.Should().Be(Vector2d.Right * expected);
        first.Position3d.Should().Be(Vector3d.Zero);
        second.LinearVelocity.Should().Be(Vector2d.Zero);
    }

    public static TheoryData<PhysicsRuntimeMode, Fixed64, Fixed64, Vector2d> MassCases
    {
        get
        {
            var cases = new TheoryData<PhysicsRuntimeMode, Fixed64, Fixed64, Vector2d>();
            foreach (PhysicsRuntimeMode mode in new[]
                { PhysicsRuntimeMode.TwoD, PhysicsRuntimeMode.ThreeD, PhysicsRuntimeMode.Mixed })
            foreach (Vector2d normal in new[] { Vector2d.Right,
                new Vector2d(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5)) })
            foreach ((Fixed64 first, Fixed64 second) in new[]
            {
                (Fixed64.Zero, Fixed64.FromRaw(3)),
                (Fixed64.FromRaw(3), Fixed64.Zero),
                (Fixed64.FromRaw(5), Fixed64.FromRaw(5)),
                (Fixed64.FromRaw(3), Fixed64.FromRaw(3)),
                (Fixed64.FromRaw(3), Fixed64.FromRaw(5)),
                (Fixed64.FromRaw(3), Fixed64.One),
                (Fixed64.One, Fixed64.FromRaw(3)),
                (Fixed64.FromRaw(1), Fixed64.One),
                (Fixed64.One, Fixed64.FromRaw(2)),
                (Fixed64.Half, Fixed64.One),
                (Fixed64.One, Fixed64.Half),
                (Fixed64.Half, Fixed64.FromFraction(1, 4)),
                (Fixed64.FromFraction(1, 4), Fixed64.Half),
                (Fixed64.One, Fixed64.One),
                (Fixed64.Zero, Fixed64.MaxValue),
                (Fixed64.MaxValue, Fixed64.Zero)
            })
                cases.Add(mode, first, second, normal);
            return cases;
        }
    }

    [Theory]
    [MemberData(nameof(MassCases))]
    public void Resolve_WithUnequalOrExtremeMasses_ShouldPreserveWeightedCorrection(
        PhysicsRuntimeMode mode, Fixed64 massA, Fixed64 massB, Vector2d normal)
    {
        using GravitasWorldContext context = CreateContext();
        Fixed64 inverseA = massA > Fixed64.Zero ? Fixed64.One / massA : Fixed64.Zero;
        Fixed64 inverseB = massB > Fixed64.Zero ? Fixed64.One / massB : Fixed64.Zero;
        Fixed64 depth = Fixed64.Two;
        Vector3d spatialNormal = new(normal.X, Fixed64.Zero, normal.Y);
        Fixed64 weightedDepth = mode == PhysicsRuntimeMode.TwoD
            ? depth - CollisionResponse2D.PenetrationSlop
            : (depth - CollisionResponse.PenetrationSlop) * CollisionResponse.PenetrationCorrectionPercent;
        Fixed64 distanceA = ExactWeightedDistance(weightedDepth, inverseA, inverseA, inverseB);
        Fixed64 distanceB = ExactWeightedDistance(weightedDepth, inverseB, inverseA, inverseB);
        Vector2d actualA, actualB;
        if (mode == PhysicsRuntimeMode.TwoD)
        {
            SolidBody2D first = CreateCircle(context, Fixed64.One, massA);
            SolidBody2D second = CreateCircle(context, Fixed64.One, massB);
            var pair = new CollisionPair2D(first.Collider, second.Collider);
            pair.Manifold.SetContact(normal, -normal, depth, normal);
            CollisionResponse2D.Resolve(pair);
            actualA = first.Position;
            actualB = second.Position;
            first.LinearVelocity.Should().Be(Vector2d.Zero);
            second.LinearVelocity.Should().Be(Vector2d.Zero);
        }
        else if (mode == PhysicsRuntimeMode.ThreeD)
        {
            SolidBody first = CreateSphere(context, Fixed64.One, massA);
            SolidBody second = CreateSphere(context, Fixed64.One, massB);
            var pair = new CollisionPair(first.Collider, second.Collider);
            pair.Manifold.SetContact(spatialNormal, -spatialNormal, depth, spatialNormal);
            CollisionResponse.CalculateImpulse(pair);
            actualA = first.Position3d.ToVector2d();
            actualB = second.Position3d.ToVector2d();
            first.Position3d.Y.Should().Be(Fixed64.Zero);
            second.Position3d.Y.Should().Be(Fixed64.Zero);
            first.LinearVelocity.Should().Be(Vector3d.Zero);
            second.LinearVelocity.Should().Be(Vector3d.Zero);
        }
        else
        {
            SolidBody first = CreateSphere(context, Fixed64.One, massA);
            SolidBody2D second = CreateCircle(context, Fixed64.One, massB);
            var pair = new CollisionPairMixed(first.Collider, second.Collider);
            var contact = new MixedContact(spatialNormal, -spatialNormal, spatialNormal, depth);
            pair.Manifold.SetContact(contact.Anchor3D, contact.Anchor2D, depth, spatialNormal);
            CollisionResponseMixed.Resolve(pair, contact, 0, 1,
                applyPositionCorrection: true, authoritativeNormal: true);
            actualA = first.Position3d.ToVector2d();
            actualB = second.Position;
            first.Position3d.Y.Should().Be(Fixed64.Zero);
            first.LinearVelocity.Should().Be(Vector3d.Zero);
            second.LinearVelocity.Should().Be(Vector2d.Zero);
        }

        // Existing compact arithmetic may round at each product; its unit-or-smaller
        // inverse masses must not amplify those errors beyond two raw units.
        Vector2d errorA = actualA + normal * distanceA;
        Vector2d errorB = actualB - normal * distanceB;
        Fixed64 tolerance = Fixed64.FromRaw(2);
        FixedMath.Abs(errorA.X).Should().BeLessThanOrEqualTo(tolerance);
        FixedMath.Abs(errorA.Y).Should().BeLessThanOrEqualTo(tolerance);
        FixedMath.Abs(errorB.X).Should().BeLessThanOrEqualTo(tolerance);
        FixedMath.Abs(errorB.Y).Should().BeLessThanOrEqualTo(tolerance);
    }

    [Theory]
    [InlineData(1L, 0L)]
    [InlineData(3L, 2L)]
    [InlineData(5L, 2L)]
    public void Resolve2D_WithEqualTinyMasses_ShouldRoundHalfCorrectionToEven(long excessRaw, long expectedRaw)
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D first = CreateCircle(context, Fixed64.One, Fixed64.FromRaw(5));
        SolidBody2D second = CreateCircle(context, Fixed64.One, Fixed64.FromRaw(5));
        var pair = new CollisionPair2D(first.Collider, second.Collider);
        pair.Manifold.SetContact(Vector2d.Right, Vector2d.Left,
            CollisionResponse2D.PenetrationSlop + Fixed64.FromRaw(excessRaw), Vector2d.Right);

        CollisionResponse2D.Resolve(pair);

        Vector2d expected = Vector2d.Right * Fixed64.FromRaw(expectedRaw);
        first.Position.Should().Be(-expected);
        second.Position.Should().Be(expected);
    }

    [Theory]
    [InlineData(PhysicsRuntimeMode.TwoD)]
    [InlineData(PhysicsRuntimeMode.ThreeD)]
    [InlineData(PhysicsRuntimeMode.Mixed)]
    public void FullStep_WithTinyMass_ShouldRetainCorrectionMargin(PhysicsRuntimeMode mode)
    {
        using GravitasWorldContext context = CreateContext();
        context.Settings.RuntimeMode = mode;
        context.World.TryAddGrid(new GridConfiguration(new Vector3d(-4, -4, -4),
            new Vector3d(4, 4, 4)), out _).Should().BeTrue();
        SolidBody2D? planar = null;
        SolidBody? spatial = null;
        if (mode == PhysicsRuntimeMode.TwoD)
        {
            _ = CreateCircle(context, Fixed64.One, Fixed64.Zero);
            planar = CreateCircle(context, Fixed64.One, Fixed64.FromRaw(3));
            planar.SetPosition(Vector2d.Right);
        }
        else
        {
            _ = CreateSphere(context, Fixed64.One, Fixed64.Zero);
            if (mode == PhysicsRuntimeMode.ThreeD)
            {
                spatial = CreateSphere(context, Fixed64.One, Fixed64.FromRaw(3));
                spatial.SetPosition(Vector3d.Right);
            }
            else
            {
                planar = CreateCircle(context, Fixed64.One, Fixed64.FromRaw(3));
                planar.SetPosition(Vector2d.Right);
            }
        }

        context.Simulate();
        context.LateSimulate();

        Fixed64 correction = mode == PhysicsRuntimeMode.TwoD
            ? Fixed64.One - CollisionResponse2D.PenetrationSlop
            : (Fixed64.One - CollisionResponse.PenetrationSlop) * CollisionResponse.PenetrationCorrectionPercent;
        Vector2d final = planar?.Position ?? spatial!.Position3d.ToVector2d();
        final.Should().Be(Vector2d.Right * (Fixed64.One + correction));
        if (spatial != null) spatial.Position3d.Y.Should().Be(Fixed64.Zero);
    }

    private static Fixed64 ExactWeightedDistance(Fixed64 depth, Fixed64 inverseMass,
        Fixed64 inverseA, Fixed64 inverseB)
    {
        BigInteger denominator = (BigInteger)inverseA.m_rawValue + inverseB.m_rawValue;
        BigInteger quotient = BigInteger.DivRem((BigInteger)depth.m_rawValue * inverseMass.m_rawValue,
            denominator, out BigInteger remainder);
        int half = (remainder * 2).CompareTo(denominator);
        if (half > 0 || half == 0 && !quotient.IsEven) quotient++;
        return Fixed64.FromRaw((long)quotient);
    }

    private static GravitasWorldContext CreateContext()
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        return context;
    }

    private static SolidBody2D CreateCircle(GravitasWorldContext context, Fixed64 radius, Fixed64 mass)
    {
        var body = new SolidBody2D(new TestMatterAgent(context), new LSCircleCollider2D(radius))
        {
            Mass = mass == Fixed64.Zero ? Fixed64.One : mass,
            SleepEnabled = false
        };
        body.Initialize(Vector2d.Zero, motionType: mass == Fixed64.Zero
            ? BodyMotionType.Static : BodyMotionType.Dynamic);
        return body;
    }

    private static SolidBody CreateSphere(GravitasWorldContext context, Fixed64 radius, Fixed64 mass)
    {
        var body = new SolidBody(new TestMatterAgent(context), new LSSphereCollider { Radius = radius })
        {
            Mass = mass == Fixed64.Zero ? Fixed64.One : mass,
            SleepEnabled = false
        };
        // This fixture isolates contact response from automatic support-height snapping.
        body.UseManualGrounding();
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity, mass == Fixed64.Zero
            ? BodyMotionType.Static : BodyMotionType.Dynamic);
        return body;
    }
}
