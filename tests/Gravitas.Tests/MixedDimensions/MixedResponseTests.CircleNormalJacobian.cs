using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

/// <content>Checks circular normal force lines without removing real offset torque or surface friction.</content>
public sealed partial class MixedResponseTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void CircularSlabNormal_ShouldPreserveFrictionlessSpinThroughTheFullHostLoop(bool zeroCore, bool freezePosition)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 64);
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        context.Environment.MinSpeed = Fixed64.Zero;
        LSMeshCollider mesh = CreateCircularNormalMesh(context, tilted: true, PhysicsMaterial.Frictionless);
        SolidBody2D circle = CreateCircularNormalBody(context, zeroCore, Vector2d.Zero);
        circle.FreezeAxes = freezePosition ? BodyFreezeAxes2D.Position : BodyFreezeAxes2D.None;
        circle.ApplyCollisionAngularVelocityDelta(Fixed64.One);

        Step(context);

        circle.AngularVelocity.Should().Be(Fixed64.One,
            "yaw leaves a centered circular slab unchanged, so frictionless normal forces cannot oppose spin");
        circle.LinearVelocity.Should().Be(Vector2d.Zero);
        context.MixedCollisions.ActivePairCount.Should().Be(1);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void DetectedCircularNormal_ShouldKeepTheShapeCenterToBodyComTorque(bool compound)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSMeshCollider mesh = CreateCircularNormalMesh(context, tilted: false, PhysicsMaterial.Frictionless,
            faceX: compound ? Fixed64.FromFraction(1, 5) : Fixed64.FromFraction(2, 5), limitedZ: compound);
        SolidBody2D body;
        if (compound)
        {
            var collider = new LSCompoundCollider2D(
                CompoundColliderPart2D.Circle(Fixed64.Quarter, Vector2d.Forward * Fixed64.Half, PhysicsMaterial.Frictionless),
                CompoundColliderPart2D.Circle(Fixed64.Quarter, Vector2d.Backward * Fixed64.Half, PhysicsMaterial.Frictionless));
            body = new SolidBody2D(new TestMatterAgent(context,
                new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)), collider) { Mass = Fixed64.One };
            body.Initialize(Vector2d.Zero);
            body.UseManualGrounding();
        }
        else
        {
            body = CreateCircularNormalBody(context, zeroCore: false, Vector2d.Forward * Fixed64.Half);
            body.LocalCenterOfMassOffset = Vector2d.Zero;
        }
        body.ApplyCollisionLinearVelocityDelta(Vector2d.Right);
        var manifold = new ContactManifold();
        CollisionDetectionMixed.TryBuildManifold(mesh, body.Collider, manifold).Should().BeTrue();
        manifold.Count.Should().Be(2, "a vertical circular support line retains both cap endpoints");
        var pair = new CollisionPairMixed(mesh, body.Collider);
        pair.MarkResting(context.FrameCount, manifold);

        CollisionResponseMixed.ResolveManifold(pair);

        // The force line goes through Z=1/2 rather than the body COM at zero.
        // With I=3/8 (single circle) or 9/32 (two equal quarter-radius leaves),
        // J=1/(1+(1/2)^2/I), so the retained yaw changes by J*(1/2)/I.
        Fixed64 expectedX = compound ? Fixed64.FromFraction(8, 17) : Fixed64.FromFraction(2, 5);
        Fixed64 expectedYaw = compound ? Fixed64.FromFraction(16, 17) : Fixed64.FromFraction(4, 5);
        (body.LinearVelocity.X - expectedX).Abs().Should().BeLessThanOrEqualTo(Fixed64.FromRaw(8));
        (body.AngularVelocity - expectedYaw).Abs().Should().BeLessThanOrEqualTo(Fixed64.FromRaw(8));
        body.AngularVelocity.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void DetectedCircularNormal_ShouldKeepTheRealSurfaceLeverForRoughFriction()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var rough = new PhysicsMaterial(Fixed64.One, Fixed64.One, Fixed64.Zero);
        LSMeshCollider mesh = CreateCircularNormalMesh(context, tilted: false, rough);
        SolidBody2D circle = CreateCircularNormalBody(context, zeroCore: false, Vector2d.Zero);
        circle.Collider.Material = rough;
        circle.ApplyCollisionLinearVelocityDelta(Vector2d.Right);
        circle.ApplyCollisionAngularVelocityDelta(Fixed64.One);
        var manifold = new ContactManifold();
        CollisionDetectionMixed.TryBuildManifold(mesh, circle.Collider, manifold).Should().BeTrue();
        manifold.Count.Should().Be(2, "a vertical circular support line retains both cap endpoints");
        var pair = new CollisionPairMixed(mesh, circle.Collider);
        pair.MarkResting(context.FrameCount, manifold);

        CollisionResponseMixed.ResolveManifold(pair);

        circle.LinearVelocity.X.Should().Be(Fixed64.Zero);
        circle.LinearVelocity.Y.Should().BeLessThan(Fixed64.Zero);
        circle.AngularVelocity.Should().BeInRange(Fixed64.Zero, Fixed64.One - Fixed64.Epsilon,
            "friction still acts at the radius-1/2 surface and must damp incoming spin");
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CircularNormalCcd_ShouldNotInventAClosingRowEvenAtMaximumYaw(bool zeroCore)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSMeshCollider mesh = CreateCircularNormalMesh(context, tilted: true, PhysicsMaterial.Frictionless);
        SolidBody2D circle = CreateCircularNormalBody(context, zeroCore, Vector2d.Zero);
        circle.FreezeAxes = BodyFreezeAxes2D.Position;
        var manifold = new ContactManifold();
        CollisionDetectionMixed.TryBuildManifold(mesh, circle.Collider, manifold).Should().BeTrue();
        ContactAnchor center = ContactAnchor.FromWorldPoint(new Vector3d(Fixed64.Zero, -Fixed64.Quarter, Fixed64.Zero));

        MixedContact.TrySelectClosingContact(manifold, ContactAnchor.FromWorldPoint(Vector3d.Zero),
            Vector3d.Zero, Vector3d.Zero, center, Vector2d.Zero, Fixed64.MaxValue, out _, circle.Collider)
            .Should().BeFalse();
        var impact = new ContactResponseSnapshot(Vector3d.Zero, center.Origin,
            ContactAnchor.FromWorldPoint(Vector3d.Zero), center, Vector3d.Zero, Vector3d.Zero,
            Vector3d.Zero, new Vector3d(Fixed64.Zero, -Fixed64.MaxValue, Fixed64.Zero), Fixed64.MaxValue);
        ContactWarmStartCache cache = default;
        ExactContactImpulseState?[]? exact = null;
        CollisionResponseMixed.TrySolveManifoldVelocity(mesh, circle.Collider, manifold, impact,
            ref cache, ref exact, out ContactResponseSnapshot solved).Should().BeFalse();
        solved.PlanarAngularB.Should().Be(Fixed64.MaxValue);
        solved.LinearB.Should().Be(Vector3d.Zero);
    }

    private static SolidBody2D CreateCircularNormalBody(GravitasWorldContext context, bool zeroCore, Vector2d offset)
    {
        LSCollider2D collider = zeroCore ? new LSCapsuleCollider2D(Fixed64.Half, Fixed64.One)
            : new LSCircleCollider2D(Fixed64.Half);
        collider.Material = PhysicsMaterial.Frictionless;
        collider.LocalOffset = offset;
        collider.MixedHalfThicknessOverride = Fixed64.Half;
        var body = new SolidBody2D(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Down * Fixed64.Quarter, FixedQuaternion.Identity, Vector3d.One)), collider) { Mass = Fixed64.One };
        body.Initialize(Vector2d.Zero);
        body.UseManualGrounding();
        return body;
    }

    private static LSMeshCollider CreateCircularNormalMesh(GravitasWorldContext context, bool tilted,
        PhysicsMaterial material, Fixed64? faceX = null, bool limitedZ = false)
    {
        Fixed64 x = faceX ?? Fixed64.FromFraction(2, 5);
        Fixed64 zMin = limitedZ ? Fixed64.Quarter : (Fixed64)(-2);
        Fixed64 zMax = limitedZ ? Fixed64.FromFraction(3, 4) : (Fixed64)2;
        Vector3d[] vertices = tilted ? new[]
        {
            new Vector3d(-4, 12, -4), new Vector3d(4, 4, -4),
            new Vector3d(-4, -4, 4), new Vector3d(4, -12, 4)
        } : new[]
        {
            new Vector3d(x, (Fixed64)(-2), zMin), new Vector3d(x, (Fixed64)2, zMin),
            new Vector3d(x, (Fixed64)(-2), zMax), new Vector3d(x, (Fixed64)2, zMax)
        };
        var mesh = new LSMeshCollider(vertices, new[] { 0, 2, 1, 1, 2, 3 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation) { Material = material };
        mesh.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        return mesh;
    }
}
