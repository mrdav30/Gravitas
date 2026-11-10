using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class PlanarSupportIslandTests
{
    [Theory]
    [InlineData(10, false)]
    [InlineData(100, false)]
    [InlineData(100, true)]
    public void FreeTopHeavyStack_ShouldKeepBothSupportInterfaces(int upperMass, bool centered)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(frameRate: 60, extent: 16);
        var material = new PhysicsMaterial(Fixed64.Half, Fixed64.FromFraction(1, 3), Fixed64.Zero);
        LSAABBoxCollider2D? floor = centered ? null : CreateFloor(context, material);
        SolidBody2D lower = CreateBox(context, new Vector2d(Fixed64.Zero, Fixed64.FromFraction(17, 32)), 1, material);
        SolidBody2D upper = CreateBox(context, new Vector2d(centered ? Fixed64.Zero : Fixed64.FromFraction(1, 64),
            Fixed64.FromFraction(25, 16)), upperMass, material);
        floor ??= CreateFloor(context, material);
        lower.Gravity = upper.Gravity = new Vector2d(Fixed64.Zero, -Fixed64.FromFraction(49, 5));
        Fixed64 depthBound = Fixed64.FromFraction(1, 16);

        for (int frame = 1; frame <= 80; frame++)
        {
            context.Simulate();
            context.LateSimulate();

            Assert.True(lower.WorldCenterOfMass.Y > Fixed64.Zero, $"Frame {frame}: lower COM reached the floor plane.");
            Assert.False(CollisionDetection2D.TryCollide(floor, upper.Collider, out _),
                $"Frame {frame}: upper body reached the floor.");
            if (frame < 8)
                continue;

            CheckSupport(floor, lower.Collider, "floor/lower");
            CheckSupport(lower.Collider, upper.Collider, "lower/upper");
            if (frame == 8)
                Assert.True(lower.LinearVelocity.Y.Abs() <= Fixed64.FromFraction(1, 256),
                    $"Frame {frame}: the lower body retained vertical speed {lower.LinearVelocity.Y} after both supports engaged.");

            void CheckSupport(LSCollider2D a, LSCollider2D b, string name)
            {
                Assert.True(CollisionDetection2D.TryCollide(a, b, out Contact2D contact), $"Frame {frame}: lost {name}.");
                Assert.True(contact.Depth <= depthBound, $"Frame {frame}: {name} depth {contact.Depth} exceeds {depthBound}.");
            }
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    public void CoupledSeed_ShouldResolveBothIncomingNormalSpeeds(int floorLever)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        PhysicsMaterial material = PhysicsMaterial.Frictionless;
        var floor = CreateFloor(context, material);
        SolidBody2D lower = CreateBox(context, Vector2d.Forward * Fixed64.Half, 1, material);
        SolidBody2D upper = CreateBox(context, Vector2d.Forward * Fixed64.FromFraction(3, 2), 2, material);
        var support = Contact(floor, lower.Collider, Vector2d.Right * (Fixed64)floorLever);
        var load = Contact(lower.Collider, upper.Collider, Vector2d.Forward);
        lower.ApplyCollisionLinearVelocityDelta(-Vector2d.Forward);
        upper.ApplyCollisionLinearVelocityDelta(-Vector2d.Forward);

        Assert.True(CollisionResponse2D.TrySeedSupportedIslandNormals(load, support));
        Fixed64 tolerance = Fixed64.FromRaw(4096);
        Assert.True((lower.LinearVelocity.Y + lower.AngularVelocity * floorLever).Abs() <= tolerance);
        Assert.True((upper.LinearVelocity.Y - lower.LinearVelocity.Y).Abs() <= tolerance);
    }

    public enum SeedBoundary { Restitution, FixedLoad, AngularResponse, EffectiveMassMatrix, UnresolvedPivot }

    [Theory]
    [InlineData(SeedBoundary.Restitution)]
    [InlineData(SeedBoundary.FixedLoad)]
    [InlineData(SeedBoundary.AngularResponse)]
    [InlineData(SeedBoundary.EffectiveMassMatrix)]
    [InlineData(SeedBoundary.UnresolvedPivot)]
    public void CoupledSeed_ShouldLeaveUnsupportedResponseToThePairSolver(SeedBoundary boundary)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        PhysicsMaterial material = boundary == SeedBoundary.Restitution
            ? new PhysicsMaterial(Fixed64.Zero, Fixed64.Zero, Fixed64.Half) : PhysicsMaterial.Frictionless;
        var floor = CreateFloor(context, material, width: 256);
        int lever = boundary == SeedBoundary.AngularResponse ? 2 : boundary == SeedBoundary.EffectiveMassMatrix ? 64 : 0;
        SolidBody2D lower = CreateBox(context, Vector2d.Forward * Fixed64.Half, 1, material, halfWidth: System.Math.Max(1, lever));
        if (lever != 0)
            lower.Mass = Fixed64.MinIncrement;
        SolidBody2D upper = CreateBox(context, Vector2d.Forward * Fixed64.FromFraction(3, 2),
            boundary == SeedBoundary.UnresolvedPivot ? 100_000_000 : 100, material);
        LSCollider2D loadCollider = upper.Collider;
        if (boundary == SeedBoundary.FixedLoad)
        {
            loadCollider = new LSAABBoxCollider2D(ColliderShapeDefinition2D.AABBox(Vector2d.One, material));
            loadCollider.InitializeWithNoBody(AgentAt(context, upper.Position));
        }
        var support = Contact(floor, lower.Collider, Vector2d.Right * (Fixed64)lever);
        var load = Contact(lower.Collider, loadCollider, Vector2d.Forward);
        upper.ApplyCollisionLinearVelocityDelta(-Vector2d.Forward);

        Assert.False(CollisionResponse2D.TrySeedSupportedIslandNormals(support, load));
        Assert.Equal(Vector2d.Zero, lower.LinearVelocity);
        Assert.Equal(Fixed64.Zero, lower.AngularVelocity);
        Assert.Equal(-Vector2d.Forward, upper.LinearVelocity);
    }

    [Fact]
    public void CoupledSeed_ShouldRejectNormalThatDependsOnSpinBelowThePublishedDeadband()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(frameRate: 60);
        PhysicsMaterial material = PhysicsMaterial.Frictionless;
        LSAABBoxCollider2D floor = CreateFloor(context, material, width: 2048);
        SolidBody2D lower = CreateBox(context, Vector2d.Forward * Fixed64.Half, 1, material, halfWidth: 1024);
        SolidBody2D upper = CreateBox(context, Vector2d.Forward * Fixed64.FromFraction(3, 2), 100, material);
        var support = new CollisionPair2D(floor, lower.Collider);
        var load = new CollisionPair2D(lower.Collider, upper.Collider);
        Fixed64 lever = (Fixed64)1000;
        Vector2d supportPoint = Vector2d.Right * lever;
        support.Manifold.SetContact(supportPoint, supportPoint, Fixed64.Zero, Vector2d.Forward);
        load.Manifold.SetContact(Vector2d.Forward, Vector2d.Forward, Fixed64.Zero, Vector2d.Forward);
        ContactGroupKey group = support.Manifold.GetGroup(0).Key;
        ulong contactId = support.Manifold.PrimaryContact.ContactId;
        support.StoreWarmStartImpulse(group, contactId, Fixed64.One, Fixed64.Half);

        // The unrounded normal solve needs sub-deadband spin to hold the off-center support.
        Fixed64 targetSpin = Fixed64.Epsilon * Fixed64.Half;
        Fixed64 normalImpulse = Fixed64.FromFraction(1, 4096);
        lower.ApplyCollisionLinearVelocityDelta(-Vector2d.Forward * (targetSpin * lever + normalImpulse));
        lower.ApplyCollisionAngularVelocityDelta(targetSpin - lever * lower.EffectiveInverseMomentOfInertia * normalImpulse);
        upper.ApplyCollisionLinearVelocityDelta(Vector2d.Forward);
        Vector2d incoming = lower.LinearVelocity;
        Fixed64 spin = lower.AngularVelocity;
        Assert.True(spin.Abs() > Fixed64.Epsilon);

        Assert.False(CollisionResponse2D.TrySeedSupportedIslandNormals(load, support));
        Assert.Equal(incoming, lower.LinearVelocity);
        Assert.Equal(spin, lower.AngularVelocity);
        Assert.Equal(Vector2d.Forward, upper.LinearVelocity);
        Assert.True(support.TryGetWarmStartImpulse(group, contactId, out var cached));
        Assert.Equal(Fixed64.One, cached.NormalImpulse);
        Assert.Equal(Fixed64.Half, cached.TangentImpulse);
    }

    private static LSAABBoxCollider2D CreateFloor(GravitasWorldContext context, PhysicsMaterial material, int width = 16)
    {
        var floor = new LSAABBoxCollider2D(ColliderShapeDefinition2D.AABBox(new Vector2d(width, 1), material));
        floor.InitializeWithNoBody(AgentAt(context, -Vector2d.Forward * Fixed64.Half));
        return floor;
    }

    private static CollisionPair2D Contact(LSCollider2D lower, LSCollider2D upper, Vector2d witness)
    {
        var pair = new CollisionPair2D(lower, upper);
        pair.Manifold.SetContact(witness, witness, Fixed64.Zero,
            pair.ColliderA == lower ? Vector2d.Forward : -Vector2d.Forward);
        return pair;
    }

    private static SolidBody2D CreateBox(GravitasWorldContext context, Vector2d position, int mass,
        PhysicsMaterial material, int halfWidth = 1)
    {
        Fixed64 x = (Fixed64)halfWidth;
        var shape = ColliderShapeDefinition2D.ConvexPolygon(material,
            new Vector2d(-x, -Fixed64.Half), new Vector2d(x, -Fixed64.Half),
            new Vector2d(x, Fixed64.Half), new Vector2d(-x, Fixed64.Half));
        var body = new SolidBody2D(AgentAt(context, position), new LSPolygonCollider2D(shape))
        {
            Mass = (Fixed64)mass,
            SleepEnabled = true
        };
        body.UseManualGrounding();
        body.Initialize(position);
        return body;
    }

    private static TestMatterAgent AgentAt(GravitasWorldContext context, Vector2d position) => new(context,
        new FixedTransform(position.ToVector3d(Fixed64.Zero), FixedQuaternion.Identity, Vector3d.One));
}
