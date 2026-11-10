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
    [InlineData(32, 8, false)]
    [InlineData(60, 121, true)]
    public void FreeTopHeavyStack_ShouldBoundBothDepthsAfterSettlingOrWake(int rate, int lastFrame, bool pulse)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(frameRate: rate, extent: 16);
        var material = new PhysicsMaterial(Fixed64.Half, Fixed64.FromFraction(1, 3), Fixed64.Zero);
        LSAABBoxCollider2D floor = CreateFloor(context, material);
        SolidBody2D lower = CreateBox(context, Vector2d.Forward * Fixed64.FromFraction(17, 32), 1, material);
        SolidBody2D upper = CreateBox(context, Vector2d.Forward * Fixed64.FromFraction(25, 16), 100, material);
        lower.Gravity = upper.Gravity = -Vector2d.Forward * Fixed64.FromFraction(49, 5);
        lower.SleepFrameThreshold = upper.SleepFrameThreshold = rate / 2;

        for (int frame = 1; frame <= lastFrame; frame++)
        {
            if (pulse && frame == lastFrame)
            {
                Assert.True(lower.IsSleeping && upper.IsSleeping);
                upper.AddLinearImpulse(-Vector2d.Forward * (upper.Mass * Fixed64.Quarter));
            }
            context.Simulate();
            context.LateSimulate();
        }

        CheckSupportDepth(floor, lower.Collider, lastFrame, "floor/lower");
        CheckSupportDepth(lower.Collider, upper.Collider, lastFrame, "lower/upper");
        Assert.False(CollisionDetection2D.TryCollide(floor, upper.Collider, out _));
        if (pulse)
            Assert.False(lower.IsSleeping || upper.IsSleeping);
    }

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

        for (int frame = 1; frame <= 80; frame++)
        {
            context.Simulate();
            context.LateSimulate();

            Assert.True(lower.WorldCenterOfMass.Y > Fixed64.Zero, $"Frame {frame}: lower COM reached the floor plane.");
            Assert.False(CollisionDetection2D.TryCollide(floor, upper.Collider, out _),
                $"Frame {frame}: upper body reached the floor.");
            if (frame < 8)
                continue;

            CheckSupportDepth(floor, lower.Collider, frame, "floor/lower");
            CheckSupportDepth(lower.Collider, upper.Collider, frame, "lower/upper");
            if (frame == 8)
                Assert.True(lower.LinearVelocity.Y.Abs() <= Fixed64.FromFraction(1, 256),
                    $"Frame {frame}: the lower body retained vertical speed {lower.LinearVelocity.Y} after both supports engaged.");
        }
    }

    private static void CheckSupportDepth(LSCollider2D a, LSCollider2D b, int frame, string name)
    {
        Assert.True(CollisionDetection2D.TryCollide(a, b, out Contact2D contact), $"Frame {frame}: lost {name}.");
        Assert.True(contact.Depth <= Fixed64.FromFraction(1, 16), $"Frame {frame}: {name} depth {contact.Depth} exceeds 1/16.");
    }

    [Theory]
    [InlineData(5, true)]
    [InlineData(50, false)]
    public void CoupledTranslation_ShouldCorrectBothInterfacesIncludingShallowLoad(int loadMillidepth, bool reverse)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        PhysicsMaterial material = PhysicsMaterial.Frictionless;
        var floor = CreateFloor(context, material);
        Fixed64 lowerHeight = Fixed64.FromFraction(47, 100);
        Fixed64 loadDepth = Fixed64.FromFraction(loadMillidepth, 1000);
        SolidBody2D lower = CreateBox(context, Vector2d.Forward * lowerHeight, 1, material);
        SolidBody2D upper = CreateBox(context, Vector2d.Forward * (lowerHeight + Fixed64.One - loadDepth), 100, material);
        var support = new CollisionPair2D(floor, lower.Collider);
        var load = new CollisionPair2D(lower.Collider, upper.Collider);
        Assert.True(CollisionDetection2D.TryCollide(support, support.Manifold, frame: 0));
        Assert.True(CollisionDetection2D.TryCollide(load, load.Manifold, frame: 0));

        Assert.True(CollisionResponse2D.TryCorrectSeededSupportPositions(reverse ? load : support, reverse ? support : load));

        Fixed64 tolerance = Fixed64.FromRaw(4096);
        Assert.True((lower.Position.Y - Fixed64.FromFraction(49, 100)).Abs() <= tolerance);
        Assert.True((upper.Position.Y - Fixed64.FromFraction(148, 100)).Abs() <= tolerance);
        Assert.Equal(Fixed64.Zero, lower.Rotation);
        Assert.Equal(Fixed64.Zero, upper.Rotation);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CoupledTranslation_ShouldKeepLowerPoseWhenFloorIsAtSlop(bool excessLoadDepth)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        PhysicsMaterial material = PhysicsMaterial.Frictionless;
        var floor = CreateFloor(context, material);
        Vector2d lowerPosition = Vector2d.Forward * Fixed64.Half;
        Vector2d upperPosition = lowerPosition + Vector2d.Forward;
        SolidBody2D lower = CreateBox(context, lowerPosition, 1, material);
        SolidBody2D upper = CreateBox(context, upperPosition, 2, material);
        Fixed64 excess = excessLoadDepth ? Fixed64.FromFraction(1, 8) : Fixed64.Zero;
        var support = Contact(floor, lower.Collider, Vector2d.Zero, CollisionResponse2D.PenetrationSlop);
        var load = Contact(lower.Collider, upper.Collider, Vector2d.Forward, CollisionResponse2D.PenetrationSlop + excess);

        Assert.True(CollisionResponse2D.TryCorrectSeededSupportPositions(support, load));
        Assert.Equal(lowerPosition, lower.Position);
        Assert.Equal(upperPosition + Vector2d.Forward * excess, upper.Position);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CoupledTranslation_ShouldLeaveBothPosesWhenUpperPoseCannotBePrepared(bool bodyPositionOverflows)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        PhysicsMaterial material = PhysicsMaterial.Frictionless;
        var floor = CreateFloor(context, material);
        Vector2d upperPosition = Vector2d.Forward * (Fixed64.MaxValue - Fixed64.FromFraction(9, 16));
        Vector2d lowerPosition = upperPosition - Vector2d.Forward;
        SolidBody2D lower = CreateBox(context, lowerPosition, 1, material);
        SolidBody2D upper = CreateBox(context, upperPosition, 100, material);
        if (!bodyPositionOverflows)
            upper.Collider.LocalOffset = Vector2d.Forward * Fixed64.Half;
        Vector2d upperCenter = upper.Collider.Center;
        Fixed64 depth = bodyPositionOverflows ? Fixed64.Half : Fixed64.FromFraction(1, 8);
        var support = Contact(floor, lower.Collider, lowerPosition, depth);
        var load = Contact(lower.Collider, upper.Collider, upperPosition, depth);

        Assert.False(CollisionResponse2D.TryCorrectSeededSupportPositions(support, load));
        Assert.Equal(lowerPosition, lower.Position);
        Assert.Equal(upperPosition, upper.Position);
        Assert.Equal(upperCenter, upper.Collider.Center);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CoupledTranslation_ShouldLeaveMixedDirectionsToPairCorrection(bool lowerLeads)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        PhysicsMaterial material = PhysicsMaterial.Frictionless;
        LSAABBoxCollider2D? floor = lowerLeads ? null : CreateFloor(context, material);
        SolidBody2D lower = CreateBox(context, Vector2d.Forward * Fixed64.Half, 1, material);
        SolidBody2D upper = CreateBox(context, Vector2d.Forward * Fixed64.FromFraction(3, 2), 100, material);
        floor ??= CreateFloor(context, material);
        Fixed64 depth = Fixed64.FromFraction(1, 8);
        var support = Contact(floor, lower.Collider, Vector2d.Zero, depth);
        Assert.Equal(lowerLeads, support.ColliderA == lower.Collider);
        support.Manifold.AddContact(Vector2d.Right, Vector2d.Right, depth,
            lowerLeads ? -Vector2d.Right : Vector2d.Right);
        var load = Contact(lower.Collider, upper.Collider, Vector2d.Forward, depth);

        Assert.False(CollisionResponse2D.TryCorrectSeededSupportPositions(support, load));
        Assert.Equal(Vector2d.Forward * Fixed64.Half, lower.Position);
        Assert.Equal(Vector2d.Forward * Fixed64.FromFraction(3, 2), upper.Position);
        CollisionResponse2D.CorrectSeededSupportPositions(support, load);
        Assert.True(lower.Position.X > Fixed64.Zero);
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
        if (boundary is SeedBoundary.FixedLoad or SeedBoundary.UnresolvedPivot)
        {
            Fixed64 depth = Fixed64.FromFraction(1, 8);
            support = Contact(floor, lower.Collider, Vector2d.Zero, depth);
            load = Contact(lower.Collider, loadCollider, Vector2d.Forward, depth);
            Assert.False(CollisionResponse2D.TryCorrectSeededSupportPositions(support, load));
        }
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

    private static CollisionPair2D Contact(LSCollider2D lower, LSCollider2D upper, Vector2d witness, Fixed64 depth = default)
    {
        var pair = new CollisionPair2D(lower, upper);
        pair.Manifold.SetContact(witness, witness, depth,
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
