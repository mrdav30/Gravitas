using Chronicler.Hashing;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Support;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedResponseTests
{
    [Fact]
    public void Step_WithCapsuleCrossingAnOffcenterPrism_ShouldRetainItsRealNormalLeverAndReplay()
    {
        RunFinitePrismWitnessSteps(capsule: true).Should().Be(RunFinitePrismWitnessSteps(capsule: true));
    }

    [Fact]
    public void Step_WithContainedSphere_ShouldApplyFrictionAtTheOpposingPrismFaceAndReplay()
    {
        RunFinitePrismWitnessSteps(capsule: false).Should().Be(RunFinitePrismWitnessSteps(capsule: false));
    }

    private static (ChronicleHash Impact, ChronicleHash Rotated, ChronicleHash Cleared,
        int Enter, int Stay, int Exit) RunFinitePrismWitnessSteps(bool capsule)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 64);
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        context.Environment.MinSpeed = Fixed64.Zero;
        Vector3d center = capsule ? Vector3d.Up * Fixed64.Half : Vector3d.Zero;
        FixedQuaternion rotation = capsule
            ? new(Fixed64.Half, Fixed64.Half, -Fixed64.Half, Fixed64.Half)
            : FixedQuaternion.Identity;
        LSCollider curved = capsule
            ? new LSCapsuleCollider { Radius = Fixed64.Quarter,
                Size = new Vector3d(Fixed64.Half, Fixed64.FromFraction(21, 2), Fixed64.Half) }
            : new LSSphereCollider { Radius = Fixed64.FromFraction(3, 4) };
        curved.Material = PhysicsMaterialTestHelper.WithFrictionAndRestitution(
            capsule ? Fixed64.Zero : Fixed64.One, Fixed64.Zero);
        var spatialAgent = new TestMatterAgent(context, new FixedTransform(center, rotation, Vector3d.One));
        SolidBody? spatial = null;
        if (capsule)
        {
            spatial = new SolidBody(spatialAgent, curved)
            {
                Mass = Fixed64.One, SleepEnabled = false, LinearDragCoefficient = Fixed64.Zero,
                FreezeAxes = BodyFreezeAxes3D.RotationX | BodyFreezeAxes3D.RotationY
            };
            spatial.Initialize(center, rotation);
            spatial.UseManualGrounding();
        }
        else curved.InitializeWithNoBody(spatialAgent);
        Vector3d slabCenter = capsule ? Vector3d.Right * Fixed64.FromFraction(5, 2) : Vector3d.Zero;
        var slab = new LSAABBoxCollider2D(capsule ? new Vector2d(1, 2) : new Vector2d(2, 6))
        {
            MixedHalfThicknessOverride = capsule ? Fixed64.One : Fixed64.Two,
            Material = curved.Material
        };
        var planarAgent = new TestMatterAgent(context, new FixedTransform(slabCenter, FixedQuaternion.Identity, Vector3d.One));
        SolidBody2D? planar = null;
        if (capsule) slab.InitializeWithNoBody(planarAgent);
        else
        {
            planar = new SolidBody2D(planarAgent, slab) { Mass = Fixed64.One, SleepEnabled = false };
            planar.Initialize(Vector2d.Zero);
            planar.UseManualGrounding();
        }
        int entered = 0, stayed = 0, exited = 0;
        curved.OnMixedContactEnter += _ => entered++;
        slab.OnMixedContactEnter += _ => entered++;
        curved.OnMixedContact += _ => stayed++;
        slab.OnMixedContact += _ => stayed++;
        curved.OnMixedContactExit += _ => exited++;
        slab.OnMixedContactExit += _ => exited++;
        bool initialHit = CollisionDetectionMixed.TryCollide(curved, slab, out MixedContact initial);
        // Integration must reach the exact authored collision pose, rather than
        // changing the face tie or off-center lever before narrow phase runs.
        if (capsule)
        {
            spatial!.SetPosition(center + Vector3d.Up * context.DeltaTime);
            spatial.AddLinearImpulse(Vector3d.Down);
        }
        else
        {
            Vector2d incoming = Vector2d.Right + Vector2d.Forward;
            planar!.SetPosition(-incoming * context.DeltaTime);
            planar.AddLinearImpulse(incoming);
        }

        Step(context);

        initialHit.Should().BeTrue();
        initial.Normal3DTo2D.Should().Be(capsule ? Vector3d.Down : Vector3d.Left);
        initial.Depth.Should().Be(Fixed64.FromFraction(capsule ? 3 : 7, 4));
        initial.DepthIsClamped.Should().BeFalse();
        context.MixedCollisions.ActivePairCount.Should().Be(1);
        entered.Should().Be(2); stayed.Should().Be(2); exited.Should().Be(0);
        if (capsule)
        {
            // Every valid paired support has X in [2,3]. Its upward normal
            // impulse has positive Z torque; the old midpoint at X=0 has none.
            spatial!.AngularVelocity.Z.Should().BeGreaterThan(Fixed64.Zero);
            spatial.Position3d.Y.Should().BeGreaterThan(center.Y);
            spatial.LinearVelocity.Y.Should().BeGreaterThan(-Fixed64.One);
            initial.Point3D.X.Should().BeInRange(Fixed64.Two, (Fixed64)3);
            initial.Point3D.Y.Should().Be(Fixed64.Quarter);
            initial.Point3D.Z.Should().Be(Fixed64.Zero);
            initial.Point2D.Should().Be(new Vector3d(initial.Point3D.X, Fixed64.One, Fixed64.Zero));
        }
        else
        {
            // Friction opposes +Z at the prism's +X face: physical +Y torque
            // is negative planar yaw. Choosing the -X face reverses that torque.
            planar!.AngularVelocity.Should().BeLessThan(Fixed64.Zero);
            planar.Position.X.Should().BeLessThan(Fixed64.Zero);
            planar.LinearVelocity.Y.Should().BeLessThan(Fixed64.One);
            initial.Point3D.Should().Be(Vector3d.Left * Fixed64.FromFraction(3, 4));
            initial.Point2D.Should().Be(Vector3d.Right);
        }
        ChronicleHash impact = context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches);
        Step(context);
        if (capsule) spatial!.Rotation.Should().NotBe(rotation);
        else planar!.Rotation.Should().BeLessThan(Fixed64.Zero);
        ChronicleHash rotated = context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        if (capsule) spatial!.SetPosition(Vector3d.Up * (Fixed64)20);
        else planar!.SetPosition(Vector2d.Right * (Fixed64)10);
        Step(context);
        // Rotation-independent support bounds: the capsule fits a radius-21/4
        // ball; the prism's planar half diagonal sqrt(10) is less than four.
        Fixed64 clearance = capsule
            ? spatial!.Position3d.Y - Fixed64.FromFraction(21, 4) - Fixed64.One
            : planar!.Position.X - (Fixed64)4 - Fixed64.FromFraction(3, 4);
        clearance.Should().BeGreaterThan(Fixed64.One);
        CollisionDetectionMixed.TryCollide(curved, slab, out _).Should().BeFalse();
        context.MixedCollisions.ActivePairCount.Should().Be(0);
        entered.Should().Be(2); stayed.Should().Be(4); exited.Should().Be(2);
        return (impact, rotated, context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches),
            entered, stayed, exited);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void Step_WithContainedRoundPrismContact_ShouldResolveTheNearestExitAndReplay(bool capsule, bool moving3D)
    {
        var first = RunContainedRoundPrismSteps(capsule, moving3D);
        var replay = RunContainedRoundPrismSteps(capsule, moving3D);
        replay.Should().Be(first);
    }

    private static (ChronicleHash Corrected, ChronicleHash Touching, ChronicleHash Cleared,
        int Enter, int Stay, int Exit) RunContainedRoundPrismSteps(bool capsule, bool moving3D)
    {
        using GravitasWorldContext context = CreateMixedContext();
        Vector3d start = (moving3D ? Vector3d.Up : Vector3d.Right) * Fixed64.Half;
        Vector3d normal = moving3D ? Vector3d.Down : Vector3d.Left;
        // This exact axis permutation maps local Up to Right. The horizontal
        // fixture instead keeps local Up, perpendicular to its X exit.
        FixedQuaternion rotation = capsule && moving3D
            ? new(Fixed64.Half, Fixed64.Half, -Fixed64.Half, Fixed64.Half)
            : FixedQuaternion.Identity;
        LSCollider curved = capsule
            ? new LSCapsuleCollider { Radius = Fixed64.Quarter,
                Size = new Vector3d(Fixed64.Half, Fixed64.FromFraction(3, 2), Fixed64.Half) }
            : new LSSphereCollider { Radius = Fixed64.Quarter };
        curved.Material = PhysicsMaterialTestHelper.WithFrictionAndRestitution(Fixed64.Zero, Fixed64.Zero);
        var spatialAgent = new TestMatterAgent(context, new FixedTransform(start, rotation, Vector3d.One));
        SolidBody? spatial = null;
        if (moving3D)
        {
            spatial = new SolidBody(spatialAgent, curved)
            {
                Mass = Fixed64.One, SleepEnabled = false, FreezeAxes = BodyFreezeAxes3D.Rotation,
                LinearDragCoefficient = Fixed64.Zero
            };
            spatial.Initialize(start, rotation);
        }
        else curved.InitializeWithNoBody(spatialAgent);
        var slab = new LSAABBoxCollider2D(new Vector2d(moving3D ? (Fixed64)4 : Fixed64.Two, (Fixed64)4))
        {
            MixedHalfThicknessOverride = moving3D ? Fixed64.One : Fixed64.Two,
            Material = PhysicsMaterialTestHelper.WithFrictionAndRestitution(Fixed64.Zero, Fixed64.Zero)
        };
        var planarAgent = new TestMatterAgent(context, new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One));
        SolidBody2D? planar = null;
        if (moving3D) slab.InitializeWithNoBody(planarAgent);
        else
        {
            planar = new SolidBody2D(planarAgent, slab)
            {
                Mass = Fixed64.One, SleepEnabled = false, FreezeAxes = BodyFreezeAxes2D.Rotation
            };
            planar.Initialize(Vector2d.Zero);
        }
        int entered = 0, stayed = 0, exited = 0;
        curved.OnMixedContactEnter += _ => entered++;
        slab.OnMixedContactEnter += _ => entered++;
        curved.OnMixedContact += _ => stayed++;
        slab.OnMixedContact += _ => stayed++;
        curved.OnMixedContactExit += _ => exited++;
        slab.OnMixedContactExit += _ => exited++;
        bool initialHit = CollisionDetectionMixed.TryCollide(curved, slab, out MixedContact initial);

        Step(context);

        // The contained sphere/capsule has exit-axis extent[1/4,3/4].
        // The nearest slab face is at1, so its complete first exit is3/4.
        // The capsule core is perpendicular to this axis and remains finite.
        initialHit.Should().BeTrue();
        initial.Depth.Should().Be(Fixed64.FromFraction(3, 4));
        initial.Normal3DTo2D.Should().Be(normal);
        initial.DepthIsClamped.Should().BeFalse();
        context.MixedCollisions.LastBroadPhaseCandidateCount.Should().Be(1);
        context.MixedCollisions.ActivePairCount.Should().Be(1);
        entered.Should().Be(2); stayed.Should().Be(2); exited.Should().Be(0);
        // 3D response updates the authoritative body before its collider's next refresh.
        Fixed64 correction = moving3D ? spatial!.Position3d.Y - start.Y : -slab.Center.X;
        correction.Should().BeGreaterThan(Fixed64.Zero);
        correction.Should().BeLessThan(initial.Depth); // Deliberate penetration slop remains.
        (spatial?.Position3d ?? curved.Center).ToVector2d().Should().Be(moving3D ? Vector2d.Zero : start.ToVector2d());
        curved.Rotation.Should().Be(rotation);
        if (curved is LSCapsuleCollider finiteCapsule)
        {
            finiteCapsule.AxisLength.Should().Be(Fixed64.One);
            finiteCapsule.WorldAxis.Should().Be(moving3D ? Vector3d.Right : Vector3d.Up);
        }
        planarAgent.Transform.WorldPosition.Y.Should().Be(Fixed64.Zero);
        (spatial?.LinearVelocity ?? Vector3d.Zero).Should().Be(Vector3d.Zero);
        (planar?.LinearVelocity ?? Vector2d.Zero).Should().Be(Vector2d.Zero);
        ChronicleHash corrected = context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        // Author the independently known boundary, then run another complete
        // step: exact touching must retain the pair without positional drift.
        if (moving3D) spatial!.SetPosition(Vector3d.Up * Fixed64.FromFraction(5, 4));
        else planar!.SetPosition(Vector2d.Left * Fixed64.FromFraction(3, 4));
        Step(context);
        (spatial?.Position3d ?? curved.Center).Should().Be(moving3D ? Vector3d.Up * Fixed64.FromFraction(5, 4) : start);
        slab.Center.Should().Be(moving3D ? Vector2d.Zero : Vector2d.Left * Fixed64.FromFraction(3, 4));
        CollisionDetectionMixed.TryCollide(curved, slab, out MixedContact touching).Should().BeTrue();
        touching.Depth.Should().Be(Fixed64.Zero);
        context.MixedCollisions.ActivePairCount.Should().Be(1);
        entered.Should().Be(2); stayed.Should().Be(4); exited.Should().Be(0);
        ChronicleHash touchHash = context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        if (moving3D) spatial!.AddLinearImpulse(Vector3d.Up);
        else planar!.AddLinearImpulse(Vector2d.Left);
        Step(context);
        // These exact cardinal support planes separate the complete finite
        // shapes; the narrow-phase miss is an additional behavior assertion.
        Fixed64 clearance = moving3D ? spatial!.Position3d.Y - Fixed64.Quarter - Fixed64.One
            : start.X - Fixed64.Quarter - (slab.Center.X + Fixed64.One);
        clearance.Should().BeGreaterThan(Fixed64.FromFraction(1, 8));
        CollisionDetectionMixed.TryCollide(curved, slab, out _).Should().BeFalse();
        context.MixedCollisions.ActivePairCount.Should().Be(0);
        entered.Should().Be(2); stayed.Should().Be(4); exited.Should().Be(2);
        return (corrected, touchHash,
            context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches), entered, stayed, exited);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void Step_WithConeGeneratorContact_ShouldRespectExactDepthAndReplay(bool inward, bool moving3D)
    {
        var first = RunConeGeneratorHostSteps(inward, moving3D);
        var replay = RunConeGeneratorHostSteps(inward, moving3D);

        replay.Should().Be(first);
    }

    private static (Vector3d First3D, Vector2d First2D, Vector3d Final3D, Vector2d Final2D,
        Vector3d Velocity3D, Vector2d Velocity2D, (int Enter, int Stay, int Exit) Events)
        RunConeGeneratorHostSteps(bool inward, bool moving3D)
    {
        using GravitasWorldContext context = CreateMixedContext();
        Fixed64 delta = inward ? Fixed64.FromFraction(1, 64) : Fixed64.Zero;
        Vector3d slabOrigin = new(Fixed64.One - Fixed64.Two * delta, Fixed64.One - delta, Fixed64.Zero);
        var cone = new LSConeCollider
        {
            Radius = Fixed64.One,
            Size = new Vector3d(2, 2, 2),
            Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Zero)
        };
        var spatial = new SolidBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)), cone)
        {
            Mass = Fixed64.One,
            SleepEnabled = false,
            FreezeAxes = BodyFreezeAxes3D.Rotation,
            LinearDragCoefficient = Fixed64.Zero
        };
        spatial.Initialize(Vector3d.Zero, FixedQuaternion.Identity,
            moving3D ? BodyMotionType.Dynamic : BodyMotionType.Static);
        var slab = new LSAABBoxCollider2D(new Vector2d(Fixed64.One, Fixed64.Two))
        {
            MixedHalfThicknessOverride = Fixed64.One,
            Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Zero)
        };
        var planar = new SolidBody2D(new TestMatterAgent(context,
            new FixedTransform(slabOrigin, FixedQuaternion.Identity, Vector3d.One)), slab)
        {
            Mass = Fixed64.One,
            SleepEnabled = false,
            FreezeAxes = BodyFreezeAxes2D.Rotation
        };
        Vector2d initial2D = slabOrigin.ToVector2d();
        planar.Initialize(initial2D, motionType: moving3D ? BodyMotionType.Static : BodyMotionType.Dynamic);
        int entered = 0, stayed = 0, exited = 0;
        cone.OnMixedContactEnter += _ => entered++;
        slab.OnMixedContactEnter += _ => entered++;
        cone.OnMixedContact += _ => stayed++;
        slab.OnMixedContact += _ => stayed++;
        cone.OnMixedContactExit += _ => exited++;
        slab.OnMixedContactExit += _ => exited++;

        CollisionDetectionMixed.TryCollide(cone, slab, out MixedContact contact).Should().BeTrue();
        // 2x+y<=1 supports the entire H=2,R=1 cone. The slab's lower
        // near edge is (1/2,0,0)-delta*(2,1,0), giving the exact seam
        // tangency or minimum sqrt(5)/64 already proved in the core tests.
        Fixed64 expectedDepth = Fixed64.FromRaw(inward ? 150_059_982L : 0);
        contact.Depth.Should().Be(expectedDepth);
        contact.DepthIsClamped.Should().BeFalse();
        contact.Normal3DTo2D.Should().Be(new Vector3d(
            Fixed64.FromRaw(3_841_535_534L), Fixed64.FromRaw(1_920_767_767L), Fixed64.Zero));
        contact.Point3D.Should().Be(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        contact.Point2D.Should().Be(new Vector3d(Fixed64.Half - Fixed64.Two * delta, -delta, Fixed64.Zero));

        Step(context);

        Assert.Equal(1, context.MixedCollisions.LastBroadPhaseCandidateCount);
        Assert.Equal(1, context.MixedCollisions.ActivePairCount);
        entered.Should().Be(2);
        stayed.Should().Be(2);
        exited.Should().Be(0);
        spatial.Rotation.Should().Be(FixedQuaternion.Identity);
        planar.Rotation.Should().Be(Fixed64.Zero);
        spatial.LinearVelocity.Should().Be(Vector3d.Zero);
        spatial.AngularVelocity.Should().Be(Vector3d.Zero);
        planar.LinearVelocity.Should().Be(Vector2d.Zero);
        planar.AngularVelocity.Should().Be(Fixed64.Zero);
        Vector3d first3D = spatial.Position3d;
        Vector2d first2D = planar.Position;
        if (!inward)
        {
            first3D.Should().Be(Vector3d.Zero);
            first2D.Should().Be(initial2D);
        }
        else
        {
            if (moving3D)
            {
                first3D.X.Should().BeLessThan(Fixed64.Zero);
                first3D.Y.Should().BeLessThan(Fixed64.Zero);
                (-first3D.X).Should().BeLessThan(expectedDepth);
                first2D.Should().Be(initial2D);
                spatial.AddLinearImpulse(-contact.Normal3DTo2D);
            }
            else
            {
                first3D.Should().Be(Vector3d.Zero);
                first2D.X.Should().BeGreaterThan(initial2D.X);
                (first2D.X - initial2D.X).Should().BeLessThan(expectedDepth * Fixed64.Two);
                planar.Agent.Transform.WorldPosition.Y.Should().Be(slabOrigin.Y);
                planar.AddLinearImpulse(Vector2d.Right);
            }
            // Positional response deliberately retains penetration slop.
            // An authored outward velocity takes the next full host step beyond
            // that resting region, where an independent support plane proves
            // that the complete shapes, rather than only their anchors, clear.
            Step(context);
            Fixed64 coneMaximum = spatial.Position3d.X * Fixed64.Two + spatial.Position3d.Y + Fixed64.One;
            Fixed64 prismMinimum = (planar.Position.X - Fixed64.Half) * Fixed64.Two
                + planar.Agent.Transform.WorldPosition.Y - Fixed64.One;
            (prismMinimum - coneMaximum).Should().BeGreaterThan(Fixed64.FromFraction(1, 8));
            CollisionDetectionMixed.TryCollide(cone, slab, out _).Should().BeFalse();
            Assert.Equal(0, context.MixedCollisions.ActivePairCount);
            entered.Should().Be(2);
            stayed.Should().Be(2);
            exited.Should().Be(2);
        }

        return (first3D, first2D, spatial.Position3d, planar.Position,
            spatial.LinearVelocity, planar.LinearVelocity, (entered, stayed, exited));
    }
}
