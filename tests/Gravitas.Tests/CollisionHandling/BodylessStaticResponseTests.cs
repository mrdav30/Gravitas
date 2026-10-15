using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class BodylessStaticResponseTests
{
    public static TheoryData<int, bool, bool> SupportedShapes
    {
        get
        {
            var cases = new TheoryData<int, bool, bool>();
            foreach (int shape in new[] { 0, 1, 2 })
            foreach (bool mesh in new[] { false, true })
            foreach (bool floorFirst in new[] { false, true })
                cases.Add(shape, mesh, floorFirst);
            return cases;
        }
    }

    [Theory]
    [MemberData(nameof(SupportedShapes))]
    public void SupportedMotion_ShouldMatchExplicitStaticBodyThroughTheCompleteHostLoop(
        int shape, bool mesh, bool floorFirst)
    {
        using Scene bodyless = CreateScene(true, mesh, floorFirst, shape);
        using Scene control = CreateScene(false, mesh, floorFirst, shape);
        for (int frame = 0; frame < 32; frame++)
        {
            Step(bodyless);
            Step(control);
            AssertSameMotion(bodyless.Body, control.Body);
        }
        bodyless.Body.Position3d.Y.Should().BeGreaterThan(Fixed64.Zero);
        bodyless.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        bodyless.Body.AddLinearImpulse(Vector3d.Right);
        control.Body.AddLinearImpulse(Vector3d.Right);
        for (int frame = 0; frame < 32; frame++)
        {
            Step(bodyless);
            Step(control);
            AssertSameMotion(bodyless.Body, control.Body);
        }
        bodyless.Body.LinearVelocity.X.Should().Be(Fixed64.One);
        bodyless.Floor.Body.Should().BeNull();
        bodyless.Floor.Position.Should().Be(control.Floor.Position);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void FrictionAndRestitution_ShouldMatchStaticResponseIncludingAngularMotion(
        bool bounce, bool floorFirst)
    {
        PhysicsMaterial material = bounce
            ? new PhysicsMaterial(Fixed64.Zero, Fixed64.Zero, Fixed64.One)
            : new PhysicsMaterial(Fixed64.Half, Fixed64.Half, Fixed64.Zero);
        using Scene bodyless = CreateScene(true, false, floorFirst, 0, material, freezeRotation: bounce);
        using Scene control = CreateScene(false, false, floorFirst, 0, material, freezeRotation: bounce);
        bodyless.Context.Environment.Gravity = control.Context.Environment.Gravity = Fixed64.Zero;
        bodyless.Context.Settings.RestitutionVelocityThreshold = control.Context.Settings.RestitutionVelocityThreshold = Fixed64.Zero;
        Vector3d impulse = bounce ? Vector3d.Down * Fixed64.Two : Vector3d.Right + Vector3d.Down;
        bodyless.Body.AddLinearImpulse(impulse);
        control.Body.AddLinearImpulse(impulse);

        Step(bodyless);
        Step(control);

        AssertSameMotion(bodyless.Body, control.Body);
        if (bounce)
            bodyless.Body.LinearVelocity.Y.Should().BeGreaterThan(Fixed64.Zero);
        else
        {
            bodyless.Body.LinearVelocity.X.Should().BeInRange(Fixed64.MinIncrement, Fixed64.One - Fixed64.MinIncrement);
            bodyless.Body.AngularVelocity.Z.Should().BeLessThan(Fixed64.Zero);
            bodyless.Body.LinearVelocity.Y.Should().Be(Fixed64.Zero);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SharedStaticFloor_ShouldSolveConnectedBodiesWithoutJoiningAnUnrelatedSleeper(bool floorFirst)
    {
        using Scene bodyless = CreateScene(true, false, floorFirst, 0);
        using Scene control = CreateScene(false, false, floorFirst, 0);
        bodyless.Body.SetPosition(new Vector3d(-Fixed64.Quarter, Fixed64.Half, Fixed64.Zero));
        control.Body.SetPosition(bodyless.Body.Position3d);
        SolidBody connected = AddSphere(bodyless.Context, new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Zero));
        SolidBody controlConnected = AddSphere(control.Context, connected.Position3d);
        SolidBody unrelated = AddSphere(bodyless.Context, new Vector3d((Fixed64)3, Fixed64.Half, Fixed64.Zero));
        SolidBody controlUnrelated = AddSphere(control.Context, unrelated.Position3d);
        connected.SleepEnabled = controlConnected.SleepEnabled = true;
        unrelated.SleepEnabled = controlUnrelated.SleepEnabled = true;
        connected.Sleep();
        controlConnected.Sleep();
        unrelated.Sleep();
        controlUnrelated.Sleep();
        bodyless.Body.AddLinearImpulse(Vector3d.Right);
        control.Body.AddLinearImpulse(Vector3d.Right);

        for (int frame = 0; frame < 8; frame++)
        {
            Step(bodyless);
            Step(control);
            AssertSameMotion(bodyless.Body, control.Body);
            AssertSameMotion(connected, controlConnected);
            AssertSameMotion(unrelated, controlUnrelated);
        }

        connected.IsSleeping.Should().BeFalse();
        unrelated.IsSleeping.Should().BeTrue("a shared immovable collider does not connect dynamic islands");
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContactNotifications_ShouldRetainTheBodylessOwnersEnterStayAndExitLifecycle(bool floorFirst)
    {
        using Scene bodyless = CreateScene(true, false, floorFirst, 0);
        using Scene control = CreateScene(false, false, floorFirst, 0);
        int enters = 0, contacts = 0, exits = 0, controlEnters = 0, controlContacts = 0, controlExits = 0;
        bodyless.Floor.OnContactEnter += other => { other.Should().BeSameAs(bodyless.Body); enters++; };
        bodyless.Floor.OnContact += other => { other.Should().BeSameAs(bodyless.Body); contacts++; };
        bodyless.Floor.OnContactExit += other => { other.Should().BeSameAs(bodyless.Body); exits++; };
        control.Floor.OnContactEnter += _ => controlEnters++;
        control.Floor.OnContact += _ => controlContacts++;
        control.Floor.OnContactExit += _ => controlExits++;
        for (int frame = 0; frame < 8; frame++) { Step(bodyless); Step(control); }
        enters.Should().Be(1);
        contacts.Should().BeGreaterThan(enters);
        // A nearby gap retains shared broad-phase cells for the next separation check.
        bodyless.Body.SetPosition(new Vector3d(bodyless.Body.Position3d.X, Fixed64.One, Fixed64.Zero));
        control.Body.SetPosition(bodyless.Body.Position3d);
        Step(bodyless);
        Step(control);

        (enters, contacts, exits).Should().Be((controlEnters, controlContacts, controlExits));
        exits.Should().Be(1);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RepeatedBodylessResponse_ShouldRetainFullLoopReplayHashes(bool mesh)
    {
        using Scene first = CreateScene(true, mesh, true, 1);
        using Scene second = CreateScene(true, mesh, true, 1);
        first.Body.AddLinearImpulse(Vector3d.Right);
        second.Body.AddLinearImpulse(Vector3d.Right);
        for (int frame = 0; frame < 32; frame++)
        {
            Step(first);
            Step(second);
            first.Context.ComputeReplayHash().Should().Be(second.Context.ComputeReplayHash());
        }
    }

    private static Scene CreateScene(bool bodyless, bool mesh, bool floorFirst, int shape,
        PhysicsMaterial? material = null, bool freezeRotation = true)
    {
        PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        GravitasWorldContext context = scenario.Context;
        context.SetFrameRate(16);
        context.Environment.Gravity = Fixed64.One;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.MinSpeed = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        LSCollider floor = mesh ? new LSMeshCollider(new[]
        {
            new Vector3d(-8, 0, -4), new Vector3d(8, 0, -4),
            new Vector3d(-8, 0, 4), new Vector3d(8, 0, 4)
        }, new[] { 0, 2, 1, 1, 2, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)
            : new LSCuboidCollider { Size = new Vector3d(16, 1, 8) };
        floor.Material = material ?? PhysicsMaterial.Frictionless;
        Vector3d floorPosition = mesh ? Vector3d.Zero : Vector3d.Down * Fixed64.Half;
        SolidBody? mover = null;
        if (!floorFirst) mover = CreateMover();
        if (bodyless) scenario.InitializeStaticCollider(floor, floorPosition);
        else scenario.CreateBody(floor, floorPosition, FixedQuaternion.Identity, immovable: true);
        mover ??= CreateMover();
        return new Scene(scenario, floor, mover);

        SolidBody CreateMover()
        {
            LSCollider collider = shape switch
            {
                1 => new LSCapsuleCollider { Radius = Fixed64.Half, Size = new Vector3d(1, 2, 1) },
                2 => new LSCuboidCollider(),
                _ => new LSSphereCollider { Radius = Fixed64.Half }
            };
            collider.Material = material ?? PhysicsMaterial.Frictionless;
            SolidBody body = scenario.CreateBody(collider,
                new Vector3d((Fixed64)(-2), shape == 1 ? Fixed64.One : Fixed64.Half, Fixed64.Zero),
                FixedQuaternion.Identity, preventAngularForces: freezeRotation).Body;
            body.SleepEnabled = false;
            body.UseManualGrounding();
            return body;
        }
    }

    private static SolidBody AddSphere(GravitasWorldContext context, Vector3d position)
    {
        var collider = new LSSphereCollider { Radius = Fixed64.Half, Material = PhysicsMaterial.Frictionless };
        var body = new SolidBody(new TestMatterAgent(context,
            new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One)), collider)
        { Mass = Fixed64.One, FreezeAxes = BodyFreezeAxes3D.Rotation, SleepEnabled = false };
        body.Initialize(position, FixedQuaternion.Identity);
        body.UseManualGrounding();
        return body;
    }

    private static void Step(Scene scene) { scene.Context.Simulate(); scene.Context.LateSimulate(); }

    private static void AssertSameMotion(SolidBody actual, SolidBody expected)
    {
        actual.Position3d.Should().Be(expected.Position3d);
        actual.Rotation.Should().Be(expected.Rotation);
        actual.LinearVelocity.Should().Be(expected.LinearVelocity);
        actual.AngularVelocity.Should().Be(expected.AngularVelocity);
        actual.IsSleeping.Should().Be(expected.IsSleeping);
    }

    private sealed class Scene : IDisposable
    {
        private readonly PhysicsScenarioBuilder _scenario;
        public Scene(PhysicsScenarioBuilder scenario, LSCollider floor, SolidBody body)
        {
            _scenario = scenario;
            Floor = floor;
            Body = body;
        }
        public GravitasWorldContext Context => _scenario.Context;
        public LSCollider Floor { get; }
        public SolidBody Body { get; }
        public void Dispose() => _scenario.Dispose();
    }
}
