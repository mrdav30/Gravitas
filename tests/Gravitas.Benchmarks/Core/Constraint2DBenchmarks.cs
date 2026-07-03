using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Constraints;
using System;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class Constraint2DBenchmarks
{
    private const int WarmupFrames = 16;
    private GravitasWorldContext _chainContext = null!;
    private GravitasWorldContext _ragdollContext = null!;
    private GravitasWorldContext _stackContext = null!;
    private GravitasWorldContext _motorContext = null!;
    private GravitasWorldContext _toggleContext = null!;
    private GravitasWorldContext _inactiveContext = null!;
    private SolidBody2D[] _chainBodies = Array.Empty<SolidBody2D>();
    private Joint2D[] _chainJoints = Array.Empty<Joint2D>();
    private Joint2D[] _ragdollJoints = Array.Empty<Joint2D>();
    private Joint2D[] _stackJoints = Array.Empty<Joint2D>();
    private Joint2D[] _motorJoints = Array.Empty<Joint2D>();
    private RagdollRuntime2D _toggleRagdoll = null!;

    [Params(32)]
    public int LinkCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _chainContext = CreateConstraintContext(BenchmarkPhysicsScene.GridExtentForLine(LinkCount), Fixed64.Zero, 10);
        CreatePinLine(_chainContext, LinkCount, out _chainBodies, out _chainJoints);
        Warm(_chainContext, WarmupFrames);

        _ragdollContext = CreateConstraintContext(48, (Fixed64)10, 12);
        CreateRagdoll(_ragdollContext, Vector2d.Zero, out _, out _ragdollJoints, sleepEnabled: false);
        Warm(_ragdollContext, 80);

        _stackContext = CreateConstraintContext(64, (Fixed64)10, 12);
        CreateRagdoll(_stackContext, new Vector2d((Fixed64)(-2), Fixed64.Zero), out _, out Joint2D[] leftStack, sleepEnabled: false);
        CreateRagdoll(_stackContext, new Vector2d((Fixed64)2, Fixed64.Zero), out _, out Joint2D[] rightStack, sleepEnabled: false);
        _stackJoints = Combine(leftStack, rightStack);
        Warm(_stackContext, 80);

        _motorContext = CreateConstraintContext(BenchmarkPhysicsScene.GridExtentForLine(LinkCount), Fixed64.Zero, 8);
        CreateMotorLine(_motorContext, LinkCount, out _motorJoints);
        Warm(_motorContext, WarmupFrames);

        _toggleContext = CreateConstraintContext(BenchmarkPhysicsScene.GridExtentForLine(LinkCount), Fixed64.Zero, 8);
        CreatePinLine(_toggleContext, LinkCount, out SolidBody2D[] toggleBodies, out _);
        _toggleRagdoll = _toggleContext.Constraints2D.RegisterRagdoll(CreateLineRagdoll(toggleBodies));
        _toggleRagdoll.DeactivateToKinematic();
        _toggleRagdoll.ActivateDynamic();
        _toggleRagdoll.DeactivateToKinematic();
        _toggleRagdoll.ActivateDynamic();
        Warm(_toggleContext, WarmupFrames);

        _inactiveContext = CreateConstraintContext(BenchmarkPhysicsScene.GridExtentForLine(LinkCount), Fixed64.Zero, 8);
        CreatePinLine(_inactiveContext, LinkCount, out SolidBody2D[] inactiveBodies, out _);
        RagdollRuntime2D inactiveRagdoll = _inactiveContext.Constraints2D.RegisterRagdoll(CreateLineRagdoll(inactiveBodies));
        inactiveRagdoll.DeactivateToKinematic();
        Warm(_inactiveContext, WarmupFrames);
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _chainContext.Dispose();
        _ragdollContext.Dispose();
        _stackContext.Dispose();
        _motorContext.Dispose();
        _toggleContext.Dispose();
        _inactiveContext.Dispose();
        _chainBodies = Array.Empty<SolidBody2D>();
        _chainJoints = Array.Empty<Joint2D>();
        _ragdollJoints = Array.Empty<Joint2D>();
        _stackJoints = Array.Empty<Joint2D>();
        _motorJoints = Array.Empty<Joint2D>();
        _toggleRagdoll = null!;
    }

    [Benchmark]
    public int SimulateLongConstraintChain()
    {
        _chainContext.Simulate();
        _chainContext.LateSimulate();
        return SumRows(_chainJoints);
    }

    [Benchmark]
    public int SimulatePlanarRagdollResting()
    {
        _ragdollContext.Simulate();
        _ragdollContext.LateSimulate();
        return SumRows(_ragdollJoints);
    }

    [Benchmark]
    public int SimulateContactHeavyRagdollStack()
    {
        _stackContext.Simulate();
        _stackContext.LateSimulate();
        return SumRows(_stackJoints);
    }

    [Benchmark]
    public int SimulateMotorDrivenConstraintChain()
    {
        _motorContext.Simulate();
        _motorContext.LateSimulate();
        return SumRows(_motorJoints);
    }

    [Benchmark]
    public int LinkedCollisionFilterLookup()
    {
        int filtered = 0;
        for (int i = 0; i < _chainBodies.Length - 1; i++)
        {
            if (_chainContext.Constraints2D.ShouldExcludeLinkedCollision(_chainBodies[i].Collider, _chainBodies[i + 1].Collider))
                filtered++;
        }

        return filtered;
    }

    [Benchmark]
    public bool ToggleRagdollActivation()
    {
        if (_toggleRagdoll.IsActive)
            _toggleRagdoll.DeactivateToKinematic();
        else
            _toggleRagdoll.ActivateDynamic();

        return _toggleRagdoll.IsActive;
    }

    [Benchmark]
    public int SimulateInactiveRagdoll()
    {
        _inactiveContext.Simulate();
        _inactiveContext.LateSimulate();
        return _inactiveContext.Constraints2D.EnabledJointCount;
    }

    private static GravitasWorldContext CreateConstraintContext(int gridExtent, Fixed64 gravity, int solverIterations)
    {
        GravitasWorldContext context = BenchmarkPhysicsScene.CreateContext(gridExtent, clearAllPools: true);
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        context.Environment.Gravity = gravity;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        context.Settings.DiscreteSolverIterations = solverIterations;
        return context;
    }

    private static void CreatePinLine(
        GravitasWorldContext context,
        int count,
        out SolidBody2D[] bodies,
        out Joint2D[] joints)
    {
        bodies = new SolidBody2D[count];
        for (int i = 0; i < bodies.Length; i++)
        {
            bool dynamic = i != 0;
            bodies[i] = CreateCircle(context, new Vector2d((Fixed64)(i * 2), Fixed64.Zero), dynamic, sleepEnabled: false);
        }

        joints = new Joint2D[count - 1];
        for (int i = 0; i < joints.Length; i++)
        {
            joints[i] = context.Constraints2D.RegisterJoint(new JointDefinition2D(
                bodies[i],
                bodies[i + 1],
                LocalFrame(Vector2d.Right * Fixed64.Half),
                LocalFrame(-Vector2d.Right * Fixed64.Half),
                JointType2D.Pin,
                JointLimit2D.Unrestricted,
                JointMotor2D.Disabled,
                JointCollisionPolicy.SuppressLinked));
        }

        bodies[^1].AddForce(new Vector2d((Fixed64)6, (Fixed64)2));
    }

    private static void CreateMotorLine(
        GravitasWorldContext context,
        int count,
        out Joint2D[] joints)
    {
        SolidBody2D[] bodies = CreateCircleLineBodies(context, count, Fixed64.One, sleepEnabled: false);
        var motor = JointMotor2D.Angular(Fixed64.HalfPi, (Fixed64)12, Fixed64.Zero, Fixed64.FromFraction(1, 8));
        var links = new RagdollLinkDefinition2D[bodies.Length];
        var definitions = new RagdollJointDefinition2D[bodies.Length - 1];
        for (int i = 0; i < bodies.Length; i++)
            links[i] = Link(i, bodies[i]);
        for (int i = 1; i < bodies.Length; i++)
            bodies[i].SetRotation(Fixed64.FromFraction(1, 4));

        for (int i = 0; i < definitions.Length; i++)
        {
            definitions[i] = new RagdollJointDefinition2D(
                i,
                i + 1,
                JointType2D.Pin,
                LocalFrame(Vector2d.Right * Fixed64.Half),
                LocalFrame(-Vector2d.Right * Fixed64.Half),
                JointLimit2D.Unrestricted,
                motor,
                JointCollisionPolicy.SuppressLinked);
        }

        RagdollRuntime2D runtime = context.Constraints2D.RegisterRagdoll(new RagdollDefinition2D(
            links,
            definitions,
            RagdollSelfCollisionPolicy.SuppressAdjacentLinks));
        joints = new Joint2D[runtime.JointCount];
        for (int i = 0; i < joints.Length; i++)
            joints[i] = runtime.GetJoint(i);
    }

    private static void CreateRagdoll(
        GravitasWorldContext context,
        Vector2d offset,
        out SolidBody2D[] bodies,
        out Joint2D[] joints,
        bool sleepEnabled)
    {
        CreateStaticFloor(context, offset);
        bodies = new[]
        {
            CreateBox(context, offset + new Vector2d(Fixed64.Zero, Fixed64.FromFraction(3, 2)), sleepEnabled),
            CreateCircle(context, offset + new Vector2d(Fixed64.Zero, (Fixed64)3), dynamic: true, sleepEnabled: sleepEnabled),
            CreateCircle(context, offset + new Vector2d((Fixed64)(-2), Fixed64.FromFraction(3, 2)), dynamic: true, sleepEnabled: sleepEnabled),
            CreateCircle(context, offset + new Vector2d((Fixed64)2, Fixed64.FromFraction(3, 2)), dynamic: true, sleepEnabled: sleepEnabled),
            CreateCircle(context, offset + new Vector2d(Fixed64.FromFraction(-3, 4), Fixed64.FromFraction(1, 4)), dynamic: true, sleepEnabled: sleepEnabled),
            CreateCircle(context, offset + new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.FromFraction(1, 4)), dynamic: true, sleepEnabled: sleepEnabled)
        };

        RagdollRuntime2D runtime = context.Constraints2D.RegisterRagdoll(new RagdollDefinition2D(
            new[]
            {
                Link(0, bodies[0]),
                Link(1, bodies[1]),
                Link(2, bodies[2]),
                Link(3, bodies[3]),
                Link(4, bodies[4]),
                Link(5, bodies[5])
            },
            new[]
            {
                RagdollJoint(0, 1, new Vector2d(Fixed64.Zero, Fixed64.One), new Vector2d(Fixed64.Zero, -Fixed64.One)),
                RagdollJoint(0, 2, -Vector2d.Right, Vector2d.Right),
                RagdollJoint(0, 3, Vector2d.Right, -Vector2d.Right),
                RagdollJoint(0, 4, new Vector2d(-Fixed64.Half, -Fixed64.One), new Vector2d(Fixed64.Zero, Fixed64.One)),
                RagdollJoint(0, 5, new Vector2d(Fixed64.Half, -Fixed64.One), new Vector2d(Fixed64.Zero, Fixed64.One))
            },
            RagdollSelfCollisionPolicy.SuppressAdjacentLinks));
        joints = new Joint2D[runtime.JointCount];
        for (int i = 0; i < joints.Length; i++)
            joints[i] = runtime.GetJoint(i);
    }

    private static RagdollDefinition2D CreateLineRagdoll(SolidBody2D[] bodies)
    {
        var links = new RagdollLinkDefinition2D[bodies.Length];
        for (int i = 0; i < bodies.Length; i++)
            links[i] = Link(i, bodies[i]);

        var joints = new RagdollJointDefinition2D[bodies.Length - 1];
        for (int i = 0; i < joints.Length; i++)
            joints[i] = RagdollJoint(i, i + 1, Vector2d.Right * Fixed64.Half, -Vector2d.Right * Fixed64.Half);

        return new RagdollDefinition2D(links, joints, RagdollSelfCollisionPolicy.SuppressAdjacentLinks);
    }

    private static SolidBody2D[] CreateCircleLineBodies(
        GravitasWorldContext context,
        int count,
        Fixed64 spacing,
        bool sleepEnabled)
    {
        var bodies = new SolidBody2D[count];
        for (int i = 0; i < bodies.Length; i++)
        {
            bool dynamic = i != 0;
            bodies[i] = CreateCircle(
                context,
                new Vector2d(spacing * i, Fixed64.Zero),
                dynamic,
                sleepEnabled);
        }

        return bodies;
    }

    private static SolidBody2D CreateCircle(
        GravitasWorldContext context,
        Vector2d position,
        bool dynamic,
        bool sleepEnabled)
    {
        return CreateBody(context, new LSCircleCollider2D(Fixed64.Half), position, dynamic, sleepEnabled, Fixed64.One);
    }

    private static SolidBody2D CreateBox(
        GravitasWorldContext context,
        Vector2d position,
        bool sleepEnabled)
    {
        var collider = new LSAABBoxCollider2D(new Vector2d(Fixed64.One, Fixed64.FromFraction(3, 2)));
        return CreateBody(context, collider, position, dynamic: true, sleepEnabled: sleepEnabled, mass: (Fixed64)3);
    }

    private static SolidBody2D CreateBody(
        GravitasWorldContext context,
        LSCollider2D collider,
        Vector2d position,
        bool dynamic,
        bool sleepEnabled,
        Fixed64 mass)
    {
        var agent = new BenchmarkMatterAgent(context, new Vector3d(position.X, Fixed64.Zero, position.Y));
        var body = new SolidBody2D(agent, collider)
        {
            Mass = mass,
            SleepEnabled = sleepEnabled
        };

        body.Initialize(position, isDynamic: dynamic);
        return body;
    }

    private static void CreateStaticFloor(GravitasWorldContext context, Vector2d offset)
    {
        var collider = new LSAABBoxCollider2D(new Vector2d((Fixed64)24, Fixed64.One));
        var agent = new BenchmarkMatterAgent(
            context,
            new Vector3d(offset.X, Fixed64.Zero, offset.Y - Fixed64.One));
        var body = new SolidBody2D(agent, collider)
        {
            Mass = Fixed64.One,
            FreezeAxes = BodyFreezeAxes2D.Position
        };
        body.Initialize(offset + new Vector2d(Fixed64.Zero, -Fixed64.One));
    }

    private static RagdollLinkDefinition2D Link(int id, SolidBody2D body) =>
        new(id, body, body.Collider);

    private static RagdollJointDefinition2D RagdollJoint(
        int first,
        int second,
        Vector2d frameA,
        Vector2d frameB) =>
        new(first, second, JointType2D.Pin, LocalFrame(frameA), LocalFrame(frameB));

    private static JointFrame2D LocalFrame(Vector2d anchor) => new(anchor, Fixed64.Zero);

    private static Joint2D[] Combine(Joint2D[] first, Joint2D[] second)
    {
        var result = new Joint2D[first.Length + second.Length];
        Array.Copy(first, 0, result, 0, first.Length);
        Array.Copy(second, 0, result, first.Length, second.Length);
        return result;
    }

    private static void Warm(GravitasWorldContext context, int frames)
    {
        for (int i = 0; i < frames; i++)
        {
            context.Simulate();
            context.LateSimulate();
        }
    }

    private static int SumRows(ReadOnlySpan<Joint2D> joints)
    {
        int rows = 0;
        for (int i = 0; i < joints.Length; i++)
            rows += joints[i].LastSolveMetrics.PreparedRowCount;

        return rows;
    }
}
