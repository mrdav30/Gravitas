using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Constraints;
using System;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class Constraint3DBenchmarks
{
    private const int WarmupFrames = 16;
    private GravitasWorldContext _chainContext = null!;
    private GravitasWorldContext _ragdollContext = null!;
    private GravitasWorldContext _stackContext = null!;
    private GravitasWorldContext _motorContext = null!;
    private GravitasWorldContext _toggleContext = null!;
    private SolidBody[] _chainBodies = Array.Empty<SolidBody>();
    private Joint3D[] _chainJoints = Array.Empty<Joint3D>();
    private Joint3D[] _ragdollJoints = Array.Empty<Joint3D>();
    private Joint3D[] _stackJoints = Array.Empty<Joint3D>();
    private Joint3D[] _motorJoints = Array.Empty<Joint3D>();
    private RagdollRuntime3D _toggleRagdoll = null!;

    [Params(32)]
    public int LinkCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _chainContext = CreateConstraintContext(BenchmarkPhysicsScene.GridExtentForLine(LinkCount), Fixed64.Zero, 10);
        CreateBallSocketLine(_chainContext, LinkCount, out _chainBodies, out _chainJoints);
        Warm(_chainContext, WarmupFrames);

        _ragdollContext = CreateConstraintContext(48, (Fixed64)10, 12);
        CreateRagdoll(_ragdollContext, Vector3d.Zero, out _, out _ragdollJoints, sleepEnabled: false);
        Warm(_ragdollContext, 80);

        _stackContext = CreateConstraintContext(64, (Fixed64)10, 12);
        CreateRagdoll(_stackContext, new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero), out _, out Joint3D[] leftStack, sleepEnabled: false);
        CreateRagdoll(_stackContext, new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero), out _, out Joint3D[] rightStack, sleepEnabled: false);
        _stackJoints = Combine(leftStack, rightStack);
        Warm(_stackContext, 80);

        _motorContext = CreateConstraintContext(BenchmarkPhysicsScene.GridExtentForLine(LinkCount), Fixed64.Zero, 8);
        CreateMotorLine(_motorContext, LinkCount, out _motorJoints);
        Warm(_motorContext, WarmupFrames);

        _toggleContext = CreateConstraintContext(BenchmarkPhysicsScene.GridExtentForLine(LinkCount), Fixed64.Zero, 8);
        CreateBallSocketLine(_toggleContext, LinkCount, out SolidBody[] toggleBodies, out _);
        _toggleRagdoll = _toggleContext.Constraints3D.RegisterRagdoll(CreateLineRagdoll(toggleBodies));
        _toggleRagdoll.DeactivateToKinematic();
        _toggleRagdoll.ActivateDynamic();
        _toggleRagdoll.DeactivateToKinematic();
        _toggleRagdoll.ActivateDynamic();
        Warm(_toggleContext, WarmupFrames);
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _chainContext.Dispose();
        _ragdollContext.Dispose();
        _stackContext.Dispose();
        _motorContext.Dispose();
        _toggleContext.Dispose();
        _chainBodies = Array.Empty<SolidBody>();
        _chainJoints = Array.Empty<Joint3D>();
        _ragdollJoints = Array.Empty<Joint3D>();
        _stackJoints = Array.Empty<Joint3D>();
        _motorJoints = Array.Empty<Joint3D>();
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
    public int SimulateHumanoidRagdollResting()
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
            if (_chainContext.Constraints3D.ShouldExcludeLinkedCollision(_chainBodies[i].Collider, _chainBodies[i + 1].Collider))
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

    private static GravitasWorldContext CreateConstraintContext(int gridExtent, Fixed64 gravity, int solverIterations)
    {
        GravitasWorldContext context = BenchmarkPhysicsScene.CreateContext(gridExtent, clearAllPools: true);
        context.Environment.Gravity = gravity;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        context.Settings.DiscreteSolverIterations = solverIterations;
        return context;
    }

    private static void CreateBallSocketLine(
        GravitasWorldContext context,
        int count,
        out SolidBody[] bodies,
        out Joint3D[] joints,
        bool seedImpulse = true)
    {
        bodies = new SolidBody[count];
        for (int i = 0; i < bodies.Length; i++)
        {
            bool dynamic = i != 0;
            bodies[i] = CreateSphere(context, new Vector3d((Fixed64)(i * 2), Fixed64.Zero, Fixed64.Zero), dynamic, sleepEnabled: false);
        }

        joints = new Joint3D[count - 1];
        for (int i = 0; i < joints.Length; i++)
        {
            joints[i] = context.Constraints3D.RegisterJoint(new JointDefinition3D(
                bodies[i],
                bodies[i + 1],
                LocalFrame(Vector3d.Right * Fixed64.Half),
                LocalFrame(-Vector3d.Right * Fixed64.Half),
                JointType3D.BallSocket,
                JointLimit3D.Unrestricted,
                JointMotor3D.Disabled,
                JointCollisionPolicy.SuppressLinked));
        }

        if (seedImpulse)
            bodies[^1].AddLinearImpulse(new Vector3d((Fixed64)6, Fixed64.Zero, (Fixed64)2));
    }

    private static void CreateMotorLine(
        GravitasWorldContext context,
        int count,
        out Joint3D[] joints)
    {
        SolidBody[] bodies = CreateSphereLineBodies(context, count, Fixed64.One, sleepEnabled: false);
        var motor = new JointMotor3D(
            FixedQuaternion.FromAxisAngle(Vector3d.Right, Fixed64.HalfPi),
            (Fixed64)12,
            Fixed64.Zero,
            Fixed64.FromFraction(1, 8));
        var links = new RagdollLinkDefinition3D[bodies.Length];
        var definitions = new RagdollJointDefinition3D[bodies.Length - 1];
        for (int i = 0; i < bodies.Length; i++)
            links[i] = Link(i, bodies[i]);
        for (int i = 1; i < bodies.Length; i++)
            bodies[i].SetRotation(FixedQuaternion.FromAxisAngle(Vector3d.Right, Fixed64.FromFraction(1, 4)));

        for (int i = 0; i < definitions.Length; i++)
        {
            definitions[i] = new RagdollJointDefinition3D(
                i,
                i + 1,
                JointType3D.BallSocket,
                LocalFrame(Vector3d.Right * Fixed64.Half),
                LocalFrame(-Vector3d.Right * Fixed64.Half),
                JointLimit3D.Unrestricted,
                motor,
                JointCollisionPolicy.SuppressLinked);
        }

        RagdollRuntime3D runtime = context.Constraints3D.RegisterRagdoll(new RagdollDefinition3D(
            links,
            definitions,
            RagdollSelfCollisionPolicy.SuppressAllLinks));
        joints = new Joint3D[runtime.JointCount];
        for (int i = 0; i < joints.Length; i++)
            joints[i] = runtime.GetJoint(i);
    }

    private static void CreateRagdoll(
        GravitasWorldContext context,
        Vector3d offset,
        out SolidBody[] bodies,
        out Joint3D[] joints,
        bool sleepEnabled)
    {
        CreateStaticFloor(context, offset);
        bodies = new[]
        {
            CreateCuboid(context, offset + new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 2), Fixed64.Zero), sleepEnabled),
            CreateSphere(context, offset + new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero), dynamic: true, sleepEnabled: sleepEnabled),
            CreateSphere(context, offset + new Vector3d((Fixed64)(-2), Fixed64.FromFraction(3, 2), Fixed64.Zero), dynamic: true, sleepEnabled: sleepEnabled),
            CreateSphere(context, offset + new Vector3d((Fixed64)2, Fixed64.FromFraction(3, 2), Fixed64.Zero), dynamic: true, sleepEnabled: sleepEnabled),
            CreateSphere(context, offset + new Vector3d(Fixed64.FromFraction(-3, 4), Fixed64.FromFraction(1, 4), Fixed64.Zero), dynamic: true, sleepEnabled: sleepEnabled),
            CreateSphere(context, offset + new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.FromFraction(1, 4), Fixed64.Zero), dynamic: true, sleepEnabled: sleepEnabled)
        };

        RagdollRuntime3D runtime = context.Constraints3D.RegisterRagdoll(new RagdollDefinition3D(
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
                RagdollJoint(0, 1, Vector3d.Up, -Vector3d.Up),
                RagdollJoint(0, 2, -Vector3d.Right, Vector3d.Right),
                RagdollJoint(0, 3, Vector3d.Right, -Vector3d.Right),
                RagdollJoint(0, 4, new Vector3d(-Fixed64.Half, -Fixed64.One, Fixed64.Zero), Vector3d.Up),
                RagdollJoint(0, 5, new Vector3d(Fixed64.Half, -Fixed64.One, Fixed64.Zero), Vector3d.Up)
            },
            RagdollSelfCollisionPolicy.SuppressAdjacentLinks));
        joints = new Joint3D[runtime.JointCount];
        for (int i = 0; i < joints.Length; i++)
            joints[i] = runtime.GetJoint(i);
    }

    private static RagdollDefinition3D CreateLineRagdoll(SolidBody[] bodies)
    {
        var links = new RagdollLinkDefinition3D[bodies.Length];
        for (int i = 0; i < bodies.Length; i++)
            links[i] = Link(i, bodies[i]);

        var joints = new RagdollJointDefinition3D[bodies.Length - 1];
        for (int i = 0; i < joints.Length; i++)
            joints[i] = RagdollJoint(i, i + 1, Vector3d.Right * Fixed64.Half, -Vector3d.Right * Fixed64.Half);

        return new RagdollDefinition3D(links, joints, RagdollSelfCollisionPolicy.SuppressAdjacentLinks);
    }

    private static SolidBody[] CreateSphereLineBodies(
        GravitasWorldContext context,
        int count,
        Fixed64 spacing,
        bool sleepEnabled)
    {
        var bodies = new SolidBody[count];
        for (int i = 0; i < bodies.Length; i++)
        {
            bool dynamic = i != 0;
            bodies[i] = CreateSphere(
                context,
                new Vector3d(spacing * i, Fixed64.Zero, Fixed64.Zero),
                dynamic,
                sleepEnabled);
        }

        return bodies;
    }

    private static SolidBody CreateSphere(
        GravitasWorldContext context,
        Vector3d position,
        bool dynamic,
        bool sleepEnabled)
    {
        return CreateBody(context, new LSSphereCollider(), position, dynamic, sleepEnabled, Fixed64.One);
    }

    private static SolidBody CreateCuboid(
        GravitasWorldContext context,
        Vector3d position,
        bool sleepEnabled)
    {
        var collider = new LSCuboidCollider { Size = new Vector3d(Fixed64.One, Fixed64.FromFraction(3, 2), Fixed64.Half) };
        return CreateBody(context, collider, position, dynamic: true, sleepEnabled: sleepEnabled, mass: (Fixed64)3);
    }

    private static SolidBody CreateBody(
        GravitasWorldContext context,
        LSCollider collider,
        Vector3d position,
        bool dynamic,
        bool sleepEnabled,
        Fixed64 mass)
    {
        var agent = new BenchmarkMatterAgent(context, position);
        var body = new SolidBody(agent, collider)
        {
            Mass = mass,
            SleepEnabled = sleepEnabled
        };

        body.Initialize(position, FixedQuaternion.Identity, dynamic);
        return body;
    }

    private static void CreateStaticFloor(GravitasWorldContext context, Vector3d offset)
    {
        var floor = new LSCuboidCollider { Size = new Vector3d((Fixed64)24, Fixed64.One, (Fixed64)24) };
        var agent = new BenchmarkMatterAgent(context, offset + new Vector3d(Fixed64.Zero, -Fixed64.One, Fixed64.Zero));
        floor.InitializeWithNoBody(agent);
    }

    private static RagdollLinkDefinition3D Link(int id, SolidBody body) =>
        new(id, body);

    private static RagdollJointDefinition3D RagdollJoint(
        int first,
        int second,
        Vector3d frameA,
        Vector3d frameB) =>
        new(first, second, JointType3D.BallSocket, LocalFrame(frameA), LocalFrame(frameB));

    private static FixedTransform LocalFrame(Vector3d position) =>
        new(position, FixedQuaternion.Identity, Vector3d.One);

    private static Joint3D[] Combine(Joint3D[] first, Joint3D[] second)
    {
        var result = new Joint3D[first.Length + second.Length];
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

    private static int SumRows(ReadOnlySpan<Joint3D> joints)
    {
        int rows = 0;
        for (int i = 0; i < joints.Length; i++)
            rows += joints[i].LastSolveMetrics.PreparedRowCount;

        return rows;
    }
}
