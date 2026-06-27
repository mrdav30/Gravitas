using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Constraints;
using SwiftCollections;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class Constraint3DBenchmarks
{
    private readonly SwiftList<SolidBody> _bodies = new();
    private GravitasWorldContext _context;
    private RagdollRuntime3D _ragdoll;

    [Params(32)]
    public int LinkCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = BenchmarkPhysicsScene.CreateContext(
            BenchmarkPhysicsScene.GridExtentForGrid(LinkCount),
            clearAllPools: true);
        _context.Environment.Gravity = Fixed64.Zero;
        _context.Environment.AirDensity = Fixed64.Zero;
        _context.Environment.DampingFactor = Fixed64.Zero;
        _context.Settings.DiscreteSolverIterations = 8;

        BenchmarkPhysicsScene.CreateDynamicSphereGrid(_context, LinkCount, _bodies);
        _ragdoll = _context.Constraints3D.RegisterRagdoll(CreateRagdollDefinition());
        for (int i = 0; i < 4; i++)
        {
            _context.Simulate();
            _context.LateSimulate();
        }
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _context.Dispose();
        _context = null;
        _ragdoll = null;
        _bodies.FastClear();
    }

    [Benchmark]
    public int SimulateConstraintChain()
    {
        _context.Simulate();
        _context.LateSimulate();
        return _context.FrameCount;
    }

    [Benchmark]
    public int LinkedCollisionFilterLookup()
    {
        int filtered = 0;
        for (int i = 0; i < _bodies.Count - 1; i++)
        {
            if (_context.Constraints3D.ShouldExcludeLinkedCollision(_bodies[i].Collider, _bodies[i + 1].Collider))
                filtered++;
        }

        return filtered;
    }

    [Benchmark]
    public bool ToggleRagdollActivation()
    {
        if (_ragdoll.IsActive)
            _ragdoll.DeactivateToKinematic();
        else
            _ragdoll.ActivateDynamic();

        return _ragdoll.IsActive;
    }

    private RagdollDefinition3D CreateRagdollDefinition()
    {
        var links = new RagdollLinkDefinition3D[_bodies.Count];
        for (int i = 0; i < _bodies.Count; i++)
            links[i] = new RagdollLinkDefinition3D(i, _bodies[i], _bodies[i].Collider);

        var joints = new RagdollJointDefinition3D[_bodies.Count - 1];
        for (int i = 0; i < joints.Length; i++)
            joints[i] = new RagdollJointDefinition3D(i, i + 1, JointType3D.BallSocket, LocalFrame(), LocalFrame());

        return new RagdollDefinition3D(links, joints, RagdollSelfCollisionPolicy.SuppressAdjacentLinks);
    }

    private static FixedTransform LocalFrame() =>
        new(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
}
