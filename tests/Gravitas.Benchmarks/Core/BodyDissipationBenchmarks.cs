using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Materials;
using System.Numerics;

namespace Gravitas.Benchmarks;

// The measured operation includes public pose/motion resets and one complete host
// frame. Small-body/high-coefficient stopping failures are correctness-only cases.
[MemoryDiagnoser]
public class BodyDissipationBenchmarks
{
    [Params("LinearDrag", "AngularDrag", "DefaultDamping", "GroundedCombined", "FrozenAnisotropic")]
    public string Scenario { get; set; }
    private GravitasWorldContext _context;
    private SolidBody[] _bodies;
    private Vector3d[] _positions;
    private Vector3d[] _angularImpulses;
    private FixedQuaternion _rotation;
    private bool _grounded;
    private bool _linear;
    private bool _angular;
    private BigInteger _initialEnergy;
    public const int Count = 64;

    [GlobalSetup]
    public void Setup()
    {
        bool combined = Scenario == "GroundedCombined" || Scenario == "FrozenAnisotropic";
        bool frozen = Scenario == "FrozenAnisotropic";
        _grounded = Scenario == "GroundedCombined";
        _linear = Scenario == "LinearDrag" || combined;
        _angular = Scenario != "LinearDrag";
        _rotation = frozen ? FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)37, Fixed64.Zero) : FixedQuaternion.Identity;
        _context = BenchmarkPhysicsScene.CreateContext(80);
        _context.Settings.RuntimeMode = PhysicsRuntimeMode.ThreeD;
        _context.SetFrameRate(64);
        _context.Environment.Gravity = _grounded ? Fixed64.One : Fixed64.Zero;
        _context.Environment.AirDensity = Scenario == "DefaultDamping" ? Fixed64.Zero : Fixed64.One;
        _context.Environment.DampingFactor = Scenario == "DefaultDamping" || combined ? PhysicsEnvironment.DefaultDampingFactor : Fixed64.Zero;
        _context.Environment.MinSpeed = Fixed64.Zero;
        _context.Environment.MaxSpeed = (Fixed64)100;
        _bodies = new SolidBody[Count]; _positions = new Vector3d[Count]; _angularImpulses = new Vector3d[Count];
        var material = _grounded ? new PhysicsMaterial(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Zero) : PhysicsMaterial.Frictionless;
        for (int i = 0; i < Count; i++)
        {
            _positions[i] = new Vector3d((Fixed64)(i % 8 * 8), Fixed64.Zero, (Fixed64)(i / 8 * 8));
            var collider = new LSCuboidCollider { Size = frozen ? new Vector3d(1, 2, 3) : Vector3d.One, Material = material };
            var body = new SolidBody(new BenchmarkMatterAgent(_context, _positions[i]), collider)
            {
                Mass = Fixed64.One, SleepEnabled = false,
                LinearDragCoefficient = _linear ? Fixed64.One : Fixed64.Zero,
                FreezeAxes = frozen ? BodyFreezeAxes3D.RotationY : BodyFreezeAxes3D.None
            };
            body.Initialize(_positions[i], _rotation); body.UseManualGrounding(); _bodies[i] = body;
            // The body default angular coefficient remains 0.75; no reflective
            // setup is needed. A weak ordinary step must already dissipate on the baseline.
            _angularImpulses[i] = frozen ? WorldInertia(body) * ((Vector3d.Right + Vector3d.Forward) / (Fixed64)8) : Vector3d.Up / (Fixed64)16;
        }
        ResetInputs(); _initialEnergy = Energy(_bodies[0]);
        for (int warm = 0; warm < 32; warm++) ResetAndStep64();
        Validate();
    }

    [Benchmark]
    public long ResetAndStep64()
    {
        ResetInputs(); _context.Simulate(); _context.LateSimulate();
        long checksum = 0;
        for (int i = 0; i < Count; i++)
        {
            var body = _bodies[i];
            checksum += body.AngularVelocity.X.m_rawValue ^ body.AngularVelocity.Y.m_rawValue
                ^ body.AngularVelocity.Z.m_rawValue ^ body.LinearVelocity.X.m_rawValue;
        }
        return checksum;
    }

    private void ResetInputs()
    {
        for (int i = 0; i < Count; i++)
        {
            var body = _bodies[i]; body.ResetPosition(_positions[i], _rotation);
            if (_grounded) body.SetManualGrounding(_positions[i], Vector3d.Up);
            if (_linear) body.AddLinearImpulse(Vector3d.Right * Fixed64.FromFraction(3, 8));
            if (_angular) body.AddAngularImpulse(_angularImpulses[i]);
        }
    }

    public void Validate()
    {
        var body = _bodies[0];
        if (body.IsGrounded != _grounded || Energy(body) > _initialEnergy)
            throw new System.InvalidOperationException("Ordinary dissipation must not add kinetic energy or change grounding.");
        if (_linear && body.LinearVelocity.X <= Fixed64.Zero)
            throw new System.InvalidOperationException("Ordinary linear dissipation reversed or exhausted its small step.");
        if (_angular && body.AngularVelocity.MagnitudeSquared <= Fixed64.Zero)
            throw new System.InvalidOperationException("Ordinary angular dissipation exhausted its small step.");
        if (Scenario == "FrozenAnisotropic" && body.AngularVelocity.Y != Fixed64.Zero)
            throw new System.InvalidOperationException("Frozen rotation axis moved.");
    }

    public long[] Output()
    {
        var b = _bodies[0];
        return new[] { b.LinearVelocity.X.m_rawValue, b.LinearVelocity.Y.m_rawValue, b.LinearVelocity.Z.m_rawValue,
            b.AngularVelocity.X.m_rawValue, b.AngularVelocity.Y.m_rawValue, b.AngularVelocity.Z.m_rawValue, b.IsGrounded ? 1L : 0L };
    }

    // Validation-only exact integer energy, outside the measured loop. Both mass
    // v² and wᵀIw use raw Q96, so their sum can be compared without scalar rounding.
    private static BigInteger Energy(SolidBody body)
    {
        var v = body.LinearVelocity; var w = body.AngularVelocity; var m = WorldInertia(body);
        BigInteger x = w.X.m_rawValue, y = w.Y.m_rawValue, z = w.Z.m_rawValue;
        return (BigInteger)body.Mass.m_rawValue * ((BigInteger)v.X.m_rawValue * v.X.m_rawValue + (BigInteger)v.Y.m_rawValue * v.Y.m_rawValue + (BigInteger)v.Z.m_rawValue * v.Z.m_rawValue)
            + x * (m.M11.m_rawValue * x + m.M12.m_rawValue * y + m.M13.m_rawValue * z)
            + y * (m.M21.m_rawValue * x + m.M22.m_rawValue * y + m.M23.m_rawValue * z)
            + z * (m.M31.m_rawValue * x + m.M32.m_rawValue * y + m.M33.m_rawValue * z);
    }
    private static Fixed3x3 WorldInertia(SolidBody body) => body.Rotation.Conjugate().ToMatrix3x3()
        * body.Collider.CalculateInertiaTensor(body.Mass, body.LocalCenterOfMassOffset) * body.Rotation.ToMatrix3x3();
    [GlobalCleanup] public void Cleanup() => _context?.Dispose();
}

