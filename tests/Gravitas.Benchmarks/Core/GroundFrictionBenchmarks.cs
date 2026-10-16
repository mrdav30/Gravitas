using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Materials;

namespace Gravitas.Benchmarks;

// Complete fixed-input motion integration: public state reset and one full host
// frame are included. Overshooting inputs are diagnostic-only, never speed gates.
[MemoryDiagnoser]
public class GroundFrictionBenchmarks
{
    [Params("GroundUnitAngular", "GroundAnisotropicAngular", "AirAngular", "GroundLinear")]
    public string Scenario { get; set; }
    private GravitasWorldContext _context;
    private SolidBody[] _bodies;
    private Vector3d[] _positions;
    private Vector3d _angularImpulse;
    private bool _grounded;
    private bool _linear;
    public const int Count = 64;
    [GlobalSetup]
    public void Setup()
    {
        _context = BenchmarkPhysicsScene.CreateContext(80);
        _context.Settings.RuntimeMode = PhysicsRuntimeMode.ThreeD;
        _context.SetFrameRate(64);
        _context.Environment.Gravity = Fixed64.One;
        _context.Environment.AirDensity = Fixed64.Zero;
        _context.Environment.DampingFactor = Fixed64.Zero;
        _context.Environment.MinSpeed = Fixed64.Zero;
        _context.Environment.MaxSpeed = (Fixed64)100;
        _grounded = Scenario != "AirAngular";
        _linear = Scenario == "GroundLinear";
        bool anisotropic = Scenario == "GroundAnisotropicAngular";
        _angularImpulse = anisotropic ? (Vector3d.Right + Vector3d.Up) / (Fixed64)8 : Vector3d.Up / (Fixed64)16;
        _bodies = new SolidBody[Count]; _positions = new Vector3d[Count];
        var material = new PhysicsMaterial(Fixed64.One, Fixed64.One, Fixed64.Zero);
        for (int i = 0; i < Count; i++)
        {
            _positions[i] = new Vector3d((Fixed64)(i % 8 * 8), Fixed64.Zero, (Fixed64)(i / 8 * 8));
            var collider = new LSCuboidCollider { Size = anisotropic ? new Vector3d(1, 2, 3) : Vector3d.One, Material = material };
            var body = new SolidBody(new BenchmarkMatterAgent(_context, _positions[i]), collider) { Mass = Fixed64.One, SleepEnabled = false };
            body.Initialize(_positions[i], FixedQuaternion.Identity); body.UseManualGrounding(); _bodies[i] = body;
        }
        for (int warm = 0; warm < 32; warm++) ResetAndStep64();
        Validate();
    }
    [Benchmark]
    public long ResetAndStep64()
    {
        for (int i = 0; i < Count; i++)
        {
            SolidBody body = _bodies[i];
            body.ResetPosition(_positions[i], FixedQuaternion.Identity);
            if (_grounded) body.SetManualGrounding(_positions[i], Vector3d.Up);
            else body.UseManualGrounding();
            if (_linear) body.AddLinearImpulse(Vector3d.Right);
            else body.AddAngularImpulse(_angularImpulse);
        }
        _context.Simulate(); _context.LateSimulate();
        long checksum = 0;
        for (int i = 0; i < Count; i++)
        {
            var body = _bodies[i];
            checksum += body.AngularVelocity.X.m_rawValue ^ body.AngularVelocity.Y.m_rawValue ^ body.LinearVelocity.X.m_rawValue;
        }
        return checksum;
    }
    public long[] Output()
    {
        var b = _bodies[0];
        return new[] { b.AngularVelocity.X.m_rawValue, b.AngularVelocity.Y.m_rawValue, b.AngularVelocity.Z.m_rawValue,
            b.LinearVelocity.X.m_rawValue, b.LinearVelocity.Y.m_rawValue, b.LinearVelocity.Z.m_rawValue, b.IsGrounded ? 1L : 0L };
    }
    public void Validate()
    {
        var b = _bodies[0];
        if (b.IsGrounded != _grounded) throw new System.InvalidOperationException("Manual grounding state changed.");
        if (_linear && (b.LinearVelocity.X < Fixed64.Zero || b.LinearVelocity.X >= Fixed64.One))
            throw new System.InvalidOperationException("Ordinary ground linear friction must dissipate without reversing.");
        if (!_linear && (b.AngularVelocity.Y <= Fixed64.Zero || b.AngularVelocity.Y > Fixed64.Half))
            throw new System.InvalidOperationException("Ordinary angular friction must retain positive bounded spin.");
    }
    [GlobalCleanup] public void Cleanup() => _context?.Dispose();
}
