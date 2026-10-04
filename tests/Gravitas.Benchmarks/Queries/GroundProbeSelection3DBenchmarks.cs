using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Diagnostics;
using Gravitas.Queries;
using Gravitas.Support;
using SwiftCollections;
using System;

namespace Gravitas.Benchmarks;

/// <summary>Forced automatic 3D probes with eligible or physically ignored raw targets.</summary>
[MemoryDiagnoser]
public class GroundProbeSelection3DBenchmarks
{
    private const int BodyCount = 64;
    private GravitasWorldContext _context = null!;
    private readonly SolidBody[] _bodies = new SolidBody[BodyCount];
    private readonly LSCollider[] _rawNearest = new LSCollider[BodyCount];
    private readonly LSCollider[] _supports = new LSCollider[BodyCount];
    private readonly SwiftList<Physics3DHit> _hits = new(8);

    [Params(false, true)]
    public bool UseSphere { get; set; }

    private GroundProbeMode ProbeMode => UseSphere ? GroundProbeMode.SweptSphere : GroundProbeMode.Ray;

    [Params(1, 8)]
    public int TargetsPerProbe { get; set; }

    [Params(false, true)]
    public bool Supported { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = BenchmarkPhysicsScene.CreateContext(48);
        _context.Settings.GroundCheckLayerMask = new PhysicsLayerMask((1 << 1) | (1 << 2));
        for (int i = 0; i < BodyCount; i++)
        {
            Vector3d position = new((i % 8) * 6, 4, (i / 8) * 6);
            var source = new LSSphereCollider
            {
                Layer = new PhysicsLayer(0),
                IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(1)
            };
            var body = new SolidBody(new BenchmarkMatterAgent(_context, position), source)
            {
                GroundProbeMode = ProbeMode,
                GroundProbeRadius = Fixed64.Half,
                GroundOriginOffset = Fixed64.Zero,
                GroundedDistanceRay = (Fixed64)4,
                GroundDownDistanceOnAir = (Fixed64)4
            };
            body.Initialize(position, FixedQuaternion.Identity);
            _bodies[i] = body;

            // Every target intersects the same probe. Ignored near targets
            // remain raw query hits; the accepted witness is the far floor.
            for (int j = 0; j < TargetsPerProbe - 1; j++)
            {
                var rejected = new LSCuboidCollider
                {
                    Layer = new PhysicsLayer(1),
                    Size = new Vector3d(Fixed64.One, Fixed64.FromFraction(1, 8), Fixed64.One)
                };
                Vector3d targetPosition = new(position.X,
                    (Fixed64)3 - Fixed64.FromFraction(j, 4), position.Z);
                BenchmarkPhysicsScene.CreateStaticCollider(_context, rejected, targetPosition);
                if (j == 0)
                    _rawNearest[i] = rejected;
            }
            var support = new LSCuboidCollider
            {
                Layer = new PhysicsLayer(Supported ? 2 : 1),
                Size = Vector3d.One
            };
            BenchmarkPhysicsScene.CreateStaticCollider(_context, support,
                new Vector3d(position.X, Fixed64.Zero, position.Z));
            _supports[i] = support;
            if (TargetsPerProbe == 1)
                _rawNearest[i] = support;
        }

        ValidateWitnesses();
        _context.Diagnostics.Disable();
        for (int i = 0; i < 8; i++)
        {
            ForceAutomaticGroundProbes();
            ClosestRawHits();
        }
        int supported = ForceAutomaticGroundProbes();
        if (supported != (Supported ? BodyCount : 0) || ClosestRawHits() != BodyCount)
            throw new InvalidOperationException($"Incorrect warmed ground probes: {supported} supports.");
    }

    private void ValidateWitnesses()
    {
        _context.Diagnostics.Enable();
        for (int i = 0; i < BodyCount; i++)
        {
            SolidBody body = _bodies[i];
            Vector3d start = body.Position3d;
            Vector3d end = start + Vector3d.Down * body.GroundedDistanceRay;
            _context.Diagnostics.Clear();
            if (ProbeMode == GroundProbeMode.Ray)
                _context.Query3D.RaycastAll(start, end, _context.Settings.GroundCheckLayerMask, _hits);
            else
                _context.Query3D.SweepSphereAll(start, end, body.GroundProbeRadius,
                    _context.Settings.GroundCheckLayerMask, _hits, body.Collider);
            if (_hits.Count != TargetsPerProbe || !ReferenceEquals(_hits[0].Collider, _rawNearest[i]))
                throw new InvalidOperationException($"Raw ground target count or nearest owner changed for body {i}.");
            GravitasDiagnosticEvent raw = _context.Diagnostics.Events[0];
            Physics3DHit support = _hits[_hits.Count - 1];
            Vector3d expectedPoint = new(start.X, Fixed64.Half, start.Z);
            Fixed64 expectedDistance = (Fixed64)4 - Fixed64.Half
                - (ProbeMode == GroundProbeMode.SweptSphere ? Fixed64.Half : Fixed64.Zero);
            if (!ReferenceEquals(support.Collider, _supports[i]) || !support.TryGetPoint(out Vector3d point)
                || point != expectedPoint || support.Normal != Vector3d.Up || support.Distance != expectedDistance)
                throw new InvalidOperationException($"Far support witness changed for body {i}.");
            int candidates = _context.Query3D.LastQueryCandidateCount;
            if (candidates != (ProbeMode == GroundProbeMode.Ray ? 0 : TargetsPerProbe))
                throw new InvalidOperationException($"Raw ground candidate count changed for body {i}.");
            if (!TryFindClosestRawHit(body, out Physics3DHit closest) || !closest.Equals(_hits[0]))
                throw new InvalidOperationException($"Public closest witness changed for body {i}.");
            _context.Diagnostics.Clear();

            body.CheckGround();

            if (_context.Diagnostics.Events.Length != 2 || !_context.Diagnostics.Events[0].Equals(raw)
                || _context.Query3D.LastQueryCandidateCount != candidates || body.IsGrounded != Supported)
                throw new InvalidOperationException($"Automatic grounding diagnostics changed for body {i}.");
            GravitasDiagnosticEvent probe = _context.Diagnostics.Events[1];
            if (probe.Kind != GravitasDiagnosticEventKind.GroundProbe || probe.Hit != Supported
                || probe.ColliderBId != (Supported ? _supports[i].Id : -1)
                || (Supported && (body.HitPoint != expectedPoint || body.GroundNormal != Vector3d.Up
                    || !ReferenceEquals(body.HitPlatform, _supports[i].Transform)
                    || probe.PointA != expectedPoint || probe.ScalarB != expectedDistance)))
                throw new InvalidOperationException($"Automatic grounding witness changed for body {i}.");
        }
    }

    [Benchmark]
    public int ForceAutomaticGroundProbes()
    {
        int supported = 0;
        for (int i = 0; i < _bodies.Length; i++)
        {
            _bodies[i].CheckGround();
            if (_bodies[i].IsGrounded)
                supported++;
        }
        return supported;
    }

    [Benchmark]
    public int ClosestRawHits()
    {
        int hitCount = 0;
        for (int i = 0; i < _bodies.Length; i++)
            if (TryFindClosestRawHit(_bodies[i], out _))
                hitCount++;
        return hitCount;
    }

    private bool TryFindClosestRawHit(SolidBody body, out Physics3DHit hit) => UseSphere
        ? _context.Query3D.SweepSphere(body.Position3d, body.GroundProbeRadius, Vector3d.Down,
            body.GroundedDistanceRay, out hit, _context.Settings.GroundCheckLayerMask, body.Collider)
        : _context.Query3D.Raycast(body.Position3d, Vector3d.Down, body.GroundedDistanceRay,
            out hit, _context.Settings.GroundCheckLayerMask);

    [GlobalCleanup]
    public void Cleanup() => _context?.Dispose();
}
