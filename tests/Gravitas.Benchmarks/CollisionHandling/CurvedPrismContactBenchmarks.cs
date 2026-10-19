using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class CurvedPrismContactBenchmarks
{
    private GravitasWorldContext _context;
    private (LSCollider Curved, LSCollider2D Slab) _coneSeparated, _coneFace, _coneOblique;
    private (LSCollider Curved, LSCollider2D Slab) _cylinderSeparated, _cylinderFace, _cylinderOblique;

    [GlobalSetup]
    public void Setup()
    {
        _context = BenchmarkPhysicsScene.CreateMixedContext(16, 16, clearAllPools: true);
        Vector3d coneNormal = new(Fixed64.FromRaw(2716375826), Fixed64.FromRaw(1920767767), Fixed64.FromRaw(2716375826));
        Vector3d coneCenter = new(Fixed64.FromRaw(-565576967), Fixed64.FromRaw(-557169159), Fixed64.FromRaw(-565576967));
        Fixed64 coneMaximum = Vector3d.Dot(coneCenter, coneNormal) + FixedMath.Max(
            Fixed64.Half * coneNormal.Y,
            Fixed64.Half * FixedMath.Sqrt(coneNormal.X * coneNormal.X + coneNormal.Z * coneNormal.Z)
                - Fixed64.Half * coneNormal.Y);
        // The box occupies [0,4] x [0,2] x [0,4]; its minimum projection is zero.
        if (-coneMaximum <= Fixed64.FromFraction(1, 2048))
            throw new InvalidOperationException("The cone fixture must have strict whole-shape separation.");
        _coneSeparated = CreatePair(true, coneCenter, FixedQuaternion.Identity,
            new LSAABBoxCollider2D(new Vector2d(4, 4)), new Vector3d(2, 1, 2), Fixed64.One);

        FixedQuaternion obliqueRotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)22, (Fixed64)17, (Fixed64)9);
        Vector3d cylinderNormal = new(Fixed64.Zero, Fixed64.FromRaw(-3980925575), Fixed64.FromRaw(-1612133878));
        Vector3d cylinderCenter = new Vector3d(Fixed64.Quarter, Fixed64.FromFraction(3, 4), Fixed64.Quarter)
            - cylinderNormal * (Fixed64.FromRaw(1918558518) + Fixed64.FromFraction(1, 1024));
        Vector3d localNormal = obliqueRotation.Conjugate().Rotate(cylinderNormal);
        Fixed64 cylinderMaximum = Vector3d.Dot(cylinderCenter, cylinderNormal)
            + FixedMath.Sqrt(localNormal.X * localNormal.X + localNormal.Z * localNormal.Z) + localNormal.Y.Abs();
        // This prism has Y in [-8,0] and Z in [-1/2,1/2].
        if (cylinderNormal.Z * Fixed64.Half - cylinderMaximum <= Fixed64.FromFraction(1, 2048))
            throw new InvalidOperationException("The cylinder fixture must have strict whole-shape separation.");
        _cylinderSeparated = CreatePair(false, cylinderCenter, obliqueRotation,
            new LSPolygonCollider2D(new Vector2d(-Fixed64.One, -Fixed64.Half), new Vector2d(Fixed64.One, -Fixed64.Half),
                new Vector2d(Fixed64.One, Fixed64.Half), new Vector2d(-Fixed64.One, Fixed64.Half)),
            new Vector3d(0, -4, 0), (Fixed64)4);

        _coneFace = CreateOverlap(true, false, obliqueRotation);
        _coneOblique = CreateOverlap(true, true, obliqueRotation);
        _cylinderFace = CreateOverlap(false, false, obliqueRotation);
        _cylinderOblique = CreateOverlap(false, true, obliqueRotation);
        // Separated queries intentionally have no expected-result gate here: their old
        // false positives must remain measurable in the pre-fix baseline.
    }

    [Benchmark]
    public bool ConeSeparated() => CollisionDetectionMixed.TryCollide(_coneSeparated.Curved, _coneSeparated.Slab, out _);

    [Benchmark]
    public bool ConeFace() => CollisionDetectionMixed.TryCollide(_coneFace.Curved, _coneFace.Slab, out _);

    [Benchmark]
    public bool ConeOblique() => CollisionDetectionMixed.TryCollide(_coneOblique.Curved, _coneOblique.Slab, out _);

    [Benchmark]
    public bool CylinderSeparated() => CollisionDetectionMixed.TryCollide(_cylinderSeparated.Curved, _cylinderSeparated.Slab, out _);

    [Benchmark]
    public bool CylinderFace() => CollisionDetectionMixed.TryCollide(_cylinderFace.Curved, _cylinderFace.Slab, out _);

    [Benchmark]
    public bool CylinderOblique() => CollisionDetectionMixed.TryCollide(_cylinderOblique.Curved, _cylinderOblique.Slab, out _);

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();

    private (LSCollider Curved, LSCollider2D Slab) CreateOverlap(bool cone, bool oblique, FixedQuaternion rotation)
    {
        Vector3d center = oblique ? new Vector3d(Fixed64.FromFraction(7, 4), Fixed64.FromFraction(3, 4), Fixed64.FromFraction(7, 4))
            : Vector3d.Up * (cone ? Fixed64.FromFraction(5, 4) : Fixed64.FromFraction(7, 4));
        var pair = CreatePair(cone, center, oblique ? rotation : FixedQuaternion.Identity,
            new LSAABBoxCollider2D(new Vector2d(4, 4)), Vector3d.Zero, Fixed64.One);
        // Oblique centers are inside both solids. Face fixtures share the axis
        // point (0,7/8,0), above the curved base and below the slab's upper face.
        if (!CollisionDetectionMixed.TryCollide(pair.Curved, pair.Slab, out MixedContact contact)
            || contact.Depth <= Fixed64.Zero || contact.DepthIsClamped)
            throw new InvalidOperationException("The ordinary curved/prism fixture must penetrate.");
        return pair;
    }

    private (LSCollider Curved, LSCollider2D Slab) CreatePair(bool cone, Vector3d center, FixedQuaternion rotation,
        LSCollider2D slab, Vector3d slabCenter, Fixed64 halfThickness)
    {
        LSCollider curved = cone ? new LSConeCollider() : new LSCylinderCollider();
        curved.Radius = cone ? Fixed64.Half : Fixed64.One;
        curved.Size = cone ? Vector3d.One : Vector3d.One * Fixed64.Two;
        var agent = new BenchmarkMatterAgent(_context, center);
        agent.Transform.LocalRotation = rotation;
        curved.InitializeWithNoBody(agent);
        slab.MixedHalfThicknessOverride = halfThickness;
        slab.InitializeWithNoBody(new BenchmarkMatterAgent(_context, slabCenter));
        return (curved, slab);
    }
}
