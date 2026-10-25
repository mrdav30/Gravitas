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
    private (LSCollider Curved, LSCollider2D Slab) _coneApexTouch, _coneSideTouch, _cylinderSideTouch;
    private (LSCollider Curved, LSCollider2D Slab) _sphereContained, _capsuleContained;
    private (LSCollider Curved, LSCollider2D Slab) _capsuleCrossingOffcenter, _sphereContainedOpposingFace;
    private (LSCollider Curved, LSCollider2D Slab) _sphereHorizontalRim;

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
        _coneApexTouch = CreateTouch(true);
        _cylinderSideTouch = CreateTouch(false);
        _coneSideTouch = CreatePair(true, Vector3d.Zero, FixedQuaternion.Identity,
            new LSAABBoxCollider2D(new Vector2d(Fixed64.Half, Fixed64.Two)),
            new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.Zero), Fixed64.One);
        // The cone has 2X+Y<=1/2; the prism has 2X+Y>=1/2. Their
        // unique common point is inside the smooth cone generator.
        Vector3d sidePoint = new(Fixed64.Quarter, Fixed64.Zero, Fixed64.Zero);
        Vector3d sideNormal = new(Fixed64.FromRaw(3_841_535_534L), Fixed64.FromRaw(1_920_767_767L), Fixed64.Zero);
        if (!CollisionDetectionMixed.TryCollide(_coneSideTouch.Curved, _coneSideTouch.Slab, out MixedContact side)
            || side.Depth != Fixed64.Zero || side.DepthIsClamped || side.Normal3DTo2D != sideNormal
            || !side.TryGetPoint3D(out Vector3d first) || !side.TryGetPoint2D(out Vector3d second)
            || first != sidePoint || second != sidePoint)
            throw new InvalidOperationException("The cone/prism fixture must touch at its finite generator point.");
        _sphereContained = CreateContained(false);
        _capsuleContained = CreateContained(true);
        _capsuleCrossingOffcenter = CreateFiniteWitnessPair(true);
        _sphereContainedOpposingFace = CreateFiniteWitnessPair(false);
        _sphereHorizontalRim = CreateHorizontalRim();
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

    [Benchmark]
    public bool ConeApexTouch() => CollisionDetectionMixed.TryCollide(_coneApexTouch.Curved, _coneApexTouch.Slab, out _);

    [Benchmark]
    public bool ConeSideTouch() => CollisionDetectionMixed.TryCollide(_coneSideTouch.Curved, _coneSideTouch.Slab, out _);

    [Benchmark]
    public bool CylinderSideTouch() => CollisionDetectionMixed.TryCollide(_cylinderSideTouch.Curved, _cylinderSideTouch.Slab, out _);

    [Benchmark]
    public bool SphereContained() => CollisionDetectionMixed.TryCollide(_sphereContained.Curved, _sphereContained.Slab, out _);

    [Benchmark]
    public bool CapsuleContained() => CollisionDetectionMixed.TryCollide(_capsuleContained.Curved, _capsuleContained.Slab, out _);

    [Benchmark]
    public bool CapsuleCrossingOffcenter() => CollisionDetectionMixed.TryCollide(_capsuleCrossingOffcenter.Curved, _capsuleCrossingOffcenter.Slab, out _);

    [Benchmark]
    public bool SphereContainedOpposingFace() => CollisionDetectionMixed.TryCollide(_sphereContainedOpposingFace.Curved, _sphereContainedOpposingFace.Slab, out _);

    [Benchmark]
    public bool SphereHorizontalRim() => CollisionDetectionMixed.TryCollide(_sphereHorizontalRim.Curved, _sphereHorizontalRim.Slab, out _);

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();

    private (LSCollider Curved, LSCollider2D Slab) CreateHorizontalRim()
    {
        var sphere = new LSSphereCollider { Radius = Fixed64.Two };
        sphere.InitializeWithNoBody(new BenchmarkMatterAgent(_context, new Vector3d(-1, -1, 1)));
        var slab = new LSAABBoxCollider2D(new Vector2d(2, 2)) { MixedHalfThicknessOverride = Fixed64.One };
        slab.InitializeWithNoBody(new BenchmarkMatterAgent(_context, Vector3d.One));
        // The cube's closest point is (0,0,1), on its horizontal rim.
        // Subtracting these irrational values from integers preserves nearest-even rounding.
        Fixed64 rootTwo = FixedMath.Sqrt(Fixed64.Two);
        Fixed64 normalComponent = FixedMath.Sqrt(Fixed64.Half);
        Vector3d expectedNormal = new(normalComponent, normalComponent, Fixed64.Zero);
        Vector3d expectedSpherePoint = new(rootTwo - Fixed64.One, rootTwo - Fixed64.One, Fixed64.One);
        if (!CollisionDetectionMixed.TryCollide(sphere, slab, out MixedContact contact)
            || contact.Depth != Fixed64.Two - rootTwo || contact.DepthIsClamped
            || contact.Normal3DTo2D != expectedNormal
            || !contact.TryGetPoint3D(out Vector3d first) || !contact.TryGetPoint2D(out Vector3d second)
            || first != expectedSpherePoint || second != Vector3d.Forward)
            throw new InvalidOperationException("The sphere must retain its finite horizontal-rim contact.");
        return (sphere, slab);
    }

    private (LSCollider Curved, LSCollider2D Slab) CreateFiniteWitnessPair(bool capsule)
    {
        LSCollider curved = capsule
            ? new LSCapsuleCollider { Radius = Fixed64.Quarter,
                Size = new Vector3d(Fixed64.Half, Fixed64.FromFraction(21, 2), Fixed64.Half) }
            : new LSSphereCollider { Radius = Fixed64.FromFraction(3, 4) };
        var agent = new BenchmarkMatterAgent(_context, capsule ? Vector3d.Up * Fixed64.Half : Vector3d.Zero);
        if (capsule)
            agent.Transform.LocalRotation = new(Fixed64.Half, Fixed64.Half, -Fixed64.Half, Fixed64.Half);
        curved.InitializeWithNoBody(agent);
        var slab = new LSAABBoxCollider2D(capsule ? new Vector2d(1, 2) : new Vector2d(2, 6))
        {
            MixedHalfThicknessOverride = capsule ? Fixed64.One : Fixed64.Two
        };
        slab.InitializeWithNoBody(new BenchmarkMatterAgent(_context,
            capsule ? Vector3d.Right * Fixed64.FromFraction(5, 2) : Vector3d.Zero));
        // The capsule's finite paired side supports share X in [2,3]. The
        // contained sphere instead pairs its -X surface with the prism's +X face.
        if (!CollisionDetectionMixed.TryCollide(curved, slab, out MixedContact contact)
            || contact.Depth != Fixed64.FromFraction(capsule ? 3 : 7, 4) || contact.DepthIsClamped
            || contact.Normal3DTo2D != (capsule ? Vector3d.Down : Vector3d.Left)
            || !contact.TryGetPoint3D(out Vector3d first) || !contact.TryGetPoint2D(out Vector3d second)
            || (capsule
                ? first.X < Fixed64.Two || first.X > (Fixed64)3
                    || first.Y != Fixed64.Quarter || first.Z != Fixed64.Zero
                    || second != new Vector3d(first.X, Fixed64.One, Fixed64.Zero)
                : first != Vector3d.Left * Fixed64.FromFraction(3, 4) || second != Vector3d.Right)
            || first - second != contact.Normal3DTo2D * contact.Depth)
            throw new InvalidOperationException("The sphere/capsule must retain its finite opposing support witnesses.");
        return (curved, slab);
    }

    private (LSCollider Curved, LSCollider2D Slab) CreateContained(bool capsule)
    {
        LSCollider curved = capsule
            ? new LSCapsuleCollider { Radius = Fixed64.Quarter,
                Size = new Vector3d(Fixed64.Half, Fixed64.FromFraction(3, 2), Fixed64.Half) }
            : new LSSphereCollider { Radius = Fixed64.Quarter };
        var agent = new BenchmarkMatterAgent(_context, Vector3d.Up * Fixed64.Half);
        if (capsule)
            agent.Transform.LocalRotation = new(Fixed64.Half, Fixed64.Half, -Fixed64.Half, Fixed64.Half);
        curved.InitializeWithNoBody(agent);
        var slab = new LSAABBoxCollider2D(new Vector2d(4, 4)) { MixedHalfThicknessOverride = Fixed64.One };
        slab.InitializeWithNoBody(new BenchmarkMatterAgent(_context, Vector3d.Zero));
        // Both shapes have Y extent[1/4,3/4]. The perpendicular capsule core
        // stays inside the prism, so its complete nearest exit is upward3/4.
        if (!CollisionDetectionMixed.TryCollide(curved, slab, out MixedContact contact)
            || contact.Depth != Fixed64.FromFraction(3, 4) || contact.DepthIsClamped
            || contact.Normal3DTo2D != Vector3d.Down
            || !contact.TryGetPoint3D(out Vector3d first) || !contact.TryGetPoint2D(out Vector3d second)
            || first != Vector3d.Up * Fixed64.Quarter || second != Vector3d.Up)
            throw new InvalidOperationException("The contained sphere/capsule must retain its nearest exit and finite paired witnesses.");
        return (curved, slab);
    }

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

    private (LSCollider Curved, LSCollider2D Slab) CreateTouch(bool cone)
    {
        FixedQuaternion rotation = cone ? FixedQuaternion.Identity
            : new(Fixed64.Half, Fixed64.Half, Fixed64.Half, Fixed64.Half);
        // The cone apex is at Y=-1. This exactly representable rotation maps
        // the cylinder axis to world Z and local X to world Up, keeping
        // its bottom side exactly Y=2-radius=1 after transform preparation.
        Vector3d center = Vector3d.Up * (cone ? Fixed64.FromFraction(-3, 2) : Fixed64.Two);
        var pair = CreatePair(cone, center, rotation, new LSAABBoxCollider2D(new Vector2d(20, 20)),
            Vector3d.Zero, Fixed64.One);
        if (!CollisionDetectionMixed.TryCollide(pair.Curved, pair.Slab, out MixedContact contact)
            || contact.Depth != Fixed64.Zero || contact.DepthIsClamped
            || contact.Normal3DTo2D != (cone ? Vector3d.Up : Vector3d.Down)
            || !contact.TryGetPoint3D(out Vector3d first) || !contact.TryGetPoint2D(out Vector3d second)
            || first != second || first.Y != (cone ? -Fixed64.One : Fixed64.One)
            || first.X.Abs() > Fixed64.One || first.Z.Abs() > Fixed64.One)
            throw new InvalidOperationException("The curved/prism fixture must touch at its exact supporting plane.");
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
