using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class CapsuleSlabContactBenchmarks
{
    private GravitasWorldContext _context;
    private LSCapsuleCollider _capsule;
    private LSCapsuleCollider2D _slab;

    [Params("Side", "Cap", "StraightRim", "SeparatedRim", "EndRegion", "Containment", "ObliqueInteriorRim",
        "ObliquePositiveRim", "ObliqueIrrationalRim", "ObliqueFanWins")]
    public string Geometry { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = GravitasWorldContext.CreateOwned();
        _context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        Fixed64 radius = Fixed64.One;
        Fixed64 coreLength = Fixed64.Two;
        Fixed64 slabRadius = Fixed64.One;
        Fixed64 slabCoreLength = Fixed64.Two;
        Fixed64 halfThickness = Fixed64.One;
        Vector3d center = new(Fixed64.FromFraction(7, 4), Fixed64.Zero, Fixed64.Zero);
        FixedQuaternion rotation = FixedQuaternion.Identity;
        bool expectedHit = true;
        Fixed64 expectedDepth = Fixed64.FromFraction(1, 4);
        switch (Geometry)
        {
            case "Side":
                // Side escape: radius sum 2 minus center X=7/4.
                break;
            case "Cap":
                // Cap escape: H + capsule half-core + radius - center Y.
                center = new Vector3d(Fixed64.Zero, Fixed64.FromFraction(11, 4), Fixed64.Zero);
                break;
            case "StraightRim":
            case "SeparatedRim":
                Fixed64 q = Fixed64.FromRaw(3_037_000_500L);
                rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, -q, q);
                coreLength = (Fixed64)20;
                slabRadius = (Fixed64)10;
                expectedHit = Geometry == "StraightRim";
                radius = expectedHit ? Fixed64.FromFraction(3, 2) : Fixed64.One;
                center = new Vector3d(Fixed64.FromFraction(83, 4),
                    expectedHit ? Fixed64.Two : Fixed64.FromFraction(7, 4), Fixed64.Zero);
                // Closest endpoint X=43/4. Straight rim residual (3/4,1,0)
                // has length 5/4, so depth=1/4. GRV088's (3/4,3/4,0)
                // has squared distance 9/8 > radius^2=1: separated.
                expectedDepth = expectedHit ? expectedDepth : Fixed64.Zero;
                break;
            case "EndRegion":
                // Positive-Z end: slab core half-length 1 + radii 2 - 11/4.
                center = new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.FromFraction(11, 4));
                break;
            case "Containment":
                radius = Fixed64.Half;
                coreLength = Fixed64.One;
                slabRadius = Fixed64.Two;
                slabCoreLength = (Fixed64)4;
                halfThickness = (Fixed64)3;
                center = Vector3d.Zero;
                // Whole-slab minimum is the straight side: 2+1/2.
                // Either constituent end cylinder alone exits at only 1/2.
                expectedDepth = Fixed64.FromFraction(5, 2);
                break;
            case "ObliqueInteriorRim":
                const long scale = 858_993_459L;
                rotation = new FixedQuaternion(Fixed64.FromRaw(scale), Fixed64.FromRaw(2 * scale),
                    Fixed64.FromRaw(-4 * scale), Fixed64.FromRaw(2 * scale));
                radius = Fixed64.FromFraction(21, 4);
                center = new Vector3d(0, 5, -5);
                // Exact axis (20,-9,-12)/25 is perpendicular to n=(0,4,-3)/5.
                // Negative-end rim p=(0,1,-2), core center q=p+5*n and n.Z<0
                // certify a whole-slab closest pair with depth 21/4-5=1/4.
                break;
            case "ObliquePositiveRim":
            case "ObliqueIrrationalRim":
            case "ObliqueFanWins":
                // Independent whole-shape fixtures from the upstream contact tests:
                // positive stationary minimum, irrational winner, and two admitted
                // roots that both lose to the earlier analytic boundary.
                // Normalize the proportional frame as authored upstream so the host
                // transform preserves the same exact reduced rotation columns.
                Fixed64 authoredScale = Fixed64.One / FixedMath.Sqrt((Fixed64)(Geometry == "ObliqueFanWins" ? 10 : 25));
                radius = Fixed64.FromFraction(1, 4);
                if (Geometry == "ObliqueFanWins")
                {
                    rotation = new FixedQuaternion(authoredScale, -2 * authoredScale, -authoredScale, 2 * authoredScale);
                    coreLength = (Fixed64)20000; slabRadius = (Fixed64)3125; halfThickness = (Fixed64)899;
                    center = new Vector3d(-1024, 0, -1);
                    expectedDepth = (Fixed64)2101 + radius;
                }
                else
                {
                    rotation = new FixedQuaternion(4 * authoredScale, -2 * authoredScale, authoredScale, 2 * authoredScale);
                    coreLength = (Fixed64)2000; slabRadius = (Fixed64)625; halfThickness = (Fixed64)500;
                    bool irrational = Geometry == "ObliqueIrrationalRim";
                    center = irrational ? new Vector3d(0, 446, 273) : new Vector3d(0, 392, 545);
                    // Irrational core gap=45*sqrt(21), independently integer-square bracketed.
                    expectedDepth = (irrational ? Fixed64.FromRaw(885_690_573_358L) : (Fixed64)135) + radius;
                }
                break;
            default:
                throw new InvalidOperationException("Unknown capsule/slab contact geometry.");
        }

        _capsule = new LSCapsuleCollider
        {
            Radius = radius,
            Size = new Vector3d(radius * Fixed64.Two, coreLength + radius * Fixed64.Two, radius * Fixed64.Two)
        };
        var agent = new BenchmarkMatterAgent(_context, center);
        agent.Transform.LocalRotation = rotation;
        _capsule.InitializeWithNoBody(agent);
        _slab = new LSCapsuleCollider2D(slabRadius, slabCoreLength + slabRadius * Fixed64.Two)
        {
            MixedHalfThicknessOverride = halfThickness
        };
        _slab.InitializeWithNoBody(new BenchmarkMatterAgent(_context, Vector3d.Zero));

        bool hit = CollisionDetectionMixed.TryCollide(_capsule, _slab, out MixedContact contact);
        if (hit != expectedHit || contact.Depth != expectedDepth || contact.DepthIsClamped)
            throw new InvalidOperationException($"Incorrect capsule/slab contact for {Geometry}: hit {hit}/{expectedHit}, " +
                $"raw depth {contact.Depth.m_rawValue}/{expectedDepth.m_rawValue}, clamped {contact.DepthIsClamped}.");
    }

    [Benchmark]
    public bool Contact() => CollisionDetectionMixed.TryCollide(_capsule, _slab, out _);

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();
}
