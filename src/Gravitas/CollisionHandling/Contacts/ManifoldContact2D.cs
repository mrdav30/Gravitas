using FixedMathSharp;

namespace Gravitas.CollisionHandling;

/// <summary>
/// A deterministic pure 2D narrow-phase contact point for a collision manifold.
/// </summary>
public readonly struct ManifoldContact2D
{
    public ManifoldContact2D(
        ulong contactId,
        Vector2d pointA,
        Vector2d pointB,
        Fixed64 depth,
        Vector2d normal)
    {
        ContactId = contactId;
        PointA = pointA;
        PointB = pointB;
        Depth = depth.Abs();
        Normal = normal.MagnitudeSquared > Fixed64.Epsilon
            ? normal.Normalized
            : Vector2d.Zero;
    }

    /// <summary>
    /// Stable identity derived from the unordered pair of world-space contact points.
    /// </summary>
    public ulong ContactId { get; }

    /// <summary>
    /// World-space X/Z-plane contact point on collider A.
    /// </summary>
    public Vector2d PointA { get; }

    /// <summary>
    /// World-space X/Z-plane contact point on collider B.
    /// </summary>
    public Vector2d PointB { get; }

    /// <summary>
    /// Penetration depth along <see cref="Normal"/>.
    /// </summary>
    public Fixed64 Depth { get; }

    /// <summary>
    /// Unit normal pointing from collider A toward collider B.
    /// </summary>
    public Vector2d Normal { get; }
}
