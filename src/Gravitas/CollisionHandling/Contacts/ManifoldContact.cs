using FixedMathSharp;

namespace Gravitas.CollisionHandling;

/// <summary>
/// A deterministic narrow-phase contact point for a collision manifold.
/// </summary>
public readonly struct ManifoldContact
{
    public ManifoldContact(
        ulong contactId,
        Vector3d pointA,
        Vector3d pointB,
        Fixed64 depth,
        Vector3d normal,
        Vector3d immovableCollisionDirection = default)
    {
        ContactId = contactId;
        PointA = pointA;
        PointB = pointB;
        Depth = depth.Abs();
        Normal = normal.MagnitudeSquared > Fixed64.Epsilon
            ? normal.Normalized
            : Vector3d.Zero;
        ImmovableCollisionDirection = immovableCollisionDirection;
    }

    /// <summary>
    /// Stable identity derived from the unordered pair of world-space contact points.
    /// </summary>
    public ulong ContactId { get; }

    /// <summary>
    /// World-space contact point on collider A.
    /// </summary>
    public Vector3d PointA { get; }

    /// <summary>
    /// World-space contact point on collider B.
    /// </summary>
    public Vector3d PointB { get; }

    /// <summary>
    /// Penetration depth along <see cref="Normal"/>.
    /// </summary>
    public Fixed64 Depth { get; }

    /// <summary>
    /// Unit normal pointing from collider A toward collider B.
    /// </summary>
    public Vector3d Normal { get; }

    /// <summary>
    /// Optional direction used by immovable-body response handling.
    /// </summary>
    public Vector3d ImmovableCollisionDirection { get; }

    public ManifoldContact WithImmovableDirection(Vector3d direction) =>
        new(ContactId, PointA, PointB, Depth, Normal, direction);
}
