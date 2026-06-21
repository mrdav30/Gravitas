//=======================================================================
// SolverContact.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Solver-ready 3D contact data, including the deterministic tangent frame and cached impulses.
/// </summary>
internal readonly struct SolverContact
{
    public SolverContact(
        ulong contactId,
        ResponseBody bodyA,
        ResponseBody bodyB,
        Vector3d pointA,
        Vector3d pointB,
        Vector3d relativeA,
        Vector3d relativeB,
        Fixed64 depth,
        Vector3d normal,
        Fixed64 cachedNormalImpulse,
        Fixed64 cachedTangentImpulse,
        Fixed64 cachedSecondaryTangentImpulse)
    {
        ContactId = contactId;
        A = bodyA;
        B = bodyB;
        PointA = pointA;
        PointB = pointB;
        RelativeA = relativeA;
        RelativeB = relativeB;
        Depth = depth;
        Normal = normal;
        Tangent = CreateTangent(normal);
        SecondaryTangent = Vector3d.Cross(normal, Tangent).Normalized;
        CachedNormalImpulse = cachedNormalImpulse;
        CachedTangentImpulse = cachedTangentImpulse;
        CachedSecondaryTangentImpulse = cachedSecondaryTangentImpulse;
    }

    public ulong ContactId { get; }

    public ResponseBody A { get; }

    public ResponseBody B { get; }

    public Vector3d PointA { get; }

    public Vector3d PointB { get; }

    public Vector3d RelativeA { get; }

    public Vector3d RelativeB { get; }

    public Fixed64 Depth { get; }

    public Vector3d Normal { get; }

    public Vector3d Tangent { get; }

    public Vector3d SecondaryTangent { get; }

    public Fixed64 CachedNormalImpulse { get; }

    public Fixed64 CachedTangentImpulse { get; }

    public Fixed64 CachedSecondaryTangentImpulse { get; }

    public Fixed64 TotalInverseMass => A.InverseMass + B.InverseMass;

    private static Vector3d CreateTangent(Vector3d normal)
    {
        Vector3d absolute = Vector3d.Abs(normal);
        Vector3d reference = absolute.X <= absolute.Y && absolute.X <= absolute.Z
            ? Vector3d.Right
            : absolute.Y <= absolute.Z
                ? Vector3d.Up
                : Vector3d.Forward;

        Vector3d tangent = Vector3d.Cross(reference, normal);
        return tangent.MagnitudeSquared > Fixed64.Epsilon
            ? tangent.Normalized
            : Vector3d.Right;
    }
}
