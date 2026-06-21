//=======================================================================
// SolverContact2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Solver-ready scalar pure 2D contact data.
/// </summary>
internal readonly struct SolverContact2D
{
    public SolverContact2D(
        ulong contactId,
        ResponseBody2D bodyA,
        ResponseBody2D bodyB,
        Vector2d pointA,
        Vector2d pointB,
        Vector2d relativeA,
        Vector2d relativeB,
        Fixed64 depth,
        Vector2d normal,
        Fixed64 cachedNormalImpulse,
        Fixed64 cachedTangentImpulse)
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
        Tangent = normal.RightHandNormal;
        CachedNormalImpulse = cachedNormalImpulse;
        CachedTangentImpulse = cachedTangentImpulse;
    }

    public ulong ContactId { get; }

    public ResponseBody2D A { get; }

    public ResponseBody2D B { get; }

    public Vector2d PointA { get; }

    public Vector2d PointB { get; }

    public Vector2d RelativeA { get; }

    public Vector2d RelativeB { get; }

    public Fixed64 Depth { get; }

    public Vector2d Normal { get; }

    public Vector2d Tangent { get; }

    public Fixed64 CachedNormalImpulse { get; }

    public Fixed64 CachedTangentImpulse { get; }

    public Fixed64 TotalInverseMass => A.InverseMass + B.InverseMass;
}
