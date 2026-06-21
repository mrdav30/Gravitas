//=======================================================================
// SolverContact.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Represents a contact point between two bodies during collision resolution, 
/// containing necessary information for the solver to compute impulses and resolve the collision.
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
        Vector3d normal)
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

    public Fixed64 TotalInverseMass => A.InverseMass + B.InverseMass;
}
