//=======================================================================
// ResponseBody.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Represents a body involved in a collision response, containing necessary information for collision resolution.
/// </summary>
internal readonly struct ResponseBody
{
    private ResponseBody(SolidBody body, Fixed64 inverseMass)
    {
        Body = body;
        InverseMass = inverseMass;
    }

    public SolidBody? Body { get; }

    public Fixed64 InverseMass { get; }

    public bool HasSolverMobility => Body?.HasSolverMobility == true;

    public Fixed64 GetConstrainedInverseMass(Vector3d axis) =>
        Body?.GetConstrainedInverseMass(axis) ?? Fixed64.Zero;

    public static ResponseBody Create(LSCollider collider)
    {
        SolidBody? body = collider.Body;
        return body == null ? default : new ResponseBody(body, body.EffectiveInverseMass);
    }
}
