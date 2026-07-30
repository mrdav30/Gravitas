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

    public SolidBody Body { get; }

    public Fixed64 InverseMass { get; }

    public bool HasSolverMobility => Body.HasSolverMobility;

    public bool CanRotate => Body.CanRotate;

    public Fixed64 GetConstrainedInverseMass(Vector3d axis) =>
        Body.GetConstrainedInverseMass(axis);

    public static ResponseBody Create(LSCollider collider)
    {
        SolidBody body = collider.Body!;
        return Create(body);
    }

    public static ResponseBody Create(SolidBody body)
    {
        Fixed64 inverseMass = body.EffectiveInverseMass;
        return new ResponseBody(body, inverseMass);
    }
}
