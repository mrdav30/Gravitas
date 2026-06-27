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
    private ResponseBody(SolidBody body, Fixed64 inverseMass, Fixed3x3 inverseInertiaTensor)
    {
        Body = body;
        InverseMass = inverseMass;
        InverseInertiaTensor = inverseInertiaTensor;
    }

    public SolidBody Body { get; }

    public Fixed64 InverseMass { get; }

    public Fixed3x3 InverseInertiaTensor { get; }

    public bool CanMove => InverseMass > Fixed64.Zero;

    public bool CanRotate => Body.CanRotate;

    public Fixed64 GetConstrainedInverseMass(Vector3d axis) =>
        Body.GetConstrainedInverseMass(axis);

    public Vector3d ApplyConstrainedInverseInertia(Vector3d torqueAxis) =>
        Body.ApplyConstrainedInverseInertia(torqueAxis);

    public static ResponseBody Create(LSCollider collider)
    {
        SolidBody body = collider.Body!;
        return Create(body);
    }

    public static ResponseBody Create(SolidBody body)
    {
        Fixed64 inverseMass = body.EffectiveInverseMass;
        Fixed3x3 inverseInertiaTensor = body.EffectiveInverseInertiaTensor;

        return new ResponseBody(body, inverseMass, inverseInertiaTensor);
    }
}
