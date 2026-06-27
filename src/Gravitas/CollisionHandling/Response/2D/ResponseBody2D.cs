//=======================================================================
// ResponseBody2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Scalar pure 2D response body data for one collider participant.
/// </summary>
internal readonly struct ResponseBody2D
{
    private ResponseBody2D(SolidBody2D? body, Fixed64 inverseMass, Fixed64 inverseMoment)
    {
        Body = body;
        InverseMass = inverseMass;
        InverseMoment = inverseMoment;
    }

    public SolidBody2D? Body { get; }

    public Fixed64 InverseMass { get; }

    public Fixed64 InverseMoment { get; }

    public bool CanTranslate => Body?.CanTranslate == true;

    public bool CanRotate => Body?.CanRotate == true;

    public Fixed64 GetConstrainedInverseMass(Vector2d axis) =>
        Body?.GetConstrainedInverseMass(axis) ?? Fixed64.Zero;

    public static ResponseBody2D Create(LSCollider2D collider)
    {
        SolidBody2D? body = collider.Body;
        if (body == null)
            return default;

        return new ResponseBody2D(
            body,
            body.EffectiveInverseMass,
            body.EffectiveInverseMomentOfInertia);
    }
}
