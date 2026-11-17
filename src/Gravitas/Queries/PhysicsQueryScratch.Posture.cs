//=======================================================================
// PhysicsQueryScratch.Posture.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Queries;

/// <content>Holds detached shapes for allocation-free posture clearance.</content>
public sealed partial class PhysicsQueryScratch
{
    private readonly LSCircleCollider2D _postureCircle = new(Fixed64.Half);
    private readonly LSCapsuleCollider2D _postureCapsule2D = new(Fixed64.Half, Fixed64.One);
    private readonly LSSphereCollider _postureSphere = new();
    private readonly LSCapsuleCollider _postureCapsule3D = new();

    internal LSCollider2D GetPostureCandidate(ColliderShapeDefinition2D definition)
    {
        if (definition.Kind == ColliderShapeDefinition2DKind.Circle)
        {
            _postureCircle.SetDetachedPostureDimensions(definition);
            return _postureCircle;
        }

        _postureCapsule2D.SetDetachedPostureDimensions(definition);
        return _postureCapsule2D;
    }

    internal LSCollider GetPostureCandidate(ColliderShapeDefinition definition)
    {
        LSCollider candidate = definition.Kind == ColliderShapeDefinitionKind.Sphere
            ? _postureSphere : _postureCapsule3D;
        candidate.SetDetachedPostureDimensions(definition);
        return candidate;
    }

    internal static bool ChargePosturePreparation(FixedTransform transform, ref PhysicsQueryMeter meter)
    {
        if (!meter.ChargeGeometry(1))
            return false;

        // The planar path can walk the host ancestry five times: to retain its
        // world Y, verify the position round trip, and validate and compose scale.
        for (FixedTransform? parent = transform.Parent; parent != null; parent = parent.Parent)
            if (!meter.ChargeGeometry(6)) // include this admission walk
                return false;
        return true;
    }
}
