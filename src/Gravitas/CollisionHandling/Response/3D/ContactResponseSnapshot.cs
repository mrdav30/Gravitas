//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.CollisionHandling;

/// <summary>Pre-warm-start motion and lever frames for one discrete solver pass.</summary>
internal readonly struct ContactResponseSnapshot
{
    internal ContactResponseSnapshot(SolidBody first, SolidBody second,
        Vector3d linearA, Vector3d angularA, Vector3d linearB, Vector3d angularB)
        : this(first.Position3d, second.Position3d, first.GetCenterOfMassAnchor(),
            second.GetCenterOfMassAnchor(), linearA, angularA, linearB, angularB)
    {
    }

    internal ContactResponseSnapshot(Vector3d positionA, Vector3d positionB,
        ContactAnchor centerA, ContactAnchor centerB, Vector3d linearA,
        Vector3d angularA, Vector3d linearB, Vector3d angularB, Fixed64 planarAngularB = default)
    {
        IsValid = true;
        PositionA = positionA;
        PositionB = positionB;
        CenterA = centerA;
        CenterB = centerB;
        LinearA = linearA;
        AngularA = angularA;
        LinearB = linearB;
        AngularB = angularB;
        PlanarAngularB = planarAngularB;
    }

    internal readonly bool IsValid;
    internal readonly Vector3d PositionA, PositionB, LinearA, AngularA, LinearB, AngularB;
    internal readonly ContactAnchor CenterA, CenterB;
    // Mixed yaw retains its own signed domain: negating MinValue cannot fit a spatial Fixed64 Y.
    internal readonly Fixed64 PlanarAngularB;
}
