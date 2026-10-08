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
    {
        IsValid = true;
        PositionA = first.Position3d;
        PositionB = second.Position3d;
        CenterA = first.GetCenterOfMassAnchor();
        CenterB = second.GetCenterOfMassAnchor();
        LinearA = linearA;
        AngularA = angularA;
        LinearB = linearB;
        AngularB = angularB;
    }

    internal readonly bool IsValid;
    internal readonly Vector3d PositionA, PositionB, LinearA, AngularA, LinearB, AngularB;
    internal readonly ContactAnchor CenterA, CenterB;
}
