//=======================================================================
// CollisionResult.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.CollisionHandling;

public readonly struct CollisionResult
{
    public readonly (Vector3d Vector, Fixed64 Depth) AxisPenetration;
    public readonly (Vector3d Point1, Vector3d Point2) PointsOfContact;

    public CollisionResult((Vector3d Point1, Vector3d Point2) pointsOfContact, (Vector3d Vector, Fixed64 Depth) axisPenetration)
    {
        AxisPenetration = axisPenetration;
        PointsOfContact = pointsOfContact;
    }
}
