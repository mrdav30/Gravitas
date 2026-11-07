//=======================================================================
// TriangleShellMassProperties.TriangleCoordinates.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;

namespace Gravitas.Colliders;

/// <content>Defines the nested TriangleCoordinates type.</content>
internal static partial class TriangleShellMassProperties
{
    internal readonly struct TriangleCoordinates
    {
        internal readonly Signed192 AX, AY, AZ, BX, BY, BZ, CX, CY, CZ;
        internal TriangleCoordinates(in FixedTriangle triangle, in ScaledRigidFrame3d frame, bool exactScale)
        {
            if (exactScale)
            {
                frame.GetScaledLocalPointNumerators(triangle.A, out AX, out AY, out AZ);
                frame.GetScaledLocalPointNumerators(triangle.B, out BX, out BY, out BZ);
                frame.GetScaledLocalPointNumerators(triangle.C, out CX, out CY, out CZ);
            }
            else
            {
                AX = Signed192.Raw(triangle.A.X); AY = Signed192.Raw(triangle.A.Y); AZ = Signed192.Raw(triangle.A.Z);
                BX = Signed192.Raw(triangle.B.X); BY = Signed192.Raw(triangle.B.Y); BZ = Signed192.Raw(triangle.B.Z);
                CX = Signed192.Raw(triangle.C.X); CY = Signed192.Raw(triangle.C.Y); CZ = Signed192.Raw(triangle.C.Z);
            }
        }
    }
}
