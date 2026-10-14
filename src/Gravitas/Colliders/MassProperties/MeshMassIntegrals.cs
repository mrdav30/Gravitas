//=======================================================================
// MeshMassIntegrals.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.Colliders;

/// <summary>Retains weighted first moments and inertia about a mesh reference before scalar projection.</summary>
internal readonly struct MeshMassIntegrals
{
    internal readonly ExactMassWeight Weight;
    internal readonly Vector3d Reference;
    // First moments use 60*Q192; inertia uses 60*Q224. These factors cancel
    // tetrahedron/shell divisors and leave final compound normalization exact.
    internal readonly Signed576 FirstX, FirstY, FirstZ;
    internal readonly Signed576 Ixx, Iyy, Izz, Ixy, Ixz, Iyz;

    private MeshMassIntegrals(ExactMassWeight weight, Vector3d reference,
        Signed576 firstX, Signed576 firstY, Signed576 firstZ,
        Signed576 ixx, Signed576 iyy, Signed576 izz,
        Signed576 ixy, Signed576 ixz, Signed576 iyz)
    {
        Weight = weight;
        Reference = reference;
        FirstX = firstX; FirstY = firstY; FirstZ = firstZ;
        Ixx = ixx; Iyy = iyy; Izz = izz;
        Ixy = ixy; Ixz = ixz; Iyz = iyz;
    }

    internal static MeshMassIntegrals FromSolid(Vector3d reference, Signed576 volume,
        Signed576 firstX, Signed576 firstY, Signed576 firstZ,
        Signed576 x2, Signed576 y2, Signed576 z2,
        Signed576 xy, Signed576 xz, Signed576 yz)
    {
        int sign = volume.Sign;
        Signed576 magnitude = sign < 0 ? WideArithmetic.SubtractSigned576(default, volume) : volume;
        var weight = new ExactMassWeight(Signed320.NarrowValue(
            WideArithmetic.MultiplySigned576(magnitude, Signed192.One)));
        Signed192 qSquared = Signed192.NarrowValue(
            WideArithmetic.MultiplySigned192(Signed192.One, Signed192.One));
        return new MeshMassIntegrals(weight, reference,
            Scale(firstX, 15 * sign, qSquared), Scale(firstY, 15 * sign, qSquared), Scale(firstZ, 15 * sign, qSquared),
            Scale(WideArithmetic.AddSigned576(y2, z2), 6 * sign, qSquared),
            Scale(WideArithmetic.AddSigned576(x2, z2), 6 * sign, qSquared),
            Scale(WideArithmetic.AddSigned576(x2, y2), 6 * sign, qSquared),
            Scale(xy, -3 * sign, qSquared), Scale(xz, -3 * sign, qSquared), Scale(yz, -3 * sign, qSquared));
    }

    internal static MeshMassIntegrals FromShell(ExactMassWeight weight,
        Signed576 firstX, Signed576 firstY, Signed576 firstZ,
        Signed576 x2, Signed576 y2, Signed576 z2,
        Signed576 xy, Signed576 xz, Signed576 yz) =>
        new(weight, Vector3d.Zero,
            Scale(firstX, 20, Signed192.One), Scale(firstY, 20, Signed192.One), Scale(firstZ, 20, Signed192.One),
            Scale(WideArithmetic.AddSigned576(y2, z2), 10, Signed192.One),
            Scale(WideArithmetic.AddSigned576(x2, z2), 10, Signed192.One),
            Scale(WideArithmetic.AddSigned576(x2, y2), 10, Signed192.One),
            Scale(xy, -5, Signed192.One), Scale(xz, -5, Signed192.One), Scale(yz, -5, Signed192.One));

    private static Signed576 Scale(Signed576 value, long coefficient, Signed192 padding) =>
        WideArithmetic.MultiplySigned576(value, Signed192.Signed(coefficient), padding);
}
