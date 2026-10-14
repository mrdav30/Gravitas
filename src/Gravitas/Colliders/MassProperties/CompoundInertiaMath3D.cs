//=======================================================================
// CompoundInertiaMath3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using FixedMathSharp;

namespace Gravitas.Colliders;

/// <summary>Stores an aggregate's unmaterialized weighted tensor about its requested reference.</summary>
// Full-domain shape weights fit 256 bits and semantic displacements fit 162.
// Squared shifts, factor sixty and an int-bounded part sum stay below 650 bits;
// applying full-domain mass fits 704. Intrinsic/rotation terms fit 576 bits.
internal struct CompoundInertiaTensor
{
    internal Signed704 Xx, Yy, Zz, Xy, Xz, Yz;

    internal Fixed3x3 Materialize(Fixed64 mass, ExactMassWeight weight)
    {
        Signed576 denominator = WideArithmetic.MultiplySigned320(
            CompoundInertiaMath3D.ScaleWeight(weight), CompoundInertiaMath3D.Q192);
        bool representable = TryMoment(Xx, mass, denominator, out Fixed64 xx)
            & TryMoment(Yy, mass, denominator, out Fixed64 yy)
            & TryMoment(Zz, mass, denominator, out Fixed64 zz)
            & TryMoment(Xy, mass, denominator, out Fixed64 xy)
            & TryMoment(Xz, mass, denominator, out Fixed64 xz)
            & TryMoment(Yz, mass, denominator, out Fixed64 yz);
        if (!representable)
            throw new InvalidOperationException("The compound collider's inertia tensor is outside the Fixed64 scalar domain.");
        return new Fixed3x3(xx, xy, xz, xy, yy, yz, xz, yz, zz);
    }

    private static bool TryMoment(Signed704 geometry, Fixed64 mass, Signed576 denominator, out Fixed64 moment) =>
        Fixed64.TryGetSignedRawRatio(
            WideArithmetic.MultiplySigned704ToSigned832(geometry, Signed192.Raw(mass)),
            Signed832.ExtendValue(denominator), numeratorLeftShift: 0, out moment);
}

/// <summary>Accumulates homogeneous weighted moments without child mass or tensor rounding.</summary>
internal static class CompoundInertiaMath3D
{
    internal static readonly Signed320 Q192 = Signed320.NarrowValue(
        WideArithmetic.MultiplySigned576(Signed576.ExtendValue(
            WideArithmetic.MultiplySigned192(Signed192.One, Signed192.One)),
            Signed192.One, Signed192.One, Signed192.One, Signed192.One));

    internal static Signed320 ScaleWeight(ExactMassWeight weight) =>
        Signed320.NarrowValue(WideArithmetic.MultiplySigned320(weight.Numerator, Signed192.Signed(60)));

    internal static void Add(ref CompoundInertiaTensor tensor, LSCollider part,
        ExactMassWeight weight, Vector3d reference)
    {
        Span<Signed576> intrinsic = stackalloc Signed576[6];
        intrinsic.Clear();
        ExactMassPoint3D point;
        MeshMassIntegrals mesh = default;
        if (part is LSMeshCollider meshCollider)
        {
            mesh = meshCollider.Mesh.GetCompoundMassIntegrals(usePrepared: false, meshCollider.InertiaPolicy);
            intrinsic[0] = mesh.Ixx; intrinsic[1] = mesh.Iyy; intrinsic[2] = mesh.Izz;
            intrinsic[3] = mesh.Ixy; intrinsic[4] = mesh.Ixz; intrinsic[5] = mesh.Iyz;
            point = part.TransformRelativeMassPropertyPointExact(mesh.Reference);
        }
        else
        {
            GetIntrinsic(part, weight, intrinsic);
            point = part.CalculateLocalMassPoint();
        }

        // Q224 intrinsic moments acquire Q64 rotation products and Q32 padding.
        // Q96 semantic displacements square directly into the same Q320 frame.
        AddIntrinsic(ref tensor, intrinsic, part.CompoundLocalRotation);
        Signed320 dx = Difference(point.XNumerator, reference.X);
        Signed320 dy = Difference(point.YNumerator, reference.Y);
        Signed320 dz = Difference(point.ZNumerator, reference.Z);
        AddParallelAxis(ref tensor, ScaleWeight(weight), dx, dy, dz);
        AddFirstMoments(ref tensor, mesh, part.CompoundLocalRotation, dx, dy, dz);
    }

    private static Signed320 Difference(Signed320 point, Fixed64 reference) =>
        WideArithmetic.SubtractSigned320(point, Signed320.NarrowValue(
            WideArithmetic.MultiplySigned320(
                WideArithmetic.MultiplySigned192(Signed192.Raw(reference), Signed192.One), Signed192.One)));

    private static Signed576 Weighted(ExactMassWeight weight, Signed192 geometry, int factor = 60) =>
        WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned320(weight.Numerator, geometry), Signed192.Signed(factor));

    private static void GetIntrinsic(LSCollider part, ExactMassWeight weight, Span<Signed576> values)
    {
        switch (part)
        {
            case LSSphereCollider sphere:
                Fixed64 radius = sphere.GetMassPropertyRadius();
                Signed576 spherical = Weighted(weight,
                    ExactMassProperties.InertiaGeometryTerm(Fixed64.FromFraction(2, 5), radius, radius));
                values[0] = values[1] = values[2] = spherical;
                break;
            case LSCuboidCollider box:
                Vector3d extents = box.GetMassPropertyHalfExtents();
                Signed576 x = Weighted(weight, ExactMassProperties.InertiaGeometryTerm(Fixed64.One, extents.X, extents.X), 20);
                Signed576 y = Weighted(weight, ExactMassProperties.InertiaGeometryTerm(Fixed64.One, extents.Y, extents.Y), 20);
                Signed576 z = Weighted(weight, ExactMassProperties.InertiaGeometryTerm(Fixed64.One, extents.Z, extents.Z), 20);
                values[0] = WideArithmetic.AddSigned576(y, z);
                values[1] = WideArithmetic.AddSigned576(x, z);
                values[2] = WideArithmetic.AddSigned576(x, y);
                break;
            case LSCylinderCollider cylinder:
                cylinder.GetInertiaGeometry(out Signed192 cylinderTransverse, out Signed192 cylinderAxial);
                values[0] = values[2] = Weighted(weight, cylinderTransverse);
                values[1] = Weighted(weight, cylinderAxial);
                break;
            case LSConeCollider cone:
                cone.GetInertiaGeometry(out Signed192 coneTransverse, out Signed192 coneAxial);
                values[0] = values[2] = Weighted(weight, coneTransverse);
                values[1] = Weighted(weight, coneAxial);
                break;
            default:
                // Authored compound definitions admit only this closed family.
                GetCapsuleIntrinsic((LSCapsuleCollider)part, weight, values);
                break;
        }
    }

    private static void GetCapsuleIntrinsic(LSCapsuleCollider capsule, ExactMassWeight weight, Span<Signed576> values)
    {
        capsule.GetMassPropertyGeometry(out Fixed64 radius, out Fixed64 axis);
        Signed192 sphereAxial = ExactMassProperties.InertiaGeometryTerm(Fixed64.FromFraction(2, 5), radius, radius);
        if (axis <= Fixed64.Epsilon)
        {
            values[0] = values[1] = values[2] = Weighted(weight, sphereAxial);
            return;
        }
        LSCapsuleCollider.GetComponentInertiaGeometry(radius, axis, sphereAxial,
            out Signed192 cylinderTransverse, out Signed192 sphereTransverse);
        ExactMassWeight cylinderWeight = LSCapsuleCollider.GetCylinderWeight(radius, axis);
        ExactMassWeight capWeight = LSCapsuleCollider.GetCapWeight(radius);
        values[0] = values[2] = WideArithmetic.AddSigned576(
            Weighted(cylinderWeight, cylinderTransverse), Weighted(capWeight, sphereTransverse));
        values[1] = WideArithmetic.AddSigned576(
            Weighted(cylinderWeight, ExactMassProperties.InertiaGeometryTerm(Fixed64.Half, radius, radius)),
            Weighted(capWeight, sphereAxial));
    }

    private static void AddIntrinsic(ref CompoundInertiaTensor tensor, ReadOnlySpan<Signed576> values, FixedQuaternion rotation)
    {
        if (rotation == FixedQuaternion.Identity)
        {
            tensor.Xx = AddPadded(tensor.Xx, values[0]);
            tensor.Yy = AddPadded(tensor.Yy, values[1]);
            tensor.Zz = AddPadded(tensor.Zz, values[2]);
            tensor.Xy = AddPadded(tensor.Xy, values[3]);
            tensor.Xz = AddPadded(tensor.Xz, values[4]);
            tensor.Yz = AddPadded(tensor.Yz, values[5]);
            return;
        }
        Fixed3x3 matrix = rotation.ToMatrix3x3().Transpose();
        Vector3d x = new(matrix.M11, matrix.M12, matrix.M13);
        Vector3d y = new(matrix.M21, matrix.M22, matrix.M23);
        Vector3d z = new(matrix.M31, matrix.M32, matrix.M33);
        tensor.Xx = AddRotated(tensor.Xx, values, x, x);
        tensor.Yy = AddRotated(tensor.Yy, values, y, y);
        tensor.Zz = AddRotated(tensor.Zz, values, z, z);
        tensor.Xy = AddRotated(tensor.Xy, values, x, y);
        tensor.Xz = AddRotated(tensor.Xz, values, x, z);
        tensor.Yz = AddRotated(tensor.Yz, values, y, z);
    }

    private static Signed704 AddPadded(Signed704 total, Signed576 value) =>
        WideArithmetic.AddSigned704(total, Signed704.ExtendValue(
            WideArithmetic.MultiplySigned576(value, Signed192.One, Signed192.One, Signed192.One)));

    private static Signed704 AddRotated(Signed704 total, ReadOnlySpan<Signed576> values, Vector3d row, Vector3d column)
    {
        Signed576 value = WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(RotationTerm(values[0], row.X, column.X), RotationTerm(values[1], row.Y, column.Y)),
            RotationTerm(values[2], row.Z, column.Z));
        value = WideArithmetic.AddSigned576(value, RotationProduct(values[3], row.X, column.Y, row.Y, column.X));
        value = WideArithmetic.AddSigned576(value, RotationProduct(values[4], row.X, column.Z, row.Z, column.X));
        value = WideArithmetic.AddSigned576(value, RotationProduct(values[5], row.Y, column.Z, row.Z, column.Y));
        return WideArithmetic.AddSigned704(total, Signed704.ExtendValue(
            WideArithmetic.MultiplySigned576(value, Signed192.One)));
    }

    private static Signed576 RotationTerm(Signed576 value, Fixed64 first, Fixed64 second) =>
        WideArithmetic.MultiplySigned576(value, Fixed64.GetExactRawProduct(first, second));

    private static Signed576 RotationProduct(Signed576 value,
        Fixed64 first, Fixed64 second, Fixed64 third, Fixed64 fourth) =>
        WideArithmetic.MultiplySigned576(value, WideArithmetic.AddSigned192(
            Fixed64.GetExactRawProduct(first, second), Fixed64.GetExactRawProduct(third, fourth)));

    private static void AddParallelAxis(ref CompoundInertiaTensor tensor, Signed320 weight,
        Signed320 dx, Signed320 dy, Signed320 dz)
    {
        Signed704 x = WideArithmetic.MultiplySigned320(dx, dx, weight);
        Signed704 y = WideArithmetic.MultiplySigned320(dy, dy, weight);
        Signed704 z = WideArithmetic.MultiplySigned320(dz, dz, weight);
        tensor.Xx = WideArithmetic.AddSigned704(tensor.Xx, WideArithmetic.AddSigned704(y, z));
        tensor.Yy = WideArithmetic.AddSigned704(tensor.Yy, WideArithmetic.AddSigned704(x, z));
        tensor.Zz = WideArithmetic.AddSigned704(tensor.Zz, WideArithmetic.AddSigned704(x, y));
        tensor.Xy = WideArithmetic.SubtractSigned704(tensor.Xy, WideArithmetic.MultiplySigned320(dx, dy, weight));
        tensor.Xz = WideArithmetic.SubtractSigned704(tensor.Xz, WideArithmetic.MultiplySigned320(dx, dz, weight));
        tensor.Yz = WideArithmetic.SubtractSigned704(tensor.Yz, WideArithmetic.MultiplySigned320(dy, dz, weight));
    }

    private static void AddFirstMoments(ref CompoundInertiaTensor tensor, MeshMassIntegrals mesh,
        FixedQuaternion rotation, Signed320 dx, Signed320 dy, Signed320 dz)
    {
        if (mesh.FirstX.IsZero && mesh.FirstY.IsZero && mesh.FirstZ.IsZero)
            return;
        Fixed3x3 matrix = rotation.ToMatrix3x3().Transpose();
        Signed576 fx = RotateFirst(mesh, matrix.M11, matrix.M12, matrix.M13);
        Signed576 fy = RotateFirst(mesh, matrix.M21, matrix.M22, matrix.M23);
        Signed576 fz = RotateFirst(mesh, matrix.M31, matrix.M32, matrix.M33);
        // First moments retain the centroid's fractional residual. Omitting
        // these cross terms would make translation depend on a rounded centroid.
        tensor.Xx = AddFirstDiagonal(tensor.Xx, fy, dy, fz, dz);
        tensor.Yy = AddFirstDiagonal(tensor.Yy, fx, dx, fz, dz);
        tensor.Zz = AddFirstDiagonal(tensor.Zz, fx, dx, fy, dy);
        tensor.Xy = SubtractFirstProduct(tensor.Xy, fx, dy, fy, dx);
        tensor.Xz = SubtractFirstProduct(tensor.Xz, fx, dz, fz, dx);
        tensor.Yz = SubtractFirstProduct(tensor.Yz, fy, dz, fz, dy);
    }

    private static Signed576 RotateFirst(MeshMassIntegrals mesh, Fixed64 x, Fixed64 y, Fixed64 z) =>
        WideArithmetic.AddSigned576(WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned576(mesh.FirstX, Signed192.Raw(x)),
            WideArithmetic.MultiplySigned576(mesh.FirstY, Signed192.Raw(y))),
            WideArithmetic.MultiplySigned576(mesh.FirstZ, Signed192.Raw(z)));

    private static Signed576 FirstProduct(Signed576 first, Signed320 displacement) =>
        WideArithmetic.MultiplySigned576(first, Signed192.NarrowValue(displacement));

    private static Signed704 AddFirstDiagonal(Signed704 total,
        Signed576 first, Signed320 firstDisplacement, Signed576 second, Signed320 secondDisplacement)
    {
        Signed576 shift = WideArithmetic.AddSigned576(
            FirstProduct(first, firstDisplacement), FirstProduct(second, secondDisplacement));
        return WideArithmetic.AddSigned704(total, Signed704.ExtendValue(WideArithmetic.AddSigned576(shift, shift)));
    }

    private static Signed704 SubtractFirstProduct(Signed704 total,
        Signed576 first, Signed320 firstDisplacement, Signed576 second, Signed320 secondDisplacement) =>
        WideArithmetic.SubtractSigned704(total, Signed704.ExtendValue(WideArithmetic.AddSigned576(
            FirstProduct(first, firstDisplacement), FirstProduct(second, secondDisplacement))));
}
