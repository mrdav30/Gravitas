//=======================================================================
// MeshRuntimeInertia.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System.Runtime.CompilerServices;
using EnclosedCoefficient = FixedMathSharp.EnclosedQ96Coefficient;

namespace Gravitas.Colliders;

/// <summary>Retains mesh geometry coefficients and certifies their final mass-applied rounding.</summary>
internal sealed class MeshRuntimeInertia
{
    private EnclosedCoefficient _xx, _yy, _zz, _xy, _xz, _yz;
    private Signed192 _firstX, _firstY, _firstZ;
    private Vector3d _reference;
    private bool _firstZero, _hasResult;
    private Fixed64 _lastMass;
    private Vector3d _lastReference;
    private Fixed3x3 _lastTensor;

    internal bool Ready;
    internal Vector3d Center;

    internal void Build(in MeshMassIntegrals integrals, Vector3d center)
    {
        Signed320 denominator = CompoundInertiaMath3D.ScaleWeight(integrals.Weight);
        _xx = new EnclosedCoefficient(WideArithmetic.DivideSigned576ToSigned192(integrals.Ixx, denominator), 9);
        _yy = new EnclosedCoefficient(WideArithmetic.DivideSigned576ToSigned192(integrals.Iyy, denominator), 9);
        _zz = new EnclosedCoefficient(WideArithmetic.DivideSigned576ToSigned192(integrals.Izz, denominator), 9);
        _xy = new EnclosedCoefficient(WideArithmetic.DivideSigned576ToSigned192(integrals.Ixy, denominator), 9);
        _xz = new EnclosedCoefficient(WideArithmetic.DivideSigned576ToSigned192(integrals.Ixz, denominator), 9);
        _yz = new EnclosedCoefficient(WideArithmetic.DivideSigned576ToSigned192(integrals.Iyz, denominator), 9);
        _firstX = WideArithmetic.DivideSigned576ToSigned192(
            WideArithmetic.MultiplySigned576(integrals.FirstX, Signed192.One, Signed192.One), denominator);
        _firstY = WideArithmetic.DivideSigned576ToSigned192(
            WideArithmetic.MultiplySigned576(integrals.FirstY, Signed192.One, Signed192.One), denominator);
        _firstZ = WideArithmetic.DivideSigned576ToSigned192(
            WideArithmetic.MultiplySigned576(integrals.FirstZ, Signed192.One, Signed192.One), denominator);
        _firstZero = integrals.FirstX.IsZero && integrals.FirstY.IsZero && integrals.FirstZ.IsZero;
        _reference = integrals.Reference;
        Center = center;
        _hasResult = false;
        Ready = true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal Fixed3x3 Calculate(Fixed64 mass, Vector3d reference, in MeshMassIntegrals integrals)
    {
        if (_hasResult && mass == _lastMass && reference == _lastReference)
            return _lastTensor;
        Fixed3x3 tensor = CalculateUncached(mass, reference, integrals);
        _lastMass = mass;
        _lastReference = reference;
        _lastTensor = tensor;
        _hasResult = true;
        return tensor;
    }

    private Fixed3x3 CalculateUncached(Fixed64 mass, Vector3d reference, in MeshMassIntegrals integrals)
    {
        // Keep reference translation's wide temporaries off changing-mass calls.
        if (reference != _reference) return CalculateTranslated(mass, reference, integrals);
        return TryMaterialize(mass, _xx, _yy, _zz, _xy, _xz, _yz, out Fixed3x3 tensor)
            ? tensor : CalculateExact(mass, reference, integrals);
    }

    private Fixed3x3 CalculateTranslated(Fixed64 mass, Vector3d reference, in MeshMassIntegrals integrals)
    {
        Signed192 xx = _xx.Value, yy = _yy.Value, zz = _zz.Value, xy = _xy.Value, xz = _xz.Value, yz = _yz.Value;
        if (!Vector3d.TrySubtract(_reference, reference, out Vector3d delta))
            return CalculateExact(mass, reference, integrals);
        Signed192 dx = Signed192.Raw(delta.X), dy = Signed192.Raw(delta.Y), dz = Signed192.Raw(delta.Z);
        Signed192 x2 = Parallel(delta.X, delta.X), y2 = Parallel(delta.Y, delta.Y), z2 = Parallel(delta.Z, delta.Z);
        xx = WideArithmetic.AddSigned192(xx, WideArithmetic.AddSigned192(y2, z2));
        yy = WideArithmetic.AddSigned192(yy, WideArithmetic.AddSigned192(x2, z2));
        zz = WideArithmetic.AddSigned192(zz, WideArithmetic.AddSigned192(x2, y2));
        xy = WideArithmetic.SubtractSigned192(xy, Parallel(delta.X, delta.Y));
        xz = WideArithmetic.SubtractSigned192(xz, Parallel(delta.X, delta.Z));
        yz = WideArithmetic.SubtractSigned192(yz, Parallel(delta.Y, delta.Z));
        if (!_firstZero)
        {
            xx = WideArithmetic.AddSigned192(xx, Twice(First(dy, _firstY), First(dz, _firstZ)));
            yy = WideArithmetic.AddSigned192(yy, Twice(First(dx, _firstX), First(dz, _firstZ)));
            zz = WideArithmetic.AddSigned192(zz, Twice(First(dx, _firstX), First(dy, _firstY)));
            xy = WideArithmetic.SubtractSigned192(xy, WideArithmetic.AddSigned192(First(dx, _firstY), First(dy, _firstX)));
            xz = WideArithmetic.SubtractSigned192(xz, WideArithmetic.AddSigned192(First(dx, _firstZ), First(dz, _firstX)));
            yz = WideArithmetic.SubtractSigned192(yz, WideArithmetic.AddSigned192(First(dy, _firstZ), First(dz, _firstY)));
        }
        return TryMaterialize(mass, new EnclosedCoefficient(xx, 9), new EnclosedCoefficient(yy, 9),
            new EnclosedCoefficient(zz, 9), new EnclosedCoefficient(xy, 9), new EnclosedCoefficient(xz, 9),
            new EnclosedCoefficient(yz, 9), out Fixed3x3 tensor)
            ? tensor : CalculateExact(mass, reference, integrals);
    }

    private static bool TryMaterialize(Fixed64 mass, in EnclosedCoefficient xx, in EnclosedCoefficient yy, in EnclosedCoefficient zz,
        in EnclosedCoefficient xy, in EnclosedCoefficient xz, in EnclosedCoefficient yz, out Fixed3x3 tensor)
    {
        // Intrinsic Q96 projection loses <1 unit. Each translated Q128 first
        // moment loses <2 Q96 units, so diagonal error is <1+2*(2+2)=9.
        // Parallel terms are exact. Coefficients fit 165 bits; mass fits 63.
        bool xFits = xx.TryMultiplyRaw(mass, out long x);
        bool yFits;
        long y;
        if (xx.Equals(yy)) { y = x; yFits = xFits; }
        else yFits = yy.TryMultiplyRaw(mass, out y);
        bool zFits;
        long z;
        if (xx.Equals(zz)) { z = x; zFits = xFits; }
        else zFits = zz.TryMultiplyRaw(mass, out z);
        bool fits = xFits & yFits & zFits
            & xy.TryMultiplyRaw(mass, out long a)
            & xz.TryMultiplyRaw(mass, out long b)
            & yz.TryMultiplyRaw(mass, out long c);
        if (fits)
        {
            tensor = new Fixed3x3(
                Fixed64.FromRaw(x), Fixed64.FromRaw(a), Fixed64.FromRaw(b),
                Fixed64.FromRaw(a), Fixed64.FromRaw(y), Fixed64.FromRaw(c),
                Fixed64.FromRaw(b), Fixed64.FromRaw(c), Fixed64.FromRaw(z));
            return true;
        }
        tensor = default;
        return false;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static Fixed3x3 CalculateExact(Fixed64 mass, Vector3d reference, in MeshMassIntegrals integrals)
    {
        // A half tie or unsupported certificate width still uses every retained bit.
        CompoundInertiaTensor exact = default;
        CompoundInertiaMath3D.AddMesh(ref exact, integrals, FixedQuaternion.Identity,
            ExactMassPoint3D.FromPoint(integrals.Reference), reference);
        return exact.MaterializeSaturated(mass, integrals.Weight);
    }

    private static Signed192 Parallel(Fixed64 first, Fixed64 second) =>
        WideArithmetic.ShiftLeftSigned192By32(Fixed64.GetExactRawProduct(first, second));

    private static Signed192 First(Signed192 displacement, Signed192 firstMoment) =>
        WideArithmetic.TruncateSigned320By64(WideArithmetic.MultiplySigned192(displacement, firstMoment));

    private static Signed192 Twice(Signed192 first, Signed192 second)
    {
        Signed192 sum = WideArithmetic.AddSigned192(first, second);
        return WideArithmetic.AddSigned192(sum, sum);
    }
}
