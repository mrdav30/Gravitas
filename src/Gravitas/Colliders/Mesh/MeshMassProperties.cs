//=======================================================================
// MeshMassProperties.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using static Gravitas.Colliders.MeshCheckedMath;

namespace Gravitas.Colliders;

/// <summary>
/// Stores deterministic closed-volume mass properties for an immutable mesh topology.
/// </summary>
public readonly struct MeshMassProperties
{
    /// <summary>Creates deterministic closed-volume mass properties.</summary>
    public MeshMassProperties(
        Fixed64 volume,
        Vector3d centerOfMass,
        Vector3d inertiaReferencePoint,
        Fixed3x3 unitMassInertiaTensor)
    {
        Volume = volume;
        CenterOfMass = centerOfMass;
        InertiaReferencePoint = inertiaReferencePoint;
        UnitMassInertiaTensor = unitMassInertiaTensor;
    }

    /// <summary>
    /// Absolute solid volume in local mesh units.
    /// </summary>
    public Fixed64 Volume { get; }

    /// <summary>
    /// Homogeneous center of mass in local mesh coordinates.
    /// </summary>
    public Vector3d CenterOfMass { get; }

    /// <summary>
    /// Local reference point about which <see cref="UnitMassInertiaTensor"/> was computed.
    /// </summary>
    public Vector3d InertiaReferencePoint { get; }

    /// <summary>
    /// Solid-volume inertia tensor for unit mass about <see cref="InertiaReferencePoint"/>.
    /// </summary>
    public Fixed3x3 UnitMassInertiaTensor { get; }

    /// <summary>
    /// Calculates an inertia tensor for the supplied mass about a local point
    /// parallel to the mesh center-of-mass axes.
    /// </summary>
    public Fixed3x3 CalculateInertiaTensor(Fixed64 mass, Vector3d localReferencePoint)
    {
        Fixed3x3 referenceTensor = UnitMassInertiaTensor * mass;
        Fixed3x3 centerTensor = InertiaTensorMath.SubtractParallelAxisTensor(
            referenceTensor,
            mass,
            InertiaReferencePoint - CenterOfMass);
        return InertiaTensorMath.AddParallelAxisTensor(centerTensor, mass, localReferencePoint - CenterOfMass);
    }

    internal MeshMassScaleResult TryScale(Vector3d scale, out MeshMassProperties properties)
    {
        properties = default;
        bool valid = TryMultiply(Volume, scale.X, out Fixed64 scaledVolume);
        valid &= TryMultiply(scaledVolume, scale.Y, out scaledVolume);
        valid &= TryMultiply(scaledVolume, scale.Z, out scaledVolume);
        if (!valid)
        {
            return MeshMassScaleResult.NonRepresentableVolume;
        }

        valid = TrySubtract(InertiaReferencePoint, CenterOfMass, out Vector3d referenceOffset);
        valid &= TryCreateParallelAxisTensor(referenceOffset, out Fixed3x3 referenceShift);
        valid &= TrySubtract(UnitMassInertiaTensor, referenceShift, out Fixed3x3 centerTensor);
        valid &= TryRecoverCovariance(centerTensor, out Fixed3x3 covariance);
        valid &= TryScaleCovariance(covariance, scale, out Fixed3x3 scaledCovariance);
        valid &= TryCreateInertiaFromCovariance(scaledCovariance, out Fixed3x3 scaledCenterTensor);
        valid &= TryMultiply(CenterOfMass, scale, out Vector3d scaledCenter);
        valid &= TryMultiply(InertiaReferencePoint, scale, out Vector3d scaledReference);
        valid &= TrySubtract(scaledReference, scaledCenter, out Vector3d scaledReferenceOffset);
        valid &= TryCreateParallelAxisTensor(scaledReferenceOffset, out Fixed3x3 scaledReferenceShift);
        valid &= TryAdd(scaledCenterTensor, scaledReferenceShift, out Fixed3x3 scaledReferenceTensor);
        if (!valid)
        {
            return MeshMassScaleResult.NonRepresentableMassProperties;
        }

        properties = new MeshMassProperties(
            scaledVolume,
            scaledCenter,
            scaledReference,
            scaledReferenceTensor);
        return MeshMassScaleResult.Valid;
    }

    private static bool TryRecoverCovariance(Fixed3x3 tensor, out Fixed3x3 covariance)
    {
        covariance = default;
        bool valid = TryAdd(tensor.M22, tensor.M33, out Fixed64 xSum);
        valid &= TrySubtract(xSum, tensor.M11, out Fixed64 xTwice);
        valid &= TryMultiply(xTwice, Fixed64.Half, out Fixed64 x);
        valid &= TryAdd(tensor.M11, tensor.M33, out Fixed64 ySum);
        valid &= TrySubtract(ySum, tensor.M22, out Fixed64 yTwice);
        valid &= TryMultiply(yTwice, Fixed64.Half, out Fixed64 y);
        valid &= TryAdd(tensor.M11, tensor.M22, out Fixed64 zSum);
        valid &= TrySubtract(zSum, tensor.M33, out Fixed64 zTwice);
        valid &= TryMultiply(zTwice, Fixed64.Half, out Fixed64 z);
        valid &= TryNegate(tensor.M12, out Fixed64 xy);
        valid &= TryNegate(tensor.M13, out Fixed64 xz);
        valid &= TryNegate(tensor.M23, out Fixed64 yz);
        if (!valid)
        {
            return false;
        }

        covariance = new Fixed3x3(x, xy, xz, xy, y, yz, xz, yz, z);
        return true;
    }

    private static bool TryScaleCovariance(Fixed3x3 covariance, Vector3d scale, out Fixed3x3 scaled)
    {
        scaled = default;
        bool valid = TryMultiply(scale.X, scale.X, out Fixed64 xxScale);
        valid &= TryMultiply(scale.Y, scale.Y, out Fixed64 yyScale);
        valid &= TryMultiply(scale.Z, scale.Z, out Fixed64 zzScale);
        valid &= TryMultiply(scale.X, scale.Y, out Fixed64 xyScale);
        valid &= TryMultiply(scale.X, scale.Z, out Fixed64 xzScale);
        valid &= TryMultiply(scale.Y, scale.Z, out Fixed64 yzScale);
        valid &= TryMultiply(covariance.M11, xxScale, out Fixed64 xx);
        valid &= TryMultiply(covariance.M22, yyScale, out Fixed64 yy);
        valid &= TryMultiply(covariance.M33, zzScale, out Fixed64 zz);
        valid &= TryMultiply(covariance.M12, xyScale, out Fixed64 xy);
        valid &= TryMultiply(covariance.M13, xzScale, out Fixed64 xz);
        valid &= TryMultiply(covariance.M23, yzScale, out Fixed64 yz);
        if (!valid)
        {
            return false;
        }

        scaled = new Fixed3x3(xx, xy, xz, xy, yy, yz, xz, yz, zz);
        return true;
    }

    private static bool TryCreateInertiaFromCovariance(Fixed3x3 covariance, out Fixed3x3 tensor)
    {
        tensor = default;
        bool valid = TryAdd(covariance.M22, covariance.M33, out Fixed64 xx);
        valid &= TryAdd(covariance.M11, covariance.M33, out Fixed64 yy);
        valid &= TryAdd(covariance.M11, covariance.M22, out Fixed64 zz);
        valid &= TryNegate(covariance.M12, out Fixed64 xy);
        valid &= TryNegate(covariance.M13, out Fixed64 xz);
        valid &= TryNegate(covariance.M23, out Fixed64 yz);
        if (!valid)
        {
            return false;
        }

        tensor = new Fixed3x3(xx, xy, xz, xy, yy, yz, xz, yz, zz);
        return true;
    }

}

internal enum MeshMassScaleResult
{
    Valid,
    NonRepresentableVolume,
    NonRepresentableMassProperties
}
