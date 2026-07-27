//=======================================================================
// LSCollider.MassProperties.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

public abstract partial class LSCollider
{
    // Default to total area for shapes where frontal area doesn't make sense.
    public virtual Fixed64 GetFrontalArea(Vector3d direction) => Area;

    /// <summary>
    /// Calculates the body-local center of mass offset implied by this collider's current shape state.
    /// </summary>
    public virtual Vector3d CalculateLocalCenterOfMassOffset()
    {
        if (!_hasCommittedShape)
            return TransformRelativeMassPropertyPoint(Vector3d.Zero);

        SwiftThrowHelper.ThrowIfTrue(
            !_hasCommittedDefaultCenterOfMassOffset,
            nameof(CalculateLocalCenterOfMassOffset),
            "The collider's body-local center of mass is outside the Fixed64 coordinate domain.");
        return _defaultCenterOfMassOffset;
    }

    /// <summary>
    /// Calculates the deterministic relative measure used to distribute mass
    /// when this shape is owned by a compound collider.
    /// </summary>
    /// <remarks>
    /// Solid shapes return volume. Explicit surface-approximation shapes may
    /// return a documented shell measure instead.
    /// </remarks>
    protected internal abstract Fixed64 CalculateMassPropertyWeight();

    public abstract Fixed3x3 CalculateInertiaTensor(
        Fixed64 mass,
        Vector3d localCenterOfMassOffset);

    protected Fixed3x3 ShiftInertiaTensorFromLocalCenterOfMass(
        Fixed3x3 centerTensor,
        Fixed64 mass,
        Vector3d targetLocalOffset) =>
        AddParallelAxisTensor(
            centerTensor,
            mass,
            targetLocalOffset - CalculateLocalCenterOfMassOffset());

    protected static Fixed3x3 AddParallelAxisTensor(
        Fixed3x3 tensor,
        Fixed64 mass,
        Vector3d offset) =>
        InertiaTensorMath.AddParallelAxisTensor(tensor, mass, offset);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    protected Vector3d TransformRelativeMassPropertyPoint(
        Vector3d partRelativePoint)
    {
        bool representable = _compoundOwner == null
            ? Vector3d.TryComposeScaledLocalPoints(
                LocalOffset,
                GetCurrentOwnerScale(),
                Vector3d.Zero,
                Vector3d.One,
                partRelativePoint,
                FixedQuaternion.Identity,
                out Vector3d transformed)
            : Vector3d.TryComposeScaledLocalPoints(
                _compoundOwner.LocalOffset,
                GetCurrentOwnerScale(),
                LocalOffset,
                GetCurrentOwnerScale(),
                partRelativePoint,
                _compoundLocalRotation,
                out transformed);
        if (representable)
            return transformed;

        throw new System.InvalidOperationException(
            "Collider mass-property point is outside the Fixed64 coordinate domain.");
    }

    internal FixedQuaternion CompoundLocalRotation => _compoundLocalRotation;
}
