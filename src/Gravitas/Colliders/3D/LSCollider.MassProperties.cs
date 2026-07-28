//=======================================================================
// LSCollider.MassProperties.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
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
        FixedMassPoint point = CalculateLocalMassPoint();
        SwiftThrowHelper.ThrowIfTrue(
            !point.TryGetPoint(out Vector3d center),
            nameof(CalculateLocalCenterOfMassOffset),
            "The collider's body-local center of mass is outside the Fixed64 coordinate domain.");
        return center;
    }

    internal virtual FixedMassPoint CalculateLocalMassPoint() =>
        TransformRelativeMassPropertyPointExact(Vector3d.Zero);

    internal virtual FixedMassPoint CalculatePreparedLocalMassPoint() =>
        TransformPreparedRelativeMassPropertyPointExact(Vector3d.Zero);

    /// <summary>
    /// Calculates the deterministic relative measure used to distribute mass
    /// when this shape is owned by a compound collider.
    /// </summary>
    /// <remarks>
    /// Solid shapes return volume. Explicit surface-approximation shapes may
    /// return a documented shell measure instead.
    /// </remarks>
    protected internal abstract FixedMassWeight CalculateMassPropertyWeight();

    internal abstract FixedMassWeight CalculatePreparedMassPropertyWeight();

    internal virtual bool SupportsMassProperties => true;

    public virtual Fixed3x3 CalculateInertiaTensor(
        Fixed64 mass,
        Vector3d localCenterOfMassOffset)
    {
        if (mass <= Fixed64.Zero)
            return Fixed3x3.Zero;

        Fixed3x3 centerTensor =
            CalculateCenterOfMassInertiaTensor(mass);
        FixedMassPoint massPoint = CalculateLocalMassPoint();
        if (massPoint.TryAddParallelAxisTensor(
                centerTensor,
                mass,
                localCenterOfMassOffset,
                out Fixed3x3 tensor))
        {
            return tensor;
        }

        // Explicit body center overrides historically use the collider's
        // saturating public tensor contract. Semantic child centers have no
        // scalar fallback and must remain checked.
        if (!massPoint.TryGetPoint(out Vector3d center))
        {
            SwiftThrowHelper.ThrowIfTrue(
                true,
                nameof(localCenterOfMassOffset),
                "The requested inertia tensor is outside the Fixed64 scalar domain.");
        }

        return InertiaTensorMath.AddParallelAxisTensor(
            centerTensor,
            mass,
            localCenterOfMassOffset - center);
    }

    internal abstract Fixed3x3 CalculateCenterOfMassInertiaTensor(
        Fixed64 mass);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    protected FixedMassPoint TransformRelativeMassPropertyPointExact(
        Vector3d partRelativePoint) =>
        _compoundOwner == null
            ? FixedMassPoint.CreateScaledLocalComposition(
                LocalOffset,
                GetCurrentOwnerScale(),
                Vector3d.Zero,
                Vector3d.One,
                partRelativePoint,
                FixedQuaternion.Identity)
            : FixedMassPoint.CreateScaledLocalComposition(
                _compoundOwner.LocalOffset,
                GetCurrentOwnerScale(),
                LocalOffset,
                GetCurrentOwnerScale(),
                partRelativePoint,
                _compoundLocalRotation);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    protected FixedMassPoint TransformPreparedRelativeMassPropertyPointExact(
        Vector3d partRelativePoint) =>
        _compoundOwner == null
            ? FixedMassPoint.CreateScaledLocalComposition(
                _preparedSnapshot.LocalOffset,
                _preparedSnapshot.OwnerScale,
                Vector3d.Zero,
                Vector3d.One,
                partRelativePoint,
                FixedQuaternion.Identity)
            : FixedMassPoint.CreateScaledLocalComposition(
                _compoundOwner._preparedSnapshot.LocalOffset,
                _preparedSnapshot.OwnerScale,
                _preparedSnapshot.LocalOffset,
                _preparedSnapshot.OwnerScale,
                partRelativePoint,
                _preparedCompoundLocalRotation);

    internal FixedQuaternion CompoundLocalRotation => _compoundLocalRotation;
}
