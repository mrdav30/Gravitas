//=======================================================================
// SolidBody.CenterOfMass.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.CollisionHandling;
using SwiftCollections.Diagnostics;
using System;

namespace Gravitas;

public partial class SolidBody
{
    private Vector3d _localCenterOfMassOffset;
    private bool _centerOfMassOffsetExplicit;

    /// <summary>
    /// Gets or sets the authoritative body-local center-of-mass offset used by response and inertia.
    /// </summary>
    public Vector3d LocalCenterOfMassOffset
    {
        get => _localCenterOfMassOffset;
        set
        {
            if (_localCenterOfMassOffset == value && _centerOfMassOffsetExplicit)
                return;

            _localCenterOfMassOffset = value;
            _centerOfMassOffsetExplicit = true;
            if (!Active)
                return;

            Wake();
            RefreshInertiaTensor();
        }
    }

    /// <summary>
    /// Gets the authoritative world-space center of mass.
    /// </summary>
    /// <exception cref="InvalidOperationException">
    /// The center lies outside the representable coordinate domain. Use
    /// <see cref="TryGetWorldCenterOfMass(out Vector3d)"/> when that is possible.
    /// </exception>
    public Vector3d WorldCenterOfMass
    {
        get
        {
            bool succeeded = TryGetWorldCenterOfMass(out Vector3d center);
            SwiftThrowHelper.ThrowIfTrue(
                !succeeded,
                nameof(WorldCenterOfMass),
                "The world center of mass is outside the representable coordinate domain.");
            return center;
        }
    }

    /// <summary>
    /// Attempts to materialize the authoritative world-space center of mass
    /// without saturation.
    /// </summary>
    public bool TryGetWorldCenterOfMass(out Vector3d center)
        => Rotation.TryTransformPoint(
            Position3d,
            _localCenterOfMassOffset,
            out center);

    internal bool TryGetOffsetFromCenterOfMass(
        ContactAnchor anchor,
        out Vector3d offset)
        => anchor.TryGetOffsetFrom(
            new ContactAnchor(
                Position3d,
                Rotation,
                _localCenterOfMassOffset),
            out offset);

    /// <summary>
    /// Clears an explicit center-of-mass override and derives the offset from the bound collider again.
    /// </summary>
    public void ResetCenterOfMassFromCollider()
    {
        _centerOfMassOffsetExplicit = false;
        RefreshMassPropertiesFromColliderShape();
    }
}
