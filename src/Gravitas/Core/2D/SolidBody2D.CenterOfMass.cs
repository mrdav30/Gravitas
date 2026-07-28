//=======================================================================
// SolidBody2D.CenterOfMass.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.CollisionHandling;
using SwiftCollections.Diagnostics;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas;

public sealed partial class SolidBody2D
{
    /// <summary>
    /// Gets or sets the authoritative body-local center-of-mass offset in the X/Z simulation plane.
    /// </summary>
    public Vector2d LocalCenterOfMassOffset
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
            RefreshMassPropertiesFromColliderShape();
        }
    }

    /// <summary>
    /// Gets the authoritative world-space center of mass in the X/Z simulation plane.
    /// </summary>
    /// <exception cref="InvalidOperationException">
    /// The center lies outside the representable coordinate domain. Use
    /// <see cref="TryGetWorldCenterOfMass(out Vector2d)"/> when that is possible.
    /// </exception>
    public Vector2d WorldCenterOfMass
    {
        get
        {
            bool succeeded = TryGetWorldCenterOfMass(out Vector2d center);
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
    public bool TryGetWorldCenterOfMass(out Vector2d center)
        => Vector2d.TryTransformPoint(
            _position,
            _localCenterOfMassOffset,
            _rotation,
            out center);

    internal bool TryGetOffsetFromCenterOfMass(
        ContactAnchor2D anchor,
        out Vector2d offset)
        => anchor.TryGetOffsetFrom(GetCenterOfMassAnchor(), out offset);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal ContactAnchor2D GetCenterOfMassAnchor() =>
        new(
            _position,
            _rotation,
            _localCenterOfMassOffset);

    /// <summary>
    /// Clears an explicit center-of-mass override and derives the offset from the bound collider again.
    /// </summary>
    public void ResetCenterOfMassFromCollider()
    {
        _centerOfMassOffsetExplicit = false;
        RefreshMassPropertiesFromColliderShape();
    }
}
