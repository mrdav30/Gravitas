//=======================================================================
// SolidBody.Grounding.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using Gravitas.Support;
using SwiftCollections;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace Gravitas;

public partial class SolidBody
{
    #region Grounding

    private bool _skipGroundingCheck = false;

    /// <summary>
    /// Selects whether Gravitas probes for ground automatically or preserves host-supplied grounding state.
    /// </summary>
    public GroundingMode GroundingMode { get; private set; } = GroundingMode.Automatic;

    // how close to the actor's feet (or whatever touches the ground) do we check for grounding
    public Fixed64 GroundOriginOffset = (Fixed64)0.5f;

    public Fixed64 GroundedDistanceRay = (Fixed64)0.5f;

    public Fixed64 GroundDownDistanceOnAir = (Fixed64)0.5f;

    /// <summary>
    /// Selects the deterministic query primitive used for ground checks.
    /// </summary>
    public GroundProbeMode GroundProbeMode { get; set; } = GroundProbeMode.Auto;

    /// <summary>
    /// Optional explicit radius for swept-sphere ground probes. A zero value derives the radius from the collider.
    /// </summary>
    public Fixed64 GroundProbeRadius { get; set; }

    private int _lastGroundCheckFrame = 0;
    private const int _groundCheckFrameThreshold = 10;
    private readonly Fixed64 _groundCheckThreshold = (Fixed64)0.01f;
    private readonly SwiftList<Physics3DHit> _groundProbeHits = new(DefaultBodyHitBufferCapacity);

    public Fixed64 StepOffset = (Fixed64)0.5f;

    private Vector3d _groundNormal = Vector3d.Zero;
    public Vector3d GroundNormal => _groundNormal;

    private FixedTransform? _hitPlatform;
    public FixedTransform? HitPlatform => _hitPlatform;

    private Vector3d _hitPlatformPosition;

    private Vector3d _hitPoint;
    private bool _hasHitPoint;

    /// <summary>
    /// Gets whether the current ground point is representable.
    /// </summary>
    public bool HasHitPoint => _hasHitPoint;

    /// <summary>
    /// Gets the current materialized ground point.
    /// </summary>
    /// <exception cref="InvalidOperationException">
    /// No representable ground point is available.
    /// </exception>
    public Vector3d HitPoint =>
        _hasHitPoint || !_isGrounded
            ? _hitPoint
            : throw new InvalidOperationException(
                "No representable ground point is available. Use TryGetHitPoint.");

    /// <summary>
    /// Attempts to get the current materialized ground point.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetHitPoint(out Vector3d hitPoint)
    {
        hitPoint = _hitPoint;
        return _hasHitPoint;
    }

    public Action<bool>? OnGrounded;

    private bool _isGrounded;
    public bool IsGrounded
    {
        get => _isGrounded;
        private set
        {
            if (_isGrounded == value)
                return;
            _isGrounded = value;
            OnGrounded?.Invoke(value);
        }
    }

    private bool _wasGrounded;
    private bool _groundedTransitionCapturedForStep;

    /// <summary>
    /// Gets whether this body was grounded before the latest authoritative grounding refresh or manual grounding change.
    /// </summary>
    public bool WasGrounded => _wasGrounded;

    private Vector3d _lastGroundedPosition;
    public Vector3d LastGroundedPosition => _lastGroundedPosition;

    #endregion

    public void SkipGrounding(Fixed64 secs)
    {
        _skipGroundingCheck = true;
        CaptureGroundedTransitionState();
        ClearGrounding();
        Context.Coroutines.StartCoroutine(SkipGroundingCoroutine(secs));
    }

    /// <summary>
    /// Gives the host ownership of grounded state and disables automatic ground probes.
    /// </summary>
    public void UseManualGrounding(bool clearGrounding = true)
    {
        GroundingMode = GroundingMode.Manual;
        _skipGroundingCheck = false;
        if (clearGrounding)
        {
            CaptureGroundedTransitionState();
            ClearGrounding();
        }
    }

    /// <summary>
    /// Returns grounded-state ownership to Gravitas and optionally refreshes the automatic probe immediately.
    /// </summary>
    public void UseAutomaticGrounding(bool checkGroundImmediately = true)
    {
        GroundingMode = GroundingMode.Automatic;
        if (checkGroundImmediately && Active)
            CheckGround();
    }

    /// <summary>
    /// Sets host-owned grounded state for manual grounding sources such as deterministic heightmaps.
    /// </summary>
    public void SetManualGrounding(Vector3d hitPoint, Vector3d groundNormal, FixedTransform? hitPlatform = null)
    {
        SwiftThrowHelper.ThrowIfArgument(
            groundNormal.MagnitudeSquared <= Fixed64.Epsilon,
            nameof(groundNormal),
            "Manual grounding normal must be non-zero.");

        GroundingMode = GroundingMode.Manual;
        _skipGroundingCheck = false;
        CaptureGroundedTransitionState();
        SetGroundingState(hitPoint, groundNormal.Normalized, hitPlatform);
    }

    /// <summary>
    /// Clears host-owned grounded state while leaving automatic probes disabled.
    /// </summary>
    public void ClearManualGrounding()
    {
        GroundingMode = GroundingMode.Manual;
        _skipGroundingCheck = false;
        CaptureGroundedTransitionState();
        ClearGrounding();
    }

    private IEnumerator<ILockedYieldInstruction> SkipGroundingCoroutine(Fixed64 secs)
    {
        yield return Context.Coroutines.WaitForRealSeconds(secs);
        _skipGroundingCheck = false;
    }

    public void CheckGround()
    {
        CaptureGroundedTransitionState();
        CheckGround(force: true);
    }

    private void CheckGroundForSimulation() => CheckGround(force: false);

    private void CaptureGroundedTransitionState()
    {
        _wasGrounded = _isGrounded;
        _groundedTransitionCapturedForStep = true;
    }

    private void CaptureGroundedStepState()
    {
        if (!_groundedTransitionCapturedForStep)
            CaptureGroundedTransitionState();
    }

    private void CompleteGroundedStepState() => _groundedTransitionCapturedForStep = false;

    private void CheckGround(bool force)
    {
        if (GroundingMode == GroundingMode.Manual)
            return;

        if (_skipGroundingCheck || World is null)
        {
            ClearGrounding();
            return;
        }

        // Only perform SphereCast if enough frames have passed
        bool hitPlatformMoved = _hitPlatform != null && _hitPlatform.WorldPosition != _hitPlatformPosition;
        bool frameGuard = !force
            && !hitPlatformMoved
            && Vector3d.Distance(_lastPosition, Position3d) < _groundCheckThreshold
            && Context.FrameCount - _lastGroundCheckFrame < _groundCheckFrameThreshold;
        if (frameGuard)
            return;

        _lastGroundCheckFrame = Context.FrameCount;
        // We want origin to be close to the actor's feet
        Vector3d origin = Position3d;
        // but not to close...
        origin.Y += GroundOriginOffset;

        Fixed64 dis = GroundedDistanceRay;
        if (!IsGrounded)
            dis = GroundDownDistanceOnAir;

        GroundProbeMode mode = ResolveGroundProbeMode();
        Fixed64 radius = mode == GroundProbeMode.SweptSphere
            ? ResolveGroundProbeRadius()
            : Fixed64.Zero;
        Vector3d end = origin + Vector3d.Down * dis;
        bool foundGround = TryFindGroundHit(mode, radius, origin, dis, out Physics3DHit hit);
        Context.Diagnostics.EmitGroundProbe(this, mode, origin, end, radius, foundGround, hit);

        if (!foundGround)
        {
            ClearGrounding();
            return;
        }

        SetGroundingState(hit.Anchor, hit.Normal, hit.Collider!.Transform);
    }

    private bool TryFindGroundHit(
        GroundProbeMode mode,
        Fixed64 radius,
        Vector3d origin,
        Fixed64 distance,
        out Physics3DHit hit)
    {
        if (mode == GroundProbeMode.SweptSphere && TryFindGroundHitWithSweptSphere(origin, distance, radius, out hit))
            return true;

        if (mode == GroundProbeMode.SweptSphere)
        {
            hit = default;
            return false;
        }

        return TryFindGroundHitWithRay(origin, distance, out hit);
    }

    private bool TryFindGroundHitWithRay(Vector3d origin, Fixed64 distance, out Physics3DHit hit)
    {
        Vector3d end = origin + Vector3d.Down * distance;
        int hitCount = Context.Query3D.RaycastAll(origin, end, Context.Settings.GroundCheckLayerMask, _groundProbeHits);
        for (int i = 0; i < hitCount; i++)
        {
            Physics3DHit current = _groundProbeHits[i];
            if (!IsValidGroundHit(current))
                continue;

            hit = current;
            return true;
        }

        hit = default;
        return false;
    }

    private bool TryFindGroundHitWithSweptSphere(Vector3d origin, Fixed64 distance, Fixed64 radius, out Physics3DHit hit)
    {
        if (radius <= Fixed64.Epsilon)
            return TryFindGroundHitWithRay(origin, distance, out hit);

        Vector3d end = origin + Vector3d.Down * distance;
        int hitCount = Context.Query3D.SweepSphereAll(
            origin,
            end,
            radius,
            Context.Settings.GroundCheckLayerMask,
            _groundProbeHits,
            Collider);

        for (int i = 0; i < hitCount; i++)
        {
            Physics3DHit current = _groundProbeHits[i];
            if (!IsValidGroundHit(current))
                continue;

            hit = current;
            return true;
        }

        hit = default;
        return false;
    }

    private bool IsValidGroundHit(Physics3DHit hit)
    {
        LSCollider hitCollider = hit.Collider!;
        if (ReferenceEquals(hitCollider, Collider))
            return false;

        if (!ColliderCollisionFilter.AllowsPhysicalPair(Collider, hitCollider))
            return false;

        SolidBody? hitBody = hitCollider.Body;
        return hitCollider.IsStatic || hitBody!.IsKinematic;
    }

    private GroundProbeMode ResolveGroundProbeMode()
    {
        if (GroundProbeMode != GroundProbeMode.Auto)
            return GroundProbeMode;

        return Collider is LSSphereCollider
            || Collider is LSCapsuleCollider
            || Collider is LSCylinderCollider
            || (Collider is LSCuboidCollider && ResolveGroundProbeRadius() > Fixed64.FromFraction(1, 8))
            || (Collider is LSCompoundCollider && ResolveGroundProbeRadius() > Fixed64.FromFraction(1, 8))
                ? GroundProbeMode.SweptSphere
                : GroundProbeMode.Ray;
    }

    private Fixed64 ResolveGroundProbeRadius()
    {
        if (GroundProbeRadius > Fixed64.Zero)
            return GroundProbeRadius;

        return Collider switch
        {
            LSSphereCollider or
            LSCapsuleCollider or
            LSCylinderCollider or
            LSCuboidCollider or
            LSCompoundCollider =>
                Collider.CanonicalGroundProbeRadius,
            _ => Fixed64.Zero
        };
    }

    private void ClearGrounding()
    {
        IsGrounded = false;
        ResetGroundCalculations();
    }

    private void ApplyGroundedHeightOrReset()
    {
        if (_isGrounded)
        {
            if (TryGetHitPoint(out Vector3d hitPoint))
                HeightPos = hitPoint.Y;
            return;
        }

        ResetGroundCalculations();
    }

    private void SetGroundingState(Vector3d hitPoint, Vector3d groundNormal, FixedTransform? hitPlatform)
    {
        _hitPlatform = hitPlatform;
        _hitPlatformPosition = _hitPlatform?.WorldPosition ?? Vector3d.Zero;
        _hitPoint = hitPoint;
        _hasHitPoint = true;
        _groundNormal = groundNormal;

        RefreshGroundNormalForce();
        IsGrounded = true;
    }

    private void SetGroundingState(ContactAnchor anchor, Vector3d groundNormal, FixedTransform hitPlatform)
    {
        _hitPlatform = hitPlatform;
        _hitPlatformPosition = hitPlatform.WorldPosition;
        _hasHitPoint = anchor.TryGetWorldPoint(out _hitPoint);
        _groundNormal = groundNormal;

        RefreshGroundNormalForce();
        IsGrounded = true;
    }

    private void RefreshGroundNormalForce()
    {
        if (!_isGrounded && _groundNormal.MagnitudeSquared <= Fixed64.Epsilon)
        {
            _normalForce = Vector3d.Zero;
            return;
        }

        Vector3d weightVector = Weight * Vector3d.Down;
        Fixed64 weightInNormalDirection = Vector3d.Dot(weightVector, _groundNormal);
        _normalForce = weightInNormalDirection * _groundNormal;
    }

    private void ResetGroundCalculations()
    {
        _hitPlatform = null;
        _hitPlatformPosition = Vector3d.Zero;
        _hitPoint = Vector3d.Zero;
        _hasHitPoint = false;
        _groundNormal = Vector3d.Zero;
        _normalForce = Vector3d.Zero;
    }

}
