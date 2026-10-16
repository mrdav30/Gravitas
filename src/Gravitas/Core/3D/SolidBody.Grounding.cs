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
    /// <summary>Vertical offset applied to the origin of automatic ground probes.</summary>
    public Fixed64 GroundOriginOffset = (Fixed64)0.5f;

    /// <summary>Automatic ground-probe distance used while grounded.</summary>
    public Fixed64 GroundedDistanceRay = (Fixed64)0.5f;

    /// <summary>Automatic ground-probe distance used while airborne.</summary>
    public Fixed64 GroundDownDistanceOnAir = (Fixed64)0.5f;

    /// <summary>
    /// Selects the deterministic query primitive used for ground checks.
    /// </summary>
    public GroundProbeMode GroundProbeMode
    {
        get => _groundProbeMode;
        set
        {
            if (_groundProbeMode == value) return;
            _groundProbeMode = value;
            if (GroundingMode == GroundingMode.Automatic) _lastGroundCheckFrame = -1;
        }
    }

    private GroundProbeMode _groundProbeMode = GroundProbeMode.Auto;

    /// <summary>
    /// Optional explicit radius for swept-sphere ground probes. Nonpositive values derive it from the collider.
    /// </summary>
    public Fixed64 GroundProbeRadius
    {
        get => _groundProbeRadius;
        set
        {
            SwiftThrowHelper.ThrowIfArgument(value > ColliderGeometryPolicy.MaximumGeometryReach,
                nameof(value), "Ground probe radius must not exceed 4,096 world units.");
            if (_groundProbeRadius == value) return;
            _groundProbeRadius = value;
            if (GroundingMode == GroundingMode.Automatic) _lastGroundCheckFrame = -1;
        }
    }

    private Fixed64 _groundProbeRadius;

    private Fixed64 _groundMinNormalDot = Fixed64.Half;

    /// <summary>
    /// Minimum normalized support-normal dot product with world up, in [0, 1].
    /// Defaults to one half (slopes through 60 degrees). Zero permits any
    /// upward-facing normal; horizontal, downward-facing and zero normals never support.
    /// </summary>
    public Fixed64 GroundMinNormalDot
    {
        get => _groundMinNormalDot;
        set
        {
            ValidateGroundMinNormalDot(value);
            if (_groundMinNormalDot == value)
                return;

            _groundMinNormalDot = value;
            if (GroundingMode == GroundingMode.Automatic)
            {
                // Cached support and sleeping motion were accepted under the old
                // slope policy. Reconsider them on the next authoritative step.
                _lastGroundCheckFrame = -1;
                Wake();
            }
        }
    }

    private long _lastGroundCheckFrame = -1;
    private const int _groundCheckFrameThreshold = 10;
    private readonly Fixed64 _groundCheckThreshold = (Fixed64)0.01f;
    private readonly SwiftList<Physics3DHit> _groundProbeHits = new(DefaultBodyHitBufferCapacity);

    /// <summary>Minimum downward snap distance used to preserve grounding across steps.</summary>
    public Fixed64 StepOffset = (Fixed64)0.5f;

    private Vector3d _groundNormal = Vector3d.Zero;
    /// <summary>Gets the world-space normal of the current supporting surface.</summary>
    public Vector3d GroundNormal => _groundNormal;

    private FixedTransform? _hitPlatform;
    private ColliderLifetimeToken _groundSupport;
    private long _groundingStateVersion;
    private uint _groundSupportVersion;
    private uint _groundProbeShapeVersion;
    private Fixed64 _groundSupportOriginOffset;
    private Fixed64 _groundSupportProbeDistance;
    /// <summary>Gets the transform of the current supporting collider, if available.</summary>
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

    /// <summary>Raised when the grounded state changes.</summary>
    public Action<bool>? OnGrounded;

    private bool _isGrounded;
    /// <summary>Gets whether the body currently has valid ground support.</summary>
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
    /// <summary>Gets the body position captured before its latest grounded movement.</summary>
    public Vector3d LastGroundedPosition => _lastGroundedPosition;

    #endregion

    /// <summary>Temporarily disables automatic ground probes and clears grounded state.</summary>
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

    /// <summary>Immediately refreshes automatic grounded state.</summary>
    public void CheckGround()
    {
        CaptureGroundedTransitionState();
        CheckGround(force: true);
    }

    private static void ValidateGroundMinNormalDot(Fixed64 value) =>
        SwiftThrowHelper.ThrowIfArgument(value < Fixed64.Zero || value > Fixed64.One,
            nameof(value), "Ground support normal threshold must be between zero and one.");

    private void CheckGroundForSimulation() => CheckGround(force: false);

    private bool HasCurrentGroundSupport => _isGrounded
        && (GroundingMode == GroundingMode.Manual || _lastGroundCheckFrame >= 0);

    internal void WakeForContactChange()
    {
        if (!IsSleeping)
            return;
        if (GroundingMode == GroundingMode.Automatic)
            _lastGroundCheckFrame = -1;
        WakeFromCollision();
    }

    private bool RefreshAutomaticGroundSupport()
    {
        if (GroundingMode != GroundingMode.Automatic || !_isGrounded)
            return true;

        if (_lastGroundCheckFrame >= 0 && HasCurrentGroundSupportDependency())
            return true;

        var registration = new ColliderLifetimeToken(Collider);
        long lastGroundCheckFrame = _lastGroundCheckFrame;
        CaptureGroundedStepState();
        CheckGround(force: true);
        if (!registration.IsActive)
            return false;

        // Reconstructing a derived cache must not change an unchanged sleeper's saved probe clock.
        if (lastGroundCheckFrame >= 0 && IsSleeping && GroundingMode == GroundingMode.Automatic)
            _lastGroundCheckFrame = lastGroundCheckFrame;
        return true;
    }

    private bool HasCurrentGroundSupportDependency() =>
        _groundSupport.Collider != null && _groundSupport.IsActive
        && _groundSupportVersion == _groundSupport.Collider.ContactDependencyVersion
        && _groundProbeShapeVersion == Collider.ContactDependencyVersion
        && _groundSupportOriginOffset == GroundOriginOffset
        && _groundSupportProbeDistance == GroundedDistanceRay
        && IsValidGroundCollider(_groundSupport.Collider);

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
            && _lastGroundCheckFrame >= 0
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

        var registration = new ColliderLifetimeToken(Collider);
        _groundSupport = new ColliderLifetimeToken(hit.Collider!);
        _groundSupportVersion = hit.Collider!.ContactDependencyVersion;
        _groundProbeShapeVersion = Collider.ContactDependencyVersion;
        _groundSupportOriginOffset = GroundOriginOffset;
        _groundSupportProbeDistance = GroundedDistanceRay;
        long groundingStateVersion = SetGroundingState(hit.Anchor, hit.Normal, hit.Collider.Transform);
        // Callbacks may replace this body, its support, or automatic grounding ownership.
        if (registration.IsCurrentLifetime && _groundingStateVersion == groundingStateVersion
            && GroundingMode == GroundingMode.Automatic
            && (!registration.IsActive || _lastGroundCheckFrame < 0 || !HasCurrentGroundSupportDependency()))
            ClearGrounding();
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

        if (!IsValidGroundCollider(hitCollider))
            return false;

        // A sweep can start tangent to a wall or inside geometry. Its valid
        // intersection witness is not necessarily upward support. Reject those
        // normals before normalization, even with the most permissive slope limit.
        return hit.Normal.Y > Fixed64.Zero
            && hit.Normal.Normalized.Y >= GroundMinNormalDot;
    }

    private bool IsValidGroundCollider(LSCollider collider) =>
        !collider.IsTrigger
        && Context.Settings.GroundCheckLayerMask.Includes(collider.Layer)
        && ColliderCollisionFilter.AllowsPhysicalPair(Collider, collider)
        && (collider.IsStatic || collider.Body!.IsKinematic);

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
        _groundingStateVersion++;
        if (GroundingMode == GroundingMode.Automatic && _isGrounded)
            WakeFromCollision();
        ResetGroundCalculations();
        IsGrounded = false;
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
        _groundingStateVersion++;
        _groundSupport = default;
        _groundSupportVersion = 0;
        _hitPlatform = hitPlatform;
        _hitPlatformPosition = _hitPlatform?.WorldPosition ?? Vector3d.Zero;
        _hitPoint = hitPoint;
        _hasHitPoint = true;
        _groundNormal = groundNormal;

        RefreshGroundNormalForce();
        IsGrounded = true;
    }

    private long SetGroundingState(ContactAnchor anchor, Vector3d groundNormal, FixedTransform hitPlatform)
    {
        long groundingStateVersion = ++_groundingStateVersion;
        bool hadPoint = _hasHitPoint;
        Vector3d oldPoint = _hitPoint;
        _hitPlatform = hitPlatform;
        _hitPlatformPosition = hitPlatform.WorldPosition;
        _hasHitPoint = anchor.TryGetWorldPoint(out _hitPoint);
        if (_isGrounded
            && (_groundNormal != groundNormal || hadPoint != _hasHitPoint
                || (_hasHitPoint && oldPoint != _hitPoint)))
            WakeFromCollision();
        _groundNormal = groundNormal;

        RefreshGroundNormalForce();
        IsGrounded = true;
        return groundingStateVersion;
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
        _groundSupport = default;
        _groundSupportVersion = 0;
        _hitPlatformPosition = Vector3d.Zero;
        _hitPoint = Vector3d.Zero;
        _hasHitPoint = false;
        _groundNormal = Vector3d.Zero;
        _normalForce = Vector3d.Zero;
    }

}
