//=======================================================================
// SolidBody.ColliderReconfiguration.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using SwiftCollections;

namespace Gravitas;

/// <content>Owns exact, transactional 3D collider reconfiguration.</content>
public partial class SolidBody
{
    /// <summary>
    /// Attempts to replace this registered body's primitive geometry and root
    /// pose without replacing its collider, body, or host identity.
    /// </summary>
    /// <remarks>
    /// Sphere, capsule, and finite-cylinder definitions are supported. The
    /// definition must match the collider's existing family. Material, filters,
    /// hierarchy, events, rotation, motion, and runtime identity are retained.
    /// Exact touching is accepted; any physically eligible 3D collider with
    /// positive penetration, even below one raw unit, rejects the transaction.
    /// <paramref name="publishSynchronizedState"/> runs exactly once after an
    /// applied physical state is committed, or after an unchanged state is
    /// confirmed, and before separation callbacks. It does not run for a blocked
    /// transaction. Publisher or separation-callback failures do not roll back
    /// an accepted transaction; pair retirement continues, then one failure is
    /// returned directly or several are aggregated through
    /// <paramref name="notificationException"/>. This keeps the transaction
    /// result distinct from post-commit host notification failure while allowing
    /// a coordinating caller to prevent observers from seeing split state.
    /// </remarks>
    /// <param name="definition">The replacement geometry definition.</param>
    /// <param name="localOffset">The replacement unscaled collider offset.</param>
    /// <param name="position">The replacement authoritative body-root position.</param>
    /// <param name="blocker">The first stable-order collider with positive candidate penetration.</param>
    /// <param name="notificationException">
    /// A synchronized-state publisher or separation-callback failure observed
    /// after an accepted transaction, or <see langword="null"/>. One failure is
    /// returned directly; multiple failures are combined in stable order as an
    /// <see cref="AggregateException"/>.
    /// </param>
    /// <param name="publishSynchronizedState">
    /// Optional dependent-state publisher invoked after the physical result is
    /// committed or confirmed unchanged and before separation callbacks.
    /// </param>
    /// <returns>The exact transaction result.</returns>
    /// <exception cref="ArgumentException">
    /// The definition is default, unsupported, or does not match the existing
    /// collider family.
    /// </exception>
    /// <exception cref="InvalidOperationException">
    /// The body is not registered, the fixed-step transaction is active, or the
    /// candidate geometry or host pose is not representable.
    /// </exception>
    public ColliderReconfigurationStatus TryReconfigureCollider(
        ColliderShapeDefinition definition,
        Vector3d localOffset,
        Vector3d position,
        out LSCollider? blocker,
        out Exception? notificationException,
        Action? publishSynchronizedState = null)
    {
        blocker = null;
        notificationException = null;
        definition.EnsureDefined();
        ThrowIfRuntimeRegistrationMissing();
        Context.ThrowIfFixedStepMutationNotAllowed();
        ValidateReconfigurationFamily(definition);
        ChronicleGuard.ThrowIfTrue(
            !CanPublishRootPosition(position),
            nameof(position),
            "The host transform cannot represent the requested body-root position.");

        LSCollider candidate = definition.CreateRuntimeCollider();
        candidate.PrepareDetachedBodyReconfigurationCandidate(this, localOffset, position, Rotation);
        if (MatchesReconfiguration(definition, candidate, localOffset, position))
        {
            notificationException = NotifyColliderReconfiguration(ColliderReconfigurationStatus.Unchanged, publishSynchronizedState);
            return ColliderReconfigurationStatus.Unchanged;
        }

        PhysicsQueryMeter meter = default;
        FindColliderReconfigurationBlocker(candidate, ref meter, bounded: false, out blocker);
        if (blocker != null)
            return ColliderReconfigurationStatus.Blocked;

        ChronicleGuard.ThrowIfTrue(!TryCommitReconfigurationGeometry(definition, localOffset, position),
            nameof(FixedTransform), "Position transform cannot represent the requested world position.");
        notificationException = NotifyColliderReconfiguration(ColliderReconfigurationStatus.Applied, publishSynchronizedState);
        return ColliderReconfigurationStatus.Applied;
    }

    /// <summary>Attempts to replace registered sphere or capsule geometry and its root pose within a bounded clearance query.</summary>
    /// <remarks>
    /// Requires pure 3D. The caller serializes this operation with world and collider mutation. An incomplete
    /// query leaves the live state unchanged and clears outcome outputs. Work limits cover detached preparation
    /// and clearance. Successful publication, mass properties, pair cleanup, and host notifications follow clearance.
    /// </remarks>
    public PhysicsQueryExecutionStatus TryReconfigureCollider(
        ColliderShapeDefinition definition, Vector3d localOffset, Vector3d position,
        in PhysicsQueryBudget budget, PhysicsQueryScratch scratch,
        out ColliderReconfigurationStatus? result, out LSCollider? blocker,
        out Exception? notificationException, out PhysicsQueryWork work,
        Action? publishSynchronizedState = null)
    {
        result = null;
        blocker = null;
        notificationException = null;
        work = default;
        ChronicleGuard.ThrowIfNull(scratch, nameof(scratch));
        definition.EnsureDefined();
        ThrowIfRuntimeRegistrationMissing();
        if (Context.Settings.RuntimeMode != PhysicsRuntimeMode.ThreeD)
            return PhysicsQueryExecutionStatus.UnsupportedRuntimeMode;
        ValidateReconfigurationFamily(definition);
        if (definition.Kind == ColliderShapeDefinitionKind.Cylinder)
            return PhysicsQueryExecutionStatus.UnsupportedGeometry;

        var meter = new PhysicsQueryMeter(budget);
        LSCollider? candidate = null;
        scratch.Borrow();
        try
        {
            ulong sequence = Context.ObservedGridChangeSequence;
            var status = PhysicsQueryScratch.CheckView(Context, sequence, ref meter, PhysicsQueryExecutionStatus.WorldNotReady);
            if (status != PhysicsQueryExecutionStatus.Completed)
                return status;
            FixedTransform agentTransform = Agent.Transform;
            if (!PhysicsQueryScratch.ChargePosturePreparation(agentTransform, ref meter)
                || !ReferenceEquals(agentTransform, PositionTransform)
                    && !PhysicsQueryScratch.ChargePosturePreparation(PositionTransform, ref meter))
                return PhysicsQueryExecutionStatus.WorkBudgetExceeded;
            if (!CanPublishRootPosition(position))
                return PhysicsQueryExecutionStatus.Unrepresentable;

            candidate = scratch.GetPostureCandidate(definition);
            try
            {
                candidate.PrepareDetachedBodyReconfigurationCandidate(this, localOffset, position, Rotation);
            }
            catch (Exception exception) when (exception is ArgumentException or InvalidOperationException or OverflowException)
            {
                return PhysicsQueryExecutionStatus.Unrepresentable;
            }

            bool unchanged = MatchesReconfiguration(definition, candidate, localOffset, position);
            LSCollider? found = null;
            if (!unchanged)
            {
                status = FindColliderReconfigurationBlocker(candidate, ref meter, bounded: true, out found);
                if (status != PhysicsQueryExecutionStatus.Completed)
                    return status;
            }
            status = PhysicsQueryScratch.CheckView(Context, sequence, ref meter);
            if (status != PhysicsQueryExecutionStatus.Completed)
                return status;
            if (found != null)
            {
                blocker = found;
                result = ColliderReconfigurationStatus.Blocked;
                return PhysicsQueryExecutionStatus.Completed;
            }

            result = unchanged ? ColliderReconfigurationStatus.Unchanged : ColliderReconfigurationStatus.Applied;
        }
        finally
        {
            candidate?.ReleaseDetachedBodyReconfigurationCandidate();
            scratch.Release(0);
            work = meter.Work;
        }

        if (result == ColliderReconfigurationStatus.Applied && !TryCommitReconfigurationGeometry(definition, localOffset, position))
        {
            result = null;
            return PhysicsQueryExecutionStatus.Unrepresentable;
        }
        notificationException = NotifyColliderReconfiguration(result.Value, publishSynchronizedState);
        return PhysicsQueryExecutionStatus.Completed;
    }

    private void ValidateReconfigurationFamily(ColliderShapeDefinition definition)
    {
        ColliderType expectedShape = GetSupportedRuntimeShape(definition.Kind);
        ChronicleGuard.ThrowIfArgument(
            expectedShape == ColliderType.None,
            nameof(definition),
            "Collider reconfiguration supports only sphere, capsule, and finite-cylinder definitions.");
        ChronicleGuard.ThrowIfArgument(
            Collider.Shape != expectedShape,
            nameof(definition),
            "Collider reconfiguration cannot replace the existing collider family.");
    }

    private bool MatchesReconfiguration(ColliderShapeDefinition definition, LSCollider candidate,
        Vector3d localOffset, Vector3d position) => Position3d == position
        && Collider.LocalOffset == localOffset
        && Collider.Radius == definition.Radius
        && Collider.Size == definition.Size
        && Collider.MatchesCommittedReconfigurationCandidate(candidate);

    private PhysicsQueryExecutionStatus FindColliderReconfigurationBlocker(
        LSCollider candidate, ref PhysicsQueryMeter meter, bool bounded, out LSCollider? blocker)
    {
        blocker = null;
        var physics = Context.Physics;
        int count = physics.ColliderCount;
        int cursor = bounded ? physics.FirstColliderRegistrationId : 0;
        while (bounded ? cursor >= 0 : cursor < count)
        {
            if (bounded && !meter.ChargeCandidate())
                return PhysicsQueryExecutionStatus.WorkBudgetExceeded;
            LSCollider other = bounded ? physics.GetColliderByRegistrationId(cursor, out cursor)
                : physics.GetColliderByServiceIndex(cursor++);
            if (ReferenceEquals(other, Collider) || other.IsTrigger)
                continue;
            if (bounded && !meter.ChargeCandidates(Context.Constraints3D.LinkedCollisionProbeLimit))
                return PhysicsQueryExecutionStatus.WorkBudgetExceeded;
            if (!physics.RequireCollisionPair(Collider, other))
                continue;
            if (bounded && !meter.ChargeGeometry(1))
                return PhysicsQueryExecutionStatus.WorkBudgetExceeded;
            if (!candidate.Bounds.Intersects(other.Bounds))
                continue;

            LSCollider first = candidate;
            LSCollider second = other;
            if (second.Priority > first.Priority)
                (first, second) = (second, first);

            CollisionType collisionType = ColliderSettings.GetCollisionType(
                first.Shape,
                second.Shape);
            if (collisionType == CollisionType.None)
                continue;

            var status = CollisionDetection.CheckPostureCandidatePenetration(candidate, other, ref meter, bounded, out bool penetrates);
            if (status != PhysicsQueryExecutionStatus.Completed)
                return status;
            if (!penetrates)
                continue;

            blocker = other;
            break;
        }

        return PhysicsQueryExecutionStatus.Completed;
    }

    private bool TryCommitReconfigurationGeometry(ColliderShapeDefinition definition, Vector3d localOffset, Vector3d position)
    {
        Collider.PrepareBodyReconfiguration(definition, localOffset, position, Rotation);
        if (!PositionTransform.TrySetWorldPosition(position))
            return false;

        Context.Constraints3D.ClearSolverCachesForBody(this);
        InvalidateContinuousCollisionTrajectory();

        Position3d = position;
        Collider.PublishPreparedBodyReconfiguration(definition, localOffset);
        Collider.Simulate();
        if (Context.Settings.RuntimeMode.RunsMixedContacts())
            Context.MixedCollisions.Refresh3DColliderPartition(Collider);
        Wake();
        return true;
    }

    private Exception? NotifyColliderReconfiguration(ColliderReconfigurationStatus result, Action? publishSynchronizedState)
    {
        var registration = result == ColliderReconfigurationStatus.Applied ? new ColliderLifetimeToken(Collider) : default;
        SwiftList<Exception>? notificationExceptions = null;
        CaptureSynchronizedStatePublication(
            publishSynchronizedState,
            ref notificationExceptions);
        if (result == ColliderReconfigurationStatus.Applied && registration.IsCurrentLifetime)
            Context.Physics.InvalidatePairsForColliderReconfiguration(
                Collider,
                ref notificationExceptions);
        if (result == ColliderReconfigurationStatus.Applied && registration.IsCurrentLifetime)
            Context.MixedCollisions.InvalidatePairsFor3DColliderReconfiguration(
                Collider,
                ref notificationExceptions);
        return CollisionNotificationExceptions.ToException(notificationExceptions);
    }

    private static void CaptureSynchronizedStatePublication(
        Action? publishSynchronizedState,
        ref SwiftList<Exception>? notificationExceptions)
    {
        if (publishSynchronizedState == null)
            return;

        try
        {
            publishSynchronizedState();
        }
        catch (Exception exception)
        {
            CollisionNotificationExceptions.Capture(
                ref notificationExceptions,
                exception);
        }
    }

    private bool CanPublishRootPosition(Vector3d position)
    {
        FixedTransform? parent = PositionTransform.Parent;
        if (parent == null)
            return true;

        return parent.TryInverseTransformPoint(position, out Vector3d localPosition)
            && parent.TryTransformPoint(localPosition, out Vector3d roundTrip)
            && roundTrip.FuzzyEqualAbsolute(position, Fixed64.Epsilon);
    }

    private static ColliderType GetSupportedRuntimeShape(
        ColliderShapeDefinitionKind kind) =>
        kind switch
        {
            ColliderShapeDefinitionKind.Sphere => ColliderType.Sphere,
            ColliderShapeDefinitionKind.Capsule => ColliderType.Capsule,
            ColliderShapeDefinitionKind.Cylinder => ColliderType.Cylinder,
            _ => ColliderType.None
        };
}
