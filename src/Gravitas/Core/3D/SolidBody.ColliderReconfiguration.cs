//=======================================================================
// SolidBody.ColliderReconfiguration.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using SwiftCollections;
using System;

namespace Gravitas;

/// <content>Owns exact, transactional 3D collider reconfiguration.</content>
public partial class SolidBody
{
    private readonly ContactManifold _colliderReconfigurationManifold = new();

    /// <summary>
    /// Attempts to replace this registered body's primitive geometry and root
    /// pose without replacing its collider, body, or host identity.
    /// </summary>
    /// <remarks>
    /// Sphere, capsule, and finite-cylinder definitions are supported. The
    /// definition must match the collider's existing family. Material, filters,
    /// hierarchy, events, rotation, motion, and runtime identity are retained.
    /// A zero-depth support contact is accepted; any physically eligible 3D
    /// collider with positive penetration rejects the whole transaction.
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
        ColliderType expectedShape = GetSupportedRuntimeShape(definition.Kind);
        SwiftThrowHelper.ThrowIfArgument(
            expectedShape == ColliderType.None,
            nameof(definition),
            "Collider reconfiguration supports only sphere, capsule, and finite-cylinder definitions.");
        SwiftThrowHelper.ThrowIfArgument(
            Collider.Shape != expectedShape,
            nameof(definition),
            "Collider reconfiguration cannot replace the existing collider family.");
        SwiftThrowHelper.ThrowIfTrue(
            !CanPublishRootPosition(position),
            nameof(position),
            "The host transform cannot represent the requested body-root position.");

        LSCollider candidate = definition.CreateRuntimeCollider();
        candidate.PrepareDetachedBodyReconfigurationCandidate(
            this,
            localOffset,
            position,
            Rotation);

        if (Position3d == position
            && Collider.LocalOffset == localOffset
            && Collider.Radius == definition.Radius
            && Collider.Size == definition.Size
            && Collider.MatchesCommittedReconfigurationCandidate(candidate))
        {
            SwiftList<Exception>? notificationExceptions = null;
            CaptureSynchronizedStatePublication(
                publishSynchronizedState,
                ref notificationExceptions);
            notificationException = CollisionNotificationExceptions.ToException(
                notificationExceptions);
            return ColliderReconfigurationStatus.Unchanged;
        }

        if (TryFindColliderReconfigurationBlocker(candidate, out blocker))
            return ColliderReconfigurationStatus.Blocked;

        Collider.PrepareBodyReconfiguration(
            definition,
            localOffset,
            position,
            Rotation);
        notificationException = CommitColliderReconfiguration(
            definition,
            localOffset,
            position,
            publishSynchronizedState);
        return ColliderReconfigurationStatus.Applied;
    }

    private bool TryFindColliderReconfigurationBlocker(
        LSCollider candidate,
        out LSCollider? blocker)
    {
        int count = Context.Physics.ColliderCount;
        for (int i = 0; i < count; i++)
        {
            LSCollider other = Context.Physics.GetColliderByServiceIndex(i);
            if (ReferenceEquals(other, Collider)
                || other.IsTrigger
                || !Context.Physics.RequireCollisionPair(Collider, other)
                || !candidate.Bounds.Intersects(other.Bounds))
            {
                continue;
            }

            LSCollider first = candidate;
            LSCollider second = other;
            if (second.Priority > first.Priority)
                (first, second) = (second, first);

            CollisionType collisionType = ColliderSettings.GetCollisionType(
                first.Shape,
                second.Shape);
            if (collisionType == CollisionType.None)
                continue;

            _colliderReconfigurationManifold.BeginUpdate(Context.FrameCount);
            var workItem = new CollisionWorkItem(
                Context,
                first,
                second,
                collisionType,
                _colliderReconfigurationManifold);
            if (!CollisionDetection.DoCollisionCheck(workItem)
                || !HasPositivePenetration(_colliderReconfigurationManifold))
            {
                continue;
            }

            blocker = other;
            return true;
        }

        blocker = null;
        return false;
    }

    private Exception? CommitColliderReconfiguration(
        ColliderShapeDefinition definition,
        Vector3d localOffset,
        Vector3d position,
        Action? publishSynchronizedState)
    {
        Context.Constraints3D.ClearSolverCachesForBody(this);
        InvalidateContinuousCollisionTrajectory();

        Position3d = position;
        SetPositionTransformWorldPosition(position);
        Collider.PublishPreparedBodyReconfiguration(definition, localOffset);
        Collider.Simulate();
        if (Context.Settings.RuntimeMode.RunsMixedContacts())
            Context.MixedCollisions.Refresh3DColliderPartition(Collider);
        Wake();

        SwiftList<Exception>? notificationExceptions = null;
        CaptureSynchronizedStatePublication(
            publishSynchronizedState,
            ref notificationExceptions);
        Context.Physics.InvalidatePairsForColliderReconfiguration(
            Collider,
            ref notificationExceptions);
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

    private static bool HasPositivePenetration(ContactManifold manifold)
    {
        for (int i = 0; i < manifold.Count; i++)
        {
            if (manifold[i].Depth > Fixed64.Zero)
                return true;
        }

        return false;
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
