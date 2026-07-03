//=======================================================================
// GravitasConstraint2DService.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Constraints;
using SwiftCollections;
using System;

namespace Gravitas;

/// <summary>
/// Owns deterministic pure 2D joints and ragdoll articulation state for one world context.
/// </summary>
public sealed class GravitasConstraint2DService
{
    private const int DefaultJointCapacity = 64;

    private readonly GravitasWorldContext _context;
    private readonly SwiftDictionary<ulong, int> _suppressedColliderPairs = new();
    private readonly SwiftList<RagdollRuntime2D> _ragdolls = new();
    private Joint2D?[] _joints = new Joint2D?[DefaultJointCapacity];
    private int _nextRagdollId;
    private int _enabledJointCount;

    internal GravitasConstraint2DService(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        _context = context;
    }

    /// <summary>
    /// Gets the owning world context.
    /// </summary>
    public GravitasWorldContext Context => _context;

    /// <summary>
    /// Gets the highest joint ID allocated in this context.
    /// </summary>
    public int PeakJointCount { get; private set; }

    /// <summary>
    /// Gets the number of registered active joints.
    /// </summary>
    public int RegisteredJointCount { get; private set; }

    /// <summary>
    /// Gets the number of registered ragdoll runtimes.
    /// </summary>
    public int RegisteredRagdollCount => _ragdolls.Count;

    internal int EnabledJointCount => _enabledJointCount;

    internal bool HasActiveJoints => _enabledJointCount > 0;

    /// <summary>
    /// Registers a context-owned deterministic pure 2D joint.
    /// </summary>
    public Joint2D RegisterJoint(in JointDefinition2D definition)
    {
        ValidateDefinition(definition);

        int id = ++PeakJointCount;
        EnsureJointCapacity(id + 1);
        var joint = new Joint2D(this, id, definition);
        _joints[id] = joint;
        RegisteredJointCount++;
        _enabledJointCount++;

        if (joint.CollisionPolicy == JointCollisionPolicy.SuppressLinked)
            AddSuppressedColliderPair(joint.BodyA.Collider, joint.BodyB.Collider);

        _context.Diagnostics.EmitJointRegistered(joint);
        return joint;
    }

    /// <summary>
    /// Removes a context-owned pure 2D joint and releases its filter state.
    /// </summary>
    public bool RemoveJoint(int jointId)
    {
        if (!TryGetJoint(jointId, out Joint2D? joint))
            return false;

        _joints[jointId] = null;
        RegisteredJointCount--;
        if (joint!.IsEnabled)
            _enabledJointCount--;
        joint.IsActive = false;
        joint.ClearSolverCache();
        if (joint.CollisionPolicy == JointCollisionPolicy.SuppressLinked)
            RemoveSuppressedColliderPair(joint.BodyA.Collider, joint.BodyB.Collider);

        _context.Diagnostics.EmitJointRemoved(joint);
        return true;
    }

    /// <summary>
    /// Gets a registered pure 2D joint by context-local ID.
    /// </summary>
    public Joint2D GetJoint(int jointId)
    {
        if (!TryGetJoint(jointId, out Joint2D? joint))
            throw new ArgumentOutOfRangeException(nameof(jointId), "No active 2D joint exists with the supplied ID.");

        return joint!;
    }

    /// <summary>
    /// Tries to get a registered pure 2D joint by context-local ID.
    /// </summary>
    public bool TryGetJoint(int jointId, out Joint2D? joint)
    {
        if ((uint)jointId >= (uint)_joints.Length)
        {
            joint = null;
            return false;
        }

        joint = _joints[jointId];
        return joint != null && joint.IsActive;
    }

    /// <summary>
    /// Registers a pure 2D ragdoll articulation and its owned joints.
    /// </summary>
    public RagdollRuntime2D RegisterRagdoll(RagdollDefinition2D definition)
    {
        ValidateRagdollDefinition(definition);

        int linkCount = definition.Links.Length;
        var links = new SolidBody2D[linkCount];
        var linkIds = new int[linkCount];
        for (int i = 0; i < linkCount; i++)
        {
            linkIds[i] = definition.Links[i].LinkId;
            links[i] = definition.Links[i].Body;
        }

        int jointCount = definition.Joints.Length;
        var joints = new Joint2D[jointCount];
        for (int i = 0; i < jointCount; i++)
        {
            RagdollJointDefinition2D authoredJoint = definition.Joints[i];
            SolidBody2D bodyA = ResolveRagdollLink(linkIds, links, authoredJoint.LinkAId);
            SolidBody2D bodyB = ResolveRagdollLink(linkIds, links, authoredJoint.LinkBId);
            JointCollisionPolicy collisionPolicy = definition.SelfCollisionPolicy == RagdollSelfCollisionPolicy.CollideAllLinks
                ? JointCollisionPolicy.Collide
                : authoredJoint.CollisionPolicy;

            joints[i] = RegisterJoint(new JointDefinition2D(
                bodyA,
                bodyB,
                authoredJoint.LocalFrameA,
                authoredJoint.LocalFrameB,
                authoredJoint.Type,
                authoredJoint.Limits,
                authoredJoint.Motor,
                collisionPolicy));
        }

        if (definition.SelfCollisionPolicy == RagdollSelfCollisionPolicy.SuppressAllLinks)
            AddAllRagdollPairSuppressions(links);

        var runtime = new RagdollRuntime2D(
            ++_nextRagdollId,
            links,
            joints,
            definition.SelfCollisionPolicy,
            startsActive: !HasKinematicLink(links));
        _ragdolls.Add(runtime);
        return runtime;
    }

    /// <summary>
    /// Updates a pure 2D joint motor target while preserving the motor's gains and impulse cap.
    /// </summary>
    public bool SetJointMotorTarget(int jointId, Fixed64 target)
    {
        if (!TryGetJoint(jointId, out Joint2D? joint))
            return false;

        JointMotor2D current = joint!.Motor;
        JointMotor2D replacement = current.Kind == JointMotorKind2D.Linear
            ? JointMotor2D.Linear(target, current.DriveStrength, current.Damping, current.MaximumMotorImpulse)
            : JointMotor2D.Angular(target, current.DriveStrength, current.Damping, current.MaximumMotorImpulse);
        joint.SetMotor(replacement);
        return true;
    }

    /// <summary>
    /// Disables a pure 2D joint motor target and clears its cached solver impulses.
    /// </summary>
    public bool ClearJointMotorTarget(int jointId)
    {
        if (!TryGetJoint(jointId, out Joint2D? joint))
            return false;

        joint!.ClearMotor();
        return true;
    }

    /// <summary>
    /// Applies caller-owned motor payloads to every joint in a pure 2D ragdoll runtime.
    /// </summary>
    public void SetRagdollPoseTargets(RagdollRuntime2D ragdoll, ReadOnlySpan<JointMotor2D> motors)
    {
        SwiftThrowHelper.ThrowIfNull(ragdoll, nameof(ragdoll));
        SwiftThrowHelper.ThrowIfArgument(
            motors.Length != ragdoll.JointCount,
            nameof(motors),
            "Ragdoll pose target count must match the ragdoll joint count.");

        for (int i = 0; i < motors.Length; i++)
        {
            Joint2D joint = ragdoll.GetJoint(i);
            SwiftThrowHelper.ThrowIfArgument(
                !ReferenceEquals(joint.Service, this),
                nameof(ragdoll),
                "Ragdoll targets must be applied through the owning 2D constraint service.");
            joint.SetMotor(motors[i]);
        }
    }

    /// <summary>
    /// Gets whether the supplied pure 2D colliders should be excluded because a registered articulation links them.
    /// </summary>
    public bool ShouldExcludeLinkedCollision(LSCollider2D colliderA, LSCollider2D colliderB)
    {
        if (colliderA == null || colliderB == null)
            return false;
        if (colliderA.Id <= 0 || colliderB.Id <= 0 || colliderA.Id == colliderB.Id)
            return false;

        return _suppressedColliderPairs.ContainsKey(CreateColliderPairKey(colliderA.Id, colliderB.Id));
    }

    internal bool TryGetJointForSolver(int jointId, out Joint2D? joint) => TryGetJoint(jointId, out joint);

    internal void UpdateJointCollisionPolicy(
        Joint2D joint,
        JointCollisionPolicy oldPolicy,
        JointCollisionPolicy newPolicy)
    {
        if (!joint.IsActive || oldPolicy == newPolicy)
            return;

        if (oldPolicy == JointCollisionPolicy.SuppressLinked)
            RemoveSuppressedColliderPair(joint.BodyA.Collider, joint.BodyB.Collider);
        if (newPolicy == JointCollisionPolicy.SuppressLinked)
            AddSuppressedColliderPair(joint.BodyA.Collider, joint.BodyB.Collider);
    }

    internal void UpdateJointEnabledState(
        Joint2D joint,
        bool oldEnabled,
        bool newEnabled)
    {
        if (!joint.IsActive || oldEnabled == newEnabled)
            return;

        _enabledJointCount += newEnabled ? 1 : -1;
    }

    internal void Reset()
    {
        for (int i = 1; i <= PeakJointCount && i < _joints.Length; i++)
        {
            Joint2D? joint = _joints[i];
            if (joint == null)
                continue;

            joint.IsActive = false;
            joint.ClearSolverCache();
            _joints[i] = null;
        }

        _suppressedColliderPairs.Clear();
        _ragdolls.FastClear();
        PeakJointCount = 0;
        RegisteredJointCount = 0;
        _enabledJointCount = 0;
        _nextRagdollId = 0;
    }

    internal void ContributeReplayHash(
        ref ChronicleHashWriter writer,
        GravitasReplayHashMode mode)
    {
        writer.WriteSection("constraints.2d", 1);
        writer.WriteInt32(PeakJointCount);
        writer.WriteInt32(RegisteredJointCount);
        for (int jointId = 1; jointId <= PeakJointCount; jointId++)
        {
            bool hasJoint = TryGetJoint(jointId, out Joint2D? joint);
            writer.WriteBool(hasJoint);
            if (hasJoint)
                joint!.ContributeReplayHash(ref writer, mode);
        }

        writer.WriteInt32(_ragdolls.Count);
        for (int i = 0; i < _ragdolls.Count; i++)
        {
            RagdollRuntime2D ragdoll = _ragdolls[i];
            writer.WriteInt32(ragdoll.Id);
            writer.WriteEnum(ragdoll.SelfCollisionPolicy);
            writer.WriteBool(ragdoll.IsActive);
            writer.WriteInt32(ragdoll.LinkCount);
            writer.WriteInt32(ragdoll.JointCount);
        }
    }

    internal static ulong CreateColliderPairKey(int colliderAId, int colliderBId)
    {
        uint min = (uint)(colliderAId <= colliderBId ? colliderAId : colliderBId);
        uint max = (uint)(colliderAId <= colliderBId ? colliderBId : colliderAId);
        return ((ulong)min << 32) | max;
    }

    private void EnsureJointCapacity(int required)
    {
        if (required <= _joints.Length)
            return;

        int newSize = _joints.Length;
        while (newSize < required)
            newSize *= 2;

        Array.Resize(ref _joints, newSize);
    }

    private void ValidateDefinition(in JointDefinition2D definition)
    {
        SwiftThrowHelper.ThrowIfNull(definition.BodyA, nameof(definition.BodyA));
        SwiftThrowHelper.ThrowIfNull(definition.BodyB, nameof(definition.BodyB));
        SwiftThrowHelper.ThrowIfArgument(
            ReferenceEquals(definition.BodyA, definition.BodyB),
            nameof(definition),
            "A 2D joint cannot link a body to itself.");
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(definition.BodyA.Context, _context) || !ReferenceEquals(definition.BodyB.Context, _context),
            nameof(definition),
            "Both 2D joint bodies must belong to this constraint service context.");
        SwiftThrowHelper.ThrowIfArgument(
            !definition.BodyA.Active || !definition.BodyB.Active,
            nameof(definition),
            "2D joint bodies must be active before registration.");
        SwiftThrowHelper.ThrowIfArgument(
            definition.BodyA.Collider == null || definition.BodyB.Collider == null,
            nameof(definition),
            "2D joint bodies must have registered 2D colliders.");
        SwiftThrowHelper.ThrowIfArgument(
            !Joint2D.IsSupportedType(definition.Type),
            nameof(definition.Type),
            "Unsupported 2D joint type.");
        SwiftThrowHelper.ThrowIfArgument(
            definition.CollisionPolicy != JointCollisionPolicy.SuppressLinked
                && definition.CollisionPolicy != JointCollisionPolicy.Collide,
            nameof(definition.CollisionPolicy),
            "Unsupported joint collision policy.");
        definition.Limits.Validate();
        definition.Motor.Validate();
        Joint2D.ValidatePayload(definition.Type, definition.Limits, definition.Motor);
    }

    private void ValidateRagdollDefinition(RagdollDefinition2D definition)
    {
        SwiftThrowHelper.ThrowIfNull(definition, nameof(definition));
        SwiftThrowHelper.ThrowIfNull(definition.Links, nameof(definition.Links));
        SwiftThrowHelper.ThrowIfNull(definition.Joints, nameof(definition.Joints));
        SwiftThrowHelper.ThrowIfArgument(definition.Links.Length == 0, nameof(definition.Links), "A 2D ragdoll must contain at least one link.");
        SwiftThrowHelper.ThrowIfArgument(
            definition.SelfCollisionPolicy != RagdollSelfCollisionPolicy.SuppressAdjacentLinks
                && definition.SelfCollisionPolicy != RagdollSelfCollisionPolicy.CollideAllLinks
                && definition.SelfCollisionPolicy != RagdollSelfCollisionPolicy.SuppressAllLinks,
            nameof(definition.SelfCollisionPolicy),
            "Unsupported ragdoll self-collision policy.");

        for (int i = 0; i < definition.Links.Length; i++)
        {
            RagdollLinkDefinition2D link = definition.Links[i];
            SwiftThrowHelper.ThrowIfNull(link.Body, nameof(definition.Links));
            SwiftThrowHelper.ThrowIfNull(link.Collider, nameof(definition.Links));
            SwiftThrowHelper.ThrowIfArgument(
                !ReferenceEquals(link.Body.Context, _context),
                nameof(definition.Links),
                "All 2D ragdoll links must belong to this context.");
            SwiftThrowHelper.ThrowIfArgument(
                !link.Body.Active || link.Body.Collider == null,
                nameof(definition.Links),
                "2D ragdoll links must have active 2D colliders.");
            SwiftThrowHelper.ThrowIfArgument(
                !ReferenceEquals(link.Body.Collider, link.Collider),
                nameof(definition.Links),
                "2D ragdoll link collider must match the link body's collider.");

            for (int j = i + 1; j < definition.Links.Length; j++)
            {
                SwiftThrowHelper.ThrowIfArgument(
                    link.LinkId == definition.Links[j].LinkId,
                    nameof(definition.Links),
                    "2D ragdoll link IDs must be unique.");
            }
        }

        for (int i = 0; i < definition.Joints.Length; i++)
        {
            RagdollJointDefinition2D joint = definition.Joints[i];
            SolidBody2D bodyA = ResolveRagdollLink(definition.Links, joint.LinkAId);
            SolidBody2D bodyB = ResolveRagdollLink(definition.Links, joint.LinkBId);
            SwiftThrowHelper.ThrowIfArgument(
                joint.LinkAId == joint.LinkBId,
                nameof(definition.Joints),
                "A 2D ragdoll joint cannot link a body to itself.");
            JointCollisionPolicy collisionPolicy = definition.SelfCollisionPolicy == RagdollSelfCollisionPolicy.CollideAllLinks
                ? JointCollisionPolicy.Collide
                : joint.CollisionPolicy;
            ValidateDefinition(new JointDefinition2D(
                bodyA,
                bodyB,
                joint.LocalFrameA,
                joint.LocalFrameB,
                joint.Type,
                joint.Limits,
                joint.Motor,
                collisionPolicy));
        }
    }

    private static bool HasKinematicLink(SolidBody2D[] links)
    {
        for (int i = 0; i < links.Length; i++)
        {
            if (links[i].IsKinematic)
                return true;
        }

        return false;
    }

    private static SolidBody2D ResolveRagdollLink(RagdollLinkDefinition2D[] links, int linkId)
    {
        for (int i = 0; i < links.Length; i++)
        {
            if (links[i].LinkId == linkId)
                return links[i].Body;
        }

        throw new ArgumentException("2D ragdoll joint references an unknown link ID.", nameof(linkId));
    }

    private static SolidBody2D ResolveRagdollLink(int[] linkIds, SolidBody2D[] links, int linkId)
    {
        for (int i = 0; i < linkIds.Length; i++)
        {
            if (linkIds[i] == linkId)
                return links[i];
        }

        throw new ArgumentException("2D ragdoll joint references an unknown link ID.", nameof(linkId));
    }

    private void AddAllRagdollPairSuppressions(SolidBody2D[] links)
    {
        for (int i = 0; i < links.Length; i++)
        {
            for (int j = i + 1; j < links.Length; j++)
                AddSuppressedColliderPair(links[i].Collider, links[j].Collider);
        }
    }

    private void AddSuppressedColliderPair(LSCollider2D colliderA, LSCollider2D colliderB)
    {
        ulong key = CreateColliderPairKey(colliderA.Id, colliderB.Id);
        if (_suppressedColliderPairs.TryGetValue(key, out int count))
            _suppressedColliderPairs[key] = count + 1;
        else
            _suppressedColliderPairs.Add(key, 1);
    }

    private void RemoveSuppressedColliderPair(LSCollider2D colliderA, LSCollider2D colliderB)
    {
        ulong key = CreateColliderPairKey(colliderA.Id, colliderB.Id);
        if (!_suppressedColliderPairs.TryGetValue(key, out int count))
            return;

        if (count <= 1)
            _suppressedColliderPairs.Remove(key);
        else
            _suppressedColliderPairs[key] = count - 1;
    }
}
