//=======================================================================
// RagdollRuntime3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using System.Runtime.CompilerServices;

namespace Gravitas.Constraints;

/// <summary>
/// Runtime handle for a context-owned 3D ragdoll articulation.
/// </summary>
public sealed class RagdollRuntime3D : IRecordable
{
    private readonly GravitasConstraint3DService _service;
    private readonly SolidBody[] _links;
    private readonly Joint3D[] _joints;
    private bool _isActive;

    internal RagdollRuntime3D(
        GravitasConstraint3DService service,
        int id,
        SolidBody[] links,
        Joint3D[] joints,
        RagdollSelfCollisionPolicy selfCollisionPolicy,
        bool startsActive)
    {
        _service = service;
        Id = id;
        _links = links;
        _joints = joints;
        SelfCollisionPolicy = selfCollisionPolicy;
        IsRegistered = true;
        ApplyActivationState(startsActive, emitDiagnostics: false);
    }

    internal GravitasConstraint3DService Service => _service;

    internal RagdollRuntime3D? PreviousRegistered { get; set; }

    internal RagdollRuntime3D? NextRegistered { get; set; }

    /// <summary>
    /// Gets this context-local ragdoll ID.
    /// </summary>
    public int Id { get; }

    /// <summary>
    /// Gets ragdoll-internal collision filtering behavior.
    /// </summary>
    public RagdollSelfCollisionPolicy SelfCollisionPolicy { get; }

    /// <summary>
    /// Gets whether this articulation remains registered with its owning constraint service.
    /// </summary>
    public bool IsRegistered { get; private set; }

    /// <summary>
    /// Gets whether the ragdoll links are currently dynamic.
    /// </summary>
    public bool IsActive => _isActive;

    /// <summary>
    /// Gets the number of links in this ragdoll.
    /// </summary>
    public int LinkCount => _links.Length;

    /// <summary>
    /// Gets the number of joints in this ragdoll.
    /// </summary>
    public int JointCount => _joints.Length;

    /// <summary>
    /// Gets a ragdoll link by runtime index.
    /// </summary>
    public SolidBody GetLink(int index) => _links[index];

    /// <summary>
    /// Gets a ragdoll joint by runtime index.
    /// </summary>
    public Joint3D GetJoint(int index) => _joints[index];

    /// <summary>
    /// Switches all links to dynamic simulation and enables ragdoll joints.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ActivateDynamic() => ApplyActivationState(isActive: true, emitDiagnostics: true);

    /// <summary>
    /// Switches all links to kinematic host control and disables ragdoll joints.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void DeactivateToKinematic() => ApplyActivationState(isActive: false, emitDiagnostics: true);

    /// <summary>
    /// Records runtime activation state for deterministic continuation.
    /// </summary>
    public void RecordData(IChronicler chronicler)
    {
        SwiftThrowHelper.ThrowIfTrue(
            !IsRegistered,
            nameof(RagdollRuntime3D),
            "Removed ragdolls cannot participate in serialization.");
        bool isActive = _isActive;
        RecordValues.Look(chronicler, ref isActive, "IsActive", false);
        if (chronicler.Mode != SerializationMode.Loading)
            return;

        ApplyActivationState(isActive, emitDiagnostics: true);
    }

    private void ApplyActivationState(bool isActive, bool emitDiagnostics)
    {
        SwiftThrowHelper.ThrowIfTrue(
            !IsRegistered,
            nameof(RagdollRuntime3D),
            "Removed ragdolls cannot mutate simulation state.");

        if (isActive)
        {
            for (int i = 0; i < _links.Length; i++)
            {
                _links[i].IsKinematic = false;
                _links[i].Wake();
            }

            for (int i = 0; i < _joints.Length; i++)
                _joints[i].IsEnabled = true;
        }
        else
        {
            for (int i = 0; i < _joints.Length; i++)
                _joints[i].IsEnabled = false;

            for (int i = 0; i < _links.Length; i++)
                _links[i].IsKinematic = true;
        }

        _isActive = isActive;
        if (emitDiagnostics)
            _links[0].Context.Diagnostics.EmitRagdollActivated(Id, LinkCount, JointCount, isActive);
    }

    internal void Invalidate()
    {
        IsRegistered = false;
        _isActive = false;
        PreviousRegistered = null;
        NextRegistered = null;
    }
}
