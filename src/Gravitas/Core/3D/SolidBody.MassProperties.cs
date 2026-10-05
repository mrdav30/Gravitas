//=======================================================================
// SolidBody.MassProperties.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas;

public partial class SolidBody
{
    private bool CanUseAngularInertia => IsDynamic && !IsRotationFullyFrozen;

    private Fixed64 _mass;

    /// <summary>Gets or sets mass in kilograms. Non-positive mass disables solver motion.</summary>
    /// <remarks>
    /// A changed runtime value must be assigned between fixed steps. It refreshes inertia,
    /// invalidates cached impulses and CCD state, and wakes the body without changing its
    /// role, pose, velocities, or already accepted accelerations. Equal values are a no-op.
    /// </remarks>
    public Fixed64 Mass
    {
        get => _mass;
        set
        {
            if (_mass == value)
                return;

            if (!Active)
            {
                _mass = value;
                return;
            }

            ThrowIfRuntimeRegistrationMissing();
            Context.ThrowIfFixedStepMutationNotAllowed();
            Collider.ValidateCurrentRuntimeTransform();
            // Calculate before publishing mass: custom collider math may throw.
            RefreshInertiaTensor(value);
            _mass = value;
            InvalidateMassDependentSolverState();
            InvalidateContinuousCollisionTrajectory();
            _sleepFrameCount = 0;
            _isSleeping = false;
            // A zero-mass body can already be awake; Wake alone cannot add it back.
            RefreshPartitionAwakeState();
        }
    }

    private void InvalidateMassDependentSolverState()
    {
        Context.Physics.ClearWarmStartCachesForCollider(Collider);
        Context.Constraints3D.ClearSolverCachesForBody(this);
        Context.Physics.InvalidateContinuousCollisionStateForBodyMutation(this, DynamicId);
    }

    /// <summary>Gets the reciprocal mass, or zero when <see cref="Mass"/> is non-positive.</summary>
    public Fixed64 InverseMass => Mass > Fixed64.Zero
        ? Fixed64.One / Mass
        : Fixed64.Zero;

    private Fixed3x3 _inertiaTensor;
    private Fixed3x3 _worldInertiaTensor;
    private Fixed3x3 _inverseLocalInertiaTensor;
    private Fixed3x3 _inverseInertiaTensor;
    /// <summary>Gets the constrained world-space inverse inertia tensor.</summary>
    public Fixed3x3 InverseInertiaTensor => _inverseInertiaTensor;

    /// <summary>
    /// Gets whether solver-side response may translate this body.
    /// </summary>
    public bool CanTranslate => Active && _dynamicId >= 0 && IsDynamic && !IsPositionFullyFrozen && InverseMass > Fixed64.Zero;

    /// <summary>
    /// Gets whether solver-side response may rotate this body.
    /// </summary>
    public bool CanRotate => Active
        && _dynamicId >= 0
        && IsDynamic
        && !IsRotationFullyFrozen
        && _inverseInertiaTensor != Fixed3x3.Zero;

    internal bool HasSolverMobility => CanTranslate || CanRotate;

    /// <summary>
    /// Gets the inverse mass that should be used by collision response.
    /// Translation-frozen, static, and kinematic bodies expose their raw mass
    /// but contribute zero constrained inverse mass.
    /// </summary>
    public Fixed64 EffectiveInverseMass => CanTranslate ? InverseMass : Fixed64.Zero;

    /// <summary>
    /// Gets the inverse inertia tensor that should be used by collision response.
    /// Bodies that cannot rotate expose a zero tensor even when raw inertia is available.
    /// </summary>
    public Fixed3x3 EffectiveInverseInertiaTensor => CanRotate ? _inverseInertiaTensor : Fixed3x3.Zero;

    internal void RefreshMassPropertiesFromColliderShape()
    {
        if (!_centerOfMassOffsetExplicit)
            _localCenterOfMassOffset = Collider.CalculateLocalCenterOfMassOffset();

        RefreshInertiaTensor(Mass);
    }

    private void UpdateInertiaTensorOrientation()
    {
        CalculateInertiaTensorOrientation(_inertiaTensor, _inverseLocalInertiaTensor,
            out _worldInertiaTensor, out _inverseInertiaTensor);
    }

    private void CalculateInertiaTensorOrientation(
        Fixed3x3 inertia,
        Fixed3x3 inverseLocalInertia,
        out Fixed3x3 worldInertia,
        out Fixed3x3 inverseInertia)
    {
        if (inertia == Fixed3x3.Zero || inverseLocalInertia == Fixed3x3.Zero)
        {
            worldInertia = Fixed3x3.Zero;
            inverseInertia = Fixed3x3.Zero;
            return;
        }

        Fixed3x3 inverseOrientation = Rotation.Conjugate().ToMatrix3x3();
        Fixed3x3 orientation = Rotation.ToMatrix3x3();

        worldInertia = orientation * inertia * inverseOrientation;
        inverseInertia = orientation * inverseLocalInertia * inverseOrientation;
    }

    private void RefreshInertiaTensor(Fixed64 mass)
    {
        Fixed3x3 inertia = CanUseAngularInertia && mass > Fixed64.Zero
            ? Collider.CalculateInertiaTensor(mass, _localCenterOfMassOffset)
            : Fixed3x3.Zero;
        Fixed3x3 inverseLocalInertia = InertiaTensorMath.InvertForSolver(inertia);
        CalculateInertiaTensorOrientation(inertia, inverseLocalInertia,
            out Fixed3x3 worldInertia, out Fixed3x3 inverseInertia);

        // Publish the complete tensor state only after all derived math succeeds.
        _inertiaTensor = inertia;
        _inverseLocalInertiaTensor = inverseLocalInertia;
        _worldInertiaTensor = worldInertia;
        _inverseInertiaTensor = inverseInertia;
    }

}
