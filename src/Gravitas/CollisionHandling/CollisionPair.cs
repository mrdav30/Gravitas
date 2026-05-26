using FixedMathSharp;
using Gravitas.Colliders;
using GridForge.Grids;
using SwiftCollections;
using System;
using System.Runtime.CompilerServices;


namespace Gravitas.CollisionHandling;

/// <summary>
/// Handles collision pairs between various types of colliders using the Separating Axis Theorem and maintains related state information.
/// </summary>
public class CollisionPair
{
    public bool Debug = true;

    public bool Active { get; private set; }

    private bool _isPooledForDeactivation;

    public GravitasWorldContext Context { get; private set; } = null!;

    public GridWorld World => Context.World;

    // stores order in which they come in
    public int Id1 { get; private set; }
    public int Id2 { get; private set; }

    public LSCollider ColliderA { get; private set; } = null!;
    public LSCollider ColliderB { get; private set; } = null!;

    public uint PartitionVersion;
    public ushort PairVersion = 1;

    public int LastFrame { get; private set; }
    public int LastCollidedFrame { get; private set; }

    private Fixed64 _fastCollideDistance;
    private Fixed64 _fastDistance;
    public CollisionType CollisionType { get; private set; }
    private bool _doPhysics = true;
    private bool _preventCulling = false;

    //If negative, prevent culling altogether
    public short CullCounter { get; private set; }
    private bool _preventDistanceCull;
    private Fixed64 _fastDistanceOffset;
    private uint _lastColliderABroadPhaseVersion;
    private uint _lastColliderBBroadPhaseVersion;

    private bool _isColliding;
    private bool _isCollidingChanged;

    public ContactManifold Manifold { get; } = new();

    public CollisionPair(LSCollider c1, LSCollider c2) => Initialize(c1, c2);

    /// <summary>
    /// Initializes the CollisionPair with the given colliders.
    /// </summary>
    /// <param name="c1">The first collider.</param>
    /// <param name="c2">The second collider.</param>
    public void Initialize(LSCollider c1, LSCollider c2)
    {
        SwiftThrowHelper.ThrowIfNull(c1, nameof(c1));
        SwiftThrowHelper.ThrowIfNull(c2, nameof(c2));
        SwiftThrowHelper.ThrowIfArgument(c1 == c2, nameof(c2), "Cannot create a CollisionPair with the same collider.");
        GravitasWorldContext context = c1.Context;
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(context, c2.Context),
            nameof(c2),
            "Colliders must be in the same context to create a CollisionPair.");

        Context = context;

        Reset();

        AssignPriority(c1, c2);
        Id1 = ColliderA.Id;
        Id2 = ColliderB.Id;

        CollisionType = ColliderSettings.GetCollisionType(ColliderA.Shape, ColliderB.Shape);

        // Calculate the square of the sum of the radii of the bounding spheres
        _fastCollideDistance = ColliderA!.Bounds.Scope.Magnitude + ColliderB!.Bounds.Scope.Magnitude;
        _fastCollideDistance *= _fastCollideDistance;

        _doPhysics = ColliderA!.Body != null && ColliderB!.Body != null && !ColliderA!.IsTrigger && !ColliderB!.IsTrigger;

        if (ColliderA!.PreventCulling || ColliderB!.PreventCulling)
        {
            CullCounter = -1;  //  Never cull
            _preventCulling = true;
        }
        else
        {
            //Immediately check collision
            CullCounter = 0;
            //If collision distance is too large, don't cull based on distance
            _preventDistanceCull = _fastCollideDistance > Context.Environment.CullFastDistanceMax;
            _fastDistanceOffset = Fixed64.FromRaw((int)_fastCollideDistance) + (Fixed64.One * 2) * (Fixed64.One * 2);
        }

        LastCollidedFrame = Context.FrameCount;
        RefreshBroadPhaseVersions();
        PairVersion++;
        Active = true;
    }

    public void AssignPriority(LSCollider c1, LSCollider c2)
    {
        if (ShouldFirstColliderLead(c1, c2))
        {
            ColliderA = c1;
            ColliderB = c2;
            return;
        }

        ColliderA = c2;
        ColliderB = c1;
    }

    private static bool ShouldFirstColliderLead(LSCollider c1, LSCollider c2)
    {
        if (c1.Priority != c2.Priority)
            return c1.Priority > c2.Priority;

        if (c1.Body == null || c2.Body == null)
            return true;

        if (c1.Body.LinearSpeed != c2.Body.LinearSpeed)
            return c1.Body.LinearSpeed > c2.Body.LinearSpeed;

        return true;
    }

    /// <summary>
    /// Checks and distributes collisions between colliders.
    /// Called by Partition Manager every fixed update if 2 colliders are on the same partion.
    /// </summary>
    public void UpdateCollision()
    {
        if (!IsCollisionPairActive())
            return;

        UpdateLastFrame();
        DeactivateAndPoolIfRequired();

        _isCollidingChanged = false;
        if (!_preventCulling && IsCullStateInvalidated())
            CullCounter = 0;

        if (_preventCulling || CullCounter <= 0)
        {
            ProcessCollision();
            RefreshBroadPhaseVersions();
            if (_isCollidingChanged && !_isColliding)
                Manifold.Reset();

            HandleCullingIfNotColliding();
            return;
        }

        if (!_preventCulling) CullCounter--;  //  Culled and counter 1 step closer until checking again
    }

    private bool IsCollisionPairActive() => Active && ColliderA.IsActive && ColliderB.IsActive;

    private void UpdateLastFrame() => LastFrame = Context.FrameCount;

    private void DeactivateAndPoolIfRequired()
    {
        if (_isPooledForDeactivation)
            return;

        Context.Physics.PoolForDeactivation(this);
        _isPooledForDeactivation = true;
    }

    private void ProcessCollision()
    {
        if (!ShouldPerformCollisionCheck())
        {
            _isCollidingChanged = _isColliding;
            _isColliding = false;
            Manifold.Reset();
            return;
        }

        bool result = CheckCollision();
        if (result && Manifold.HasContact)
            Context.Diagnostics.EmitContact(this, result);

        if (result ^ _isColliding)
        {
            _isColliding = result;
            _isCollidingChanged = true;
        }

        if (!result || !_doPhysics)
            return;

        CollisionResponse.CalculateImpulse(this);
    }

    public void NotifyCollidersOfContact()
    {
        ColliderA.NotifyContact(ColliderB, _isColliding, _isCollidingChanged);
        ColliderB.NotifyContact(ColliderA, _isColliding, _isCollidingChanged);
    }

    private void HandleCullingIfNotColliding()
    {
        if (_isColliding)
        {
            LastCollidedFrame = Context.FrameCount;
            return;
        }

        if (CullCounter >= 0)  //  A Negative cull counter means a Body is preventing culling
            CalculateCullScore();
    }

    private bool CheckCollision()
    {
        if (!BroadPhaseVersionChanged() && _isColliding)
            return _isColliding;

        return CollisionDetection.DoCollisionCheck(this);
    }

    private bool IsCullStateInvalidated()
    {
        return ColliderA.PartitionChanged
            || ColliderB.PartitionChanged
            || BroadPhaseVersionChanged();
    }

    private bool BroadPhaseVersionChanged()
    {
        return ColliderA.BroadPhaseVersion != _lastColliderABroadPhaseVersion
            || ColliderB.BroadPhaseVersion != _lastColliderBBroadPhaseVersion;
    }

    private void RefreshBroadPhaseVersions()
    {
        _lastColliderABroadPhaseVersion = ColliderA.BroadPhaseVersion;
        _lastColliderBBroadPhaseVersion = ColliderB.BroadPhaseVersion;
    }

    public void SetImmovableDirection(Vector3d directionA, Vector3d directionB)
    {
        if (ColliderA.Body?.Immovable == true)
            Manifold.SetImmovableDirection(directionA);
        else if (ColliderB.Body?.Immovable == true)
            Manifold.SetImmovableDirection(directionB);
    }

    private bool ShouldPerformCollisionCheck()
    {
        // Calculate the square distance between the two bounding box centers
        // If the square distance between the centers is greater than the square of the sum of their combine scope,
        // then the bounding boxes do not overlap and no collision check should be performed
        _fastDistance = Vector3d.SqrDistance(ColliderA.Center, ColliderB.Center);
        if (_fastDistance > _fastCollideDistance)
            return false;
        // Inclusive bounds overlap preserves zero-depth touching contacts for the manifold pass.
        return BoundsOverlapInclusive(ColliderA, ColliderB);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool BoundsOverlapInclusive(LSCollider colliderA, LSCollider colliderB)
    {
        return colliderA.BoundsMin.x <= colliderB.BoundsMax.x
            && colliderA.BoundsMax.x >= colliderB.BoundsMin.x
            && colliderA.BoundsMin.y <= colliderB.BoundsMax.y
            && colliderA.BoundsMax.y >= colliderB.BoundsMin.y
            && colliderA.BoundsMin.z <= colliderB.BoundsMax.z
            && colliderA.BoundsMax.z >= colliderB.BoundsMin.z;
    }

    private void CalculateCullScore()
    {
        int distanceScore = 0;
        int velocityScore = 0;
        if (!_preventDistanceCull)
        {
            int distanceMax = Context.Environment.CullDistanceMax;
            if (distanceMax > 0)
            {
                int step = GetCullDistanceStep(World!);
                distanceScore = Math.Clamp((int)(_fastDistance - _fastDistanceOffset) / step + Context.Collisions.CullDistributor, 0, distanceMax);
            }

            int cullVelocityStep = Context.Environment.CullVelocityStep;
            if (cullVelocityStep > 0)
                velocityScore = Math.Clamp((int)(ColliderA.Velocity - ColliderB.Velocity).Magnitude / cullVelocityStep, 0, Context.Environment.CullVelocityMax);
        }

        int timeScore = 0;
        int cullTimeStep = Context.Environment.CullTimeStep;
        if (cullTimeStep > 0)
            timeScore = Math.Clamp((Context.FrameCount - LastCollidedFrame) / cullTimeStep, 0, Context.Environment.CullTimeMax);

        CullCounter = (short)Math.Clamp(distanceScore + timeScore - velocityScore, 0, short.MaxValue);
    }

    /// <summary>
    /// Defines the step value for distance-based culling. The score is increased
    /// when the distance between objects increases. Higher values make the culling more aggressive for distant objects.
    /// </summary>
    internal int GetCullDistanceStep(GridWorld world)
    {
        int distanceMax = Context.Environment.CullDistanceMax;
        if (distanceMax <= 0)
            return int.MaxValue;

        int step = ((world.VoxelSize + Fixed64.One * 2) * (world.VoxelSize + Fixed64.One * 2) / distanceMax).CeilToInt();
        return Math.Max(1, step);
    }

    public void Reset()
    {
        Manifold.Reset();
        _isColliding = false;
        _isPooledForDeactivation = false;
    }

    /// <summary>
    /// Deactivates the CollisionPair.
    /// </summary>
    public void Deactivate()
    {
        if (_isColliding)
        {
            _isColliding = false;
            _isCollidingChanged = true;
            NotifyCollidersOfContact();
        }

        Reset();
        Active = false;
    }
}
