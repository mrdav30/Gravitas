using Chronicler;
using FixedMathSharp;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using Gravitas.Support;
using GridForge.Grids;
using GridForge.Spatial;
using SwiftCollections;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

public abstract class LSCollider : IRecordable, IColliderHierarchyNode
{
    #region Fields and Properties

    protected bool _debug;
    protected bool _drawShape;
    private bool _drawPartitions;
    private bool _drawBoundingBox;

    private bool _active = true;
    public bool IsActive => _active;

    private bool _isTrigger;
    public bool IsTrigger => _isTrigger;

    private int _id;
    public int Id => _id;

    private IMatterAgent? _agent;
    internal IMatterAgent? AgentOrNull => _agent;

    private GravitasWorldContext? _context;
    public GravitasWorldContext Context
    {
        get
        {
            SwiftThrowHelper.ThrowIfTrue(
                _context == null,
                nameof(LSCollider),
                "Collider is not bound to a GravitasWorldContext.");
            return _context!;
        }
    }

    private StiffBody? _body;
    public StiffBody? Body => _body;

    private LSCompoundCollider? _compoundOwner;
    private FixedQuaternion _compoundLocalRotation = FixedQuaternion.Identity;
    private Vector3d _compoundLocalScale = Vector3d.One;

    private readonly ColliderRuntimeShapeState<ColliderShapeSnapshot> _runtimeShapeState = new();
    private ColliderPartitionState _partitionState;
    private ColliderPartitionState _mixedPartitionState;
    private ColliderQueryState _queryState;
    private ColliderPairState<CollisionPair> _pairState;
    private ColliderHierarchyState _hierarchyState;

    internal uint RuntimeShapeVersion => _runtimeShapeState.RuntimeVersion;

    internal bool HasHostBinding => _body != null || _agent != null;

    internal LSCompoundCollider? CompoundOwner => _compoundOwner;

    public virtual Vector3d Position
    {
        get => _compoundOwner?.Position
            ?? Body?.Position3d
            ?? _agent?.Transform.Position
            ?? throw new InvalidOperationException("Collider has no body or static transform.");
        set
        {
            SwiftThrowHelper.ThrowIfTrue(
                _compoundOwner != null,
                nameof(Position),
                "Compound collider parts inherit position from their owning compound collider.");

            if (_agent == null || _agent.Transform.Position == value)
                return;
            _agent.Transform.Position = value;
        }
    }

    public Fixed64 HeightPos => _compoundOwner?.HeightPos
        ?? Body?.HeightPos
        ?? _agent?.Transform.Position.y
        ?? throw new InvalidOperationException("Collider has no body or static transform.");

    public virtual FixedQuaternion Rotation
    {
        get => _compoundOwner != null
            ? _compoundOwner.Rotation * _compoundLocalRotation
            : Body?.Rotation
            ?? _agent?.Transform.Rotation
            ?? throw new InvalidOperationException("Collider has no body or static transform.");
        set
        {
            SwiftThrowHelper.ThrowIfTrue(
                _compoundOwner != null,
                nameof(Rotation),
                "Compound collider parts inherit rotation from their owning compound collider.");

            if (_agent == null || _agent.Transform.Rotation == value)
                return;
            _agent.Transform.Rotation = value;
        }
    }

    // For dynamic colliders, this is the velocity of the body. For static colliders, this is always zero.
    public Vector3d Velocity => Body?.LinearVelocity ?? Vector3d.Zero;

    public GridWorld? World => _context?.World ?? _agent?.Context.World;

    public FixedTransform Transform => _compoundOwner?.Transform
        ?? Body?.PositionTransform
        ?? _agent?.Transform
        ?? throw new InvalidOperationException("Collider has no body or static transform.");

    private PhysicsLayer _layer = new();
    public PhysicsLayer Layer
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _layer;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => _layer = value;
    }

    /// <summary>
    /// Used to prevent distance culling for very large objects.
    /// </summary>
    /// <remarks>
    /// Useful for fast-moving objects that might pass through if not checked for a frame.
    /// When enabled, the collider will not be culled based on distance for the first frame after being added to a new partition.
    /// </remarks>
    private bool _preventCulling = false;
    internal bool PreventCulling => _preventCulling;

    public bool IsPartitioned => _partitionState.IsPartitioned;

    /// <summary>
    /// Used for preventing culling for the first frame this object is added to a new partition node.
    /// </summary>
    public bool PartitionChanged
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _partitionState.PartitionChanged;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => _partitionState.PartitionChanged = value;
    }

    internal uint BroadPhaseVersion => _partitionState.BroadPhaseVersion;

    /// <summary>
    /// Center of collider in local space, used for calculating bounds and offsets. Should be set in the Setup method of each collider type.
    /// </summary>
    protected Vector3d _offset;

    /// <summary>
    /// Gets or sets the unscaled local center offset used by bounds and shape-derived state.
    /// </summary>
    public Vector3d LocalOffset
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _offset;
        set
        {
            if (_offset == value)
                return;

            _offset = value;
            MarkShapeDirty();
        }
    }

    public abstract ColliderType Shape { get; }
    public abstract int Priority { get; }

    protected Fixed64 _radius = Fixed64.Half;

    /// <summary>
    /// Gets or sets the unscaled shape radius used by radius-based colliders.
    /// </summary>
    public Fixed64 Radius
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _radius;
        set
        {
            SwiftThrowHelper.ThrowIfArgument(
                value <= Fixed64.Zero,
                nameof(value),
                "Collider radius must be greater than zero.");

            if (_radius == value)
                return;

            _radius = value;
            OnRadiusChanged();
            MarkShapeDirty();
        }
    }

    /// <summary>
    /// Gets the bounding circle radius.
    /// For boxes, this is half of the diagonal length of the cube
    /// </summary>
    /// <value>The radius.</value>
    public virtual Fixed64 ScaledRadius => _radius * FixedMath.Max(LocalScale.z, FixedMath.Max(LocalScale.x, LocalScale.y));

    public Fixed64 ScaledRadiusSqr => ScaledRadius * ScaledRadius;

    protected Vector3d _size = Vector3d.One;

    /// <summary>
    /// Gets or sets the unscaled local size used when rebuilding collider bounds.
    /// </summary>
    public Vector3d Size
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _size;
        set
        {
            ValidateSize(value);
            Vector3d normalizedSize = NormalizeSize(value);
            if (_size == normalizedSize)
                return;

            _size = normalizedSize;
            MarkShapeDirty();
        }
    }

    public virtual Fixed64 Area { get; protected set; } = Fixed64.Zero;

    #region Grid & Partition Bounds

    public virtual Vector3d LocalScale => _compoundOwner != null
        ? Vector3d.Scale(_compoundOwner.LocalScale, _compoundLocalScale)
        : Transform.LossyScale;

    public virtual Vector3d ScaledSize => Vector3d.Scale(_size, LocalScale);

    public Vector3d ScaledOffset => Vector3d.Scale(_offset, LocalScale);

    /// <summary>
    /// Bodies position in world space + collider offset (center) value
    /// </summary>
    public Vector3d Center => Position + (Rotation * ScaledOffset);

    protected BoundingBox _bounds;
    private bool _boundsInitialized;
    public BoundingBox Bounds => _bounds;
    public Vector3d BoundsMin => _bounds.Min;
    public Vector3d BoundsMax => _bounds.Max;

    public Vector3d LastGridBoundsMin => _partitionState.LastGridBoundsMin;

    public Vector3d LastGridBoundsMax => _partitionState.LastGridBoundsMax;

    public SwiftList<WorldVoxelIndex>? PartitionCoordinates
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _partitionState.Coordinates;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => _partitionState.Coordinates = value;
    }

    internal bool IsMixedPartitioned => _mixedPartitionState.IsPartitioned;

    internal SwiftList<WorldVoxelIndex>? MixedPartitionCoordinates => _mixedPartitionState.Coordinates;

    #endregion

    public uint RaycastVersion
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _queryState.RaycastVersion;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => _queryState.RaycastVersion = value;
    }

    public uint CircleQueryVersion
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _queryState.CircleQueryVersion;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => _queryState.CircleQueryVersion = value;
    }

    internal int CollisionPairCount => _pairState.CollisionPairCount;

    internal int CollisionPairHolderCount => _pairState.CollisionPairHolderCount;

    public delegate void BodyCollisionFunc(StiffBody other);
    public event BodyCollisionFunc? OnContact;
    public event BodyCollisionFunc? OnContactEnter;
    public event BodyCollisionFunc? OnContactExit;

    public delegate void TriggerCollisionFunc(LSCollider other);
    public event TriggerCollisionFunc? OnTriggerEnter;
    public event TriggerCollisionFunc? OnTriggerExit;

    public delegate void MixedCollisionFunc(LSCollider2D other);
    public event MixedCollisionFunc? OnMixedContact;
    public event MixedCollisionFunc? OnMixedContactEnter;
    public event MixedCollisionFunc? OnMixedContactExit;
    public event MixedCollisionFunc? OnMixedTriggerEnter;
    public event MixedCollisionFunc? OnMixedTriggerExit;

    public bool IsChild => _hierarchyState.IsChild;
    public bool IsParent => _hierarchyState.IsParent;
    public int ParentId => ParentKey.Id;
    public LSCollider? Parent3D => _hierarchyState.Parent as LSCollider;
    public LSCollider2D? Parent2D => _hierarchyState.Parent as LSCollider2D;
    internal LSCollider? TopParent3D => _hierarchyState.TopParent as LSCollider;
    internal LSCollider2D? TopParent2D => _hierarchyState.TopParent as LSCollider2D;
    internal int HierarchyChildCount => _hierarchyState.ChildCount;
    internal ColliderHierarchyKey HierarchyKey => Id >= 0 ? ColliderHierarchyKey.Create3D(Id) : ColliderHierarchyKey.None;
    internal ColliderHierarchyKey ParentKey => _hierarchyState.ParentKey;
    internal ColliderHierarchyKey TopParentKey => _hierarchyState.TopParentKey;
    internal ColliderHierarchyState HierarchyState => _hierarchyState;
    ColliderHierarchyKey IColliderHierarchyNode.HierarchyKey => HierarchyKey;
    IColliderHierarchyNode? IColliderHierarchyNode.HierarchyParent => _hierarchyState.Parent;

    #endregion

    public void Initialize(StiffBody body)
    {
        ThrowIfCompoundPartLifecycle(nameof(Initialize));
        _body = body;
        InitCore(body.Agent);
    }

    public void InitializeWithNoBody(IMatterAgent agent)
    {
        ThrowIfCompoundPartLifecycle(nameof(InitializeWithNoBody));
        InitCore(agent);
    }

    private void InitCore(IMatterAgent agent)
    {
        SwiftThrowHelper.ThrowIfNull(agent, nameof(agent));
        OnBeforeInitialize(agent);

        _queryState.Reset();

        _agent = agent;
        _active = true;
        BindContext(agent.Context);
        Context.Physics.AssimilateCollider(this);
        _hierarchyState.Initialize(_agent.IsParent);

        OnInitialize();

        _partitionState.SetPreviousGridBounds(Vector3d.Zero, Vector3d.Zero);

        InitialPartition();
    }

    protected virtual void OnBeforeInitialize(IMatterAgent agent) { }

    protected virtual void OnInitialize()
    {
        RebuildRuntimeShapeState();
    }

    private void InitialPartition()
    {
        RebuildRuntimeShapeState();

        if (IsActive)
        {
            _partitionState.Coordinates ??= new();
            SwiftList<WorldVoxelIndex>? partitionCoordinates = _partitionState.Coordinates;
            Context.Collisions.PartitionObject(this, ref partitionCoordinates);
            _partitionState.Coordinates = partitionCoordinates;
            SetPreviousGridBounds();
            _partitionState.MarkPartitioned();
            MarkBroadPhaseChanged();
        }
    }

    public void LateInitialize() { }

    // Dynamic Colliders attached to a body will be updated by the body
    // Static Colliders need to be updated by whatever is updating the static collider
    // Even if the collider is inactive, if the body is active, the collider will be updated
    public void Simulate()
    {
        ThrowIfCompoundPartLifecycle(nameof(Simulate));
        PartitionChanged = false;
        if (!IsActive)
            return;

        if (!IsPartitioned)
        {
            InitialPartition();
            return;
        }

        if (RebuildRuntimeShapeState())
            UpdatePartition();
    }

    private void UpdatePartition()
    {
        MarkBroadPhaseChanged();

        if (!Context.Collisions.ClearPartitionedObject(this))
            return;

        _partitionState.MarkUnpartitioned();

        _partitionState.Coordinates ??= new();
        SwiftList<WorldVoxelIndex>? partitionCoordinates = _partitionState.Coordinates;
        bool partitioned = Context.Collisions.PartitionObject(this, ref partitionCoordinates);
        _partitionState.Coordinates = partitionCoordinates;
        if (!partitioned)
            return;

        SetPreviousGridBounds();

        _partitionState.MarkPartitioned();
    }

    public void SetParent(LSCollider parent)
    {
        ThrowIfCompoundPartLifecycle(nameof(SetParent));
        _hierarchyState.SetParent(this, parent);
    }

    public void SetParent(LSCollider2D parent)
    {
        ThrowIfCompoundPartLifecycle(nameof(SetParent));
        _hierarchyState.SetParent(this, parent);
    }

    public void ClearParent()
    {
        ThrowIfCompoundPartLifecycle(nameof(ClearParent));
        _hierarchyState.ClearParent(this);
    }

    public bool IsSibling(LSCollider other)
    {
        return _hierarchyState.ExcludesCollisionWith(other._hierarchyState, HierarchyKey, other.HierarchyKey);
    }

    internal bool ExcludesMixedCollisionWith(LSCollider2D other) =>
        _hierarchyState.ExcludesCollisionWith(other.HierarchyState, HierarchyKey, other.HierarchyKey);

    private bool RebuildRuntimeShapeState()
    {
        ColliderShapeSnapshot snapshot = CaptureShapeSnapshot();
        if (!_runtimeShapeState.ShouldRebuild(snapshot))
            return false;

        RebuildRuntimeShape();
        _runtimeShapeState.Commit(snapshot);
        return true;
    }

    protected virtual void RebuildRuntimeShape()
    {
        BuildBoundingBox();
        BuildShape();
    }

    private ColliderShapeSnapshot CaptureShapeSnapshot() =>
        new(Center, Rotation, LocalScale, _offset, _size, _radius);

    protected virtual void BuildBoundingBox()
    {
        if (!_boundsInitialized)
        {
            _bounds = new BoundingBox(Center, ScaledSize);
            _boundsInitialized = true;
        }
        else
        {
            _bounds.Orient(Center, ScaledSize);
        }

        CalculateBoundLimits();
    }

    protected void SetBounds(BoundingBox bounds)
    {
        _bounds = bounds;
        _boundsInitialized = true;
    }

    protected void SetBoundsMinMax(Vector3d min, Vector3d max)
    {
        if (!_boundsInitialized)
        {
            _bounds = new BoundingBox((min + max) * Fixed64.Half, max - min);
            _boundsInitialized = true;
            return;
        }

        _bounds.SetMinMax(min, max);
    }

    private void CalculateBoundLimits()
    {
        if (Rotation == FixedQuaternion.Identity || Shape == ColliderType.Mesh)
            return;

        // Calculate the axis-aligned bounding box (AABB) of the OBB
        Vector3d min = _bounds.Center;
        Vector3d max = _bounds.Center;
        for (int i = 0; i < _bounds.Vertices.Length; i++)
        {
            Vector3d orientedVertex = _bounds.Vertices[i].Rotate(_bounds.Center, Rotation);
            min = Vector3d.Min(min, orientedVertex);
            max = Vector3d.Max(max, orientedVertex);
        }

        _bounds.SetMinMax(min, max);
    }

    protected abstract void BuildShape();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    protected void MarkShapeDirty()
    {
        _runtimeShapeState.MarkDirty();
        if (_compoundOwner != null)
        {
            _compoundOwner.MarkShapeDirty();
            return;
        }

        _body?.Wake();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void MarkBroadPhaseChanged() => _partitionState.MarkBroadPhaseChanged();

    protected virtual void OnRadiusChanged() { }

    protected virtual Vector3d NormalizeSize(Vector3d value) => value;

    private static void ValidateSize(Vector3d value)
    {
        SwiftThrowHelper.ThrowIfArgument(
            value.x <= Fixed64.Zero || value.y <= Fixed64.Zero || value.z <= Fixed64.Zero,
            nameof(value),
            "Collider size components must be greater than zero.");
    }

    // default to total area for shapes where frontal area doesn't make sense
    public virtual Fixed64 GetFrontalArea(Vector3d direction) => Area;

    // we're considering the inertial tensor for these shapes to be diagonal matrices with the principal moments of inertia along the diagonal.
    // The non-diagonal elements of the inertial tensor matrix are zero for these symmetrical shapes,
    // under the assumption that the center of mass is the reference point and the coordinate axes are principal axes of inertia.
    public abstract Fixed3x3 CalculateInertiaTensor(Fixed64 mass);

    internal void NotifyContact(LSCollider other, bool isColliding, bool isChanged)
    {
        if (!IsActive)
            return;

        if (isColliding)
        {
            // Only called once per collision
            if (isChanged)
            {
                if (IsTrigger)
                    OnTriggerEnter?.Invoke(other);
                else if (other.Body != null)
                    OnContactEnter?.Invoke(other.Body);
            }

            if (other.Body != null)
                OnContact?.Invoke(other.Body);

            return;
        }

        if (!isChanged)
            return;

        if (IsTrigger)
            OnTriggerExit?.Invoke(other);
        if (other.Body != null)
            OnContactExit?.Invoke(other.Body);
    }

    internal void NotifyMixedContact(LSCollider2D other, bool isColliding, bool isChanged, bool isTriggerPair)
    {
        if (!IsActive)
            return;

        if (isColliding)
        {
            if (isTriggerPair)
            {
                if (isChanged && IsTrigger)
                    OnMixedTriggerEnter?.Invoke(other);

                return;
            }

            if (isChanged)
                OnMixedContactEnter?.Invoke(other);

            OnMixedContact?.Invoke(other);
            return;
        }

        if (!isChanged)
            return;

        if (isTriggerPair)
        {
            if (IsTrigger)
                OnMixedTriggerExit?.Invoke(other);

            return;
        }

        OnMixedContactExit?.Invoke(other);
    }

    public void SetStatus(bool status)
    {
        ThrowIfCompoundPartLifecycle(nameof(SetStatus));
        _active = status;
    }

    /// <summary>
    /// The point on the surface of the capsule that's nearest to the given point
    /// </summary>
    /// <param name="other"></param>
    /// <returns></returns>
    public abstract Vector3d ClosestPointOnSurface(Vector3d other);

    /// <summary>
    /// The direction pointing outward from the surface of the collider at the point
    /// </summary>
    /// <param name="point"></param>
    /// <returns></returns>
    public abstract Vector3d GetNormalAtPoint(Vector3d point);

    /// <summary>
    /// Checks if position is in the padded bounds of the collider, used for broad phase collision detection
    /// </summary>
    /// <param name="voxelSize">The size of the voxel to pad the bounds</param>
    /// <param name="position">The position to check</param>
    /// <returns>True if position within bounds</returns>
    public bool IsPositionInBounds(Fixed64 voxelSize, Vector3d position)
    {
        return position.x + voxelSize >= BoundsMin.x
            && position.x - voxelSize <= BoundsMax.x
            && position.y + voxelSize >= BoundsMin.y
            && position.y - voxelSize <= BoundsMax.y
            && position.z + voxelSize >= BoundsMin.z
            && position.z - voxelSize <= BoundsMax.z;
    }

    /// <summary>
    /// Checks if this object overlaps the line formed by p1 and p2
    /// </summary>
    /// <param name="worker">The prepared ray-axis worker for the owning query service.</param>
    /// <param name="outputIntersectionPoints"></param>
    /// <returns></returns>
    public abstract bool ColliderOverlapsRay(RaycastSegmentWorker worker, ref SwiftList<Vector3d> outputIntersectionPoints);

    public void SetPreviousGridBounds()
    {
        GridWorld? world = World;
        if (world == null)
        {
            _partitionState.SetPreviousGridBounds(Vector3d.Zero, Vector3d.Zero);
            return;
        }

        (Vector3d min, Vector3d max) = world.SnapBoundsToVoxelSize(BoundsMin, BoundsMax, Fixed64.Half);
        _partitionState.SetPreviousGridBounds(min, max);
    }

    internal bool TryGetCollisionPair(int otherId, out CollisionPair? collisionPair) =>
        _pairState.TryGetCollisionPair(otherId, out collisionPair);

    internal bool TryAddCollisionPair(int otherId, CollisionPair collisionPair)
    {
        if (_pairState.TryAddCollisionPair(otherId, collisionPair) != true)
        {
            GravitasLogger.Channel.Warn($"Collision pair with collider ID {otherId} already exists.");
            return false;
        }
        return true;
    }

    internal bool TryRemoveCollisionPair(int otherId)
    {
        if (!_pairState.TryRemoveCollisionPair(otherId, out CollisionPair? collisionPair))
            return false;

        if (collisionPair != null && collisionPair.Active)
            Context.Physics.DeactivateAndPoolPair(collisionPair);
        return true;
    }

    internal bool TryAddCollisionPairHolder(int otherId) => _pairState.TryAddCollisionPairHolder(otherId);

    internal bool TryRemoveCollisionPairHolder(int otherId) => _pairState.TryRemoveCollisionPairHolder(otherId);

    public void Deactivate()
    {
        ThrowIfCompoundPartLifecycle(nameof(Deactivate));
        if (IsMixedPartitioned)
            Context.MixedCollisions.ClearPartitioned3DCollider(this, force: true);

        if (IsPartitioned)
        {
            Context.Collisions.ClearPartitionedObject(this, true);
            _partitionState.ClearCoordinates();
            _partitionState.MarkUnpartitioned();
        }

        // Remove all collision pairs involving this collider
        SwiftDictionary<int, CollisionPair>? collisionPairs = _pairState.CollisionPairs;
        if (collisionPairs != null)
        {
            foreach (var kvp in collisionPairs)
            {
                int otherId = kvp.Key;
                CollisionPair collisionPair = kvp.Value;
                if (!Context.Physics.TryGetColliderById(otherId, out LSCollider? other))
                    continue;
                other!.TryRemoveCollisionPairHolder(Id);
                // Remove the pair regardless of whether the other collider has already removed it,
                // to ensure it's cleaned up properly and to avoid potential issues with colliders
                // that might still reference this collider in their pairs.
                Context.Physics.DeactivateAndPoolPair(collisionPair);
            }
        }
        _pairState.ClearCollisionPairs();

        // Remove this collider from the collision pair holders of all colliders it has pairs with.
        SwiftHashSet<int>? collisionPairHolders = _pairState.CollisionPairHolders;
        if (collisionPairHolders != null)
        {
            foreach (int holderId in collisionPairHolders)
            {
                if (!Context.Physics.TryGetColliderById(holderId, out LSCollider? other))
                    continue;

                if (other!.TryRemoveCollisionPair(Id) != true)
                    GravitasLogger.DebugChannel.Info($"Collider with ID {Id} was not found in the collision pairs of collider with ID {holderId} during deactivation. This may indicate that the pair was already removed or that there is an inconsistency in the collision management.");
            }
        }
        _pairState.ClearCollisionPairHolders();
        ClearChildParentReferences();
        ClearParent();

        Context.Physics.DessimilateCollider(this);
        //  IsInCollision = false;
        _active = false;
    }

    private void ClearChildParentReferences()
    {
        SwiftHashSet<ulong>? children = _hierarchyState.Children;
        if (children == null)
            return;

        foreach (ulong childPackedKey in children)
        {
            ColliderHierarchyKey childKey = ColliderHierarchyKey.FromPacked(childPackedKey);
            if (((IColliderHierarchyNode)this).TryGetHierarchyColliderByKey(childKey, out IColliderHierarchyNode? child))
                child!.ClearParentReference();
        }

        _hierarchyState.ClearChildren();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void BindContext(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        SwiftThrowHelper.ThrowIfArgument(
            _context != null && !ReferenceEquals(_context, context),
            nameof(context),
            "Collider is already bound to a different GravitasWorldContext.");
        _context = context;
    }

    internal void BindCompoundPart(
        LSCompoundCollider owner,
        FixedQuaternion localRotation,
        Vector3d localScale,
        GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(owner, nameof(owner));
        SwiftThrowHelper.ThrowIfArgument(
            HasHostBinding,
            nameof(owner),
            "Compound collider parts cannot be initialized as standalone colliders.");
        SwiftThrowHelper.ThrowIfArgument(
            _compoundOwner != null && !ReferenceEquals(_compoundOwner, owner),
            nameof(owner),
            "Compound collider part is already owned by another compound collider.");

        _compoundOwner = owner;
        _compoundLocalRotation = localRotation;
        _compoundLocalScale = localScale;
        BindContext(context);
        RebuildRuntimeShapeState();
    }

    internal void ReserveCompoundPart(LSCompoundCollider owner)
    {
        SwiftThrowHelper.ThrowIfNull(owner, nameof(owner));
        SwiftThrowHelper.ThrowIfArgument(
            HasHostBinding,
            nameof(owner),
            "Compound collider parts cannot be initialized as standalone colliders.");
        SwiftThrowHelper.ThrowIfArgument(
            _compoundOwner != null && !ReferenceEquals(_compoundOwner, owner),
            nameof(owner),
            "Compound collider part is already owned by another compound collider.");

        _compoundOwner = owner;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void ThrowIfCompoundPartLifecycle(string operation)
    {
        SwiftThrowHelper.ThrowIfTrue(
            _compoundOwner != null,
            operation,
            "Compound collider parts are geometry owned by LSCompoundCollider and cannot run standalone lifecycle operations.");
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool TryGetBoundContext(out GravitasWorldContext? context)
    {
        context = _context;
        return context != null;
    }

    void IColliderHierarchyNode.AddChild(ColliderHierarchyKey key)
    {
        ThrowIfCompoundPartLifecycle(nameof(IColliderHierarchyNode.AddChild));
        if (_hierarchyState.AddChild(key) != true)
            GravitasLogger.Channel.Warn($"Collider hierarchy key {key.Packed} is already a child.");
    }

    void IColliderHierarchyNode.RemoveChild(ColliderHierarchyKey key)
    {
        ThrowIfCompoundPartLifecycle(nameof(IColliderHierarchyNode.RemoveChild));
        if (_hierarchyState.RemoveChild(key) != true)
            GravitasLogger.Channel.Warn($"Cannot remove. Collider hierarchy key {key.Packed} is not a child.");
    }

    void IColliderHierarchyNode.ClearParentReference() => _hierarchyState.ClearParentReference();

    bool IColliderHierarchyNode.TryGetHierarchyColliderByKey(ColliderHierarchyKey key, out IColliderHierarchyNode? collider)
    {
        collider = null;
        if (!key.IsValid || _context == null)
            return false;

        if (key.Is3D && _context.Physics.TryGetColliderById(key.Id, out LSCollider? collider3D))
        {
            collider = collider3D;
            return true;
        }

        if (key.Is2D && _context.Physics2D.TryGetColliderById(key.Id, out LSCollider2D? collider2D))
        {
            collider = collider2D;
            return true;
        }

        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void SetPhysicsId(int id)
    {
        SwiftThrowHelper.ThrowIfNegative(id, nameof(id));
        _id = id;
    }

    internal SwiftList<WorldVoxelIndex> GetOrCreateMixedPartitionCoordinates()
    {
        _mixedPartitionState.Coordinates ??= new();
        return _mixedPartitionState.Coordinates;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool MatchesMixedPartitionGridBounds(Vector3d min, Vector3d max) =>
        _mixedPartitionState.IsPartitioned
        && _mixedPartitionState.LastGridBoundsMin == min
        && _mixedPartitionState.LastGridBoundsMax == max;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void MarkMixedPartitioned(Vector3d min, Vector3d max)
    {
        _mixedPartitionState.SetPreviousGridBounds(min, max);
        _mixedPartitionState.MarkPartitioned();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void MarkMixedUnpartitioned() => _mixedPartitionState.MarkUnpartitioned();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void ClearMixedPartitionCoordinates() => _mixedPartitionState.ClearCoordinates();

    #region Serialization

    public void RecordData(IChronicler chronicler)
    {
        RecordValues.Look(chronicler, ref _debug, "Debug", false);
        RecordValues.Look(chronicler, ref _drawShape, "DrawShape", false);
        RecordValues.Look(chronicler, ref _drawPartitions, "DrawPartitions", false);
        RecordValues.Look(chronicler, ref _drawBoundingBox, "DrawBoundingBox", false);
        RecordValues.Look(chronicler, ref _active, "Active", true);
        RecordValues.Look(chronicler, ref _layer, "Layer", new());
        RecordValues.Look(chronicler, ref _isTrigger, "IsTrigger", false);
        RecordValues.Look(chronicler, ref _preventCulling, "PreventCulling", false);
        RecordValues.Look(chronicler, ref _offset, "Offset", Vector3d.Zero);
        RecordValues.Look(chronicler, ref _radius, "Radius", Fixed64.Half);
        RecordValues.Look(chronicler, ref _size, "Size", Vector3d.One);

        if (chronicler.Mode == SerializationMode.Loading)
            ApplyLoadedState();
        else
            _runtimeShapeState.MarkDirty();
    }

    private void ApplyLoadedState()
    {
        _runtimeShapeState.MarkDirty();
        RebuildRuntimeShapeState();

        if (_context == null || _compoundOwner != null)
            return;

        if (!_active)
        {
            if (IsPartitioned)
                _context.Collisions.ClearPartitionedObject(this, force: true);
            if (IsMixedPartitioned)
                _context.MixedCollisions.ClearPartitioned3DCollider(this, force: true);
            return;
        }

        if (IsPartitioned)
            UpdatePartition();
        if (_context.Settings.RuntimeMode.RunsMixedContacts())
            _context.MixedCollisions.Refresh3DColliderPartition(this);
    }

    #endregion
}
