//=======================================================================
// LSCollider.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using FixedMathSharp;
using FixedMathSharp.Bounds;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Queries;
using Gravitas.Support;
using GridForge.Grids;
using GridForge.Spatial;
using SwiftCollections;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

public abstract partial class LSCollider : IRecordable, IColliderHierarchyNode, IPhysicsColliderRegistryItem
{
    #region Fields and Properties

    protected bool _debug;
    protected bool _drawShape;
    private bool _drawPartitions;
    private bool _drawBoundingBox;

    private bool _active = true;
    private bool _deactivationInProgress;
    public bool IsActive
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _active;
        set
        {
            ThrowIfCompoundPartLifecycle(nameof(IsActive));

            if (_active == value)
                return;

            _active = value;
            if (_context == null || _id < 0)
                return;

            if (_active)
            {
                InitialPartition();
                if (_context.Settings.RuntimeMode.RunsMixedContacts())
                    _context.MixedCollisions.Refresh3DColliderPartition(this);
                return;
            }

            if (IsPartitioned)
                _context.Collisions.ClearPartitionedObject(this, force: true);

            if (IsMixedPartitioned)
            {
                _context.MixedCollisions.ClearPartitioned3DCollider(this, force: true);
                MarkMixedUnpartitioned();
                ClearMixedPartitionCoordinates();
            }
        }
    }

    private bool _isTrigger;

    /// <summary>
    /// Gets or sets whether this bodyless collider is a trigger volume.
    /// Trigger volumes raise trigger enter/stay/exit callbacks for valid overlap
    /// pairs and never apply physical response.
    /// </summary>
    public bool IsTrigger
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _isTrigger;
        set => SetTrigger(value);
    }

    private int _id = -1;
    public int Id => _id;

    private int _serviceIndex = -1;

    private int _replayOrder = -1;

    private int _replayOrdinal = -1;
    internal int ReplayOrdinal => _replayOrdinal;

    private long _lifetimeVersion;
    internal long LifetimeVersion => _lifetimeVersion;

    internal bool IsDeactivationInProgress => _deactivationInProgress;

    private int _serviceRefreshIndex = -1;
    internal int ServiceRefreshIndex => _serviceRefreshIndex;

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

    private SolidBody? _body;
    public SolidBody? Body => _body;

    /// <summary>
    /// Gets whether this collider is static-style for partition mobility.
    /// </summary>
    public bool IsStatic => _body == null || _body.DynamicId < 0 || _body.IsPositionFullyFrozen;

    internal bool RequiresServiceSideRefresh => _body == null || _body.DynamicId < 0;

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

    internal bool HasHostBinding => _agent != null;

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
            SwiftThrowHelper.ThrowIfTrue(
                _agent == null,
                nameof(Position),
                "Collider is not bound to a static transform.");

            if (_agent.Transform.Position == value)
                return;
            _agent.Transform.Position = value;
        }
    }

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
            SwiftThrowHelper.ThrowIfTrue(
                _agent == null,
                nameof(Rotation),
                "Collider is not bound to a static transform.");

            if (_agent.Transform.Rotation == value)
                return;
            _agent.Transform.Rotation = value;
        }
    }

    // For dynamic colliders, this is the velocity of the body. For static colliders, this is always zero.
    public Vector3d Velocity => Body?.LinearVelocity ?? Vector3d.Zero;

    public GridWorld World => Context.World;

    public FixedTransform Transform => _compoundOwner?.Transform
        ?? Body?.PositionTransform
        ?? _agent?.Transform
        ?? throw new InvalidOperationException("Collider has no body or static transform.");

    private PhysicsLayer _layer = new();
    private PhysicsLayerMask _ignoredCollisionLayers = PhysicsLayerMask.None;

    public PhysicsLayer Layer
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _layer;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => _layer = value;
    }

    /// <summary>
    /// Gets or sets physical layers this collider ignores for collider-to-collider
    /// interactions. Public queries continue to use the caller's query mask.
    /// </summary>
    public PhysicsLayerMask IgnoredCollisionLayers
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _ignoredCollisionLayers;
        set
        {
            if (_ignoredCollisionLayers == value)
                return;

            _ignoredCollisionLayers = value;
            _body?.Wake();
        }
    }

    private PhysicsMaterial _material = PhysicsMaterial.Default;

    /// <summary>
    /// Gets or sets the deterministic surface material used by collision
    /// response for this collider.
    /// </summary>
    public PhysicsMaterial Material
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _material;
        set
        {
            if (_material == value)
                return;

            _material = value;
            OnMaterialChanged();
            _body?.Wake();
        }
    }

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
    public virtual Fixed64 ScaledRadius => _radius * FixedMath.Max(LocalScale.Z, FixedMath.Max(LocalScale.X, LocalScale.Y));

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
        ? Vector3d.Multiply(_compoundOwner.LocalScale, _compoundLocalScale)
        : Transform.LossyScale;

    public virtual Vector3d ScaledSize => Vector3d.Multiply(_size, LocalScale);

    public Vector3d ScaledOffset => Vector3d.Multiply(_offset, LocalScale);

    /// <summary>
    /// Bodies position in world space + collider offset (center) value
    /// </summary>
    public Vector3d Center => Position + (Rotation * ScaledOffset);

    protected FixedBoundBox _bounds;
    private bool _boundsInitialized;
    public FixedBoundBox Bounds => _bounds;
    public Vector3d BoundsMin => _bounds.Min;
    public Vector3d BoundsMax => _bounds.Max;

    internal SwiftList<WorldVoxelIndex>? PartitionCoordinates => _partitionState.Coordinates;

    internal int PartitionKind => _partitionState.LastPartitionKind;

    internal bool IsMixedPartitioned => _mixedPartitionState.IsPartitioned;

    internal SwiftList<WorldVoxelIndex>? MixedPartitionCoordinates => _mixedPartitionState.Coordinates;

    internal int MixedPartitionKind => _mixedPartitionState.LastPartitionKind;

    #endregion

    internal uint RaycastVersion
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _queryState.RaycastVersion;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => _queryState.RaycastVersion = value;
    }

    internal uint CircleQueryVersion
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _queryState.CircleQueryVersion;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => _queryState.CircleQueryVersion = value;
    }

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

    internal void Initialize(SolidBody body)
    {
        ThrowIfCompoundPartLifecycle(nameof(Initialize));
        ThrowIfTriggerWouldAttachToBody(nameof(Initialize));
        _body = body;
        InitCore(body.Agent);
    }

    public void InitializeWithNoBody(IMatterAgent agent)
    {
        ThrowIfCompoundPartLifecycle(nameof(InitializeWithNoBody));
        SwiftThrowHelper.ThrowIfNull(agent, nameof(agent));
        SwiftThrowHelper.ThrowIfArgument(
            Id >= 0 || (HasHostBinding && !ReferenceEquals(_agent, agent)),
            nameof(agent),
            "Collider is already registered or bound to another host agent.");
        InitCore(agent);
    }

    private void InitCore(IMatterAgent agent)
    {
        SwiftThrowHelper.ThrowIfNull(agent, nameof(agent));
        _lifetimeVersion++;
        OnBeforeInitialize(agent);

        _queryState.Reset();

        _agent = agent;
        _active = true;
        _deactivationInProgress = false;
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
        if (!IsActive)
            return;

        _partitionState.Coordinates ??= new();
        SwiftList<WorldVoxelIndex> partitionCoordinates = _partitionState.Coordinates!;
        Context.Collisions.PartitionObject(this, ref partitionCoordinates);
        _partitionState.Coordinates = partitionCoordinates;
        SetPreviousGridBounds();
        _partitionState.MarkPartitioned();
        MarkBroadPhaseChanged();
    }

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

        if (RebuildRuntimeShapeState() || Context.Collisions.IsPartitionRefreshRequired(this))
            UpdatePartition();
    }

    private void UpdatePartition()
    {
        MarkBroadPhaseChanged();

        if (!Context.Collisions.ClearPartitionedObject(this))
            return;

        SwiftList<WorldVoxelIndex> partitionCoordinates = _partitionState.Coordinates!;
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

    private bool RebuildRuntimeShapeState(bool refreshMassProperties = true)
    {
        ColliderShapeSnapshot snapshot = CaptureShapeSnapshot();
        if (!_runtimeShapeState.ShouldRebuild(snapshot))
            return false;

        RebuildRuntimeShape();
        _runtimeShapeState.Commit(snapshot);
        if (refreshMassProperties)
            _body?.RefreshMassPropertiesFromColliderShape();
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool RebuildRuntimeShapeOnly(bool refreshMassProperties = true) =>
        RebuildRuntimeShapeState(refreshMassProperties);

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
            _bounds = FixedBoundBox.FromCenterAndSize(Center, ScaledSize);
            _boundsInitialized = true;
        }
        else
        {
            _bounds.Orient(Center, ScaledSize);
        }

        CalculateBoundLimits();
    }

    protected void SetBounds(FixedBoundBox bounds)
    {
        _bounds = bounds;
        _boundsInitialized = true;
    }

    protected void SetBoundsMinMax(Vector3d min, Vector3d max)
    {
        if (!_boundsInitialized)
        {
            _bounds = FixedBoundBox.FromMinMax(min, max);
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
        Span<Vector3d> vertices = stackalloc Vector3d[FixedBoundBox.CornerCount];
        _bounds.CopyCorners(vertices);
        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3d orientedVertex = vertices[i].Rotate(_bounds.Center, Rotation);
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

    protected virtual void OnMaterialChanged() { }

    protected virtual Vector3d NormalizeSize(Vector3d value) => value;

    private static void ValidateSize(Vector3d value)
    {
        SwiftThrowHelper.ThrowIfArgument(
            value.X <= Fixed64.Zero || value.Y <= Fixed64.Zero || value.Z <= Fixed64.Zero,
            nameof(value),
            "Collider size components must be greater than zero.");
    }

    private void SetTrigger(bool value)
    {
        if (_isTrigger == value)
            return;

        if (value)
            ThrowIfCannotEnableTrigger(nameof(IsTrigger));

        _isTrigger = value;
    }

    private void ThrowIfCannotEnableTrigger(string operation)
    {
        SwiftThrowHelper.ThrowIfArgument(
            _body != null,
            operation,
            "Trigger colliders must be initialized without a SolidBody. Use InitializeWithNoBody for trigger volumes.");
        SwiftThrowHelper.ThrowIfArgument(
            _compoundOwner != null,
            operation,
            "Compound collider parts are not trigger identities. Set IsTrigger on the owning compound collider.");
    }

    private void ThrowIfTriggerWouldAttachToBody(string operation)
    {
        SwiftThrowHelper.ThrowIfArgument(
            _isTrigger,
            operation,
            "Trigger colliders must be initialized without a SolidBody. Use InitializeWithNoBody for trigger volumes.");
    }

    private void ThrowIfLoadedTriggerHasBody(string operation)
    {
        SwiftThrowHelper.ThrowIfArgument(
            _isTrigger && _body != null,
            operation,
            "Loaded trigger state is invalid for a collider attached to a SolidBody.");
    }

    // default to total area for shapes where frontal area doesn't make sense
    public virtual Fixed64 GetFrontalArea(Vector3d direction) => Area;

    /// <summary>
    /// Calculates the body-local center of mass offset implied by this collider's current shape state.
    /// </summary>
    public virtual Vector3d CalculateLocalCenterOfMassOffset() => ScaledOffset;

    public abstract Fixed3x3 CalculateInertiaTensor(Fixed64 mass, Vector3d localCenterOfMassOffset);

    protected Fixed3x3 ShiftInertiaTensorFromLocalCenterOfMass(
        Fixed3x3 centerTensor,
        Fixed64 mass,
        Vector3d targetLocalOffset) =>
        AddParallelAxisTensor(centerTensor, mass, targetLocalOffset - CalculateLocalCenterOfMassOffset());

    protected static Fixed3x3 AddParallelAxisTensor(Fixed3x3 tensor, Fixed64 mass, Vector3d offset) =>
        InertiaTensorMath.AddParallelAxisTensor(tensor, mass, offset);

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
    /// <param name="cellPadding">The topology cell edge used to conservatively pad the bounds.</param>
    /// <param name="position">The position to check</param>
    /// <returns>True if position within bounds</returns>
    public bool IsPositionInBounds(Fixed64 cellPadding, Vector3d position)
    {
        return position.X + cellPadding >= BoundsMin.X
            && position.X - cellPadding <= BoundsMax.X
            && position.Y + cellPadding >= BoundsMin.Y
            && position.Y - cellPadding <= BoundsMax.Y
            && position.Z + cellPadding >= BoundsMin.Z
            && position.Z - cellPadding <= BoundsMax.Z;
    }

    /// <summary>
    /// Checks if this object overlaps the line formed by p1 and p2
    /// </summary>
    /// <param name="worker">The prepared ray-axis worker for the owning query service.</param>
    /// <param name="outputIntersectionPoints"></param>
    /// <returns></returns>
    public abstract bool ColliderOverlapsRay(RaycastSegmentWorker worker, ref SwiftList<Vector3d> outputIntersectionPoints);

    void IColliderHierarchyNode.AddChild(ColliderHierarchyKey key)
    {
        ThrowIfCompoundPartLifecycle(nameof(IColliderHierarchyNode.AddChild));
        _hierarchyState.AddChild(key);
    }

    void IColliderHierarchyNode.RemoveChild(ColliderHierarchyKey key)
    {
        ThrowIfCompoundPartLifecycle(nameof(IColliderHierarchyNode.RemoveChild));
        _hierarchyState.RemoveChild(key);
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
    internal void SetPhysicsState(int id, int serviceIndex, int replayOrder)
    {
        SwiftThrowHelper.ThrowIfNegative(id, nameof(id));
        SwiftThrowHelper.ThrowIfNegative(serviceIndex, nameof(serviceIndex));
        SwiftThrowHelper.ThrowIfNegative(replayOrder, nameof(replayOrder));
        _id = id;
        _serviceIndex = serviceIndex;
        _replayOrder = replayOrder;
        _replayOrdinal = -1;
    }

    internal void SetServiceIndex(int serviceIndex)
    {
        SwiftThrowHelper.ThrowIfNegative(serviceIndex, nameof(serviceIndex));
        _serviceIndex = serviceIndex;
    }

    internal void SetReplayOrdinal(int replayOrdinal)
    {
        SwiftThrowHelper.ThrowIfNegative(replayOrdinal, nameof(replayOrdinal));
        _replayOrdinal = replayOrdinal;
    }

    internal void SetServiceRefreshIndex(int serviceRefreshIndex)
    {
        SwiftThrowHelper.ThrowIfNegative(serviceRefreshIndex, nameof(serviceRefreshIndex));
        _serviceRefreshIndex = serviceRefreshIndex;
    }

    internal void ClearServiceRefreshIndex()
    {
        _serviceRefreshIndex = -1;
    }

    internal void ClearPhysicsState()
    {
        _partitionState.MarkUnpartitioned();
        _partitionState.ClearCoordinates();
        _mixedPartitionState.MarkUnpartitioned();
        _mixedPartitionState.ClearCoordinates();
        _pairState.ClearCollisionPairs();
        _pairState.ClearCollisionPairHolders();
        _id = -1;
        _serviceIndex = -1;
        _replayOrder = -1;
        _replayOrdinal = -1;
        _serviceRefreshIndex = -1;
    }

    void IPhysicsColliderRegistryItem.SetRegistryState(int id, int serviceIndex, int replayOrder) =>
        SetPhysicsState(id, serviceIndex, replayOrder);

    int IPhysicsColliderRegistryItem.ServiceIndex => _serviceIndex;

    int IPhysicsColliderRegistryItem.ReplayOrder => _replayOrder;

    void IPhysicsColliderRegistryItem.SetRegistryServiceIndex(int serviceIndex) =>
        SetServiceIndex(serviceIndex);

    void IPhysicsColliderRegistryItem.SetRegistryReplayOrdinal(int replayOrdinal) =>
        SetReplayOrdinal(replayOrdinal);

    void IPhysicsColliderRegistryItem.ClearRegistryState() => ClearPhysicsState();

    internal SwiftList<WorldVoxelIndex> GetOrCreateMixedPartitionCoordinates()
    {
        _mixedPartitionState.Coordinates ??= new();
        return _mixedPartitionState.Coordinates;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool MatchesPartitionGridBounds(Vector3d min, Vector3d max, int partitionKind) =>
        _partitionState.IsPartitioned
        && _partitionState.LastGridBoundsMin == min
        && _partitionState.LastGridBoundsMax == max
        && _partitionState.LastPartitionKind == partitionKind;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void MarkUnpartitioned() => _partitionState.MarkUnpartitioned();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void ClearPartitionCoordinates() => _partitionState.ClearCoordinates();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool MatchesMixedPartitionGridBounds(Vector3d min, Vector3d max, int partitionKind) =>
        _mixedPartitionState.IsPartitioned
        && _mixedPartitionState.LastGridBoundsMin == min
        && _mixedPartitionState.LastGridBoundsMax == max
        && _mixedPartitionState.LastPartitionKind == partitionKind;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void MarkMixedPartitioned(Vector3d min, Vector3d max, int partitionKind)
    {
        _mixedPartitionState.SetPreviousGridBounds(min, max, partitionKind);
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
        RecordValues.Look(chronicler, ref _ignoredCollisionLayers, "IgnoredCollisionLayers", PhysicsLayerMask.None);
        RecordValues.Look(chronicler, ref _material, "Material", PhysicsMaterial.Default);
        RecordValues.Look(chronicler, ref _isTrigger, "IsTrigger", false);
        RecordValues.Look(chronicler, ref _offset, "Offset", Vector3d.Zero);
        RecordValues.Look(chronicler, ref _radius, "Radius", Fixed64.Half);
        RecordValues.Look(chronicler, ref _size, "Size", Vector3d.One);

        if (chronicler.Mode == SerializationMode.Loading)
        {
            ThrowIfLoadedTriggerHasBody(nameof(IsTrigger));
            ApplyLoadedState();
        }
        else
        {
            _runtimeShapeState.MarkDirty();
        }
    }

    private void ApplyLoadedState()
    {
        _runtimeShapeState.MarkDirty();
        if (_context == null)
            return;

        RebuildRuntimeShapeState();

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
        else
            InitialPartition();
        if (_context.Settings.RuntimeMode.RunsMixedContacts())
            _context.MixedCollisions.Refresh3DColliderPartition(this);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool IgnoresCollisionLayer(PhysicsLayer layer) => _ignoredCollisionLayers.Includes(layer);

    #endregion
}
