//=======================================================================
// LSCCollider2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using FixedMathSharp;
using FixedMathSharp.Bounds;
using Gravitas.Materials;
using Gravitas.Support;
using GridForge.Spatial;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

internal readonly struct ColliderLifetimeToken2D
{
    internal ColliderLifetimeToken2D(LSCollider2D collider)
    {
        Collider = collider;
        LifetimeVersion = collider.LifetimeVersion;
    }

    internal LSCollider2D Collider { get; }

    internal long LifetimeVersion { get; }

    internal bool IsActive => Collider.IsActive && IsCurrentLifetime;

    internal bool IsCurrentLifetime => Collider.LifetimeVersion == LifetimeVersion;
}

/// <summary>
/// Base type for pure 2D collider shapes.
/// </summary>
public abstract partial class LSCollider2D : IRecordable, IColliderHierarchyNode, IPhysicsColliderRegistryItem
{
    private SolidBody2D? _body;
    private IMatterAgent? _agent;
    private GravitasWorldContext? _context;
    private LSCompoundCollider2D? _compoundOwner;
    private Fixed64 _compoundLocalRotation;
    private Vector2d _compoundLocalScale = Vector2d.One;
    private int _id = -1;
    private int _serviceIndex = -1;
    private int _replayOrder = -1;
    private int _replayOrdinal = -1;
    private long _lifetimeVersion;
    private int _serviceRefreshIndex = -1;
    private bool _isActive = true;
    private bool _isTrigger;
    private PhysicsLayer _layer = new();
    private PhysicsLayerMask _ignoredCollisionLayers = PhysicsLayerMask.None;
    private PhysicsMaterial _material = PhysicsMaterial.Default;
    private Vector2d _localOffset;
    private FixedBoundArea _bounds;
    private FixedBoundBox _mixedBounds3D;
    private Fixed64? _mixedHalfThicknessOverride;
    private Fixed64 _mixedHalfThickness;
    private Fixed64 _mixedSlabCenterY;
    private bool _mixedBoundsInitialized;
    private uint _shapeVersion;
    private readonly ColliderRuntimeShapeState<ColliderShapeSnapshot2D> _runtimeShapeState = new();
    private ColliderPartitionState2D _partitionState;
    private ColliderPartitionState _mixedPartitionState;
    private ColliderQueryState _queryState;
    private ColliderPairState<CollisionPair2D> _pairState;
    private ColliderHierarchyState _hierarchyState;

    public delegate void Body2DCollisionFunc(SolidBody2D other);
    public delegate void Trigger2DCollisionFunc(LSCollider2D other);
    public delegate void Mixed2DCollisionFunc(LSCollider other);

    /// <summary>
    /// Raised while this collider is touching another collider that owns a 2D body.
    /// </summary>
    public event Body2DCollisionFunc? OnContact;

    /// <summary>
    /// Raised on the first simulation frame this non-trigger collider touches another 2D body.
    /// </summary>
    public event Body2DCollisionFunc? OnContactEnter;

    /// <summary>
    /// Raised when this collider stops touching another 2D body.
    /// </summary>
    public event Body2DCollisionFunc? OnContactExit;

    /// <summary>
    /// Raised on the first simulation frame this collider participates in a valid trigger pair.
    /// </summary>
    public event Trigger2DCollisionFunc? OnTriggerEnter;

    /// <summary>
    /// Raised each simulation frame this collider participates in an overlapped valid trigger pair.
    /// </summary>
    public event Trigger2DCollisionFunc? OnTriggerStay;

    /// <summary>
    /// Raised when this collider stops participating in a valid trigger pair.
    /// </summary>
    public event Trigger2DCollisionFunc? OnTriggerExit;

    public event Mixed2DCollisionFunc? OnMixedContact;
    public event Mixed2DCollisionFunc? OnMixedContactEnter;
    public event Mixed2DCollisionFunc? OnMixedContactExit;

    /// <summary>
    /// Raised on the first mixed 2D/3D simulation frame this collider participates in a valid trigger pair.
    /// </summary>
    public event Mixed2DCollisionFunc? OnMixedTriggerEnter;

    /// <summary>
    /// Raised each mixed 2D/3D simulation frame this collider participates in an overlapped valid trigger pair.
    /// </summary>
    public event Mixed2DCollisionFunc? OnMixedTriggerStay;

    /// <summary>
    /// Raised when this collider stops participating in a valid mixed 2D/3D trigger pair.
    /// </summary>
    public event Mixed2DCollisionFunc? OnMixedTriggerExit;

    public int Id => _id;

    internal int ReplayOrdinal => _replayOrdinal;

    internal long LifetimeVersion => _lifetimeVersion;

    internal int ServiceRefreshIndex => _serviceRefreshIndex;

    internal bool IsPartitioned => _partitionState.IsPartitioned;

    internal SwiftList<WorldVoxelIndex>? PartitionCoordinates => _partitionState.Coordinates;

    internal int PartitionKind => _partitionState.LastPartitionKind;

    internal bool IsMixedPartitioned => _mixedPartitionState.IsPartitioned;

    internal SwiftList<WorldVoxelIndex>? MixedPartitionCoordinates => _mixedPartitionState.Coordinates;

    internal int MixedPartitionKind => _mixedPartitionState.LastPartitionKind;

    internal uint BroadPhaseVersion => _partitionState.BroadPhaseVersion;

    internal uint RuntimeShapeVersion => _runtimeShapeState.RuntimeVersion;

    internal bool HasHostBinding => _agent != null;

    internal int CollisionPairCount => _pairState.CollisionPairCount;

    internal int CollisionPairHolderCount => _pairState.CollisionPairHolderCount;

    internal SwiftDictionary<int, CollisionPair2D>? CollisionPairs => _pairState.CollisionPairs;

    internal SwiftHashSet<int>? CollisionPairHolders => _pairState.CollisionPairHolders;

    public SolidBody2D? Body => _body;

    /// <summary>
    /// Gets whether this collider is static-style for partition mobility.
    /// </summary>
    public bool IsStatic => _body == null || _body.DynamicId < 0 || _body.IsPositionFullyFrozen;

    internal bool RequiresServiceSideRefresh => _body == null || _body.DynamicId < 0;

    public IMatterAgent Agent
    {
        get
        {
            SwiftThrowHelper.ThrowIfTrue(
                _agent == null,
                nameof(LSCollider2D),
                "2D collider is not bound to an IMatterAgent.");
            return _agent!;
        }
    }

    internal IMatterAgent? AgentOrNull => _agent;

    public GravitasWorldContext Context
    {
        get
        {
            SwiftThrowHelper.ThrowIfTrue(
                _context == null,
                nameof(LSCollider2D),
                "2D collider is not bound to a GravitasWorldContext.");
            return _context!;
        }
    }

    public bool IsActive
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _isActive;
        set
        {
            ThrowIfCompoundPartLifecycle(nameof(IsActive));

            if (_isActive == value)
                return;

            _isActive = value;
            if (_context == null || _id < 0)
                return;

            if (_isActive)
            {
                _context.Collisions2D.RefreshColliderPartition(this);
                if (_context.Settings.RuntimeMode.RunsMixedContacts())
                    _context.MixedCollisions.Refresh2DColliderPartition(this);
            }
            else
            {
                _context.Collisions2D.ClearPartitionedCollider(this, force: true);
                if (IsMixedPartitioned)
                    _context.MixedCollisions.ClearPartitioned2DCollider(this, force: true);
            }
        }
    }

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

    /// <summary>
    /// Gets or sets the single physics layer used by 2D collision matrix and query-mask filtering.
    /// </summary>
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

    /// <summary>
    /// Gets or sets the deterministic surface material used by pure 2D and
    /// mixed collision response for this collider.
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

    public abstract ColliderType2D Shape { get; }

    public virtual int Priority => ColliderSettings2D.GetPriority(Shape);

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

    public LSCollider2D? Parent2D => _hierarchyState.Parent as LSCollider2D;

    public LSCollider? Parent3D => _hierarchyState.Parent as LSCollider;

    internal LSCollider2D? TopParent2D => _hierarchyState.TopParent as LSCollider2D;

    internal LSCollider? TopParent3D => _hierarchyState.TopParent as LSCollider;

    internal int HierarchyChildCount => _hierarchyState.ChildCount;

    internal ColliderHierarchyKey HierarchyKey => Id >= 0 ? ColliderHierarchyKey.Create2D(Id) : ColliderHierarchyKey.None;

    internal ColliderHierarchyKey ParentKey => _hierarchyState.ParentKey;

    internal ColliderHierarchyKey TopParentKey => _hierarchyState.TopParentKey;

    internal ColliderHierarchyState HierarchyState => _hierarchyState;

    ColliderHierarchyKey IColliderHierarchyNode.HierarchyKey => HierarchyKey;

    IColliderHierarchyNode? IColliderHierarchyNode.HierarchyParent => _hierarchyState.Parent;

    public Vector2d LocalOffset
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _localOffset;
        set
        {
            if (_localOffset == value)
                return;

            _localOffset = value;
            MarkShapeDirty();
        }
    }

    public Vector2d Position => _compoundOwner != null
        ? _compoundOwner.Center
        : ResolveStandalonePosition();

    public Fixed64 Rotation => _compoundOwner != null
        ? _compoundOwner.Rotation + _compoundLocalRotation
        : ResolveStandaloneRotation();

    public virtual Vector2d LocalScale => _compoundOwner != null
        ? Vector2d.Multiply(_compoundOwner.LocalScale, _compoundLocalScale)
        : _agent?.Transform.LossyScale.ToVector2d() ?? Vector2d.One;

    public Vector2d ScaledLocalOffset => Vector2d.Multiply(_localOffset, LocalScale);

    public Vector2d Center => Position + Rotate(ScaledLocalOffset, Rotation);

    public FixedBoundArea Bounds => _bounds;

    internal FixedBoundBox MixedBounds3D => _mixedBounds3D;

    internal Fixed64 MixedHalfThickness => _mixedHalfThickness;

    internal Fixed64 MixedSlabCenterY => _mixedSlabCenterY;

    /// <summary>
    /// Gets or sets the optional half-thickness used when this 2D collider is embedded into mixed 2D/3D contacts.
    /// </summary>
    public Fixed64? MixedHalfThicknessOverride
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _mixedHalfThicknessOverride;
        set
        {
            if (value.HasValue)
            {
                SwiftThrowHelper.ThrowIfArgument(
                    value.Value <= Fixed64.Zero,
                    nameof(value),
                    "2D mixed half-thickness override must be greater than zero.");
            }

            if (_mixedHalfThicknessOverride == value)
                return;

            _mixedHalfThicknessOverride = value;
            MarkShapeDirty();
        }
    }

    public Fixed64 MinX => _bounds.Min.X;

    public Fixed64 MaxX => _bounds.Max.X;

    public Fixed64 MinY => _bounds.Min.Y;

    public Fixed64 MaxY => _bounds.Max.Y;

    internal void Initialize(SolidBody2D body)
    {
        PreflightBodyInitialization(body);
        InitCore(body.Agent, body);
    }

    public void InitializeWithNoBody(IMatterAgent agent)
    {
        ThrowIfCompoundPartLifecycle(nameof(InitializeWithNoBody));
        SwiftThrowHelper.ThrowIfNull(agent, nameof(agent));
        SwiftThrowHelper.ThrowIfArgument(
            Id >= 0 || (HasHostBinding && !ReferenceEquals(_agent, agent)),
            nameof(agent),
            "2D collider is already registered or bound to another host agent.");
        PreflightInitialization(agent);
        InitCore(agent, null);
        Context.Physics2D.AssimilateCollider(this);
    }

    internal void PreflightBodyInitialization(SolidBody2D body)
    {
        ThrowIfCompoundPartLifecycle(nameof(Initialize));
        SwiftThrowHelper.ThrowIfNull(body, nameof(body));
        ThrowIfTriggerWouldAttachToBody(nameof(Initialize));
        PreflightInitialization(body.Agent);
    }

    private void PreflightInitialization(IMatterAgent agent)
    {
        SwiftThrowHelper.ThrowIfNull(agent, nameof(agent));
        ColliderScalePolicy.ValidatePlanar(agent.Transform);
        OnBeforeInitialize(agent);
    }

    private void InitCore(IMatterAgent agent, SolidBody2D? body)
    {
        _lifetimeVersion++;
        _body = body;
        _agent = agent;
        _context = agent.Context;
        _isActive = true;
        _queryState.Reset();
        _hierarchyState.Initialize(agent.IsParent);
        _runtimeShapeState.MarkDirty();
        RebuildRuntimeShapeState();
    }

    protected virtual void OnBeforeInitialize(IMatterAgent agent) { }

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

    internal void ClearBindingState()
    {
        _body = null;
        _agent = null;
        _context = null;
    }

    public void Deactivate()
    {
        ThrowIfCompoundPartLifecycle(nameof(Deactivate));

        if (_body != null)
        {
            _body.Deactivate();
            return;
        }

        if (_id >= 0)
            _context!.Physics2D.DessimilateCollider(this);

        _isActive = false;
        ClearBindingState();
    }

    public void Simulate()
    {
        ThrowIfCompoundPartLifecycle(nameof(Simulate));

        if (!IsActive)
            return;

        Rebuild();
    }

    internal bool Rebuild()
    {
        if (!RebuildRuntimeShapeState())
            return false;

        if (_id >= 0)
            return _context!.Collisions2D.RefreshColliderPartitionAfterShapeChange(this);

        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool RebuildRuntimeShapeOnly() => RebuildRuntimeShapeState();

    public void SetParent(LSCollider2D parent)
    {
        ThrowIfCompoundPartLifecycle(nameof(SetParent));
        _hierarchyState.SetParent(this, parent);
    }

    public void SetParent(LSCollider parent)
    {
        ThrowIfCompoundPartLifecycle(nameof(SetParent));
        _hierarchyState.SetParent(this, parent);
    }

    public void ClearParent()
    {
        ThrowIfCompoundPartLifecycle(nameof(ClearParent));
        _hierarchyState.ClearParent(this);
    }

    public bool IsSibling(LSCollider2D other) =>
        _hierarchyState.ExcludesCollisionWith(other._hierarchyState, HierarchyKey, other.HierarchyKey);

    internal bool ExcludesMixedCollisionWith(LSCollider other) =>
        _hierarchyState.ExcludesCollisionWith(other.HierarchyState, HierarchyKey, other.HierarchyKey);

    internal SwiftList<WorldVoxelIndex> GetOrCreatePartitionCoordinates()
    {
        _partitionState.Coordinates ??= new();
        return _partitionState.Coordinates;
    }

    internal SwiftList<WorldVoxelIndex> GetOrCreateMixedPartitionCoordinates()
    {
        _mixedPartitionState.Coordinates ??= new();
        return _mixedPartitionState.Coordinates;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool MatchesPartitionGridBounds(Vector2d min, Vector2d max, int partitionKind) =>
        _partitionState.MatchesGridBounds(min, max, partitionKind);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool MatchesMixedPartitionGridBounds(Vector3d min, Vector3d max, int partitionKind) =>
        _mixedPartitionState.IsPartitioned
        && _mixedPartitionState.LastGridBoundsMin == min
        && _mixedPartitionState.LastGridBoundsMax == max
        && _mixedPartitionState.LastPartitionKind == partitionKind;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void MarkPartitioned(Vector2d min, Vector2d max, int partitionKind)
    {
        _partitionState.SetPreviousGridBounds(min, max, partitionKind);
        _partitionState.MarkPartitioned();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void MarkMixedPartitioned(Vector3d min, Vector3d max, int partitionKind)
    {
        _mixedPartitionState.SetPreviousGridBounds(min, max, partitionKind);
        _mixedPartitionState.MarkPartitioned();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void MarkUnpartitioned() => _partitionState.MarkUnpartitioned();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void MarkMixedUnpartitioned() => _mixedPartitionState.MarkUnpartitioned();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void ClearPartitionCoordinates() => _partitionState.ClearCoordinates();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void ClearMixedPartitionCoordinates() => _mixedPartitionState.ClearCoordinates();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool IsPositionInPlanarBounds(Fixed64 cellEdge, Vector3d worldPosition)
    {
        Fixed64 padding = cellEdge * Fixed64.Half;
        return worldPosition.X >= MinX - padding
            && worldPosition.X <= MaxX + padding
            && worldPosition.Z >= MinY - padding
            && worldPosition.Z <= MaxY + padding;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool IsPositionInMixedBounds(Fixed64 cellEdge, Vector3d worldPosition)
    {
        Fixed64 padding = cellEdge * Fixed64.Half;
        return worldPosition.X >= _mixedBounds3D.Min.X - padding
            && worldPosition.X <= _mixedBounds3D.Max.X + padding
            && worldPosition.Y >= _mixedBounds3D.Min.Y - padding
            && worldPosition.Y <= _mixedBounds3D.Max.Y + padding
            && worldPosition.Z >= _mixedBounds3D.Min.Z - padding
            && worldPosition.Z <= _mixedBounds3D.Max.Z + padding;
    }

    internal bool TryGetCollisionPair(int otherId, out CollisionPair2D? collisionPair) =>
        _pairState.TryGetCollisionPair(otherId, out collisionPair);

    internal bool TryAddCollisionPair(int otherId, CollisionPair2D collisionPair) =>
        _pairState.TryAddCollisionPair(otherId, collisionPair);

    internal bool TryRemoveCollisionPair(int otherId, out CollisionPair2D? collisionPair) =>
        _pairState.TryRemoveCollisionPair(otherId, out collisionPair);

    internal bool TryAddCollisionPairHolder(int otherId) => _pairState.TryAddCollisionPairHolder(otherId);

    internal bool TryRemoveCollisionPairHolder(int otherId) => _pairState.TryRemoveCollisionPairHolder(otherId);

    internal void ClearCollisionPairState()
    {
        _pairState.ClearCollisionPairs();
        _pairState.ClearCollisionPairHolders();
    }

    internal void ClearRuntimeRelationships()
    {
        ClearChildParentReferences();
        ClearParent();
    }

    internal void NotifyContact(LSCollider2D other, bool isColliding, bool isChanged) =>
        NotifyContact(
            other,
            other.Body,
            isColliding,
            isChanged,
            allowInactive: false,
            new ColliderLifetimeToken2D(this),
            new ColliderLifetimeToken2D(other),
            IsTrigger || other.IsTrigger,
            ColliderTriggerEventPolicy.ShouldRaise(this, other));

    internal void NotifyContact(
        LSCollider2D other,
        SolidBody2D? otherBody,
        bool isColliding,
        bool isChanged,
        bool allowInactive,
        in ColliderLifetimeToken2D registration,
        in ColliderLifetimeToken2D otherRegistration,
        bool isTriggerPair,
        bool shouldRaiseTrigger)
    {
        if (isColliding
            ? !registration.IsActive || !otherRegistration.IsActive
            : allowInactive ? !registration.IsCurrentLifetime : !registration.IsActive)
        {
            return;
        }

        if (isColliding)
        {
            if (isTriggerPair)
            {
                if (shouldRaiseTrigger)
                {
                    if (isChanged)
                    {
                        OnTriggerEnter?.Invoke(other);
                        if (!registration.IsActive || !otherRegistration.IsActive)
                            return;
                    }

                    OnTriggerStay?.Invoke(other);
                }

                return;
            }

            if (isChanged && otherBody != null)
            {
                OnContactEnter?.Invoke(otherBody);
                if (!registration.IsActive || !otherRegistration.IsActive)
                    return;
            }

            if (otherBody != null)
                OnContact?.Invoke(otherBody);
            return;
        }

        if (!isChanged)
            return;

        if (isTriggerPair)
        {
            if (shouldRaiseTrigger)
                OnTriggerExit?.Invoke(other);

            return;
        }

        if (otherBody != null)
            OnContactExit?.Invoke(otherBody);
    }

    internal void NotifyMixedContact(LSCollider other, bool isColliding, bool isChanged, bool isTriggerPair) =>
        NotifyMixedContact(
            other,
            isColliding,
            isChanged,
            isTriggerPair,
            allowInactive: false,
            new ColliderLifetimeToken2D(this),
            new ColliderLifetimeToken(other),
            ColliderTriggerEventPolicy.ShouldRaise(other, this));

    internal void NotifyMixedContact(
        LSCollider other,
        bool isColliding,
        bool isChanged,
        bool isTriggerPair,
        bool allowInactive,
        in ColliderLifetimeToken2D registration,
        in ColliderLifetimeToken otherRegistration,
        bool shouldRaiseTrigger)
    {
        if (isColliding
            ? !registration.IsActive || !otherRegistration.IsActive
            : allowInactive
                ? !registration.IsCurrentLifetime || !otherRegistration.IsCurrentLifetime
                : !registration.IsActive)
        {
            return;
        }

        if (isColliding)
        {
            if (isTriggerPair)
            {
                if (shouldRaiseTrigger)
                {
                    if (isChanged)
                    {
                        OnMixedTriggerEnter?.Invoke(other);
                        if (!registration.IsActive || !otherRegistration.IsActive)
                            return;
                    }

                    OnMixedTriggerStay?.Invoke(other);
                }

                return;
            }

            if (isChanged)
            {
                OnMixedContactEnter?.Invoke(other);
                if (!registration.IsActive || !otherRegistration.IsActive)
                    return;
            }

            OnMixedContact?.Invoke(other);
            return;
        }

        if (!isChanged)
            return;

        if (isTriggerPair)
        {
            if (shouldRaiseTrigger)
                OnMixedTriggerExit?.Invoke(other);

            return;
        }

        OnMixedContactExit?.Invoke(other);
    }

    public abstract bool ContainsPoint(Vector2d point);

    public abstract Vector2d GetClosestPoint(Vector2d point);

    public abstract Vector2d GetSupportPoint(Vector2d direction);

    internal int VertexCount
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => this is IConvexVertexSource2D source ? source.VertexCount : 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal Vector2d GetVertexUnchecked(int index) =>
        ((IConvexVertexSource2D)this).GetVertexUnchecked(index);

    /// <summary>
    /// Calculates the body-local center of mass implied by this 2D collider's current shape state.
    /// </summary>
    public virtual Vector2d CalculateLocalCenterOfMassOffset() =>
        TransformMassPropertyPoint(ScaledLocalOffset);

    /// <summary>
    /// Calculates the scalar moment of inertia about a requested body-local reference point.
    /// </summary>
    public abstract Fixed64 CalculateMomentOfInertia(Fixed64 mass, Vector2d localReferencePoint);

    internal abstract Fixed64 CalculateAreaForMassProperties();

    protected abstract void RebuildShape();

    protected virtual void OnMaterialChanged() { }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    protected void MarkShapeDirty()
    {
        _shapeVersion++;
        _runtimeShapeState.MarkDirty();
        if (_compoundOwner != null)
        {
            _compoundOwner.MarkShapeDirty();
            return;
        }

        _body?.Wake();
        _body?.RefreshMassPropertiesFromColliderShape();
    }

    protected static Fixed64 ApplyParallelAxis(
        Fixed64 momentAboutCenterOfMass,
        Fixed64 mass,
        Vector2d centerOfMass,
        Vector2d localReferencePoint)
    {
        Vector2d delta = localReferencePoint - centerOfMass;
        return momentAboutCenterOfMass + mass * delta.MagnitudeSquared;
    }

    private bool RebuildRuntimeShapeState()
    {
        ColliderShapeSnapshot2D snapshot = CaptureShapeSnapshot();
        if (!_runtimeShapeState.ShouldRebuild(snapshot))
            return false;

        RebuildShape();
        RebuildMixedEmbedding(snapshot.MixedSlabCenterY, snapshot.MixedHalfThickness);
        _runtimeShapeState.Commit(snapshot);
        return true;
    }

    private ColliderShapeSnapshot2D CaptureShapeSnapshot()
    {
        if (_compoundOwner != null)
        {
            Fixed64 rotation = _compoundOwner.Rotation + _compoundLocalRotation;
            Vector2d localScale = Vector2d.Multiply(_compoundOwner.LocalScale, _compoundLocalScale);
            ColliderScalePolicy.Validate(localScale);
            Vector2d center = _compoundOwner.Center + Rotate(Vector2d.Multiply(_localOffset, localScale), rotation);
            return new(
                center,
                rotation,
                localScale,
                _localOffset,
                _shapeVersion,
                ResolveMixedSlabCenterY(),
                ResolveMixedHalfThickness());
        }

        Fixed64 standaloneRotation = ResolveStandaloneRotation();
        Vector2d standaloneScale;
        if (_agent != null)
            standaloneScale = ColliderScalePolicy.ValidatePlanar(_agent.Transform);
        else
        {
            standaloneScale = LocalScale;
            ColliderScalePolicy.Validate(standaloneScale);
        }
        Vector2d standaloneCenter = ResolveStandalonePosition()
            + Rotate(Vector2d.Multiply(_localOffset, standaloneScale), standaloneRotation);
        return new(
            standaloneCenter,
            standaloneRotation,
            standaloneScale,
            _localOffset,
            _shapeVersion,
            ResolveMixedSlabCenterY(),
            ResolveMixedHalfThickness());
    }

    private Fixed64 ResolveMixedHalfThickness() =>
        _mixedHalfThicknessOverride ?? _context?.Settings.Mixed2DHalfThickness ?? PhysicsSettings.DefaultMixed2DHalfThickness;

    private Fixed64 ResolveMixedSlabCenterY() =>
        _agent?.Transform.WorldPosition.Y ?? Fixed64.Zero;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Vector2d ResolveStandalonePosition() =>
        _body?.Position
        ?? _agent?.Transform.WorldPositionXZ
        ?? Vector2d.Zero;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Fixed64 ResolveStandaloneRotation() =>
        _body?.Rotation ?? ResolveAgentRotation();

    private void RebuildMixedEmbedding(Fixed64 slabCenterY, Fixed64 halfThickness)
    {
        _mixedHalfThickness = halfThickness;
        _mixedSlabCenterY = slabCenterY;

        Vector3d min = new(MinX, slabCenterY - halfThickness, MinY);
        Vector3d max = new(MaxX, slabCenterY + halfThickness, MaxY);
        if (!_mixedBoundsInitialized)
        {
            _mixedBounds3D = FixedBoundBox.FromMinMax(min, max);
            _mixedBoundsInitialized = true;
            return;
        }

        _mixedBounds3D.SetMinMax(min, max);
    }

    private void ClearChildParentReferences()
    {
        SwiftHashSet<ulong>? children = _hierarchyState.Children;
        if (children == null || _context == null)
            return;

        foreach (ulong childPackedKey in children)
        {
            ColliderHierarchyKey childKey = ColliderHierarchyKey.FromPacked(childPackedKey);
            if (((IColliderHierarchyNode)this).TryGetHierarchyColliderByKey(childKey, out IColliderHierarchyNode? child))
                child!.ClearParentReference();
        }

        _hierarchyState.ClearChildren();
    }

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

        if (key.Is2D && _context.Physics2D.TryGetColliderById(key.Id, out LSCollider2D? collider2D))
        {
            collider = collider2D;
            return true;
        }

        if (key.Is3D && _context.Physics.TryGetColliderById(key.Id, out LSCollider? collider3D))
        {
            collider = collider3D;
            return true;
        }

        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    protected void SetBounds(FixedBoundArea bounds) => _bounds = bounds;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    protected void SetBoundsFromMinMax(Vector2d min, Vector2d max) =>
        SetBounds(FixedBoundArea.FromMinMax(min, max));

    private Fixed64 ResolveAgentRotation()
    {
        return _agent == null
            ? Fixed64.Zero
            : _agent.Transform.WorldRotationXZRadians;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void BindContext(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        SwiftThrowHelper.ThrowIfArgument(
            _context != null && !ReferenceEquals(_context, context),
            nameof(context),
            "2D collider is already bound to a different GravitasWorldContext.");
        _context = context;
    }

    internal void BindCompoundPart(
        LSCompoundCollider2D owner,
        Fixed64 localRotation,
        Vector2d localScale,
        GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(owner, nameof(owner));
        SwiftThrowHelper.ThrowIfArgument(
            HasHostBinding,
            nameof(owner),
            "2D compound collider parts cannot be initialized as standalone colliders.");
        _compoundOwner = owner;
        _compoundLocalRotation = localRotation;
        _compoundLocalScale = localScale;
        BindContext(context);
        RebuildRuntimeShapeState();
    }

    internal void ReserveCompoundPart(
        LSCompoundCollider2D owner,
        Fixed64 localRotation,
        Vector2d localScale)
    {
        SwiftThrowHelper.ThrowIfNull(owner, nameof(owner));
        SwiftThrowHelper.ThrowIfArgument(
            HasHostBinding,
            nameof(owner),
            "2D compound collider parts cannot be initialized as standalone colliders.");
        SwiftThrowHelper.ThrowIfArgument(
            _compoundOwner != null && !ReferenceEquals(_compoundOwner, owner),
            nameof(owner),
            "2D compound collider part is already owned by another compound collider.");

        _compoundOwner = owner;
        _compoundLocalRotation = localRotation;
        _compoundLocalScale = localScale;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void ThrowIfCompoundPartLifecycle(string operation)
    {
        SwiftThrowHelper.ThrowIfTrue(
            _compoundOwner != null,
            operation,
            "2D compound collider parts are geometry owned by LSCompoundCollider2D and cannot run standalone lifecycle operations.");
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    protected Vector2d TransformMassPropertyPoint(Vector2d localPoint) =>
        _compoundOwner == null
            ? localPoint
            : _compoundOwner.ScaledLocalOffset + Rotate(localPoint, _compoundLocalRotation);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    protected static Vector2d Rotate(Vector2d value, Fixed64 radians)
    {
        if (radians == Fixed64.Zero)
            return value;

        return ClampNearZero(Vector2d.Rotate(value, radians));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    protected static Vector2d ClampNearZero(Vector2d value)
    {
        Fixed64 x = value.X.Abs() <= Fixed64.Epsilon ? Fixed64.Zero : value.X;
        Fixed64 y = value.Y.Abs() <= Fixed64.Epsilon ? Fixed64.Zero : value.Y;
        return new Vector2d(x, y);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    protected static Fixed64 ClampAxis(Fixed64 value, Fixed64 min, Fixed64 max) =>
        value < min ? min : value > max ? max : value;

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
            "2D trigger colliders must be initialized without a SolidBody2D. Use InitializeWithNoBody for trigger volumes.");
        SwiftThrowHelper.ThrowIfArgument(
            _compoundOwner != null,
            operation,
            "2D compound collider parts are not trigger identities. Set IsTrigger on the owning compound collider.");
    }

    private void ThrowIfTriggerWouldAttachToBody(string operation)
    {
        SwiftThrowHelper.ThrowIfArgument(
            _isTrigger,
            operation,
            "2D trigger colliders must be initialized without a SolidBody2D. Use InitializeWithNoBody for trigger volumes.");
    }

    private void ThrowIfLoadedTriggerHasBody(string operation)
    {
        SwiftThrowHelper.ThrowIfArgument(
            _isTrigger && _body != null,
            operation,
            "Loaded 2D trigger state is invalid for a collider attached to a SolidBody2D.");
    }

    public void RecordData(IChronicler chronicler)
    {
        RecordValues.Look(chronicler, ref _isActive, "Active", true);
        RecordValues.Look(chronicler, ref _isTrigger, "IsTrigger", false);
        RecordValues.Look(chronicler, ref _layer, "Layer", new());
        RecordValues.Look(chronicler, ref _ignoredCollisionLayers, "IgnoredCollisionLayers", PhysicsLayerMask.None);
        RecordValues.Look(chronicler, ref _material, "Material", PhysicsMaterial.Default);
        RecordValues.Look(chronicler, ref _localOffset, "LocalOffset", Vector2d.Zero);
        RecordValues.Look(chronicler, ref _mixedHalfThicknessOverride, "MixedHalfThicknessOverride");
        RecordShapeData(chronicler);

        if (chronicler.Mode == SerializationMode.Loading)
        {
            ThrowIfLoadedTriggerHasBody(nameof(IsTrigger));
            ApplyLoadedState();
        }
    }

    protected virtual void RecordShapeData(IChronicler chronicler) { }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool IgnoresCollisionLayer(PhysicsLayer layer) => _ignoredCollisionLayers.Includes(layer);

    private void ApplyLoadedState()
    {
        _runtimeShapeState.MarkDirty();
        if (_context == null)
            return;

        RebuildRuntimeShapeState();

        if (_id < 0)
            return;

        if (!_isActive)
        {
            if (IsPartitioned)
                _context.Collisions2D.ClearPartitionedCollider(this, force: true);
            MarkUnpartitioned();
            ClearPartitionCoordinates();

            if (IsMixedPartitioned)
                _context.MixedCollisions.ClearPartitioned2DCollider(this, force: true);
            MarkMixedUnpartitioned();
            ClearMixedPartitionCoordinates();
            return;
        }

        _context.Collisions2D.RefreshColliderPartition(this);
        if (_context.Settings.RuntimeMode.RunsMixedContacts())
            _context.MixedCollisions.Refresh2DColliderPartition(this);
    }
}
