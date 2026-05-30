using FixedMathSharp;
using GridForge.Spatial;
using Gravitas.Support;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

/// <summary>
/// Base type for pure 2D collider shapes.
/// </summary>
public abstract class LSCollider2D : IColliderHierarchyNode<LSCollider2D>
{
    private StiffBody2D? _body;
    private IMatterAgent? _agent;
    private GravitasWorldContext? _context;
    private int _id = -1;
    private int _serviceIndex = -1;
    private bool _isActive = true;
    private bool _isTrigger;
    private PhysicsLayer _layer = new();
    private Vector2d _localOffset;
    private BoundingArea _bounds;
    private BoundingBox _mixedBounds3D;
    private Fixed64? _mixedHalfThicknessOverride;
    private Fixed64 _mixedHalfThickness;
    private Fixed64 _mixedSlabCenterY;
    private bool _mixedBoundsInitialized;
    private uint _shapeVersion;
    private readonly ColliderRuntimeShapeState<ColliderShapeSnapshot2D> _runtimeShapeState = new();
    private ColliderPartitionState2D _partitionState;
    private ColliderQueryState _queryState;
    private ColliderPairState<CollisionPair2D> _pairState;
    private ColliderHierarchyState<LSCollider2D> _hierarchyState;

    public delegate void Body2DCollisionFunc(StiffBody2D other);
    public delegate void Trigger2DCollisionFunc(LSCollider2D other);

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
    /// Raised on the first simulation frame this trigger overlaps another 2D collider.
    /// </summary>
    public event Trigger2DCollisionFunc? OnTriggerEnter;

    /// <summary>
    /// Raised when this trigger stops overlapping another 2D collider.
    /// </summary>
    public event Trigger2DCollisionFunc? OnTriggerExit;

    public int Id => _id;

    internal int ServiceIndex => _serviceIndex;

    internal bool IsPartitioned => _partitionState.IsPartitioned;

    internal SwiftList<WorldVoxelIndex>? PartitionCoordinates => _partitionState.Coordinates;

    internal uint BroadPhaseVersion => _partitionState.BroadPhaseVersion;

    internal uint RuntimeShapeVersion => _runtimeShapeState.RuntimeVersion;

    internal int CollisionPairCount => _pairState.CollisionPairCount;

    internal int CollisionPairHolderCount => _pairState.CollisionPairHolderCount;

    internal SwiftDictionary<int, CollisionPair2D>? CollisionPairs => _pairState.CollisionPairs;

    internal SwiftHashSet<int>? CollisionPairHolders => _pairState.CollisionPairHolders;

    public StiffBody2D? Body => _body;

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
            if (_isActive == value)
                return;

            _isActive = value;
            if (_context == null || _id < 0)
                return;

            if (_isActive)
                _context.Collisions2D.RefreshColliderPartition(this);
            else
                _context.Collisions2D.ClearPartitionedCollider(this, force: true);
        }
    }

    public bool IsTrigger
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _isTrigger;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => _isTrigger = value;
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

    public abstract ColliderType2D Shape { get; }

    public virtual int Priority => ColliderSettings2D.GetPriority(Shape);

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

    public bool IsChild => _hierarchyState.IsChild;

    public bool IsParent => _hierarchyState.IsParent;

    public int ParentId => _hierarchyState.ParentId;

    public LSCollider2D? Parent => _hierarchyState.Parent;

    internal LSCollider2D? TopParent => _hierarchyState.TopParent;

    internal int HierarchyChildCount => _hierarchyState.ChildCount;

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

    public Vector2d Position => _body?.Position ?? _agent?.Transform.Position.ToVector2d() ?? Vector2d.Zero;

    public Fixed64 Rotation => _body?.Rotation ?? ResolveAgentRotation();

    public Vector2d Center => Position + Rotate(LocalOffset, Rotation);

    public BoundingArea Bounds => _bounds;

    internal BoundingBox MixedBounds3D => _mixedBounds3D;

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

    public Fixed64 MinX => _bounds.MinX;

    public Fixed64 MaxX => _bounds.MaxX;

    public Fixed64 MinY => _bounds.MinY;

    public Fixed64 MaxY => _bounds.MaxY;

    internal void Initialize(StiffBody2D body)
    {
        SwiftThrowHelper.ThrowIfNull(body, nameof(body));
        InitCore(body.Agent, body);
    }

    public void InitializeWithNoBody(IMatterAgent agent)
    {
        InitCore(agent, null);
        Context.Physics2D.AssimilateCollider(this);
    }

    private void InitCore(IMatterAgent agent, StiffBody2D? body)
    {
        SwiftThrowHelper.ThrowIfNull(agent, nameof(agent));
        _body = body;
        _agent = agent;
        _context = agent.Context;
        _isActive = true;
        _queryState.Reset();
        _hierarchyState.Initialize(agent.IsParent);
        _runtimeShapeState.MarkDirty();
        RebuildRuntimeShapeState();
    }

    internal void SetPhysicsState(int id, int serviceIndex)
    {
        SwiftThrowHelper.ThrowIfNegative(id, nameof(id));
        SwiftThrowHelper.ThrowIfNegative(serviceIndex, nameof(serviceIndex));
        _id = id;
        _serviceIndex = serviceIndex;
    }

    internal void SetServiceIndex(int serviceIndex)
    {
        SwiftThrowHelper.ThrowIfNegative(serviceIndex, nameof(serviceIndex));
        _serviceIndex = serviceIndex;
    }

    internal void ClearPhysicsState()
    {
        _partitionState.MarkUnpartitioned();
        _partitionState.ClearCoordinates();
        _pairState.ClearCollisionPairs();
        _pairState.ClearCollisionPairHolders();
        _id = -1;
        _serviceIndex = -1;
    }

    internal void ClearBindingState()
    {
        _body = null;
        _agent = null;
        _context = null;
    }

    public void Deactivate()
    {
        if (!_isActive)
            return;

        if (_context != null && _id >= 0)
            _context.Physics2D.DessimilateCollider(this);

        ClearChildParentReferences();
        ClearParent();
        _isActive = false;
        ClearBindingState();
    }

    public void Simulate()
    {
        if (!IsActive)
            return;

        Rebuild();
    }

    internal bool Rebuild()
    {
        if (!RebuildRuntimeShapeState())
            return false;

        if (_context != null && _id >= 0)
            return _context.Collisions2D.RefreshColliderPartitionAfterShapeChange(this);

        return true;
    }

    public void SetParent(LSCollider2D parent) => _hierarchyState.SetParent(this, parent);

    public void ClearParent() => _hierarchyState.ClearParent(this);

    public void AddChild(int id)
    {
        if (_hierarchyState.AddChild(id) != true)
        {
            GravitasLogger.Channel.Warn($"2D collider with ID {id} is already a child.");
            return;
        }
    }

    public void RemoveChild(int id)
    {
        if (_hierarchyState.RemoveChild(id) != true)
        {
            GravitasLogger.Channel.Warn($"Cannot remove. 2D collider with ID {id} is not a child.");
            return;
        }
    }

    public bool IsSibling(LSCollider2D other) =>
        _hierarchyState.ExcludesCollisionWith(other._hierarchyState, Id, other.Id);

    internal SwiftList<WorldVoxelIndex> GetOrCreatePartitionCoordinates()
    {
        _partitionState.Coordinates ??= new();
        return _partitionState.Coordinates;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool MatchesPartitionGridBounds(Vector2d min, Vector2d max) =>
        _partitionState.MatchesGridBounds(min, max);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void MarkPartitioned(Vector2d min, Vector2d max)
    {
        _partitionState.SetPreviousGridBounds(min, max);
        _partitionState.MarkPartitioned();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void MarkUnpartitioned() => _partitionState.MarkUnpartitioned();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void ClearPartitionCoordinates() => _partitionState.ClearCoordinates();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool IsPositionInPlanarBounds(Fixed64 voxelSize, Vector3d worldPosition)
    {
        Fixed64 padding = voxelSize * Fixed64.Half;
        return worldPosition.x >= MinX - padding
            && worldPosition.x <= MaxX + padding
            && worldPosition.z >= MinY - padding
            && worldPosition.z <= MaxY + padding;
    }

    internal bool TryGetCollisionPair(int otherId, out CollisionPair2D? collisionPair) =>
        _pairState.TryGetCollisionPair(otherId, out collisionPair);

    internal bool TryAddCollisionPair(int otherId, CollisionPair2D collisionPair)
    {
        if (_pairState.TryAddCollisionPair(otherId, collisionPair) != true)
        {
            GravitasLogger.Channel.Warn($"2D collision pair with collider ID {otherId} already exists.");
            return false;
        }

        return true;
    }

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

    internal void NotifyContact(LSCollider2D other, bool isColliding, bool isChanged)
    {
        if (!IsActive)
            return;

        if (isColliding)
        {
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

    public abstract bool ContainsPoint(Vector2d point);

    public abstract Vector2d GetClosestPoint(Vector2d point);

    public abstract Vector2d GetSupportPoint(Vector2d direction);

    internal abstract int VertexCount { get; }

    internal abstract Vector2d GetVertexUnchecked(int index);

    protected abstract void RebuildShape();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    protected void MarkShapeDirty()
    {
        _shapeVersion++;
        _runtimeShapeState.MarkDirty();
        _body?.Wake();
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

    private ColliderShapeSnapshot2D CaptureShapeSnapshot() =>
        new(
            Center,
            Rotation,
            _localOffset,
            _shapeVersion,
            ResolveMixedSlabCenterY(),
            ResolveMixedHalfThickness());

    private Fixed64 ResolveMixedHalfThickness() =>
        _mixedHalfThicknessOverride ?? _context?.Settings.Mixed2DHalfThickness ?? PhysicsSettings.DefaultMixed2DHalfThickness;

    private Fixed64 ResolveMixedSlabCenterY() =>
        _agent?.Transform.Position.y ?? Fixed64.Zero;

    private void RebuildMixedEmbedding(Fixed64 slabCenterY, Fixed64 halfThickness)
    {
        _mixedHalfThickness = halfThickness;
        _mixedSlabCenterY = slabCenterY;

        Vector3d min = new(MinX, slabCenterY - halfThickness, MinY);
        Vector3d max = new(MaxX, slabCenterY + halfThickness, MaxY);
        if (!_mixedBoundsInitialized)
        {
            _mixedBounds3D = new BoundingBox((min + max) * Fixed64.Half, max - min);
            _mixedBoundsInitialized = true;
            return;
        }

        _mixedBounds3D.SetMinMax(min, max);
    }

    private void ClearChildParentReferences()
    {
        SwiftHashSet<int>? children = _hierarchyState.Children;
        if (children == null || _context == null)
            return;

        foreach (int childId in children)
        {
            if (_context.Physics2D.TryGetColliderById(childId, out LSCollider2D? child))
                child!._hierarchyState.ClearParentReference();
        }

        _hierarchyState.ClearChildren();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    bool IColliderHierarchyNode<LSCollider2D>.TryGetHierarchyColliderById(int id, out LSCollider2D? collider) =>
        Context.Physics2D.TryGetColliderById(id, out collider);

    protected void SetBounds(BoundingArea bounds) => _bounds = bounds;

    protected void SetBoundsFromMinMax(Vector2d min, Vector2d max)
    {
        SetBounds(new BoundingArea(
            new Vector3d(min.x, min.y, Fixed64.Zero),
            new Vector3d(max.x, max.y, Fixed64.Zero)));
    }

    private Fixed64 ResolveAgentRotation()
    {
        return _agent == null
            ? Fixed64.Zero
            : FixedMath.DegToRad(_agent.Transform.EulerAngles.y);
    }

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
        Fixed64 x = value.x.Abs() <= Fixed64.Epsilon ? Fixed64.Zero : value.x;
        Fixed64 y = value.y.Abs() <= Fixed64.Epsilon ? Fixed64.Zero : value.y;
        return new Vector2d(x, y);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    protected static Fixed64 ClampAxis(Fixed64 value, Fixed64 min, Fixed64 max) =>
        value < min ? min : value > max ? max : value;

    protected static Vector2d ClosestPointOnSegment(Vector2d point, Vector2d a, Vector2d b)
    {
        Vector2d segment = b - a;
        Fixed64 lengthSquared = segment.SqrMagnitude;
        if (lengthSquared <= Fixed64.Epsilon)
            return a;

        Fixed64 t = FixedMath.Clamp01(Vector2d.Dot(point - a, segment) / lengthSquared);
        return a + segment * t;
    }
}
