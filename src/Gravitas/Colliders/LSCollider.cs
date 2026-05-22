using Chronicler;
using FixedMathSharp;
using Gravitas.CollisionHandling;
using Gravitas.Support;
using GridForge.Grids;
using GridForge.Spatial;
using SwiftCollections;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

public abstract class LSCollider : IRecordable
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

    private bool _staticPositionChanged;
    private bool _staticRotationChanged;

    public virtual Vector3d Position
    {
        get => Body?.Position3d
            ?? _agent?.Transform.Position
            ?? throw new InvalidOperationException("Collider has no body or static transform.");
        set
        {
            if (_agent == null || _agent.Transform.Position == value)
                return;
            _agent.Transform.Position = value;
            _staticPositionChanged = true;
        }
    }

    public bool PositionChanged => Body?.PositionChangePending ?? _staticPositionChanged;

    public Fixed64 HeightPos => Body?.HeightPos
        ?? _agent?.Transform.Position.y
        ?? throw new InvalidOperationException("Collider has no body or static transform.");

    public virtual FixedQuaternion Rotation
    {
        get => Body?.Rotation
            ?? _agent?.Transform.Rotation
            ?? throw new InvalidOperationException("Collider has no body or static transform.");
        set
        {
            if (_agent == null || _agent.Transform.Rotation == value)
                return;
            _agent.Transform.Rotation = value;
            _staticRotationChanged = true;
        }
    }

    public bool RotationChanged => Body?.RotationChangePending ?? _staticRotationChanged;

    // For dynamic colliders, this is the velocity of the body. For static colliders, this is always zero.
    public Vector3d Velocity => Body?.LinearVelocity ?? Vector3d.Zero;

    public GridWorld? World => _context?.World ?? _agent?.Context.World;

    public FixedTransform Transform => Body?.PositionTransform
        ?? _agent?.Transform
        ?? throw new InvalidOperationException("Collider has no body or static transform.");

    private SingleLayer _layer = new();
    public int Layer { get => _layer.LayerIndex; set => _layer.Set(value); }

    //TODO: Account for teleports when culling.
    /// <summary>
    /// Used to prevent distance culling for very large objects.
    /// </summary>
    /// <remarks>
    /// Useful for fast-moving objects that might pass through if not checked for a frame.
    /// When enabled, the collider will not be culled based on distance for the first frame after being added to a new partition.
    /// </remarks>
    private bool _preventCulling = false;
    internal bool PreventCulling => _preventCulling;

    public bool IsPartitioned { get; private set; }

    /// <summary>
    /// Used for preventing culling for the first frame this object is added to a new partition node.
    /// </summary>
    public bool PartitionChanged { get; set; }

    /// <summary>
    /// Center of collider in local space, used for calculating bounds and offsets. Should be set in the Setup method of each collider type.
    /// </summary>
    protected Vector3d _offset;

    public abstract ColliderType Shape { get; }
    public abstract int Priority { get; }

    protected Fixed64 _radius = Fixed64.Half;

    /// <summary>
    /// Gets the bounding circle radius.
    /// For boxes, this is half of the diagonal length of the cube
    /// </summary>
    /// <value>The radius.</value>
    public virtual Fixed64 ScaledRadius => _radius * FixedMath.Max(LocalScale.z, FixedMath.Max(LocalScale.x, LocalScale.y));

    public Fixed64 ScaledRadiusSqr => ScaledRadius * ScaledRadius;

    protected Vector3d _size = Vector3d.One;

    public virtual Fixed64 Area { get; protected set; } = Fixed64.Zero;

    #region Grid & Partition Bounds

    public virtual Vector3d LocalScale => Transform.LossyScale;

    public virtual Vector3d ScaledSize => Vector3d.Scale(_size, LocalScale);

    public Vector3d ScaledOffset => Vector3d.Scale(_offset, LocalScale);

    /// <summary>
    /// Bodies position in world space + collider offset (center) value
    /// </summary>
    public Vector3d Center => Position + (Rotation * ScaledOffset);

    protected BoundingBox _bounds;
    public BoundingBox Bounds => _bounds;
    public Vector3d BoundsMin => _bounds.Min;
    public Vector3d BoundsMax => _bounds.Max;

    private Vector3d _lastGridBoundsMin;
    public Vector3d LastGridBoundsMin => _lastGridBoundsMin;

    private Vector3d _lastGridBoundsMax;
    public Vector3d LastGridBoundsMax => _lastGridBoundsMax;

    [NonSerialized]
    public SwiftList<WorldVoxelIndex>? PartitionCoordinates;

    #endregion

    private SwiftDictionary<int, CollisionPair> _collisionPairs = new();
    private SwiftHashSet<int> _collisionPairHolders = new();

    // TODO: this was for something
    // public bool IsInCollision { get; private set; }

    public uint RaycastVersion { get; set; } = 0;
    public uint SpherecastVersion { get; set; } = 0;

    public delegate void BodyCollisionFunc(StiffBody other);
    public event BodyCollisionFunc? OnContact;
    public event BodyCollisionFunc? OnContactEnter;
    public event BodyCollisionFunc? OnContactExit;

    public delegate void TriggerCollisionFunc(LSCollider other);
    public event TriggerCollisionFunc? OnTriggerEnter;
    public event TriggerCollisionFunc? OnTriggerExit;

    public bool IsChild { get; private set; }
    public bool IsParent { get; private set; }
    private SwiftHashSet<int>? _children;
    public int ParentId { get; private set; }
    public LSCollider? Parent { get; private set; }

    #endregion

    public void Initialize(StiffBody body)
    {
        _body = body;
        InitCore(body.Agent);
    }

    public void InitializeWithNoBody(IMatterAgent agent) =>
        InitCore(agent);

    private void InitCore(IMatterAgent agent)
    {
        SwiftThrowHelper.ThrowIfNull(agent, nameof(agent));

        RaycastVersion = 0;
        SpherecastVersion = 0;

        _agent = agent;
        BindContext(agent.Context);
        Context.Physics.AssimilateCollider(this);

        // This is a top level parent
        if (_agent.IsParent)
        {
            IsParent = true;
            _children = new();
        }
        else
            IsChild = true;

        ParentId = -1;

        OnInitialize();

        _lastGridBoundsMin = Vector3d.Zero;
        _lastGridBoundsMax = Vector3d.Zero;

        InitialPartition();
    }

    protected virtual void OnInitialize()
    {
        GenerateBoundingBox();
        GenerateShape();
    }

    private void GenerateBoundingBox()
    {
        _bounds = new BoundingBox(Center, ScaledSize);
        CalculateBoundLimits();
    }

    protected virtual void GenerateShape() { }

    private void InitialPartition()
    {
        GenerateDynamicRuntime();

        if (IsActive)
        {
            PartitionCoordinates ??= new();
            CollisionManager.PartitionObject(this, ref PartitionCoordinates);
            SetPreviousGridBounds();
            IsPartitioned = true;
            PartitionChanged = true;
        }
    }

    public void LateInitialize()
    {
        SetParent();
    }

    // Dynamic Colliders attached to a body will be updated by the body
    // Static Colliders need to be updated by whatever is updating the static collider
    // Even if the collider is inactive, if the body is active, the collider will be updated
    public void Simulate()
    {
        PartitionChanged = false;
        if (!IsActive)
            return;

        if (!IsPartitioned)
        {
            InitialPartition();
            return;
        }

        if (PositionChanged || RotationChanged)
            UpdatePartition();

        if (_staticPositionChanged) _staticPositionChanged = false;
        if (_staticRotationChanged) _staticRotationChanged = false;
    }

    private void UpdatePartition()
    {
        GenerateDynamicRuntime();

        if (!CollisionManager.ClearPartitionedObject(this))
            return;

        IsPartitioned = false;

        PartitionCoordinates ??= new();
        if (!CollisionManager.PartitionObject(this, ref PartitionCoordinates))
            return;

        SetPreviousGridBounds();

        IsPartitioned = true;
        PartitionChanged = true;
    }

    public void SetParent()
    {
        if (!IsChild)
            return;

        LSCollider? currentParent = Parent;
        LSCollider? topParentCollider = Parent;

        // Traverse the parent hierarchy until you find the top-level parent
        do
        {
            currentParent = currentParent?.Parent;
            if (currentParent == null)
                break;

            topParentCollider = currentParent;
        }
        while (currentParent != null);

        // Make sure the top parent has a collider
        if (topParentCollider == null)
            return;

        // Add this collider to the top parent's list of children
        topParentCollider.AddChild(_id);
        ParentId = topParentCollider.Id;
    }

    public void AddChild(int id)
    {
        _children ??= new();
        if (_children.Add(id) != true)
        {
            GravitasLogger.Channel.Warn($"Collider with ID {id} is already a child.");
            return;
        }
    }

    public void RemoveChild(int id)
    {
        if (_children?.Remove(id) != true)
        {
            GravitasLogger.Channel.Warn($"Cannot remove. Collider with ID {id} is not a child.");
            return;
        }
    }

    public bool IsSibling(LSCollider other)
    {
        if (ParentId == -1) return false;
        if (IsParent && Id == other.Id) return true;
        if (other.IsParent && other.Id == ParentId) return true;

        return other.ParentId == ParentId;
    }

    private void GenerateDynamicRuntime()
    {
        BuildBoundingBox();
        BuildShape();
    }

    protected virtual void BuildBoundingBox()
    {
        _bounds.Orient(Center, ScaledSize);
        CalculateBoundLimits();
    }

    private void CalculateBoundLimits()
    {
        // TODO: rotation is causing weird issues with Mesh colliders
        // when rotating the bounding box can sometimes be larger than the actual mesh
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

    public void SetStatus(bool status) => _active = status;

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
    /// <param name="outputIntersectionPoints"></param>
    /// <returns></returns>
    public abstract bool ColliderOverlapsRay(ref SwiftList<Vector3d> outputIntersectionPoints);

    public void SetPreviousGridBounds() =>
        (_lastGridBoundsMin, _lastGridBoundsMax) = World?.SnapBoundsToVoxelSize(BoundsMin, BoundsMax, Fixed64.Half)
            ?? (Vector3d.Zero, Vector3d.Zero);

    internal bool TryGetCollisionPair(int otherId, out CollisionPair? collisionPair) =>
        _collisionPairs.TryGetValue(otherId, out collisionPair);

    internal bool TryAddCollisionPair(int otherId, CollisionPair collisionPair)
    {
        if (_collisionPairs.TryAdd(otherId, collisionPair) != true)
        {
            GravitasLogger.Channel.Warn($"Collision pair with collider ID {otherId} already exists.");
            return false;
        }
        return true;
    }

    internal bool TryRemoveCollisionPair(int otherId)
    {
        if (!_collisionPairs.Remove(otherId, out CollisionPair? collisionPair))
            return false;

        if (collisionPair.Active)
            Context.Physics.DeactivateAndPoolPair(collisionPair);
        return true;
    }

    internal bool TryAddCollisionPairHolder(int otherId) => _collisionPairHolders.Add(otherId);

    internal bool TryRemoveCollisionPairHolder(int otherId) => _collisionPairHolders.Remove(otherId);

    public void Deactivate()
    {
        if (IsPartitioned)
        {
            CollisionManager.ClearPartitionedObject(this, true);
            PartitionCoordinates?.Clear();
        }

        // Remove all collision pairs involving this collider
        foreach (var kvp in _collisionPairs)
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
        _collisionPairs.Clear();

        // Remove this collider from the collision pair holders of all colliders it has pairs with
        for (int i = 0; i < _collisionPairHolders.Count; i++)
        {
            if (!Context.Physics.TryGetColliderById(_collisionPairHolders[i], out LSCollider? other))
                continue;

            if (other!.TryRemoveCollisionPair(Id) != true)
                GravitasLogger.DebugChannel.Info($"Collider with ID {Id} was not found in the collision pairs of collider with ID {_collisionPairHolders[i]} during deactivation. This may indicate that the pair was already removed or that there is an inconsistency in the collision management.");
        }
        _collisionPairHolders.Clear();

        Context.Physics.DessimilateCollider(this);
        //  IsInCollision = false;
        _active = false;
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool TryGetBoundContext(out GravitasWorldContext? context)
    {
        context = _context;
        return context != null;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void SetPhysicsId(int id)
    {
        SwiftThrowHelper.ThrowIfNegative(id, nameof(id));
        _id = id;
    }

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
        RecordValues.Look(chronicler, ref _id, "Id", -1);
        RecordValues.Look(chronicler, ref _preventCulling, "PreventCulling", false);
        RecordValues.Look(chronicler, ref _offset, "Offset", Vector3d.Zero);
        RecordValues.Look(chronicler, ref _radius, "Radius", Fixed64.Half);
        RecordValues.Look(chronicler, ref _size, "Size", Vector3d.One);
    }

    #endregion
}
