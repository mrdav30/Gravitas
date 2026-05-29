using FixedMathSharp;
using Gravitas.Support;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

/// <summary>
/// Base type for pure 2D collider shapes.
/// </summary>
public abstract class LSCollider2D
{
    private StiffBody2D? _body;
    private GravitasWorldContext? _context;
    private int _id = -1;
    private int _serviceIndex = -1;
    private bool _isActive = true;
    private bool _isTrigger;
    private PhysicsLayer _layer = new();
    private Vector2d _localOffset;
    private Physics2DBounds _bounds;

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

    public StiffBody2D? Body => _body;

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
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => _isActive = value;
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

    public PhysicsDimension Dimension
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => PhysicsDimension.TwoD;
    }

    public abstract Collider2DType Shape { get; }

    public Vector2d LocalOffset
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _localOffset;
        set
        {
            if (_localOffset == value)
                return;

            _localOffset = value;
            Rebuild();
        }
    }

    public Vector2d Position => _body?.Position ?? Vector2d.Zero;

    public Fixed64 Rotation => _body?.Rotation ?? Fixed64.Zero;

    public Vector2d Center => Position + Rotate(LocalOffset, Rotation);

    public Physics2DBounds Bounds => _bounds;

    public Fixed64 MinX => _bounds.Area.MinX;

    public Fixed64 MaxX => _bounds.Area.MaxX;

    public Fixed64 MinY => _bounds.Area.MinY;

    public Fixed64 MaxY => _bounds.Area.MaxY;

    internal void Initialize(StiffBody2D body)
    {
        SwiftThrowHelper.ThrowIfNull(body, nameof(body));
        _body = body;
        _context = body.Context;
        _isActive = true;
        Rebuild();
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
        _id = -1;
        _serviceIndex = -1;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void Rebuild() => RebuildShape();

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

    protected void SetBounds(Physics2DBounds bounds) => _bounds = bounds;

    protected void SetBoundsFromMinMax(Vector2d min, Vector2d max)
    {
        SetBounds(Physics2DBounds.FromMinMax(min, max, Fixed64.Zero, Fixed64.Zero));
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
