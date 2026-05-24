using FixedMathSharp;

namespace Gravitas.CollisionHandling;

public class ContactPoint
{
    /// <summary>
    /// Gets whether this instance currently contains narrow-phase contact data.
    /// </summary>
    public bool HasContact { get; private set; }

    private Vector3d _relativeA;
    public Vector3d RelativeA => _relativeA;
    private Vector3d _relativeB;
    public Vector3d RelativeB => _relativeB;

    /// <summary>
    /// The depth to which one object penetrates into another during a collision. 
    /// This value is derived from the minimum penetration vector, which is calculated during collision resolution.
    /// </summary>
    private Fixed64 _depth;
    public Fixed64 Depth => _depth;
    /// <summary>
    /// The vector that represents the direction and magnitude of the separation between the two colliding objects.
    /// </summary>
    private Vector3d _normal;
    public Vector3d Normal => _normal;

    private Vector3d _immovableCollisionDirection;
    public Vector3d ImmovableCollisionDirection => _immovableCollisionDirection;

    public void SetContactPoint(Vector3d localA, Vector3d localB, Fixed64 penetrationDepth, Vector3d normal)
    {
        HasContact = true;
        _relativeA = localA;
        _relativeB = localB;
        _depth = penetrationDepth.Abs();
        _normal = normal.SqrMagnitude > Fixed64.Epsilon
            ? normal.Normal
            : Vector3d.Zero;
        _immovableCollisionDirection = Vector3d.Zero;
    }

    public void SetImmovableDirection(Vector3d immovableCollisionDirection) =>
         _immovableCollisionDirection = immovableCollisionDirection;

    public void Reset()
    {
        HasContact = false;
        _relativeA = Vector3d.Zero;
        _relativeB = Vector3d.Zero;
        _depth = Fixed64.Zero;
        _normal = Vector3d.Zero;
        _immovableCollisionDirection = Vector3d.Zero;
    }
}
