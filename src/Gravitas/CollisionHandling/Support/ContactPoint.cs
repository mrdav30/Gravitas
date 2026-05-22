using FixedMathSharp;

namespace Gravitas.CollisionHandling;

public class ContactPoint
{
    private static readonly Fixed64 _penetrationMarginTolerance = (Fixed64)0.025f;

    // where did the collision occur in the frame of each object !
    private Vector3d? _relativeA;
    public Vector3d RelativeA => _relativeA ?? Vector3d.Zero;
    private Vector3d? _relativeB;
    public Vector3d RelativeB => _relativeB ?? Vector3d.Zero;

    /// <summary>
    /// The depth to which one object penetrates into another during a collision. 
    /// This value is derived from the minimum penetration vector, which is calculated during collision resolution.
    /// </summary>
    private Fixed64? _depth;
    public Fixed64 Depth => _depth ?? Fixed64.Zero;
    /// <summary>
    /// The vector that represents the direction and magnitude of the separation between the two colliding objects.
    /// </summary>
    private Vector3d? _normal;
    public Vector3d Normal => _normal ?? Vector3d.Zero;

    private Vector3d? _immovableCollisionDirection;
    public Vector3d ImmovableCollisionDirection => _immovableCollisionDirection ?? Vector3d.Zero;

    public void SetContactPoint(Vector3d localA, Vector3d localB, Fixed64 penetrationDepth, Vector3d normal)
    {
        _relativeA = localA;
        _relativeB = localB;

        //if (penetrationDepth < _penetrationMarginTolerance)
        //    GlobalLogger.Log("small _depth");

        // Introduce a minimum penetration depth to prevent small values
        // This value impacts the amount of penetration that is resolved during collision resolution.
        // If this value is to low, the objects will not separate enough to prevent tunneling.
        _depth = FixedMath.Max(penetrationDepth.Abs(), _penetrationMarginTolerance);
        _normal = normal;
        _immovableCollisionDirection = null;
    }

    public void SetImmovableDirection(Vector3d immovableCollisionDirection) =>
         _immovableCollisionDirection = immovableCollisionDirection;

    public void Reset()
    {
        _relativeA = null;
        _relativeB = null;
        _depth = null;
        _normal = null;
        _immovableCollisionDirection = null;
    }
}
