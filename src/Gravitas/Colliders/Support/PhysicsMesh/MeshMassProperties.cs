using FixedMathSharp;

namespace Gravitas.Colliders;

/// <summary>
/// Stores deterministic closed-volume mass properties for an immutable mesh topology.
/// </summary>
public readonly struct MeshMassProperties
{
    public MeshMassProperties(
        Fixed64 volume,
        Vector3d centerOfMass,
        Vector3d inertiaReferencePoint,
        Fixed3x3 unitMassInertiaTensor)
    {
        Volume = volume;
        CenterOfMass = centerOfMass;
        InertiaReferencePoint = inertiaReferencePoint;
        UnitMassInertiaTensor = unitMassInertiaTensor;
    }

    /// <summary>
    /// Absolute solid volume in local mesh units.
    /// </summary>
    public Fixed64 Volume { get; }

    /// <summary>
    /// Homogeneous center of mass in local mesh coordinates.
    /// </summary>
    public Vector3d CenterOfMass { get; }

    /// <summary>
    /// Local reference point about which <see cref="UnitMassInertiaTensor"/> was computed.
    /// </summary>
    public Vector3d InertiaReferencePoint { get; }

    /// <summary>
    /// Solid-volume diagonal inertia tensor for unit mass about <see cref="InertiaReferencePoint"/>.
    /// </summary>
    public Fixed3x3 UnitMassInertiaTensor { get; }
}
