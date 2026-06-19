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

    /// <summary>
    /// Calculates a diagonal inertia tensor for the supplied mass about a local
    /// point parallel to the mesh center-of-mass axes.
    /// </summary>
    public Fixed3x3 CalculateInertiaTensor(Fixed64 mass, Vector3d localReferencePoint)
    {
        Fixed3x3 referenceTensor = UnitMassInertiaTensor * mass;
        Fixed3x3 centerTensor = SubtractParallelAxisTensor(
            referenceTensor,
            mass,
            InertiaReferencePoint - CenterOfMass);
        return AddParallelAxisTensor(centerTensor, mass, localReferencePoint - CenterOfMass);
    }

    private static Fixed3x3 AddParallelAxisTensor(Fixed3x3 tensor, Fixed64 mass, Vector3d offset)
    {
        if (mass <= Fixed64.Zero || offset == Vector3d.Zero)
            return tensor;

        Fixed64 xx = mass * ((offset.Y * offset.Y) + (offset.Z * offset.Z));
        Fixed64 yy = mass * ((offset.X * offset.X) + (offset.Z * offset.Z));
        Fixed64 zz = mass * ((offset.X * offset.X) + (offset.Y * offset.Y));
        tensor.M11 += xx;
        tensor.M22 += yy;
        tensor.M33 += zz;
        return tensor;
    }

    private static Fixed3x3 SubtractParallelAxisTensor(Fixed3x3 tensor, Fixed64 mass, Vector3d offset)
    {
        if (mass <= Fixed64.Zero || offset == Vector3d.Zero)
            return tensor;

        Fixed64 xx = mass * ((offset.Y * offset.Y) + (offset.Z * offset.Z));
        Fixed64 yy = mass * ((offset.X * offset.X) + (offset.Z * offset.Z));
        Fixed64 zz = mass * ((offset.X * offset.X) + (offset.Y * offset.Y));
        tensor.M11 = ClampInertia(tensor.M11 - xx);
        tensor.M22 = ClampInertia(tensor.M22 - yy);
        tensor.M33 = ClampInertia(tensor.M33 - zz);
        return tensor;
    }

    private static Fixed64 ClampInertia(Fixed64 value) =>
        value > Fixed64.Zero ? value : Fixed64.Zero;
}
