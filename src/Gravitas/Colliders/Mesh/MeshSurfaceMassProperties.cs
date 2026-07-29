//=======================================================================
// MeshSurfaceMassProperties.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using System;

namespace Gravitas.Colliders;

/// <summary>
/// Stores deterministic uniform thin-shell mass properties for scaled mesh triangles.
/// </summary>
public readonly struct MeshSurfaceMassProperties
{
    internal MeshSurfaceMassProperties(
        Fixed64 area,
        Vector3d centerOfMass,
        Vector3d inertiaReferencePoint,
        Fixed3x3 unitMassInertiaTensor)
    {
        Area = area;
        CenterOfMass = centerOfMass;
        InertiaReferencePoint = inertiaReferencePoint;
        UnitMassInertiaTensor = unitMassInertiaTensor;
    }

    /// <summary>Gets the scaled triangle surface area represented by this shell.</summary>
    public Fixed64 Area { get; }

    /// <summary>Gets the shell center of mass in scaled mesh-local coordinates.</summary>
    public Vector3d CenterOfMass { get; }

    /// <summary>Gets the scaled mesh-local point about which the cached unit tensor is expressed.</summary>
    public Vector3d InertiaReferencePoint { get; }

    /// <summary>Gets the unit-mass thin-shell inertia tensor about <see cref="InertiaReferencePoint"/>.</summary>
    public Fixed3x3 UnitMassInertiaTensor { get; }

    /// <summary>Calculates shell inertia for the supplied mass about a scaled mesh-local reference point.</summary>
    public Fixed3x3 CalculateInertiaTensor(Fixed64 mass, Vector3d localReferencePoint)
    {
        Fixed3x3 referenceTensor = UnitMassInertiaTensor * mass;
        Fixed3x3 centerTensor = InertiaTensorMath.SubtractParallelAxisTensor(
            referenceTensor,
            mass,
            InertiaReferencePoint - CenterOfMass);
        return InertiaTensorMath.AddParallelAxisTensor(
            centerTensor,
            mass,
            localReferencePoint - CenterOfMass);
    }

    internal static bool TryCreate(
        ReadOnlySpan<Vector3d> localVertices,
        ReadOnlySpan<int> triangles,
        Fixed64 totalArea,
        out ExactMassWeight totalWeight,
        out MeshSurfaceMassProperties properties)
    {
        properties = default;
        if (!TriangleShellMassProperties.TryCreateUniformShell(
                localVertices,
                triangles,
                out totalWeight,
                out Vector3d centerOfMass,
                out Fixed3x3 unitCenterTensor))
        {
            return false;
        }

        properties = new MeshSurfaceMassProperties(
            totalArea,
            centerOfMass,
            centerOfMass,
            unitCenterTensor);
        return true;
    }
}
