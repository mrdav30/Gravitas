//=======================================================================
// ColliderShapeDefinitionKind.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Colliders;

/// <summary>
/// Identifies a data-only collider shape definition that can be materialized
/// into a runtime collider.
/// </summary>
public enum ColliderShapeDefinitionKind
{
    /// <summary>No authored shape.</summary>
    Undefined = 0,
    /// <summary>An authored sphere.</summary>
    Sphere = 1,
    /// <summary>An authored capsule.</summary>
    Capsule = 2,
    /// <summary>An authored cuboid.</summary>
    Cuboid = 3,
    /// <summary>An authored finite cylinder.</summary>
    Cylinder = 4,
    /// <summary>An authored finite cone.</summary>
    Cone = 5,
    /// <summary>An authored convex triangle mesh.</summary>
    ConvexMesh = 6
}
