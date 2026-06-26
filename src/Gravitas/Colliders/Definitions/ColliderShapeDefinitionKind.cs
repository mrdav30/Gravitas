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
    Undefined = 0,
    Sphere = 1,
    Capsule = 2,
    Cuboid = 3,
    Cylinder = 4,
    ConvexMesh = 5
}
