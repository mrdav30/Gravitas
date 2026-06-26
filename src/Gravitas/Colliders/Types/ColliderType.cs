//=======================================================================
// ColliderType.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Colliders;

public enum ColliderType : byte
{
    None,
    Sphere,
    AABox,
    OBBox,
    Capsule,
    Cylinder,
    Mesh,
    Compound
}
