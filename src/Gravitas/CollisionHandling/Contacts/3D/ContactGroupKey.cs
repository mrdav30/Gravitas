//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.CollisionHandling;

/// <summary>Collision-free local provenance for one admitted surface region.</summary>
internal readonly struct ContactGroupKey
{
    internal ContactGroupKey(int namespaceA, int namespaceB, int surfaceA = 0,
        int surfaceB = 0, int region = 0)
    {
        NamespaceA = namespaceA;
        NamespaceB = namespaceB;
        SurfaceA = surfaceA;
        SurfaceB = surfaceB;
        Region = region;
    }

    internal readonly int NamespaceA, NamespaceB, SurfaceA, SurfaceB, Region;

    internal int CompareTo(in ContactGroupKey other)
    {
        int order = NamespaceA.CompareTo(other.NamespaceA);
        if (order != 0) return order;
        order = NamespaceB.CompareTo(other.NamespaceB);
        if (order != 0) return order;
        order = SurfaceA.CompareTo(other.SurfaceA);
        if (order != 0) return order;
        order = SurfaceB.CompareTo(other.SurfaceB);
        return order != 0 ? order : Region.CompareTo(other.Region);
    }

    internal ContactGroupKey Remap(int namespaceA, int namespaceB, bool reverse) =>
        new(namespaceA, namespaceB, reverse ? SurfaceB : SurfaceA,
            reverse ? SurfaceA : SurfaceB, Region);
}
