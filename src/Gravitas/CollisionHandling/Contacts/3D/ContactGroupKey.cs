//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.CollisionHandling;

/// <summary>Collision-free local provenance for one admitted surface region.</summary>
internal readonly struct ContactGroupKey
{
    internal ContactGroupKey(int namespaceA, int namespaceB, int surfaceA = 0,
        int surfaceB = 0, int region = 0, int sourceIdA = 0, int sourceIdB = 0)
    {
        NamespaceA = namespaceA;
        NamespaceB = namespaceB;
        SurfaceA = surfaceA;
        SurfaceB = surfaceB;
        Region = region;
        SourceIdA = sourceIdA;
        SourceIdB = sourceIdB;
    }

    internal readonly int NamespaceA, NamespaceB, SurfaceA, SurfaceB, Region;
    // Compound namespaces identify leaves. Supplier registry IDs plus one are
    // normalized to replay ordinals only when hashing; zero means no supplier.
    internal readonly int SourceIdA, SourceIdB;

    internal int CompareTo(in ContactGroupKey other)
    {
        int order = NamespaceA.CompareTo(other.NamespaceA);
        if (order != 0) return order;
        order = NamespaceB.CompareTo(other.NamespaceB);
        if (order != 0) return order;
        order = SurfaceA.CompareTo(other.SurfaceA);
        if (order != 0) return order;
        order = SurfaceB.CompareTo(other.SurfaceB);
        if (order != 0) return order;
        order = Region.CompareTo(other.Region);
        if (order != 0) return order;
        order = SourceIdA.CompareTo(other.SourceIdA);
        return order != 0 ? order : SourceIdB.CompareTo(other.SourceIdB);
    }

    internal ContactGroupKey Remap(int namespaceA, int namespaceB, bool reverse) =>
        new(namespaceA, namespaceB, reverse ? SurfaceB : SurfaceA,
            reverse ? SurfaceA : SurfaceB, Region,
            reverse ? SourceIdB : SourceIdA, reverse ? SourceIdA : SourceIdB);
}
