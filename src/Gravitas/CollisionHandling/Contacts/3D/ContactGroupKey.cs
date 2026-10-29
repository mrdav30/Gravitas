//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.CollisionHandling;

/// <summary>Collision-free local provenance for one admitted surface region.</summary>
internal readonly struct ContactGroupKey
{
    internal ContactGroupKey(int namespaceA, int namespaceB, int surfaceA = 0,
        int surfaceB = 0, int region = 0, int sourceIdA = 0, int sourceIdB = 0,
        int sourceOrderA = 0, int sourceOrderB = 0)
    {
        NamespaceA = namespaceA;
        NamespaceB = namespaceB;
        SurfaceA = surfaceA;
        SurfaceB = surfaceB;
        Region = region;
        _orderedSourceA = ((ulong)(uint)sourceOrderA << 32) | (uint)sourceIdA;
        _orderedSourceB = ((ulong)(uint)sourceOrderB << 32) | (uint)sourceIdB;
    }

    internal readonly int NamespaceA, NamespaceB, SurfaceA, SurfaceB, Region;
    // Compound namespaces identify leaves. Supplier registry IDs plus one are
    // normalized to replay ordinals only when hashing; zero means no supplier.
    internal int SourceIdA => (int)(uint)_orderedSourceA;
    internal int SourceIdB => (int)(uint)_orderedSourceB;
    // Joined suppliers sort by live logical registration, independently of recycled
    // registry slots. Keep the actual ID as the low-word uniqueness/lookup key;
    // ordinary groups retain their existing order and two source comparisons.
    private readonly ulong _orderedSourceA, _orderedSourceB;

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
        order = _orderedSourceA.CompareTo(other._orderedSourceA);
        return order != 0 ? order : _orderedSourceB.CompareTo(other._orderedSourceB);
    }

    internal ContactGroupKey Remap(int namespaceA, int namespaceB, bool reverse) =>
        new(namespaceA, namespaceB, reverse ? SurfaceB : SurfaceA,
            reverse ? SurfaceA : SurfaceB, Region,
            reverse ? SourceIdB : SourceIdA, reverse ? SourceIdA : SourceIdB,
            (int)((reverse ? _orderedSourceB : _orderedSourceA) >> 32),
            (int)((reverse ? _orderedSourceA : _orderedSourceB) >> 32));
}
