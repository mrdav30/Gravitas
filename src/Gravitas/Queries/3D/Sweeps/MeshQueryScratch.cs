//=======================================================================
// MeshQueryScratch.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using SwiftCollections;

namespace Gravitas.Queries;

/// <summary>Owns the fixed buffers reused for mesh sweep and witness traversal.</summary>
internal sealed class MeshQueryScratch
{
    internal MeshQueryScratch(int capacity)
    {
        Traversal = new int[capacity];
        Triangles = new SwiftList<int>(capacity);
    }

    internal int Capacity => Traversal.Length;
    internal int[] Traversal { get; }
    internal SwiftList<int> Triangles { get; }
}
