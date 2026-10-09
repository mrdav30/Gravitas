//=======================================================================
// CollisionSatScratch.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using SwiftCollections;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Owns reusable SAT and mesh-query buffers for one world context.
/// </summary>
internal sealed class CollisionSatScratch
{
    public SwiftList<int> MeshCylinderTriangles { get; } = new(8);

    public SwiftList<int> MeshTriangleCandidatesA { get; } = new(16);

    public SwiftList<int> MeshTriangleCandidatesB { get; } = new(16);

    public SwiftHashSet<int> MeshConeSurfaces { get; } = new(8);

    public MeshConeSurfaceConnectivity MeshConeConnectivity { get; } = new();

    public MeshConeFaceRegions MeshConeFaces { get; } = new();

    public MeshConeSurfaceBoundary MeshConeBoundary { get; } = new();

    public ContactManifold MeshConeManifold { get; } = new();

    public ContactManifold CompoundPartManifold { get; } = new();
}
