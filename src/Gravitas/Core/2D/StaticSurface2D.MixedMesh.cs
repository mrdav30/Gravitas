// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using SwiftCollections;
using System;

namespace Gravitas;

/// <content>Visits native moving mesh patches without filling concavities or holes.</content>
public sealed partial class StaticSurface2D
{
    private readonly SwiftList<Vector3d> _mixedPatchPoints = new();
    private readonly SwiftList<int> _mixedPatchTriangles = new(), _mixedPatchEdges = new();
    private readonly SwiftHashSet<int> _mixedPatchOwners = new();
    private int _mixedMovingFeature = -1;

    private void BuildMixedMesh(PhysicsMesh mesh)
    {
        _mixedPatchOwners.Clear();
        for (int triangle = 0; triangle < mesh.TriangleCount; triangle++)
        {
            int owner = mesh.GetManifoldSurfaceOwner(triangle);
            bool convex = !mesh.GetConvexCoplanarPatchCornerVertexIndices(triangle).IsEmpty;
            if (convex && !_mixedPatchOwners.Add(owner)) continue;
            SetMixedMovingFeature(convex ? owner : triangle);
            BuildMixedPart();
            if (_mixedRequestedReach > _mixedReach) return;
        }
    }

    private void SetMixedMovingFeature(int feature)
    {
        _mixedMovingFeature = feature;
        PhysicsMesh mesh = ((LSMeshCollider)_mixedMoving!).Mesh;
        ReadOnlySpan<int> corners = mesh.GetConvexCoplanarPatchCornerVertexIndices(feature);
        if (!corners.IsEmpty)
            MeshHullSurfaceContacts.FillPatch(mesh, corners, _mixedPatchPoints, _mixedPatchTriangles, _mixedPatchEdges);
        else
        {
            mesh.GetGeometryTriangleVertices(feature, out var a, out var b, out var c);
            _mixedPatchPoints.FastClear(); _mixedPatchPoints.Add(a); _mixedPatchPoints.Add(b); _mixedPatchPoints.Add(c);
            _mixedPatchTriangles.FastClear(); _mixedPatchTriangles.Add(0); _mixedPatchTriangles.Add(1); _mixedPatchTriangles.Add(2);
            _mixedPatchEdges.FastClear();
            _mixedPatchEdges.Add(0); _mixedPatchEdges.Add(1); _mixedPatchEdges.Add(1);
            _mixedPatchEdges.Add(2); _mixedPatchEdges.Add(2); _mixedPatchEdges.Add(0);
        }
    }

}
