//=======================================================================
// MeshSurfaceBoundaryFan.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using SwiftCollections;
using System;

namespace Gravitas.CollisionHandling;

/// <summary>Collects exact incident sectors through certified positive-length surface joins.</summary>
internal sealed class MeshSurfaceBoundaryFan
{
    private readonly MeshSurfaceBoundaryDomain _boundary = new();
    private readonly SwiftList<(int Member, int Surface)> _owners = new();
    private readonly SwiftHashSet<(int Member, int Surface)> _visited = new();
    private readonly SwiftList<WideAxis3> _rays = new();
    private bool _trusted;

    internal int RayCount => _rays.Count;

    internal bool BuildEndpoint(ReadOnlySpan<PhysicsMesh> members, MeshSurfaceBoundaryDomain source, int span, bool upper)
    {
        var edge = source.Spans[span];
        Signed320 n = upper ? edge.Interval.UpperNumerator : edge.Interval.LowerNumerator;
        Signed320 d = upper ? edge.Interval.UpperDenominator : edge.Interval.LowerDenominator;
        if (!edge.Interval.IsClipped) { n = upper ? Signed320.One : default; d = Signed320.One; }
        return Build(members, source.SourceMember, source.SourceSurface,
            new FixedSegment(edge.Start, edge.End), n, d);
    }

    internal bool BuildAtPoint(ReadOnlySpan<PhysicsMesh> members, int sourceMember, int sourceSurface, Vector3d point) =>
        Build(members, sourceMember, sourceSurface, new FixedSegment(point, point), default, Signed320.One);

    // The caller supplies an admitted point. Traverse only positive-length
    // certified joins incident to it; a point-only neighbor adds no sector.
    private bool Build(ReadOnlySpan<PhysicsMesh> members, int sourceMember, int sourceSurface,
        FixedSegment source, Signed320 n, Signed320 d)
    {
        _trusted = false; _owners.FastClear(); _visited.Clear(); _rays.FastClear();
        var first = (sourceMember, members[sourceMember].GetManifoldSurfaceOwner(sourceSurface));
        _owners.Add(first); _visited.Add(first);
        PhysicsMesh original = members[sourceMember];
        Span<WideAxis3> fan = stackalloc WideAxis3[4];
        for (int index = 0; index < _owners.Count; index++)
        {
            var key = _owners[index]; PhysicsMesh mesh = members[key.Member];
            _boundary.Build(members, key.Member, key.Surface);
            foreach (var edge in _boundary.Spans)
            {
                if (WideTriangleRelations.LocateEdgePoint(source, original.Origin, original.Rotation, n, d,
                    new FixedSegment(edge.Start, edge.End), mesh.Origin, mesh.Rotation, edge.Interval) < 0) continue;
                if (edge.Kind == MeshSurfaceBoundaryDomain.SpanKind.Ambiguous) { _rays.FastClear(); return false; }
                var neighbor = (edge.OtherMember, edge.OtherSurface);
                if (edge.Kind == MeshSurfaceBoundaryDomain.SpanKind.Joined && _visited.Add(neighbor)) _owners.Add(neighbor);
            }
            int count = GetFan(mesh, key.Surface, source, original, n, d, fan);
            // Raw endpoint differences need <65 bits; the normalized relative
            // basis adds <132, keeping common-chart rays within Signed320.
            var basis = WideRationalBasis3d.CreateRelative(new WideRationalBasis3d(original.Rotation),
                new WideRationalBasis3d(mesh.Rotation));
            for (int ray = 0; ray < count; ray++)
                _rays.Add(WideRigidProjection.TransformLocalAxis(basis, Signed192.NarrowProven(fan[ray].X),
                    Signed192.NarrowProven(fan[ray].Y), Signed192.NarrowProven(fan[ray].Z)));
        }
        _trusted = true;
        return true;
    }

    private static int GetFan(PhysicsMesh mesh, int owner, FixedSegment source, PhysicsMesh original,
        Signed320 n, Signed320 d, Span<WideAxis3> fan)
    {
        ReadOnlySpan<int> edges = mesh.GetCanonicalSurfaceBoundaryVertexPairs(owner);
        ReadOnlySpan<Vector3d> vertices = mesh.ScaledLocalVertices;
        for (int edge = 0; edge < edges.Length; edge += 2)
        {
            int first = edges[edge], second = edges[edge + 1];
            int location = WideTriangleRelations.LocateEdgePoint(source, original.Origin, original.Rotation, n, d,
                new FixedSegment(vertices[first], vertices[second]), mesh.Origin, mesh.Rotation);
            if (location < 0) continue;
            if (location == 2) (first, second) = (second, first);
            return MeshSurfaceBoundaryTopology.GetIncidentFan(mesh, owner, first, second, location == 1, fan);
        }
        mesh.GetLocalTriangleVertices(owner, out Vector3d a, out Vector3d b, out Vector3d c);
        fan[0] = MeshSurfaceBoundaryTopology.Ray(a, b); fan[1] = -fan[0];
        fan[2] = MeshSurfaceBoundaryTopology.Ray(a, c); fan[3] = -fan[2];
        return 4;
    }
    /// <summary>Tests a direction in the source member's authored chart against every incident inward ray.</summary>
    internal bool Supports(Vector3d direction)
    {
        if (!_trusted) return false;
        var axis = new WideAxis3(Signed320.ExtendValue(Signed192.Raw(direction.X)),
            Signed320.ExtendValue(Signed192.Raw(direction.Y)), Signed320.ExtendValue(Signed192.Raw(direction.Z)));
        foreach (WideAxis3 ray in _rays.AsReadOnlySpan())
            if (WideAxis3.Dot(axis, ray).Sign > 0) return false;
        return true;
    }
}
