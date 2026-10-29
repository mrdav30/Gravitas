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
    internal ReadOnlySpan<WideAxis3> Rays => _rays.AsReadOnlySpan();
    internal ReadOnlySpan<(int Member, int Surface)> Owners => _owners.AsReadOnlySpan();
    internal bool HasExposedBoundary { get; private set; }

    internal bool BuildEndpoint(ReadOnlySpan<PhysicsMesh> members, MeshSurfaceBoundaryDomain source, int span, bool upper,
        ReadOnlySpan<MeshSurfaceBoundaryDomain> prepared = default)
    {
        var edge = source.Spans[span];
        Signed832 n = upper ? edge.Interval.WideUpperNumerator : edge.Interval.WideLowerNumerator;
        Signed832 d = upper ? edge.Interval.WideUpperDenominator : edge.Interval.WideLowerDenominator;
        if (!edge.Interval.IsClipped) { n = upper ? Signed832.ExtendValue(Signed192.Signed(1)) : default; d = Signed832.ExtendValue(Signed192.Signed(1)); }
        return Build(members, source.SourceMember, source.SourceSurface,
            new FixedSegment(edge.Start, edge.End), n, d, prepared);
    }

    internal bool BuildAtPoint(ReadOnlySpan<PhysicsMesh> members, int sourceMember, int sourceSurface, Vector3d point,
        ReadOnlySpan<MeshSurfaceBoundaryDomain> prepared = default) =>
        Build(members, sourceMember, sourceSurface, new FixedSegment(point, point), default, Signed832.ExtendValue(Signed192.Signed(1)), prepared);

    internal bool BuildAtAnchor(ReadOnlySpan<PhysicsMesh> members, int sourceMember, int sourceSurface,
        in FixedPointAnchor point, ReadOnlySpan<MeshSurfaceBoundaryDomain> prepared) =>
        Build(members, sourceMember, sourceSurface, default, default, Signed832.ExtendValue(Signed192.Signed(1)),
            prepared, RationalPoint3d.FromAnchor(point));

    // The caller supplies an admitted point. Traverse only positive-length
    // certified joins incident to it; a point-only neighbor adds no sector.
    private bool Build(ReadOnlySpan<PhysicsMesh> members, int sourceMember, int sourceSurface,
        FixedSegment source, Signed832 n, Signed832 d, ReadOnlySpan<MeshSurfaceBoundaryDomain> prepared = default,
        RationalPoint3d? point = null)
    {
        _trusted = false; HasExposedBoundary = false; _owners.FastClear(); _visited.Clear(); _rays.FastClear();
        var first = (sourceMember, members[sourceMember].GetManifoldSurfaceOwner(sourceSurface));
        _owners.Add(first); _visited.Add(first);
        PhysicsMesh original = members[sourceMember];
        Span<WideAxis3> fan = stackalloc WideAxis3[4];
        for (int index = 0; index < _owners.Count; index++)
        {
            var key = _owners[index]; PhysicsMesh mesh = members[key.Member];
            MeshSurfaceBoundaryDomain boundary = _boundary;
            if (prepared.IsEmpty) boundary.Build(members, key.Member, key.Surface);
            else
            {
                MeshSurfaceBoundaryDomain? found = null;
                foreach (var candidate in prepared)
                    if (candidate.SourceMember == key.Member && candidate.SourceSurface == key.Surface) { found = candidate; break; }
                if (found == null) { HasExposedBoundary = true; continue; }
                boundary = found;
            }
            foreach (var edge in boundary.Spans)
            {
                int location = point.HasValue ? WideTriangleRelations.LocatePointOnFrameEdge(point.Value,
                    new FixedSegment(edge.Start, edge.End), mesh.GeometryFrame, edge.Interval)
                    : WideTriangleRelations.LocateEdgePoint(source, original.GeometryFrame, n, d,
                        new FixedSegment(edge.Start, edge.End), mesh.GeometryFrame, edge.Interval);
                if (location < 0) continue;
                if (edge.Kind == MeshSurfaceBoundaryDomain.SpanKind.Ambiguous) { _rays.FastClear(); return false; }
                if (edge.Kind == MeshSurfaceBoundaryDomain.SpanKind.Exposed) HasExposedBoundary = true;
                var neighbor = (edge.OtherMember, edge.OtherSurface);
                if (edge.Kind == MeshSurfaceBoundaryDomain.SpanKind.Joined && _visited.Add(neighbor)) _owners.Add(neighbor);
            }
            int count = GetFan(mesh, key.Surface, source, original, n, d, fan, point,
                key.Member == sourceMember && key.Surface == first.Item2 ? sourceSurface : key.Surface);
            if (count == 0) { _rays.FastClear(); return false; }
            // Each positive affine denominator is irrelevant to a ray's halfspace sign.
            for (int ray = 0; ray < count; ray++)
                _rays.Add(mesh.GeometryFrame.GetWorldDirectionNumerators(fan[ray]));
        }
        _trusted = true;
        return true;
    }

    private static int GetFan(PhysicsMesh mesh, int owner, FixedSegment source, PhysicsMesh original,
        Signed832 n, Signed832 d, Span<WideAxis3> fan, RationalPoint3d? point, int admittedTriangle)
    {
        ReadOnlySpan<int> edges = mesh.GetCanonicalSurfaceBoundaryVertexPairs(owner);
        ReadOnlySpan<Vector3d> vertices = mesh.GeometryVertices;
        for (int edge = 0; edge < edges.Length; edge += 2)
        {
            int first = edges[edge], second = edges[edge + 1];
            int location = point.HasValue ? WideTriangleRelations.LocatePointOnFrameEdge(point.Value,
                new FixedSegment(vertices[first], vertices[second]), mesh.GeometryFrame)
                : WideTriangleRelations.LocateEdgePoint(source, original.GeometryFrame, n, d,
                    new FixedSegment(vertices[first], vertices[second]), mesh.GeometryFrame);
            if (location < 0) continue;
            if (location == 2) (first, second) = (second, first);
            return MeshSurfaceBoundaryTopology.GetIncidentFan(mesh, owner, first, second, location == 1, fan);
        }
        if (point.HasValue)
        {
            // An authored diagonal inside a certified coplanar owner is filled
            // surface, not a rim. Final anchor rounding can put the point on
            // either incident triangle; both still require exact finite proof.
            bool admitted = PointOnTriangle(mesh, admittedTriangle, point.Value);
            // Prepared coplanar neighbors share this committed manifold owner.
            foreach (int neighbor in mesh.GetCoplanarTriangleNeighbors(admittedTriangle))
                if (!admitted)
                    admitted = PointOnTriangle(mesh, neighbor, point.Value);
            if (!admitted) return 0;
        }
        mesh.GetGeometryTriangleVertices(owner, out Vector3d a, out Vector3d b, out Vector3d c);
        fan[0] = MeshSurfaceBoundaryTopology.Ray(a, b); fan[1] = -fan[0];
        fan[2] = MeshSurfaceBoundaryTopology.Ray(a, c); fan[3] = -fan[2];
        return 4;
    }

    private static bool PointOnTriangle(PhysicsMesh mesh, int triangle, in RationalPoint3d point)
    {
        mesh.GetGeometryTriangleVertices(triangle, out var a, out var b, out var c);
        FixedTriangle face = new(a, b, c);
        var frame = mesh.GeometryFrame;
        WideDirection3d normal = WideTriangleRelations.GetScaledTriangleLocalNormal(face, frame).Transform(frame.GeometryBasis);
        return WideDirection3d.Dot(normal, WideTriangleRelations.GetScaledPointDifference(point,
            frame.GetPoint(a), frame.RotationDenominator)).IsZero && WideTriangleRelations.ContainsProjection(face, frame, point);
    }

    internal static bool SupportsTriangle(PhysicsMesh mesh, int triangle, in RationalPoint3d point,
        in WideDirection3d direction)
    {
        // A rounded closest witness can land on a seam without remaining the
        // actual closest point. Finite incidence and the triangle normal cone
        // certify that the supplied point-to-core direction still owns it.
        if (!PointOnTriangle(mesh, triangle, point)) return false;
        mesh.GetGeometryTriangleVertices(triangle, out var a, out var b, out var c);
        Span<Vector3d> vertices = stackalloc Vector3d[3] { a, b, c };
        foreach (Vector3d vertex in vertices)
            if (WideDirection3d.Dot(direction, WideTriangleRelations.GetScaledPointDifference(
                mesh.GeometryFrame.GetPoint(vertex), point,
                WideArithmetic.TruncateSigned320By64(point.Denominator))).Sign > 0) return false;
        return true;
    }
    /// <summary>Tests an exact world direction against every incident inward ray.</summary>
    internal bool Supports(in WideDirection3d direction)
    {
        if (!_trusted) return false;
        foreach (WideAxis3 ray in _rays.AsReadOnlySpan())
            if (WideDirection3d.Dot(direction, ray).Sign > 0) return false;
        return true;
    }

    internal bool Supports(Vector3d direction) => Supports(new WideDirection3d(
        Signed320.ExtendValue(Signed192.Raw(direction.X)), Signed320.ExtendValue(Signed192.Raw(direction.Y)),
        Signed320.ExtendValue(Signed192.Raw(direction.Z))));
}
