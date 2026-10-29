//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using SwiftCollections;
using System;
using System.Collections.Generic;

namespace Gravitas.CollisionHandling;

/// <summary>Shared source ownership and graph of exactly admitted joined surface regions.</summary>
internal class MeshJoinedSurfaceRegions<TFrame> where TFrame : struct
{
    internal readonly SwiftList<(LSMeshCollider Collider, int Triangle)> Inputs = new();
    internal readonly SwiftList<PhysicsMesh> Meshes = new();
    internal readonly SwiftList<int> Candidates = new();
    internal readonly SwiftList<MeshSurfaceBoundaryDomain> PreparedBoundaries = new();
    internal readonly MeshSurfaceBoundaryFan Fan = new();
    internal readonly SwiftDictionary<(PhysicsMesh Mesh, int Owner), int> MemberBySurface = new();
    internal readonly SwiftList<Member> Members = new();
    internal readonly SwiftList<int> Parents = new(), Roots = new(), RegionByRoot = new(), NodeMembers = new();
    private int[] _responseComponents = Array.Empty<int>(), _responseMembers = Array.Empty<int>();
    private readonly IComparer<(LSMeshCollider Collider, int Triangle)> _memberComparer =
        Comparer<(LSMeshCollider Collider, int Triangle)>.Create(static (a, b) =>
        {
            int order = ((IPhysicsColliderRegistryItem)a.Collider).ReplayOrder.CompareTo(((IPhysicsColliderRegistryItem)b.Collider).ReplayOrder);
            return order != 0 ? order : a.Triangle.CompareTo(b.Triangle);
        });
    internal int PreparedCount = -1;
    internal bool Active;

    internal void Prepare(ReadOnlySpan<(LSMeshCollider Collider, int Triangle)> surfaces,
        ReadOnlySpan<PhysicsMesh> preparedMeshes,
        SwiftDictionary<(PhysicsMesh, int), MeshSurfaceBoundaryDomain>? prepared)
    {
        Active = true;
        Inputs.FastClear(); MemberBySurface.Clear();
        foreach (var input in surfaces)
        {
            int owner = input.Collider.Mesh.GetManifoldSurfaceOwner(input.Triangle);
            if (MemberBySurface.TryAdd((input.Collider.Mesh, owner), 0)) Inputs.Add((input.Collider, owner));
        }
        Inputs.SortInPlace(_memberComparer);
        bool reuse = PreparedCount == Inputs.Count;
        for (int index = 0; reuse && index < Inputs.Count; index++)
        {
            Member member = Members[index]; var input = Inputs[index];
            reuse = member.Collider == input.Collider && member.Owner == input.Triangle
                && member.PreparedLifetime == input.Collider.LifetimeVersion
                && member.PreparedVersion == input.Collider.RuntimeShapeVersion;
        }
        // Only a fully completed build publishes the reusable set.
        PreparedCount = -1;
        Meshes.FastClear(); Parents.FastClear(); Roots.FastClear(); RegionByRoot.FastClear(); NodeMembers.FastClear();
        if (prepared != null)
            foreach (PhysicsMesh mesh in preparedMeshes) Meshes.Add(mesh);
        else
            foreach (var input in Inputs.AsReadOnlySpan())
                if (Meshes.Count == 0 || Meshes[Meshes.Count - 1] != input.Collider.Mesh) Meshes.Add(input.Collider.Mesh);
        PreparedBoundaries.FastClear();
        int meshIndex = -1;
        for (int index = 0; index < Inputs.Count; index++)
        {
            if (index == Members.Count) Members.Add(new Member());
            Member member = Members[index];
            member.Collider = Inputs[index].Collider; member.Triangle = member.Owner = Inputs[index].Triangle;
            PhysicsMesh mesh = member.Collider.Mesh;
            MemberBySurface[(mesh, member.Owner)] = index;
            if (index == 0 || Inputs[index - 1].Collider != member.Collider) meshIndex++;
            bool changed = !reuse;
            if (prepared != null)
            {
                MeshSurfaceBoundaryDomain boundary = prepared[(mesh, member.Owner)];
                changed |= !ReferenceEquals(member.Boundary, boundary);
                member.Boundary = boundary;
            }
            else if (!reuse) member.Boundary.Build(Meshes.AsReadOnlySpan(), meshIndex, member.Triangle);
            if (changed)
            {
                member.BoundaryTrusted = true;
                foreach (var span in member.Boundary.Spans)
                    if (span.Kind == MeshSurfaceBoundaryDomain.SpanKind.Ambiguous) member.BoundaryTrusted = false;
            }
            PreparedBoundaries.Add(member.Boundary);
            if (member.JoinedSpans.Length < member.Boundary.Spans.Length)
                Array.Resize(ref member.JoinedSpans, member.Boundary.Spans.Length);
            Array.Clear(member.JoinedSpans, 0, member.Boundary.Spans.Length);
        }
    }

    internal void AddRegions(int index)
        => AddRegions(index, Members[index].Connectivity.RegionCount);

    internal void PrepareResponseComponents()
    {
        if (_responseComponents.Length < Inputs.Count)
        {
            Array.Resize(ref _responseComponents, Inputs.Count);
            Array.Resize(ref _responseMembers, Inputs.Count);
        }
        for (int index = 0; index < Inputs.Count; index++)
        { _responseComponents[index] = index; _responseMembers[index] = -1; }
        // Geometry certifies coverage independently of finite support admission.
        // A covered rim stays covered even when its owner supplies no sample.
        for (int index = 0; index < Inputs.Count; index++)
        {
            Member member = Members[index];
            for (int span = 0; span < member.Boundary.Spans.Length; span++)
            {
                var edge = member.Boundary.Spans[span];
                // Coverage certifies this span independently. An ambiguous or
                // folded edge elsewhere still keeps its own native perimeter.
                member.JoinedSpans[span] = edge.Kind == MeshSurfaceBoundaryDomain.SpanKind.Joined
                    && MemberBySurface.TryGetValue((Meshes[edge.OtherMember], edge.OtherSurface), out int other);
                if (!member.JoinedSpans[span]) continue;
                int a = FindResponseComponent(index), b = FindResponseComponent(MemberBySurface[(Meshes[edge.OtherMember], edge.OtherSurface)]);
                _responseComponents[Math.Max(a, b)] = Math.Min(a, b);
            }
        }
    }

    internal void AddAdmittedMember(int index)
    {
        int component = FindResponseComponent(index);
        // Inputs are sorted, so the first genuine overlap is the stable carrier.
        if (_responseMembers[component] < 0) _responseMembers[component] = index;
    }

    internal int FindResponseComponent(int index)
    {
        while (_responseComponents[index] != index)
        { _responseComponents[index] = _responseComponents[_responseComponents[index]]; index = _responseComponents[index]; }
        return index;
    }

    internal int GetResponseMember(int index) => _responseMembers[FindResponseComponent(index)];

    internal void AddRegions(int index, int count)
    {
        Member member = Members[index]; member.Offset = Parents.Count;
        for (int region = 0; region < count; region++)
        { Parents.Add(Parents.Count); RegionByRoot.Add(-1); NodeMembers.Add(index); }
    }

    internal void Union(int first, int second)
    {
        int a = Find(first), b = Find(second);
        Parents[Math.Max(a, b)] = Math.Min(a, b);
    }

    internal int Find(int node)
    {
        while (Parents[node] != node) { Parents[node] = Parents[Parents[node]]; node = Parents[node]; }
        return node;
    }

    internal void CompleteRegions()
    {
        for (int node = 0; node < Parents.Count; node++)
        {
            int root = Find(node);
            if (RegionByRoot[root] >= 0) continue;
            RegionByRoot[root] = Roots.Count; Roots.Add(root);
        }
    }

    internal void Commit()
    {
        foreach (Member member in Members.AsReadOnlySpan()[..Inputs.Count])
        { member.PreparedVersion = member.Collider.RuntimeShapeVersion; member.PreparedLifetime = member.Collider.LifetimeVersion; }
        PreparedCount = Inputs.Count;
    }

    internal ReadOnlySpan<WideAxis3> GetEndpointFan(LSMeshCollider collider, int triangle, int span, bool upper, out bool suppress)
    {
        Member member = Members[MemberBySurface[(collider.Mesh, collider.Mesh.GetManifoldSurfaceOwner(triangle))]];
        bool trusted = Fan.BuildEndpoint(Meshes.AsReadOnlySpan(), member.Boundary, span, upper, PreparedBoundaries.AsReadOnlySpan());
        suppress = trusted && !Fan.HasExposedBoundary;
        // A genuine shared corner supplies one physical row in stable collider/surface order.
        if (trusted)
            foreach (var owner in Fan.Owners)
                if (MemberBySurface.TryGetValue((Meshes[owner.Member], owner.Surface), out int other)
                    && (((IPhysicsColliderRegistryItem)Members[other].Collider).ReplayOrder < ((IPhysicsColliderRegistryItem)collider).ReplayOrder || Members[other].Collider == collider
                        && collider.Mesh.GetCanonicalSurfaceOrdinal(owner.Surface) < collider.Mesh.GetCanonicalSurfaceOrdinal(member.Owner)))
                    suppress = true;
        return trusted ? Fan.Rays : default;
    }

    internal ReadOnlySpan<WideAxis3> GetCornerFan(LSMeshCollider collider, int triangle, int vertex, out bool suppress)
    {
        Member member = Members[MemberBySurface[(collider.Mesh, collider.Mesh.GetManifoldSurfaceOwner(triangle))]];
        Vector3d point = collider.Mesh.GeometryVertices[vertex];
        bool trusted = Fan.BuildAtPoint(Meshes.AsReadOnlySpan(), member.Boundary.SourceMember, member.Owner,
            point, PreparedBoundaries.AsReadOnlySpan());
        suppress = trusted && !Fan.HasExposedBoundary;
        // A finite canonical-owner corner can borrow a fan only when every
        // incident owner's complete perimeter retains its classification.
        if (trusted)
            foreach (var owner in Fan.Owners)
                if (MemberBySurface.TryGetValue((Meshes[owner.Member], owner.Surface), out int incident) && !Members[incident].BoundaryTrusted)
                { suppress = false; return default; }
        if (trusted)
            foreach (var owner in Fan.Owners)
            {
                if (!MemberBySurface.TryGetValue((Meshes[owner.Member], owner.Surface), out int other)
                    || ((IPhysicsColliderRegistryItem)Members[other].Collider).ReplayOrder > ((IPhysicsColliderRegistryItem)collider).ReplayOrder || Members[other].Collider == collider
                    && collider.Mesh.GetCanonicalSurfaceOrdinal(owner.Surface) >= collider.Mesh.GetCanonicalSurfaceOrdinal(member.Owner)) continue;
                PhysicsMesh mesh = Members[other].Collider.Mesh;
                ReadOnlySpan<int> edges = mesh.GetCanonicalSurfaceBoundaryVertexPairs(owner.Surface);
                for (int edge = 0; edge < edges.Length; edge += 2)
                {
                    int location = WideTriangleRelations.LocateEdgePoint(new FixedSegment(point, point),
                        collider.Mesh.GeometryFrame, default(Signed832), Signed832.ExtendValue(Signed192.Signed(1)),
                        new FixedSegment(mesh.GeometryVertices[edges[edge]], mesh.GeometryVertices[edges[edge + 1]]), mesh.GeometryFrame);
                    // Certified exposed/joined transitions are authored corners
                    // of some member. An interior-edge owner cannot supply one.
                    if (location == 0 || location == 2) { suppress = true; break; }
                }
            }
        return trusted ? Fan.Rays : default;
    }

    internal sealed class Member
    {
        internal LSMeshCollider Collider = null!;
        internal int Triangle, Owner, Offset;
        internal uint PreparedVersion;
        internal long PreparedLifetime;
        internal TFrame Frame;
        internal readonly MeshSurfaceConnectivity Connectivity = new();
        internal MeshSurfaceBoundaryDomain Boundary = new();
        internal readonly SwiftList<int> Admitted = new();
        internal bool[] JoinedSpans = Array.Empty<bool>();
        internal bool BoundaryTrusted;

        internal RationalSegmentInterval GetRetainedInterval(ref int span)
        {
            ReadOnlySpan<MeshSurfaceBoundaryDomain.BoundarySpan> pieces = Boundary.Spans;
            var first = pieces[span];
            // Classification cuts are not physical features. Only an actually
            // suppressed joined span can split a retained original edge.
            while (span + 1 < pieces.Length && pieces[span + 1].First == first.First && pieces[span + 1].Second == first.Second
                && !JoinedSpans[span + 1]) span++;
            var last = pieces[span];
            return new RationalSegmentInterval(first.Interval.IsClipped ? first.Interval.WideLowerNumerator : default,
                first.Interval.IsClipped ? first.Interval.WideLowerDenominator : Signed832.ExtendValue(Signed192.Signed(1)),
                last.Interval.IsClipped ? last.Interval.WideUpperNumerator : Signed832.ExtendValue(Signed192.Signed(1)),
                last.Interval.IsClipped ? last.Interval.WideUpperDenominator : Signed832.ExtendValue(Signed192.Signed(1)));
        }
    }
}
