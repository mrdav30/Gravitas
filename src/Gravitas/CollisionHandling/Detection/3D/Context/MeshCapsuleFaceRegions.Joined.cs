//=======================================================================
// MeshCapsuleFaceRegions.Joined.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using SwiftCollections;
using SwiftCollections.Query;
using System;
using System.Collections.Generic;

namespace Gravitas.CollisionHandling;

/// <content>Finite capsule sections across explicitly supplied static surface members.</content>
internal sealed partial class MeshCapsuleFaceRegions
{
    private JoinedState? _joined;

    internal void ReleaseJoined()
    {
        _joined = null;
        RegionCount = 0;
    }

    /// <summary>Prepares unique committed surface owners from bounded input; membership/filter admission belongs to the caller.</summary>
    internal void BuildJoined(ReadOnlySpan<(LSMeshCollider Collider, int Triangle)> surfaces,
        Vector3d center, FixedQuaternion rotation, Fixed64 coreLength, Fixed64 radius,
        ReadOnlySpan<PhysicsMesh> preparedMeshes = default,
        SwiftDictionary<(PhysicsMesh, int), MeshSurfaceBoundaryDomain>? prepared = null)
    {
        JoinedState state = _joined ??= new JoinedState(this);
        state.Active = true;
        state.Inputs.FastClear(); state.MemberBySurface.Clear();
        foreach (var input in surfaces)
        {
            int owner = input.Collider.Mesh.GetManifoldSurfaceOwner(input.Triangle);
            if (!state.MemberBySurface.TryAdd((input.Collider.Mesh, owner), 0)) continue;
            state.Inputs.Add((input.Collider, owner));
        }
        state.Inputs.SortInPlace(state.MemberComparer);
        bool reuseBoundaries = state.PreparedCount == state.Inputs.Count;
        for (int index = 0; reuseBoundaries && index < state.Inputs.Count; index++)
        {
            JoinedMember member = state.Members[index]; var input = state.Inputs[index];
            reuseBoundaries = member.Collider == input.Collider && member.Owner == input.Triangle
                && member.PreparedLifetime == input.Collider.LifetimeVersion
                && member.PreparedVersion == input.Collider.RuntimeShapeVersion;
        }
        // An incomplete rebuild must never qualify as the last prepared set.
        state.PreparedCount = -1;
        state.Meshes.FastClear(); state.Parents.FastClear(); state.Roots.FastClear(); state.RegionByRoot.FastClear(); state.NodeMembers.FastClear();
        _center = center; _rotation = coreLength == Fixed64.Zero ? FixedQuaternion.Identity : rotation;
        FixedBoundBox box = FixedBoundBox.FromCenteredCapsuleClippedToDomain(center, _rotation, Vector3d.Up, coreLength, radius);
        var bounds = new FixedBoundVolume(box.Min, box.Max);
        LSMeshCollider? previous = null;
        foreach (var input in state.Inputs.AsReadOnlySpan())
        {
            if (input.Collider != previous) state.Meshes.Add(input.Collider.Mesh);
            previous = input.Collider;
        }
        if (prepared != null)
        {
            state.Meshes.FastClear();
            foreach (PhysicsMesh mesh in preparedMeshes) state.Meshes.Add(mesh);
        }
        state.PreparedBoundaries.FastClear();
        int meshIndex = -1;
        for (int index = 0; index < state.Inputs.Count; index++)
        {
            if (index == state.Members.Count) state.Members.Add(new JoinedMember());
            JoinedMember member = state.Members[index];
            member.Collider = state.Inputs[index].Collider; member.Triangle = state.Inputs[index].Triangle;
            PhysicsMesh mesh = member.Collider.Mesh;
            member.Owner = member.Triangle;
            state.MemberBySurface[(mesh, member.Owner)] = index;
            if (index == 0 || state.Inputs[index - 1].Collider != member.Collider) meshIndex++;
            var finite = new CapsuleFiniteSectionFrame(mesh.Origin, mesh.Rotation, center, _rotation, coreLength, radius);
            mesh.GetLocalTriangleVertices(member.Triangle, out Vector3d a, out Vector3d b, out Vector3d c);
            member.Frame = new CapsulePlaneRayFrame(new FixedTriangle(a, b, c), finite);
            member.Admitted.FastClear();
            member.Collider.GetTrianglesInBounds(bounds, state.Candidates);
            foreach (int triangle in state.Candidates.AsReadOnlySpan())
            {
                if (mesh.GetManifoldSurfaceOwner(triangle) != member.Owner) continue;
                mesh.GetLocalTriangleVertices(triangle, out a, out b, out c);
                if (CapsulePlaneRayEvents.IntersectsTriangle(finite, new FixedTriangle(a, b, c))) member.Admitted.Add(triangle);
            }
            member.Connectivity.BuildCapsule(mesh, member.Triangle, finite, member.Admitted.AsReadOnlySpan());
            member.Offset = state.Parents.Count;
            for (int region = 0; region < member.Connectivity.RegionCount; region++)
            { state.Parents.Add(state.Parents.Count); state.RegionByRoot.Add(-1); state.NodeMembers.Add(index); }
            if (prepared != null) member.Boundary = prepared[(mesh, member.Owner)];
            else if (!reuseBoundaries) member.Boundary.Build(state.Meshes.AsReadOnlySpan(), meshIndex, member.Triangle);
            state.PreparedBoundaries.Add(member.Boundary);
            if (member.JoinedSpans.Length < member.Boundary.Spans.Length)
                Array.Resize(ref member.JoinedSpans, member.Boundary.Spans.Length);
            Array.Clear(member.JoinedSpans, 0, member.Boundary.Spans.Length);
        }
        JoinFiniteSpans(state);
        for (int node = 0; node < state.Parents.Count; node++)
        {
            int root = FindJoinedRoot(state, node);
            if (state.RegionByRoot[root] >= 0) continue;
            state.RegionByRoot[root] = state.Roots.Count; state.Roots.Add(root);
        }
        RegionCount = state.Roots.Count; HasBoundaryIntersection = false;
        Reserve(RegionCount);
        if (state.BoundaryHits.Length < RegionCount) Array.Resize(ref state.BoundaryHits, RegionCount);
        if (state.Trusted.Length < RegionCount) Array.Resize(ref state.Trusted, RegionCount);
        Array.Clear(state.BoundaryHits, 0, RegionCount); Array.Fill(state.Trusted, true, 0, RegionCount);
        foreach (JoinedMember member in state.Members.AsReadOnlySpan()[..state.Inputs.Count])
        {
            foreach (var span in member.Boundary.Spans)
            {
                if (span.Kind != MeshSurfaceBoundaryDomain.SpanKind.Ambiguous) continue;
                for (int local = 0; local < member.Connectivity.RegionCount; local++)
                    state.Trusted[state.RegionByRoot[FindJoinedRoot(state, member.Offset + local)]] = false;
            }
        }
        if (state.Maximum.Length < 2 * RegionCount) Array.Resize(ref state.Maximum, 2 * RegionCount);
        if (state.MaximumPoint.Length < 2 * RegionCount) Array.Resize(ref state.MaximumPoint, 2 * RegionCount);
        Array.Clear(_winners, 0, 2 * RegionCount);
        _events.FastClear(); _points.FastClear(); _pointWordCount = 0; state.PointSources.FastClear();
        if (!TryGatherContainedJoinedSupport(state))
        {
            // The intrinsic plane is shared by the whole joined component;
            // finite admission below still selects each actual supplying owner.
            for (int region = 0; region < RegionCount; region++)
            {
                state.CurrentRegion = region; state.CurrentMember = state.NodeMembers[state.Roots[region]];
                _frame = state.Members[state.CurrentMember].Frame;
                VisitJoinedSource(state, CapsulePlaneRayEventSource.Plane, default);
            }
            state.CurrentRegion = -1;
            for (int member = 0; member < state.Inputs.Count; member++)
            {
                if (state.Members[member].Connectivity.RegionCount == 0) continue;
                state.CurrentMember = member;
                JoinedMember current = state.Members[member]; _frame = current.Frame;
                ReadOnlySpan<MeshSurfaceBoundaryDomain.BoundarySpan> spans = current.Boundary.Spans;
                for (int span = 0; span < spans.Length; span++)
                {
                    if (current.JoinedSpans[span] || !_frame.Finite.IntersectsSegment(new FixedSegment(spans[span].Start, spans[span].End))) continue;
                    VisitJoinedSource(state, new CapsulePlaneRayEventSource(new FixedSegment(spans[span].Start, spans[span].End)), spans[span].Interval);
                }
            }
        }
        for (int region = 0; region < RegionCount; region++) ReduceJoinedSamples(state, region);
        foreach (JoinedMember member in state.Members.AsReadOnlySpan()[..state.Inputs.Count])
        {
            member.PreparedVersion = member.Collider.RuntimeShapeVersion;
            member.PreparedLifetime = member.Collider.LifetimeVersion;
        }
        state.PreparedCount = state.Inputs.Count;
    }

    internal CapsulePlaneRayFrame GetBoundaryFrame(LSMeshCollider collider, int triangle, out Vector3d normal)
    {
        if (_joined?.Active != true) { normal = _normal; return _frame; }
        var frame = _joined.Members[_joined.MemberBySurface[(collider.Mesh, collider.Mesh.GetManifoldSurfaceOwner(triangle))]].Frame;
        normal = frame.GetWorldNormal(collider.Mesh.Rotation);
        return frame;
    }

    internal int GetBoundaryOrientation(LSMeshCollider collider, int triangle, int localRegion)
    {
        if (_joined?.Active != true) return GetOrientation(localRegion);
        JoinedState state = _joined;
        JoinedMember member = state.Members[state.MemberBySurface[(collider.Mesh, collider.Mesh.GetManifoldSurfaceOwner(triangle))]];
        int region = state.RegionByRoot[FindJoinedRoot(state, member.Offset + localRegion)];
        return WideAxis3.Dot(member.Frame.Normal, GetJoinedRegionFrame(state, region).Normal).Sign * _orientations[region];
    }

    internal bool IsJoinedEdge(LSMeshCollider collider, int triangle, int edge)
    {
        if (_joined?.Active != true) return false;
        JoinedMember member = _joined.Members[_joined.MemberBySurface[(collider.Mesh, collider.Mesh.GetManifoldSurfaceOwner(triangle))]];
        // Admission currently requires complete edges, hence one classified span per canonical edge.
        return member.JoinedSpans[edge];
    }

    internal ReadOnlySpan<WideAxis3> GetJoinedCornerFan(LSMeshCollider collider, int triangle, Vector3d point, out bool suppress)
    {
        suppress = false;
        if (_joined?.Active != true) return default;
        JoinedState state = _joined;
        JoinedMember member = state.Members[state.MemberBySurface[(collider.Mesh, collider.Mesh.GetManifoldSurfaceOwner(triangle))]];
        bool trusted = state.Fan.BuildAtPoint(state.Meshes.AsReadOnlySpan(), member.Boundary.SourceMember, triangle,
            point, state.PreparedBoundaries.AsReadOnlySpan());
        suppress = trusted && !state.Fan.HasExposedBoundary;
        // A shared true-rim endpoint is one physical feature. Keep its first
        // eligible collider/surface, even though every member retains geometric overlap.
        if (trusted)
            foreach (var owner in state.Fan.Owners)
                if (state.MemberBySurface.TryGetValue((state.Meshes[owner.Member], owner.Surface), out int other)
                    && (state.Members[other].Collider.Id < collider.Id
                        || state.Members[other].Collider == collider
                        && collider.Mesh.GetCanonicalSurfaceOrdinal(owner.Surface) < collider.Mesh.GetCanonicalSurfaceOrdinal(member.Owner)))
                    suppress = true;
        return trusted ? state.Fan.Rays : default;
    }

    internal bool AppendJoinedBoundaries(LSMeshCollider collider, Gravitas.Materials.PhysicsMaterial material,
        Vector3d center, FixedQuaternion rotation, Fixed64 coreLength, Fixed64 radius,
        MeshCapsuleSurfaceBoundary boundary, ContactManifold manifold)
    {
        bool overlaps = false;
        JoinedState state = _joined!;
        foreach (JoinedMember member in state.Members.AsReadOnlySpan()[..state.Inputs.Count])
        {
            if (member.Collider != collider || member.Connectivity.RegionCount == 0) continue;
            overlaps = true;
            boundary.BuildSurfaceContacts(collider, material, center, rotation, coreLength, radius,
                member.Triangle, member.Connectivity, this, manifold);
        }
        return overlaps;
    }

    private bool TryGatherContainedJoinedSupport(JoinedState state)
    {
        if (RegionCount != 1 || !state.Trusted[0]) return false;
        ref readonly CapsulePlaneRayFrame common = ref GetJoinedRegionFrame(state, 0);
        if (common.Finite.ShapeFrame.Radius.Sign <= 0 || common.Finite.Length != Fixed64.Zero
            && (!common.Normal.X.IsZero || !common.Normal.Z.IsZero)) return false;
        // Only the complete retained perimeter certifies containment. Testing
        // full original edges is conservative when a retained span is clipped.
        foreach (JoinedMember member in state.Members.AsReadOnlySpan()[..state.Inputs.Count])
        {
            if (member.Connectivity.RegionCount == 0) continue;
            ReadOnlySpan<MeshSurfaceBoundaryDomain.BoundarySpan> spans = member.Boundary.Spans;
            for (int span = 0; span < spans.Length; span++)
                if (!member.JoinedSpans[span] && member.Frame.Finite.IntersectsSegment(new FixedSegment(spans[span].Start, spans[span].End)))
                    return false;
        }
        Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        state.CurrentRegion = 0; state.CurrentMember = state.NodeMembers[state.Roots[0]]; state.CurrentClip = default;
        _frame = state.Members[state.CurrentMember].Frame;
        for (int orientation = 1; orientation >= -1; orientation -= 2)
        {
            var support = new CapsulePlaneRaySelection(values, signs, valueSigns);
            int endpoint = _frame.Finite.Length == Fixed64.Zero ? 0 : orientation * _frame.Normal.Y.Sign > 0 ? 1 : 0;
            var descriptor = new CapsulePlaneRayEvent(CapsulePlaneRayEventKind.Support, endpoint);
            bool found = CapsulePlaneRayEvents.Reconstruct(CapsulePlaneRayEventSource.Plane, _frame, descriptor, orientation, ref support);
            System.Diagnostics.Debug.Assert(found);
            AdmitJoinedPoint(CapsulePlaneRayEventSource.Plane, _frame, descriptor, orientation, support);
        }
        return true;
    }

    private void JoinFiniteSpans(JoinedState state)
    {
        for (int member = 0; member < state.Inputs.Count; member++)
        {
            JoinedMember source = state.Members[member];
            ReadOnlySpan<MeshSurfaceBoundaryDomain.BoundarySpan> spans = source.Boundary.Spans;
            for (int span = 0; span < spans.Length; span++)
            {
                var edge = spans[span];
                if (edge.Kind != MeshSurfaceBoundaryDomain.SpanKind.Joined) continue;
                if (!state.MemberBySurface.TryGetValue((state.Meshes[edge.OtherMember], edge.OtherSurface), out int other)) continue;
                JoinedMember target = state.Members[other];
                if (!edge.Interval.IsClipped && source.Connectivity.RegionCount == 1 && target.Connectivity.RegionCount == 1)
                {
                    // A complete certified shared edge meets the convex capsule
                    // in one closed interval belonging to both unique regions.
                    if (source.Frame.Finite.IntersectsSegment(new FixedSegment(edge.Start, edge.End)))
                    {
                        UnionJoinedRegions(state, source.Offset, target.Offset);
                        source.JoinedSpans[span] = true;
                    }
                    continue;
                }
                state.CurrentMember = member; state.OtherMember = other; state.CurrentSpan = span;
                CapsulePlaneRayEvents.VisitEvents(new CapsulePlaneRayEventSource(new FixedSegment(edge.Start, edge.End)),
                    source.Frame, 1, edge.Interval, state.JoinVisitor);
            }
        }
    }

    private void JoinAtPoint(in CapsulePlaneRayEventSource source, in CapsulePlaneRayFrame frame,
        CapsulePlaneRayEvent descriptor, int orientation, scoped in CapsulePlaneRaySelection point)
    {
        JoinedState state = _joined!;
        JoinedMember first = state.Members[state.CurrentMember], second = state.Members[state.OtherMember];
        foreach (int a in first.Connectivity.TriangleIndices)
        {
            first.Collider.Mesh.GetLocalTriangleVertices(a, out Vector3d aa, out Vector3d ab, out Vector3d ac);
            if (!point.ContainsTrianglePoint(frame, new FixedTriangle(aa, ab, ac))) continue;
            foreach (int b in second.Connectivity.TriangleIndices)
            {
                second.Collider.Mesh.GetLocalTriangleVertices(b, out Vector3d ba, out Vector3d bb, out Vector3d bc);
                if (!point.ContainsTrianglePoint(second.Frame, new FixedTriangle(ba, bb, bc), frame)) continue;
                UnionJoinedRegions(state, first.Offset + first.Connectivity.GetRegionOrdinal(a),
                    second.Offset + second.Connectivity.GetRegionOrdinal(b));
                first.JoinedSpans[state.CurrentSpan] = true;
            }
        }
    }

    private static void UnionJoinedRegions(JoinedState state, int first, int second)
    {
        int rootA = FindJoinedRoot(state, first), rootB = FindJoinedRoot(state, second);
        state.Parents[Math.Max(rootA, rootB)] = Math.Min(rootA, rootB);
    }

    private static int FindJoinedRoot(JoinedState state, int node)
    {
        while (state.Parents[node] != node)
        { state.Parents[node] = state.Parents[state.Parents[node]]; node = state.Parents[node]; }
        return node;
    }

    private void VisitJoinedSource(JoinedState state, in CapsulePlaneRayEventSource source, in RationalSegmentInterval clip)
    {
        state.CurrentClip = clip;
        CapsulePlaneRayEvents.VisitEvents(source, _frame, 1, clip, state.Visitor);
        CapsulePlaneRayEvents.VisitEvents(source, _frame, -1, clip, state.Visitor);
    }

    private void AdmitJoinedPoint(in CapsulePlaneRayEventSource source, in CapsulePlaneRayFrame frame,
        CapsulePlaneRayEvent descriptor, int orientation, scoped in CapsulePlaneRaySelection candidate)
    {
        JoinedState state = _joined!;
        state.VisitedRegions.Clear();
        int first = state.CurrentRegion < 0 ? state.CurrentMember : 0;
        int end = state.CurrentRegion < 0 ? first + 1 : state.Inputs.Count;
        for (int memberIndex = first; memberIndex < end; memberIndex++)
        {
            JoinedMember member = state.Members[memberIndex];
            foreach (int triangle in member.Connectivity.TriangleIndices)
            {
                int region = state.RegionByRoot[FindJoinedRoot(state, member.Offset + member.Connectivity.GetRegionOrdinal(triangle))];
                if (state.CurrentRegion >= 0 && region != state.CurrentRegion) continue;
                member.Collider.Mesh.GetLocalTriangleVertices(triangle, out Vector3d a, out Vector3d b, out Vector3d c);
                if (!candidate.ContainsTrianglePoint(member.Frame, new FixedTriangle(a, b, c), frame) || !state.VisitedRegions.Add(region)) continue;
                KeepJoinedPoint(state, memberIndex, region, source, frame, descriptor, orientation, candidate);
            }
        }
    }

    private void KeepJoinedPoint(JoinedState state, int member, int region, in CapsulePlaneRayEventSource source,
        in CapsulePlaneRayFrame frame, CapsulePlaneRayEvent descriptor, int orientation, scoped in CapsulePlaneRaySelection candidate)
    {
        bool represented = candidate.TryGetAuthoredPoint(frame, _center, _rotation, _center, _rotation, out Vector3d p);
        System.Diagnostics.Debug.Assert(represented);
        int point = RetainJoinedPoint(candidate, frame);
        var provenance = new JoinedWitness(member, state.CurrentMember, source, descriptor, state.CurrentClip);
        state.PointSources.Add(provenance);
        foreach (Sample old in _events.AsReadOnlySpan())
        {
            if (old.Region != region || old.Point != p
                || FiniteSectionPoint.CompareLexicographic(BorrowPoint(old.ExactPoint), BorrowPoint(point)) != 0) continue;
            _pointWordCount = _points[point].Offset;
            _points.RemoveAt(point); state.PointSources.RemoveAt(point); point = old.ExactPoint;
            break;
        }
        bool retained = false;
        foreach (Sample old in _events.AsReadOnlySpan())
            if (old.Region == region && old.ExactPoint == point) { retained = true; break; }
        if (!retained) _events.Add(new Sample(source, descriptor, region, p, point));
        if (source.IsSegment) { HasBoundaryIntersection = true; state.BoundaryHits[region] = true; }
        ref readonly CapsulePlaneRayFrame common = ref GetJoinedRegionFrame(state, region);
        int aligned = WideAxis3.Dot(frame.Normal, common.Normal).Sign * orientation;
        int slot = 2 * region + (aligned > 0 ? 0 : 1);
        var winner = BorrowWinner(slot);
        int comparison = !winner.HasValue ? 1 : candidate.CompareCoplanarDepth(winner, frame,
            state.Members[state.Maximum[slot].FrameMember].Frame);
        if (comparison < 0) return;
        if (comparison == 0)
        {
            int order = FiniteSectionPoint.CompareLexicographic(BorrowPoint(point), BorrowPoint(state.MaximumPoint[slot]));
            if (order > 0 || order == 0 && CompareJoinedWitness(state, provenance, state.Maximum[slot]) >= 0) return;
        }
        candidate.Values.CopyTo(winner.Values); candidate.Signs.CopyTo(winner.Signs); candidate.ValueSigns.CopyTo(winner.ValueSigns);
        winner.Restore(candidate.Metadata); _winners[slot] = winner.Metadata;
        state.Maximum[slot] = provenance; state.MaximumPoint[slot] = point;
    }

    private static ref readonly CapsulePlaneRayFrame GetJoinedRegionFrame(JoinedState state, int region) =>
        ref state.Members[state.NodeMembers[state.Roots[region]]].Frame;

    private int RetainJoinedPoint(scoped in CapsulePlaneRaySelection point, in CapsulePlaneRayFrame frame)
    {
        int words = Math.Max(Math.Max(ContactQuadratic.ActiveWords(point.X), ContactQuadratic.ActiveWords(point.Y)),
            Math.Max(ContactQuadratic.ActiveWords(point.Z), ContactQuadratic.ActiveWords(point.Denominator))) + 3;
        int rootWords = WideArithmetic.GetActiveMagnitudeLength(point.Root);
        int index = _points.Count, needed = _pointWordCount + 8 * words + rootWords;
        if (_pointValues.Length < needed) Array.Resize(ref _pointValues, Math.Max(needed, 2 * _pointValues.Length));
        if (_pointSigns.Length < (index + 1) * FiniteSectionPoint.SignCount)
            Array.Resize(ref _pointSigns, Math.Max((index + 1) * FiniteSectionPoint.SignCount, Math.Max(64, 2 * _pointSigns.Length)));
        _points.Add(new PointStorage(_pointWordCount, words, rootWords)); _pointWordCount = needed;
        var retained = BorrowPoint(index);
        point.X.CopyTo(retained.Coordinate(0)); point.Y.CopyTo(retained.Coordinate(1)); point.Z.CopyTo(retained.Coordinate(2));
        Signed192 d = frame.Finite.ShapeFrame.Denominator;
        ContactQuadratic.Scale(point.Denominator, Signed576.ExtendValue(Signed320.ExtendValue(WideArithmetic.AddSigned192(d, d))), retained.Denominator);
        point.Root[..rootWords].CopyTo(retained.Root);
        return index;
    }

    private static int CompareJoinedWitness(JoinedState state, JoinedWitness first, JoinedWitness second)
    {
        int order = state.Members[first.Member].Collider.Id.CompareTo(state.Members[second.Member].Collider.Id);
        if (order != 0) return order;
        return state.Members[first.Member].Owner.CompareTo(state.Members[second.Member].Owner);
    }

    private int CompareJoinedPoints(Sample first, Sample second)
    {
        return FiniteSectionPoint.CompareLexicographic(BorrowPoint(first.ExactPoint), BorrowPoint(second.ExactPoint));
    }

    private void ReduceJoinedSamples(JoinedState state, int region)
    {
        var positive = BorrowWinner(2 * region); var negative = BorrowWinner(2 * region + 1);
        var positiveFrame = state.Members[state.Maximum[2 * region].FrameMember].Frame;
        var negativeFrame = state.Members[state.Maximum[2 * region + 1].FrameMember].Frame;
        int orientation = positive.CompareCoplanarDepth(negative, positiveFrame, negativeFrame) <= 0 ? 1 : -1;
        _orientations[region] = orientation;
        int slot = 2 * region + (orientation > 0 ? 0 : 1), point = state.MaximumPoint[slot];
        _pool.FastClear();
        foreach (Sample sample in _events.AsReadOnlySpan())
            if (sample.Region == region && sample.ExactPoint == point) { _pool.Add(sample); break; }
        foreach (Sample sample in _events.AsReadOnlySpan())
            if (sample.Region == region && sample.ExactPoint != point) _pool.Add(sample);
        ref readonly CapsulePlaneRayFrame common = ref GetJoinedRegionFrame(state, region);
        var metrics = new WidePlaneMetrics(common.Normal);
        // A nonempty connected section cannot leave an embedded filled union
        // without crossing its complete retained boundary. Only that global
        // certificate permits dropping off-peak samples for a strictly curved support.
        bool contained = state.Trusted[region] && !state.BoundaryHits[region];
        bool uniqueSupport = common.Finite.ShapeFrame.Radius.Sign > 0
            && (common.Finite.Length == Fixed64.Zero || common.Normal.X.IsZero && common.Normal.Z.IsZero);
        int count = contained && uniqueSupport ? 1 : Math.Min(ContactManifold.MaxContactsPerGroup, _pool.Count);
        Span<int> chosen = stackalloc int[ContactManifold.MaxContactsPerGroup]; chosen[0] = 0;
        for (int index = 0; index < count; index++)
        {
            if (index > 0) chosen[index] = MeshPlaneSampleReduction.ChooseNext(_pool.AsReadOnlySpan(), chosen[..index], metrics, state.PointComparer);
            _samples[region * ContactManifold.MaxContactsPerGroup + index] = _pool[chosen[index]];
        }
        _counts[region] = count;
    }

    internal bool TryGetJoinedSample(int region, int index, out FixedContactAnchors contact, out LSMeshCollider? supplier)
    {
        contact = default; supplier = null;
        if (_joined == null || !_joined.Active || (uint)region >= (uint)RegionCount || (uint)index >= (uint)_counts[region]) return false;
        Sample sample = _samples[region * ContactManifold.MaxContactsPerGroup + index];
        int slot = 2 * region + (_orientations[region] > 0 ? 0 : 1);
        JoinedWitness witness = index == 0 ? _joined.Maximum[slot] : _joined.PointSources[sample.ExactPoint];
        JoinedMember member = _joined.Members[witness.Member]; supplier = member.Collider;
        JoinedMember producer = _joined.Members[witness.FrameMember];
        ref readonly CapsulePlaneRayFrame common = ref GetJoinedRegionFrame(_joined, region);
        int orientation = WideAxis3.Dot(producer.Frame.Normal, common.Normal).Sign * _orientations[region];
        Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var selection = index == 0 ? BorrowWinner(slot) : new CapsulePlaneRaySelection(values, signs, valueSigns);
        if (index != 0)
        {
            bool found = CapsulePlaneRayEvents.Reconstruct(witness.Source, producer.Frame, witness.Event, orientation, witness.Clip, ref selection);
            System.Diagnostics.Debug.Assert(found);
        }
        PhysicsMesh mesh = member.Collider.Mesh;
        bool represented = selection.TryGetAuthoredPoint(producer.Frame, mesh.Origin, mesh.Rotation, _center, _rotation, out Vector3d authored)
            & selection.TryGetExitAnchor(producer.Frame, orientation, _center, _rotation, out FixedPointAnchor exit, out Fixed64 depth, out bool clamped);
        contact = new FixedContactAnchors(new FixedPointAnchor(mesh.Origin, mesh.Rotation, authored), exit,
            -producer.Frame.GetWorldNormal(producer.Collider.Mesh.Rotation) * orientation, depth, clamped);
        return represented;
    }

    private readonly struct JoinedWitness
    {
        internal readonly int Member, FrameMember;
        internal readonly CapsulePlaneRayEventSource Source;
        internal readonly CapsulePlaneRayEvent Event;
        internal readonly RationalSegmentInterval Clip;
        internal JoinedWitness(int member, int frameMember, CapsulePlaneRayEventSource source, CapsulePlaneRayEvent descriptor,
            RationalSegmentInterval clip)
        { Member = member; FrameMember = frameMember; Source = source; Event = descriptor; Clip = clip; }
    }

    private sealed class JoinedMember
    {
        internal LSMeshCollider Collider = null!;
        internal int Triangle, Owner, Offset;
        internal uint PreparedVersion;
        internal long PreparedLifetime;
        internal CapsulePlaneRayFrame Frame;
        internal readonly MeshSurfaceConnectivity Connectivity = new();
        internal MeshSurfaceBoundaryDomain Boundary = new();
        internal readonly SwiftList<int> Admitted = new();
        internal bool[] JoinedSpans = Array.Empty<bool>();
    }

    private sealed class JoinedState
    {
        internal readonly SwiftList<(LSMeshCollider Collider, int Triangle)> Inputs = new();
        internal readonly SwiftList<PhysicsMesh> Meshes = new();
        internal readonly SwiftList<int> Candidates = new();
        internal readonly SwiftList<MeshSurfaceBoundaryDomain> PreparedBoundaries = new();
        internal readonly MeshSurfaceBoundaryFan Fan = new();
        internal readonly SwiftDictionary<(PhysicsMesh Mesh, int Owner), int> MemberBySurface = new();
        internal readonly SwiftList<JoinedMember> Members = new();
        internal readonly SwiftList<int> Parents = new(), Roots = new(), RegionByRoot = new(), NodeMembers = new();
        internal readonly SwiftList<JoinedWitness> PointSources = new();
        internal readonly SwiftHashSet<int> VisitedRegions = new();
        internal JoinedWitness[] Maximum = Array.Empty<JoinedWitness>();
        internal int[] MaximumPoint = Array.Empty<int>();
        internal bool[] BoundaryHits = Array.Empty<bool>(), Trusted = Array.Empty<bool>();
        internal readonly IComparer<(LSMeshCollider Collider, int Triangle)> MemberComparer;
        internal readonly IComparer<Sample> PointComparer;
        internal readonly CapsulePlaneRayEventVisitor Visitor, JoinVisitor;
        internal int CurrentMember, OtherMember, CurrentSpan, CurrentRegion, PreparedCount = -1;
        internal bool Active;
        internal RationalSegmentInterval CurrentClip;
        internal JoinedState(MeshCapsuleFaceRegions owner)
        {
            MemberComparer = Comparer<(LSMeshCollider Collider, int Triangle)>.Create((a, b) =>
            { int order = a.Collider.Id.CompareTo(b.Collider.Id); return order != 0 ? order : a.Triangle.CompareTo(b.Triangle); });
            PointComparer = Comparer<Sample>.Create(owner.CompareJoinedPoints);
            Visitor = owner.AdmitJoinedPoint; JoinVisitor = owner.JoinAtPoint;
        }
    }
}
