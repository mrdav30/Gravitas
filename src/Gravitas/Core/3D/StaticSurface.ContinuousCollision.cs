// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using SwiftCollections;
using SwiftCollections.Query;
using System;

namespace Gravitas;

/// <content>Admits CCD witness normals against the existing exact incident surface sectors.</content>
internal sealed partial class StaticSurface
{
    private readonly MeshSurfaceBoundaryFan _continuousFan = new();
    private readonly SwiftList<MeshSurfaceBoundaryDomain> _continuousBoundaries = new();
    private readonly SwiftList<LSMeshCollider> _continuousMixedMembers = new();
    private ContactManifold? _continuousMixedFiltered;
    internal int ContinuousOrder => ((IPhysicsColliderRegistryItem)_members[0]).ReplayOrder;

    internal bool IntersectsMixedContinuous(LSCollider2D mover, Fixed64 padding = default,
        ReadOnlySpan<(LSMeshCollider Supplier, int Owner)> resolved = default)
    {
        if (!GatherMixedContinuous(mover, padding)) return true;
        foreach (var surface in _surfaces.AsReadOnlySpan())
        {
            LSMeshCollider member = surface.Collider;
            int owner = member.Mesh.GetManifoldSurfaceOwner(surface.Triangle);
            bool handled = IsHandledContinuousFamily(member, owner, resolved);
            if (padding > Fixed64.Zero && ContinuousCollisionBounds.AreSeparated(member, mover, padding)) continue;
            if (CollisionDetectionMixed.IntersectsForContinuousCollision(member, mover, padding, owner,
                handled ? SlabFaces : null)) return true;
        }
        return false;
    }

    internal void AppendMixedContinuousContacts(LSCollider2D mover, ContactManifold destination,
        ReadOnlySpan<(LSMeshCollider Supplier, int Owner)> resolved = default)
    {
        GatherMixedContinuous(mover, Fixed64.Zero);
        _continuousMixedMembers.Clear();
        try
        {
            // Native emission gathers projected suppliers again. Snapshot only
            // the local carrier candidates before those existing owners run.
            LSMeshCollider? previous = null;
            foreach (var surface in _surfaces.AsReadOnlySpan())
                if (surface.Collider != previous)
                { previous = surface.Collider; _continuousMixedMembers.Add(previous); }
            foreach (LSMeshCollider member in _continuousMixedMembers.AsReadOnlySpan())
            {
                ContactManifold target = resolved.IsEmpty ? destination : (_continuousMixedFiltered ??= new());
                if (!resolved.IsEmpty) target.BeginUpdate(_context.FrameCount);
                CollisionDetectionMixed.AppendManifold(member, mover, target, 0, 0, surfaceResponse: true);
                if (resolved.IsEmpty) continue;
                for (int index = 0; index < target.GroupCount; index++)
                {
                    ref ContactGroup group = ref target.GetGroup(index);
                    if (SlabFaces.TryGetContinuousFamily(group.Key, out LSMeshCollider supplier, out int owner)
                        && IsHandledContinuousFamily(supplier, owner, resolved)) continue;
                    for (int point = 0; point < group.Count; point++) destination.AddContact(group.Key, group[point]);
                }
            }
        }
        finally { _continuousMixedMembers.Clear(); _continuousMixedFiltered?.Reset(); }
    }

    internal void CaptureMixedContinuousFamilies(ContactManifold manifold, in ContactResponseSnapshot impact,
        LSCollider2D mover, SwiftList<(LSMeshCollider Supplier, int Owner)> destination)
    {
        destination.Clear();
        for (int index = 0; index < manifold.GroupCount; index++)
        {
            ref ContactGroup group = ref manifold.GetGroup(index);
            if (!SlabFaces.TryGetContinuousFamily(group.Key, out LSMeshCollider supplier, out int owner)) continue;
            for (int point = 0; point < group.Count; point++)
                if (MixedContact.IsClosingContact(group[point], impact.CenterA, impact.LinearA, impact.AngularA,
                    impact.CenterB, impact.LinearB, impact.AngularB, impact.PlanarAngularB, mover))
                { destination.Add((supplier, owner)); break; }
        }
    }

    private bool IsHandledContinuousFamily(LSMeshCollider member, int owner,
        ReadOnlySpan<(LSMeshCollider Supplier, int Owner)> resolved)
    {
        if (!_facesCurrent || !_mixedSlab) return false;
        foreach (var family in resolved)
            if (family.Supplier.StaticSurface == this
                && SlabFaces.IsSameContinuousFamily(member, owner, family.Supplier, family.Owner)) return true;
        return false;
    }

    private bool GatherMixedContinuous(LSCollider2D mover, Fixed64 padding)
    {
        FixedBoundBox box = mover.MixedBounds3D;
        if (padding > Fixed64.Zero)
        {
            Vector3d expansion = Vector3d.One * padding;
            if (!Vector3d.TrySubtract(box.Min, expansion, out Vector3d minimum)
                || !Vector3d.TryAdd(box.Max, expansion, out Vector3d maximum)) return false;
            box = FixedBoundBox.FromMinMax(minimum, maximum);
        }
        Gather(null, mover, new FixedBoundVolume(box.Min, box.Max));
        return true;
    }

    internal bool AdmitContinuousWitness(LSCollider? mover, LSCollider2D? embedded, Vector3d center,
        LSMeshCollider target, int triangle, in ContactAnchor witness,
        WideDirection3d? exactOutward = null, Fixed64? circleSlabHalfThickness = null)
    {
        System.Diagnostics.Debug.Assert(triangle >= 0);
        Fixed64 reach = mover != null ? mover.CanonicalCenteredProxyRadius * Fixed64.Two
            : embedded!.CanonicalCenteredProxyRadius * Fixed64.Two + embedded.MixedHalfThickness;
        FixedBoundBox box = FixedBoundBox.FromCenterAndScopeClippedToDomain(center, Vector3d.One * reach);
        Gather(mover, embedded, new(box.Min, box.Max));
        _continuousBoundaries.FastClear();
        foreach (var surface in _surfaces.AsReadOnlySpan())
            _continuousBoundaries.Add(_boundaries[(surface.Collider.Mesh,
                surface.Collider.Mesh.GetManifoldSurfaceOwner(surface.Triangle))]);
        foreach (var boundary in _continuousBoundaries.AsReadOnlySpan())
        {
            if (_meshes[boundary.SourceMember] != target.Mesh
                || boundary.SourceSurface != target.Mesh.GetManifoldSurfaceOwner(triangle)) continue;
            // Only exact original-boundary incidence can remove a conservative
            // cast. Unknown/interior witnesses keep their geometric result.
            if (_continuousFan.BuildAtAnchor(_meshes, boundary.SourceMember, triangle,
                    witness.GeometryPoint, _continuousBoundaries.AsReadOnlySpan()))
            {
                RationalPoint3d point = RationalPoint3d.FromAnchor(witness.GeometryPoint);
                if (circleSlabHalfThickness.HasValue)
                {
                    // A finite circle/slab has a vertical core. Its side ray
                    // is horizontal while the witness is inside that core;
                    // cap/rim provenance stays conservative here.
                    RationalPoint3d lower = RationalPoint3d.FromAnchor(new FixedPointAnchor(center,
                        FixedQuaternion.Identity, new Vector3d(Fixed64.Zero, -circleSlabHalfThickness.Value, Fixed64.Zero)));
                    RationalPoint3d upper = RationalPoint3d.FromAnchor(new FixedPointAnchor(center,
                        FixedQuaternion.Identity, new Vector3d(Fixed64.Zero, circleSlabHalfThickness.Value, Fixed64.Zero)));
                    exactOutward = null;
                    if (RationalPoint3d.CompareProjection(point, lower, Vector3d.Up) >= 0
                        && RationalPoint3d.CompareProjection(point, upper, Vector3d.Up) <= 0)
                    {
                        WideDirection3d radial = WideTriangleRelations.GetScaledPointDifference(RationalPoint3d.FromScalarPoint(center),
                            point, WideArithmetic.TruncateSigned320By64(point.Denominator));
                        exactOutward = new WideDirection3d(radial.X, default, radial.Z);
                    }
                }
                // A rounded normal without its producer's axis cannot certify
                // exclusion. Rounded witnesses must also retain the original
                // finite triangle's supporting-point contract.
                return !exactOutward.HasValue || exactOutward.Value.IsZero
                    || !MeshSurfaceBoundaryFan.SupportsTriangle(target.Mesh, triangle,
                        point, exactOutward.Value)
                    || _continuousFan.Supports(exactOutward.Value);
            }
        }
        return true;
    }
}
