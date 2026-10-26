// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System;
using SwiftCollections.Query;

namespace Gravitas;

/// <content>Reuses joined finite-slab response for an embedded planar participant.</content>
internal sealed partial class StaticSurface
{
    private bool _mixedSlab;
    private Fixed64 _mixedLength;

    internal bool AppendMixed(LSMeshCollider overlap, LSCollider2D mover, ContactManifold destination,
        int namespace3D, int namespace2D)
    {
        Vector3d center = MixedEmbedded2DGeometry.GetCenter3D(mover);
        Fixed64 reach = mover.CanonicalCenteredProxyRadius * Fixed64.Two + mover.MixedHalfThickness;
        FixedBoundBox box = FixedBoundBox.FromCenterAndScopeClippedToDomain(center, Vector3d.One * reach);
        Gather(null, mover, new(box.Min, box.Max));
        Fixed64 yaw = mover is LSAABBoxCollider2D ? Fixed64.Zero : mover.Rotation;
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, -yaw);
        Span<Vector2d> boxOffsets = stackalloc Vector2d[4];
        ReadOnlySpan<Vector2d> offsets = CollisionDetectionMixed.GetConvexPrismOffsets(mover, boxOffsets);
        Fixed64 length = mover is LSCapsuleCollider2D capsule ? capsule.AxisLength : Fixed64.Zero;
        Fixed64 radius = mover is LSCircleCollider2D circle ? circle.ScaledRadius
            : mover is LSCapsuleCollider2D stadium ? stadium.ScaledRadius : Fixed64.Zero;
        bool reuse = _facesCurrent && _cachedFamily == ContactFamily.Slab && _mixedSlab
            && _center == center && _rotation == rotation && _mixedLength == length && _radius == radius
            && _coreLength == mover.MixedHalfThickness && SlabFaces.PrismOffsets.Span.SequenceEqual(offsets)
            && _cachedOwners.Count == _owners.Count;
        foreach (var owner in _cachedOwners.AsReadOnlySpan()) reuse &= _owners.Contains(owner);
        if (!reuse)
        {
            _facesCurrent = false; _cachedFamily = ContactFamily.Slab;
            SlabFaces.BuildJoined(_surfaces.AsReadOnlySpan(), center, rotation, length, radius,
                mover.MixedHalfThickness, offsets,
                new(mover.MixedBounds3D.Min, mover.MixedBounds3D.Max), false, _meshes, _boundaries);
            _cachedOwners.Clear();
            foreach (var surface in _surfaces.AsReadOnlySpan())
                _cachedOwners.Add((surface.Collider.Mesh, surface.Collider.Mesh.GetManifoldSurfaceOwner(surface.Triangle)));
            _mixedSlab = true; _mixedLength = length; _radius = radius;
            _center = center; _rotation = rotation; _coreLength = mover.MixedHalfThickness; _facesCurrent = true;
        }
        return SlabFaces.AppendJoinedContacts(overlap, mover.Material, _context.CollisionScratch.MeshSlabBoundary,
            destination, mover, namespace3D, namespace2D);
    }
}
