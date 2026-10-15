// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.

using FixedMathSharp;

namespace Gravitas.Colliders;

/// <content>Preflights authored mutations before changing shape state or waking an owner.</content>
public abstract partial class LSCollider
{
    private void ValidateGeometryMutation(Vector3d offset, Vector3d size, Fixed64 radius)
    {
        LSCollider root = _compoundOwner ?? this;
        if (root._agent == null)
            return; // Detached authoring may be compensated by a later host or part scale.

        Vector3d ownerScale = ColliderScalePolicy.CaptureScale(root._agent.Transform);
        var candidate = new ColliderShapeSnapshot(default, default, ownerScale,
            _compoundOwner == null ? Vector3d.One : _compoundLocalScale, offset, size, radius);
        Fixed64 reach = root is LSCompoundCollider compound
            ? compound.GetMutationGeometryReach(this, candidate)
            : GetCandidateGeometryReach(candidate);
        if (root._body != null)
            reach = ColliderGeometryPolicy.Add(ColliderGeometryPolicy.Attachment(
                ReferenceEquals(root, this) ? offset : root._offset, ownerScale), reach);
        ColliderGeometryPolicy.ValidateReach(reach, nameof(candidate));
    }

    internal Fixed64 GetCandidateGeometryReach(in ColliderShapeSnapshot snapshot)
    {
        switch (this)
        {
            case LSSphereCollider:
                Vector3d radii = ColliderScalePolicy.ScalePositive(
                    Vector3d.One * snapshot.Radius, snapshot.OwnerScale, snapshot.PartScale, Fixed64.One);
                return FixedMath.Max(radii.X, FixedMath.Max(radii.Y, radii.Z));
            case LSCapsuleCollider:
                LSCapsuleCollider.GetScaledGeometry(snapshot, out Fixed64 radius, out Fixed64 length);
                return ColliderGeometryPolicy.Capsule(radius, length);
            case LSCuboidCollider:
                return ColliderGeometryPolicy.Magnitude(ColliderScalePolicy.ScalePositive(
                    snapshot.Size, snapshot.OwnerScale, snapshot.PartScale, Fixed64.Two));
            case LSConeCollider:
            case LSCylinderCollider:
                return ColliderGeometryPolicy.FiniteAxis(FixedMath.Max(
                    ColliderScalePolicy.ScalePositive(snapshot.Radius, snapshot.OwnerScale.X, snapshot.PartScale.X),
                    ColliderScalePolicy.ScalePositive(snapshot.Radius, snapshot.OwnerScale.Z, snapshot.PartScale.Z)),
                    ColliderScalePolicy.ScalePositive(snapshot.Size.Y, snapshot.OwnerScale.Y, snapshot.PartScale.Y));
            case LSMeshCollider mesh:
                return mesh.GeometryMesh.GetScaledLocalRadius(snapshot.OwnerScale, snapshot.PartScale);
            default:
                return Fixed64.MaxValue;
        }
    }
}
