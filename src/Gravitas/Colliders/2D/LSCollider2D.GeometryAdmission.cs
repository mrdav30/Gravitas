// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.

using FixedMathSharp;

namespace Gravitas.Colliders;

/// <content>Preflights planar mutations against the complete owner envelope before waking.</content>
public abstract partial class LSCollider2D
{
    private protected void ValidateGeometryMutation(Vector2d? offset = null,
        Fixed64? radius = null, Fixed64? height = null, Vector2d? size = null,
        Fixed64? halfThickness = null)
    {
        LSCollider2D root = _compoundOwner ?? this;
        if (root._agent == null)
            return;

        Vector2d ownerScale = ColliderScalePolicy.CapturePlanar(root._agent.Transform,
            out _, out _, captureRotation: false);
        Vector2d partScale = _compoundOwner == null ? Vector2d.One : _compoundLocalScale;
        Fixed64 intrinsic = this is LSCompoundCollider2D
            ? Fixed64.Zero : GetCandidateGeometryReach(ownerScale, partScale, radius, height, size);
        Fixed64 reach = root is LSCompoundCollider2D compound
            ? compound.GetMutationGeometryReach(this, ownerScale, intrinsic, offset ?? _localOffset)
            : intrinsic;
        if (root._body != null)
            reach = ColliderGeometryPolicy.Add(ColliderGeometryPolicy.Attachment(
                ReferenceEquals(root, this) ? offset ?? _localOffset : root._localOffset, ownerScale), reach);
        if (root._context!.Settings.RuntimeMode.RunsMixedContacts())
        {
            Fixed64 thickness = ReferenceEquals(root, this) && halfThickness.HasValue
                ? halfThickness.Value
                : root._mixedHalfThicknessOverride ?? root._context.Settings.Mixed2DHalfThickness;
            reach = ColliderGeometryPolicy.Mixed(reach, thickness);
        }
        ColliderGeometryPolicy.ValidateReach(reach, nameof(intrinsic));
    }

    internal Fixed64 GetCandidateGeometryReach(Vector2d ownerScale, Vector2d partScale,
        Fixed64? radius = null, Fixed64? height = null, Vector2d? size = null)
    {
        switch (this)
        {
            case LSCircleCollider2D circle:
                Fixed64 r = radius ?? circle.Radius;
                return FixedMath.Max(ColliderScalePolicy.ScalePositive(r, ownerScale.X, partScale.X),
                    ColliderScalePolicy.ScalePositive(r, ownerScale.Y, partScale.Y));
            case LSCapsuleCollider2D capsule:
                LSCapsuleCollider2D.GetScaledGeometry(radius ?? capsule.Radius, height ?? capsule.Height,
                    ownerScale, partScale, out Fixed64 scaledRadius, out Fixed64 length);
                return ColliderGeometryPolicy.Capsule(scaledRadius, length);
            case LSAABBoxCollider2D box:
                return ColliderGeometryPolicy.Magnitude(ColliderScalePolicy.ScalePositive(
                    size ?? box.Size, ownerScale, partScale, Fixed64.Two));
            case LSPolygonCollider2D polygon:
                return polygon.GetCandidateGeometryReach(ownerScale, partScale);
            default:
                return Fixed64.MaxValue;
        }
    }
}
