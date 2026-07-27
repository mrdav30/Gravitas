//=======================================================================
// CollisionDetection.Capsule.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;

namespace Gravitas.CollisionHandling;

public static partial class CollisionDetection
{
    #region Capsule

    private static bool DoCapsuleSphereCheck(CollisionWorkItem pair)
    {
        var capsule = (LSCapsuleCollider)pair.ColliderA;
        var sphere = (LSSphereCollider)pair.ColliderB;

        if (!FixedSegment.TryGetCenteredCapsulesContact(
                capsule.Center,
                capsule.Rotation,
                Vector3d.Up,
                capsule.AxisLength,
                capsule.ScaledRadius,
                sphere.Center,
                sphere.Rotation,
                Vector3d.Up,
                Fixed64.Zero,
                sphere.ScaledRadius,
                ResolveCenteredCapsuleFallback(capsule.Center, sphere.Center),
                out FixedContactAnchors contact))
        {
            return false;
        }

        pair.Manifold.SetContact(
            new ContactAnchor(contact.FirstAnchor),
            new ContactAnchor(contact.SecondAnchor),
            contact.Depth,
            contact.Normal,
            contact.DepthIsClamped);
        return true;
    }

    private static bool DoCapsulesCheck(CollisionWorkItem pair)
    {
        var capsule1 = (LSCapsuleCollider)pair.ColliderA;
        var capsule2 = (LSCapsuleCollider)pair.ColliderB;

        if (!FixedSegment.TryGetCenteredCapsulesContact(
                capsule1.Center,
                capsule1.Rotation,
                Vector3d.Up,
                capsule1.AxisLength,
                capsule1.ScaledRadius,
                capsule2.Center,
                capsule2.Rotation,
                Vector3d.Up,
                capsule2.AxisLength,
                capsule2.ScaledRadius,
                ResolveCenteredCapsuleFallback(capsule1.Center, capsule2.Center),
                out FixedContactAnchors contact))
        {
            return false;
        }

        pair.Manifold.SetContact(
            new ContactAnchor(contact.FirstAnchor),
            new ContactAnchor(contact.SecondAnchor),
            contact.Depth,
            contact.Normal,
            contact.DepthIsClamped);
        return true;
    }

    private static Vector3d ResolveCenteredCapsuleFallback(
        Vector3d firstCenter,
        Vector3d secondCenter)
    {
        if (secondCenter.X != firstCenter.X)
            return secondCenter.X > firstCenter.X ? Vector3d.Right : Vector3d.Left;
        if (secondCenter.Y != firstCenter.Y)
            return secondCenter.Y > firstCenter.Y ? Vector3d.Up : Vector3d.Down;
        if (secondCenter.Z != firstCenter.Z)
            return secondCenter.Z > firstCenter.Z ? Vector3d.Forward : Vector3d.Backward;
        return Vector3d.Right;
    }

    #endregion

}
