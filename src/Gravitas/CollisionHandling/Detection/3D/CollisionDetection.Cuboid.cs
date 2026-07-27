//=======================================================================
// CollisionDetection.Cuboid.cs
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
    #region Cuboid

    private static bool DoCuboidSphereCheck(CollisionWorkItem pair)
    {
        var cuboid = (LSCuboidCollider)pair.ColliderA;
        if (!cuboid.OrientedBox.TryGetSphereContact(
                pair.ColliderB.Center,
                pair.ColliderB.Rotation,
                pair.ColliderB.ScaledRadius,
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

    private static bool DoCuboidCapsuleCheck(CollisionWorkItem pair)
    {
        var cuboid = (LSCuboidCollider)pair.ColliderA;
        var capsule = (LSCapsuleCollider)pair.ColliderB;

        if (!cuboid.OrientedBox.TryGetCenteredCapsuleContact(
                capsule.Center,
                capsule.Rotation,
                Vector3d.Up,
                capsule.AxisLength,
                capsule.ScaledRadius,
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

    /// <summary>
    /// Checks for collisions between two poly-poly colliders.
    /// </summary>
    /// <returns>true if a collision is detected, false otherwise.</returns>
    private static bool DoCuboidsCheck(CollisionWorkItem pair)
    {
        var cuboidA = (LSCuboidCollider)pair.ColliderA;
        var cuboidB = (LSCuboidCollider)pair.ColliderB;

        if (cuboidA.Shape == ColliderType.AABox
            && cuboidB.Shape == ColliderType.AABox
            && CanBuildAxisAlignedManifold(cuboidA)
            && CanBuildAxisAlignedManifold(cuboidB))
        {
            return TryBuildAxisAlignedCuboidManifold(pair, cuboidA, cuboidB);
        }

        if (!cuboidA.OrientedBox.TryGetContact(
                cuboidB.OrientedBox,
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

    private static bool CanBuildAxisAlignedManifold(LSCuboidCollider cuboid)
    {
        Vector3d center = cuboid.Center;
        Vector3d halfExtents = cuboid.OrientedBox.HalfExtents;
        // The four-point fast path materializes both bounds and subtracts their
        // overlap widths. Wider conceptual boxes use the exact relative path.
        return Vector3d.TryAdd(center, halfExtents, out _)
            && Vector3d.TrySubtract(center, halfExtents, out _)
            && Vector3d.TryAdd(halfExtents, halfExtents, out _);
    }

    private static bool TryBuildAxisAlignedCuboidManifold(
        CollisionWorkItem pair,
        LSCuboidCollider cuboidA,
        LSCuboidCollider cuboidB)
    {
        Fixed64 overlapX = FixedMath.Min(cuboidA.BoundsMax.X, cuboidB.BoundsMax.X) - FixedMath.Max(cuboidA.BoundsMin.X, cuboidB.BoundsMin.X);
        Fixed64 overlapY = FixedMath.Min(cuboidA.BoundsMax.Y, cuboidB.BoundsMax.Y) - FixedMath.Max(cuboidA.BoundsMin.Y, cuboidB.BoundsMin.Y);
        Fixed64 overlapZ = FixedMath.Min(cuboidA.BoundsMax.Z, cuboidB.BoundsMax.Z) - FixedMath.Max(cuboidA.BoundsMin.Z, cuboidB.BoundsMin.Z);

        if (overlapX < Fixed64.Zero || overlapY < Fixed64.Zero || overlapZ < Fixed64.Zero)
            return false;

        Vector3d centerDelta = cuboidB.Center - cuboidA.Center;
        int axis = 0;
        Fixed64 depth = overlapX;
        if (overlapY < depth)
        {
            axis = 1;
            depth = overlapY;
        }

        if (overlapZ < depth)
        {
            axis = 2;
            depth = overlapZ;
        }

        Vector3d normal = axis switch
        {
            0 => new Vector3d(centerDelta.X < Fixed64.Zero ? -Fixed64.One : Fixed64.One, Fixed64.Zero, Fixed64.Zero),
            1 => new Vector3d(Fixed64.Zero, centerDelta.Y < Fixed64.Zero ? -Fixed64.One : Fixed64.One, Fixed64.Zero),
            _ => new Vector3d(Fixed64.Zero, Fixed64.Zero, centerDelta.Z < Fixed64.Zero ? -Fixed64.One : Fixed64.One)
        };

        AddAxisAlignedCuboidContacts(pair.Manifold, cuboidA, cuboidB, axis, depth, normal);
        return pair.Manifold.HasContact;
    }

    private static void AddAxisAlignedCuboidContacts(
        ContactManifold manifold,
        LSCuboidCollider cuboidA,
        LSCuboidCollider cuboidB,
        int axis,
        Fixed64 depth,
        Vector3d normal)
    {
        Fixed64 minX = FixedMath.Max(cuboidA.BoundsMin.X, cuboidB.BoundsMin.X);
        Fixed64 maxX = FixedMath.Min(cuboidA.BoundsMax.X, cuboidB.BoundsMax.X);
        Fixed64 minY = FixedMath.Max(cuboidA.BoundsMin.Y, cuboidB.BoundsMin.Y);
        Fixed64 maxY = FixedMath.Min(cuboidA.BoundsMax.Y, cuboidB.BoundsMax.Y);
        Fixed64 minZ = FixedMath.Max(cuboidA.BoundsMin.Z, cuboidB.BoundsMin.Z);
        Fixed64 maxZ = FixedMath.Min(cuboidA.BoundsMax.Z, cuboidB.BoundsMax.Z);

        switch (axis)
        {
            case 0:
                {
                    Fixed64 x = normal.X > Fixed64.Zero ? cuboidA.BoundsMax.X : cuboidA.BoundsMin.X;
                    AddCuboidContact(manifold, new Vector3d(x, minY, minZ), normal, depth);
                    AddCuboidContact(manifold, new Vector3d(x, minY, maxZ), normal, depth);
                    AddCuboidContact(manifold, new Vector3d(x, maxY, minZ), normal, depth);
                    AddCuboidContact(manifold, new Vector3d(x, maxY, maxZ), normal, depth);
                    break;
                }
            case 1:
                {
                    Fixed64 y = normal.Y > Fixed64.Zero ? cuboidA.BoundsMax.Y : cuboidA.BoundsMin.Y;
                    AddCuboidContact(manifold, new Vector3d(minX, y, minZ), normal, depth);
                    AddCuboidContact(manifold, new Vector3d(minX, y, maxZ), normal, depth);
                    AddCuboidContact(manifold, new Vector3d(maxX, y, minZ), normal, depth);
                    AddCuboidContact(manifold, new Vector3d(maxX, y, maxZ), normal, depth);
                    break;
                }
            default:
                {
                    Fixed64 z = normal.Z > Fixed64.Zero ? cuboidA.BoundsMax.Z : cuboidA.BoundsMin.Z;
                    AddCuboidContact(manifold, new Vector3d(minX, minY, z), normal, depth);
                    AddCuboidContact(manifold, new Vector3d(minX, maxY, z), normal, depth);
                    AddCuboidContact(manifold, new Vector3d(maxX, minY, z), normal, depth);
                    AddCuboidContact(manifold, new Vector3d(maxX, maxY, z), normal, depth);
                    break;
                }
        }
    }

    private static void AddCuboidContact(ContactManifold manifold, Vector3d pointA, Vector3d normal, Fixed64 depth)
    {
        Vector3d pointB = pointA - normal * depth;
        manifold.AddContact(pointA, pointB, depth, normal);
    }

    #endregion

}
