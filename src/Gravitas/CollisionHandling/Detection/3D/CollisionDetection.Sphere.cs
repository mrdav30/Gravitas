//=======================================================================
// CollisionDetection.Sphere.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;

namespace Gravitas.CollisionHandling;

public static partial class CollisionDetection
{
    #region Sphere

    private static bool DoSpheresCheck(CollisionWorkItem pair)
    {
        if (!FixedSegment.TryGetCenteredCapsulesContact(
                pair.ColliderA.Center,
                pair.ColliderA.Rotation,
                Vector3d.Up,
                Fixed64.Zero,
                pair.ColliderA.ScaledRadius,
                pair.ColliderB.Center,
                pair.ColliderB.Rotation,
                Vector3d.Up,
                Fixed64.Zero,
                pair.ColliderB.ScaledRadius,
                ResolveCenteredCapsuleFallback(pair.ColliderA.Center, pair.ColliderB.Center),
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

    #endregion

}
