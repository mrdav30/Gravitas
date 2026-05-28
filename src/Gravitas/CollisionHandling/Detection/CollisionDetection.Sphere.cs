using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections;
using SwiftCollections.Pool;
using SwiftCollections.Query;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

public static partial class CollisionDetection
{
    #region Sphere

    private static bool DoSpheresCheck(CollisionWorkItem pair)
    {
        Vector3d penetrationVector = pair.ColliderB.Center - pair.ColliderA.Center;
        if (penetrationVector.SqrMagnitude > (pair.ColliderA.ScaledRadius + pair.ColliderB.ScaledRadius) * (pair.ColliderA.ScaledRadius + pair.ColliderB.ScaledRadius))
            return false; // No collision if the distance squared is greater than the sum of the radii squared

        // Calculate the penetration depths and normals
        Vector3d penetrationNormal = ResolveNormal(penetrationVector, pair.ColliderB.Center - pair.ColliderA.Center);
        pair.Manifold.SetContact(
            pair.ColliderA.Center + penetrationNormal * pair.ColliderA.ScaledRadius,
            pair.ColliderB.Center - penetrationNormal * pair.ColliderB.ScaledRadius,
            penetrationVector.Magnitude - (pair.ColliderA.ScaledRadius + pair.ColliderB.ScaledRadius),
            penetrationNormal
        );
        return true;
    }

    #endregion

}
