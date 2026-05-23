using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Represents a pair of colliders in the physics simulation, handling their interaction.
/// </summary>
public static class CollisionResponse
{
    public const bool Debug = true;

    /// <summary>
    /// Applies the separation vector to a collider, updating its position and velocity.
    /// </summary>
    /// <param name="pair">The collision pair containing the colliders and contact point information.</param>
    public static void CalculateImpulse(CollisionPair pair)
    {
        if (pair.ContactPoint.Depth < Fixed64.Zero)
            GravitasLogger.DebugChannel.Info($"Negative penetration depth detected: {pair.ContactPoint.Depth}. This may indicate an issue with the collision detection phase.");
        //return;

        LSCollider collider1 = pair.ColliderA, collider2 = pair.ColliderB;
        ApplyPositionCorrection(collider1, collider2, pair.ContactPoint);

        Vector3d fullVelocityA = collider1.Body!.LinearVelocity + Vector3d.Cross(collider1.Body.AngularVelocity, pair.ContactPoint.RelativeA);
        Vector3d fullVelocityB = collider2.Body!.LinearVelocity + Vector3d.Cross(collider2.Body.AngularVelocity, pair.ContactPoint.RelativeB);
        Vector3d contactVelocity = fullVelocityB - fullVelocityA;

        if (contactVelocity == Vector3d.Zero)
            return;

        Fixed64 impulseForce = Vector3d.Dot(contactVelocity, pair.ContactPoint.Normal);

        // work out the effects of interia
        Vector3d interiaA = Vector3d.Cross(collider1.Body.InverseInteriaTensor * Vector3d.Cross(pair.ContactPoint.RelativeA, pair.ContactPoint.Normal), pair.ContactPoint.RelativeA);
        Vector3d interiaB = Vector3d.Cross(collider2.Body.InverseInteriaTensor * Vector3d.Cross(pair.ContactPoint.RelativeB, pair.ContactPoint.Normal), pair.ContactPoint.RelativeB);
        Fixed64 angularEffect = Vector3d.Dot(interiaA + interiaB, pair.ContactPoint.Normal);

        Fixed64 restitution = -(Fixed64.One + FixedMath.Max(collider1.Body.RestitutionCoefficient, collider2.Body.RestitutionCoefficient));
        Fixed64 impulseScalar = restitution * impulseForce;
        impulseScalar /= collider1.Body.InverseMass + collider2.Body.InverseMass + angularEffect;

        // direction of the impulse
        Vector3d impulseDirection = (collider2.Center - collider1.Center).Normal;
        // direction of the impulse
        Vector3d fullImpulse = impulseDirection * impulseScalar;

        SetVelocityImpulse(collider1, -fullImpulse, pair.ContactPoint.RelativeA);
        SetVelocityImpulse(collider2, fullImpulse, pair.ContactPoint.RelativeB);
    }

    /// <summary>
    /// This method is used to adjust the positions of colliding objects based on 
    /// the penetration depth calculated during the collision detection phase.
    /// It's more about correcting the overlap after the collision has been detected, 
    /// rather than preemptively adjusting positions to avoid penetration, as in CCD.
    /// </summary> 
    /// <param name="collider1">The first collider involved in the collision.</param>
    /// <param name="collider2">The second collider involved in the collision.</param>
    /// <param name="point">The contact point information, including penetration depth and normal.</param>
    private static void ApplyPositionCorrection(LSCollider collider1, LSCollider collider2, ContactPoint point)
    {
        //// Still testing this out...
        //pair.SetImmovableDirection(-pair.ColliderB.Body.LinearVelocity.Normal, -pair.ColliderA.Body.LinearVelocity.Normal);

        Vector3d direction = (collider2.Center - collider1.Center).Normal;

        // Correct positions using MPV, penetration depth, & TOI position
        if (!collider1.Body!.Immovable && !collider2.Body!.Immovable)
        {
            Fixed64 totalMass = collider1.Body.InverseMass + collider2.Body.InverseMass;
            if (!collider1.IsTrigger)
            {
                Fixed64 move1 = point.Depth * (collider1.Body.InverseMass / totalMass);
                collider1.Body.AddPositionCorrection(-direction * move1);
            }

            if (!collider2.IsTrigger)
            {
                Fixed64 move2 = point.Depth * (collider2.Body.InverseMass / totalMass);
                collider2.Body.AddPositionCorrection(direction * move2);
            }

            return;
        }

        if (!collider1.Body.Immovable && !collider1.IsTrigger)
        {
            collider1.Body.AddPositionCorrection(-direction * point.Depth);
            return;
        }

        if (!collider2.Body!.Immovable && !collider1.IsTrigger)
            collider2.Body.AddPositionCorrection(direction * point.Depth);
    }

    private static void SetVelocityImpulse(LSCollider collider, Vector3d impulse, Vector3d contactPointRelative)
    {
        if (collider.Body!.Immovable == true || collider.IsTrigger)
            return;

        collider.Body.AddLinearImpulse(impulse);

        if (collider.Body.PreventAngularForces)
            return;

        Vector3d angularImpulse = Vector3d.Cross(contactPointRelative, impulse);
        collider.Body.AddAngularImpulse(angularImpulse);
    }
}
