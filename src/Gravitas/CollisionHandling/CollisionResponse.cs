using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Solves the current primary-contact collision response for one collision pair.
/// </summary>
public static class CollisionResponse
{
    public const bool Debug = true;

    /// <summary>
    /// Penetration depth below this value is treated as contact slop and does not
    /// produce positional correction.
    /// </summary>
    public static readonly Fixed64 PenetrationSlop = (Fixed64)0.01f;

    /// <summary>
    /// Fraction of penetration above slop corrected per solver call.
    /// </summary>
    public static readonly Fixed64 PenetrationCorrectionPercent = (Fixed64)0.8f;

    /// <summary>
    /// Closing speed at or below this value is treated as resting contact and
    /// uses zero restitution to avoid small deterministic bounces.
    /// </summary>
    public static readonly Fixed64 RestitutionVelocityThreshold = (Fixed64)0.25f;

    /// <summary>
    /// Applies positional correction and a normal impulse for the collision
    /// pair's current contact.
    /// </summary>
    public static void CalculateImpulse(CollisionPair pair)
    {
        if (!TryCreateContact(pair, out SolverContact contact))
            return;

        ApplyPositionCorrection(contact);
        ApplyVelocityImpulse(pair, contact);
    }

    private static bool TryCreateContact(CollisionPair pair, out SolverContact contact)
    {
        contact = default;
        if (pair.ColliderA.IsTrigger || pair.ColliderB.IsTrigger)
            return false;

        if (pair.ColliderA.Body == null || pair.ColliderB.Body == null)
            return false;

        if (!pair.Manifold.HasContact)
            return false;

        ManifoldContact manifoldContact = pair.Manifold.PrimaryContact;
        Vector3d normal = ResolveContactNormal(manifoldContact.Normal, pair.ColliderB.Center - pair.ColliderA.Center);
        if (normal == Vector3d.Zero)
            return false;

        ResponseBody bodyA = ResponseBody.Create(pair.ColliderA);
        ResponseBody bodyB = ResponseBody.Create(pair.ColliderB);
        if (bodyA.InverseMass + bodyB.InverseMass <= Fixed64.Zero)
            return false;

        contact = new SolverContact(
            bodyA,
            bodyB,
            manifoldContact.PointA,
            manifoldContact.PointB,
            manifoldContact.PointA - pair.ColliderA.Center,
            manifoldContact.PointB - pair.ColliderB.Center,
            manifoldContact.Depth,
            normal);
        return true;
    }

    private static void ApplyPositionCorrection(SolverContact contact)
    {
        Fixed64 correctionDepth = contact.Depth - PenetrationSlop;
        if (correctionDepth <= Fixed64.Zero)
            return;

        Fixed64 totalInverseMass = contact.TotalInverseMass;
        if (totalInverseMass <= Fixed64.Zero)
            return;

        Vector3d correction = contact.Normal * (correctionDepth * PenetrationCorrectionPercent / totalInverseMass);
        contact.A.Body.ApplyCollisionPositionCorrection(-correction * contact.A.InverseMass);
        contact.B.Body.ApplyCollisionPositionCorrection(correction * contact.B.InverseMass);
    }

    private static void ApplyVelocityImpulse(CollisionPair pair, SolverContact contact)
    {
        Vector3d velocityA = contact.A.Body.LinearVelocity + Vector3d.Cross(contact.A.Body.AngularVelocity, contact.RelativeA);
        Vector3d velocityB = contact.B.Body.LinearVelocity + Vector3d.Cross(contact.B.Body.AngularVelocity, contact.RelativeB);
        Fixed64 normalVelocity = Vector3d.Dot(velocityB - velocityA, contact.Normal);

        if (normalVelocity >= Fixed64.Zero)
            return;

        Fixed64 denominator = contact.TotalInverseMass
            + ComputeAngularDenominator(contact.A, contact.RelativeA, contact.Normal)
            + ComputeAngularDenominator(contact.B, contact.RelativeB, contact.Normal);
        if (denominator <= Fixed64.Epsilon)
            return;

        Fixed64 restitution = ResolveRestitution(contact, -normalVelocity);
        Fixed64 impulseScalar = -(Fixed64.One + restitution) * normalVelocity / denominator;
        if (impulseScalar <= Fixed64.Zero)
            return;

        Vector3d impulse = contact.Normal * impulseScalar;
        pair.Context.Diagnostics.EmitResponseImpulse(pair, impulse, normalVelocity);
        ApplyImpulse(contact.A, -impulse, contact.RelativeA);
        ApplyImpulse(contact.B, impulse, contact.RelativeB);
    }

    private static Fixed64 ComputeAngularDenominator(ResponseBody body, Vector3d relativeContactPoint, Vector3d normal)
    {
        if (!body.CanRotate)
            return Fixed64.Zero;

        Vector3d angular = Vector3d.Cross(
            body.InverseInertiaTensor * Vector3d.Cross(relativeContactPoint, normal),
            relativeContactPoint);
        Fixed64 denominator = Vector3d.Dot(angular, normal);
        return denominator > Fixed64.Zero ? denominator : Fixed64.Zero;
    }

    private static void ApplyImpulse(ResponseBody body, Vector3d impulse, Vector3d relativeContactPoint)
    {
        if (!body.CanMove)
            return;

        body.Body.ApplyCollisionLinearVelocityDelta(impulse * body.InverseMass);

        if (!body.CanRotate)
            return;

        Vector3d angularVelocityDelta = body.InverseInertiaTensor * Vector3d.Cross(relativeContactPoint, impulse);
        body.Body.ApplyCollisionAngularVelocityDelta(angularVelocityDelta);
    }

    private static Fixed64 ResolveRestitution(SolverContact contact, Fixed64 closingSpeed)
    {
        if (closingSpeed <= RestitutionVelocityThreshold)
            return Fixed64.Zero;

        Fixed64 restitution = FixedMath.Min(
            contact.A.Body.RestitutionCoefficient,
            contact.B.Body.RestitutionCoefficient);
        return FixedMath.Clamp(restitution, Fixed64.Zero, Fixed64.One);
    }

    private static Vector3d ResolveContactNormal(Vector3d normal, Vector3d fallbackDirection)
    {
        Vector3d resolved = normal.SqrMagnitude > Fixed64.Epsilon
            ? normal.Normal
            : fallbackDirection.SqrMagnitude > Fixed64.Epsilon
                ? fallbackDirection.Normal
                : Vector3d.Zero;

        if (resolved == Vector3d.Zero)
            return resolved;

        return fallbackDirection.SqrMagnitude > Fixed64.Epsilon
            && Vector3d.Dot(resolved, fallbackDirection) < Fixed64.Zero
                ? -resolved
                : resolved;
    }

    private readonly struct SolverContact
    {
        public SolverContact(
            ResponseBody bodyA,
            ResponseBody bodyB,
            Vector3d pointA,
            Vector3d pointB,
            Vector3d relativeA,
            Vector3d relativeB,
            Fixed64 depth,
            Vector3d normal)
        {
            A = bodyA;
            B = bodyB;
            PointA = pointA;
            PointB = pointB;
            RelativeA = relativeA;
            RelativeB = relativeB;
            Depth = depth;
            Normal = normal;
        }

        public ResponseBody A { get; }

        public ResponseBody B { get; }

        public Vector3d PointA { get; }

        public Vector3d PointB { get; }

        public Vector3d RelativeA { get; }

        public Vector3d RelativeB { get; }

        public Fixed64 Depth { get; }

        public Vector3d Normal { get; }

        public Fixed64 TotalInverseMass => A.InverseMass + B.InverseMass;
    }

    private readonly struct ResponseBody
    {
        private ResponseBody(LSCollider collider, StiffBody body, Fixed64 inverseMass, Fixed3x3 inverseInertiaTensor)
        {
            Collider = collider;
            Body = body;
            InverseMass = inverseMass;
            InverseInertiaTensor = inverseInertiaTensor;
        }

        public LSCollider Collider { get; }

        public StiffBody Body { get; }

        public Fixed64 InverseMass { get; }

        public Fixed3x3 InverseInertiaTensor { get; }

        public bool CanMove => InverseMass > Fixed64.Zero;

        public bool CanRotate => CanMove && !Body.AngularForcesHalted && !Body.IsKinematic;

        public static ResponseBody Create(LSCollider collider)
        {
            StiffBody body = collider.Body!;
            bool movable = !body.Immovable && !body.IsKinematic;
            Fixed64 inverseMass = movable ? body.InverseMass : Fixed64.Zero;
            Fixed3x3 inverseInertiaTensor = movable && !body.AngularForcesHalted
                ? body.InverseInteriaTensor
                : Fixed3x3.Zero;

            return new ResponseBody(collider, body, inverseMass, inverseInertiaTensor);
        }
    }
}
