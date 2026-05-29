using FixedMathSharp;
using Gravitas.Colliders;
using System.Runtime.CompilerServices;

namespace Gravitas;

internal sealed class CollisionPair2D
{
    private bool _isColliding;

    public CollisionPair2D(LSCollider2D colliderA, LSCollider2D colliderB)
    {
        ColliderA = colliderA;
        ColliderB = colliderB;
    }

    public LSCollider2D ColliderA { get; private set; }

    public LSCollider2D ColliderB { get; private set; }

    public int LastFrame { get; private set; } = -1;

    public bool IsColliding => _isColliding;

    public void Initialize(LSCollider2D colliderA, LSCollider2D colliderB)
    {
        ColliderA = colliderA;
        ColliderB = colliderB;
        _isColliding = false;
        LastFrame = -1;
    }

    public void MarkColliding(int frame, Contact2D contact)
    {
        bool changed = !_isColliding;
        _isColliding = true;
        LastFrame = frame;

        if (!ColliderA.IsTrigger && !ColliderB.IsTrigger)
        {
            Resolve(contact);
            WakeBodies();
        }

        ColliderA.NotifyContact(ColliderB, true, changed);
        ColliderB.NotifyContact(ColliderA, true, changed);
    }

    public void MarkResting(int frame)
    {
        LastFrame = frame;
    }

    public void MarkSeparated()
    {
        if (!_isColliding)
            return;

        _isColliding = false;
        ColliderA.NotifyContact(ColliderB, false, true);
        ColliderB.NotifyContact(ColliderA, false, true);
    }

    private void Resolve(Contact2D contact)
    {
        StiffBody2D? bodyA = ColliderA.Body;
        StiffBody2D? bodyB = ColliderB.Body;
        if (bodyA == null && bodyB == null)
            return;

        Fixed64 inverseMassA = bodyA?.CanMove == true ? bodyA.InverseMass : Fixed64.Zero;
        Fixed64 inverseMassB = bodyB?.CanMove == true ? bodyB.InverseMass : Fixed64.Zero;
        Fixed64 totalInverseMass = inverseMassA + inverseMassB;
        if (totalInverseMass <= Fixed64.Zero)
            return;

        Vector2d normal = contact.Normal.SqrMagnitude > Fixed64.Epsilon
            ? contact.Normal.Normal
            : ResolveFallbackNormal();
        if (normal == Vector2d.Zero)
            return;

        ApplyPositionCorrection(bodyA, bodyB, normal, contact.Depth, inverseMassA, inverseMassB, totalInverseMass);
        ApplyVelocityImpulse(bodyA, bodyB, normal, inverseMassA, inverseMassB, totalInverseMass);
    }

    private static void ApplyPositionCorrection(
        StiffBody2D? bodyA,
        StiffBody2D? bodyB,
        Vector2d normal,
        Fixed64 depth,
        Fixed64 inverseMassA,
        Fixed64 inverseMassB,
        Fixed64 totalInverseMass)
    {
        Fixed64 correctionDepth = depth - CollisionResponse2D.PenetrationSlop;
        if (correctionDepth <= Fixed64.Zero)
            return;

        Vector2d correction = normal * (correctionDepth * CollisionResponse2D.PenetrationCorrectionPercent / totalInverseMass);
        bodyA?.ApplyCollisionPositionCorrection(-correction * inverseMassA);
        bodyB?.ApplyCollisionPositionCorrection(correction * inverseMassB);
    }

    private static void ApplyVelocityImpulse(
        StiffBody2D? bodyA,
        StiffBody2D? bodyB,
        Vector2d normal,
        Fixed64 inverseMassA,
        Fixed64 inverseMassB,
        Fixed64 totalInverseMass)
    {
        Vector2d velocityA = bodyA?.LinearVelocity ?? Vector2d.Zero;
        Vector2d velocityB = bodyB?.LinearVelocity ?? Vector2d.Zero;
        Vector2d relativeVelocity = velocityB - velocityA;
        Fixed64 normalVelocity = Vector2d.Dot(relativeVelocity, normal);
        if (normalVelocity >= Fixed64.Zero)
            return;

        Fixed64 restitution = bodyA != null && bodyB != null
            ? FixedMath.Min(bodyA.RestitutionCoefficient, bodyB.RestitutionCoefficient)
            : Fixed64.Zero;
        if (-normalVelocity <= CollisionResponse2D.RestitutionVelocityThreshold)
            restitution = Fixed64.Zero;

        Fixed64 impulseScalar = -(Fixed64.One + restitution) * normalVelocity / totalInverseMass;
        if (impulseScalar <= Fixed64.Zero)
            return;

        Vector2d impulse = normal * impulseScalar;
        bodyA?.ApplyCollisionLinearVelocityDelta(-impulse * inverseMassA);
        bodyB?.ApplyCollisionLinearVelocityDelta(impulse * inverseMassB);
    }

    private void WakeBodies()
    {
        StiffBody2D? bodyA = ColliderA.Body;
        StiffBody2D? bodyB = ColliderB.Body;
        if (bodyA == null || bodyB == null)
            return;

        bool bodyAAwake = !bodyA.IsSleeping;
        bool bodyBAwake = !bodyB.IsSleeping;
        if (bodyA.IsSleeping && bodyBAwake)
            bodyA.Wake();
        if (bodyB.IsSleeping && bodyAAwake)
            bodyB.Wake();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Vector2d ResolveFallbackNormal()
    {
        Vector2d direction = ColliderB.Center - ColliderA.Center;
        return direction.SqrMagnitude > Fixed64.Epsilon
            ? direction.Normal
            : Vector2d.Right;
    }
}
