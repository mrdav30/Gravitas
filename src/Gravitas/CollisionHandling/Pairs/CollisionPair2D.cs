using Gravitas.Colliders;

namespace Gravitas;

internal sealed class CollisionPair2D
{
    private bool _isColliding;

    public CollisionPair2D(LSCollider2D colliderA, LSCollider2D colliderB)
    {
        ColliderA = colliderA;
        ColliderB = colliderB;
        Initialize(colliderA, colliderB);
    }

    public LSCollider2D ColliderA { get; private set; }

    public LSCollider2D ColliderB { get; private set; }

    public int Id1 { get; private set; }

    public int Id2 { get; private set; }

    public CollisionType2D CollisionType { get; private set; }

    public int LastFrame { get; private set; } = -1;

    public bool IsColliding => _isColliding;

    public void Initialize(LSCollider2D colliderA, LSCollider2D colliderB)
    {
        AssignPriority(colliderA, colliderB);
        Id1 = ColliderA.Id;
        Id2 = ColliderB.Id;
        CollisionType = ColliderSettings2D.GetCollisionType(ColliderA.Shape, ColliderB.Shape);
        _isColliding = false;
        LastFrame = -1;
    }

    private void AssignPriority(LSCollider2D colliderA, LSCollider2D colliderB)
    {
        if (ShouldFirstColliderLead(colliderA, colliderB))
        {
            ColliderA = colliderA;
            ColliderB = colliderB;
            return;
        }

        ColliderA = colliderB;
        ColliderB = colliderA;
    }

    internal static bool ShouldFirstColliderLead(LSCollider2D colliderA, LSCollider2D colliderB)
    {
        if (colliderA.Priority != colliderB.Priority)
            return colliderA.Priority > colliderB.Priority;

        StiffBody2D? bodyA = colliderA.Body;
        StiffBody2D? bodyB = colliderB.Body;
        if (bodyA == null || bodyB == null)
            return colliderA.Id <= colliderB.Id;

        if (bodyA.LinearSpeed != bodyB.LinearSpeed)
            return bodyA.LinearSpeed > bodyB.LinearSpeed;

        return colliderA.Id <= colliderB.Id;
    }

    public void MarkColliding(int frame, Contact2D contact)
    {
        bool changed = !_isColliding;
        _isColliding = true;
        LastFrame = frame;

        if (!ColliderA.IsTrigger && !ColliderB.IsTrigger)
        {
            CollisionResponse2D.Resolve(this, contact);
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
}
