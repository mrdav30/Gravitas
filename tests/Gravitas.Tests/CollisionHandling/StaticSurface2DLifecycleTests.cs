using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Support;
using Gravitas.Tests.Support;
using System;
using System.Reflection;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests;

public sealed class StaticSurface2DLifecycleTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Replacement_ShouldReleaseOldMembersAndPermitAnEmptyDeclaration(bool polygon)
    {
        using var context = Physics2DTestWorld.CreateContext();
        LSCollider2D first = Member(context, polygon), second = Member(context, !polygon);
        using var surface = context.Physics2D.CreateStaticSurface(new[] { first });
        surface.ReplaceMembers(new[] { first, second });
        Assert.Same(surface, first.StaticSurface);
        Assert.Same(surface, second.StaticSurface);
        surface.ReplaceMembers(new[] { second });
        Assert.Null(first.StaticSurface);
        Assert.Same(surface, second.StaticSurface);
        surface.ReplaceMembers(Array.Empty<LSCollider2D>());
        Assert.Null(second.StaticSurface);
        using var other = context.Physics2D.CreateStaticSurface(new[] { first, second });
        Assert.Same(other, first.StaticSurface);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    [InlineData(5)]
    [InlineData(6)]
    [InlineData(7)]
    [InlineData(8)]
    [InlineData(9)]
    [InlineData(10)]
    public void RejectedReplacement_ShouldKeepEveryPreviousAssociation(int invalidKind)
    {
        using var context = Physics2DTestWorld.CreateContext();
        using var foreign = Physics2DTestWorld.CreateContext();
        LSCollider2D first = Member(context), second = Member(context, true);
        using var surface = context.Physics2D.CreateStaticSurface(new[] { first, second });
        LSCollider2D candidate = Member(invalidKind == 2 ? foreign : context);
        using var occupied = context.Physics2D.CreateStaticSurface(invalidKind == 6
            ? new[] { candidate } : Array.Empty<LSCollider2D>());
        if (invalidKind is 3 or 10)
        {
            candidate.Deactivate();
            var body = new SolidBody2D(new TestMatterAgent(context), candidate);
            body.Initialize(Vector2d.Zero, motionType: invalidKind == 3 ? BodyMotionType.Dynamic : BodyMotionType.Kinematic);
        }
        if (invalidKind == 4) candidate.IsActive = false;
        if (invalidKind == 5)
        {
            candidate = new LSCircleCollider2D(Fixed64.Half);
            candidate.InitializeWithNoBody(new TestMatterAgent(context));
        }
        if (invalidKind == 7) candidate = new LSAABBoxCollider2D(Vector2d.One);
        if (invalidKind is 8 or 9)
        {
            var compound = new LSCompoundCollider2D(CompoundColliderPart2D.AABBox(Vector2d.One, Vector2d.Zero));
            compound.InitializeWithNoBody(new TestMatterAgent(context));
            candidate = invalidKind == 8 ? compound : compound.GetPartCollider(0);
        }
        LSCollider2D[] invalid = invalidKind == 0 ? new LSCollider2D[] { null! }
            : invalidKind == 1 ? new[] { first, first } : new[] { first, candidate };
        uint version = first.ContactDependencyVersion;
        Assert.Throws<ArgumentException>(() => surface.ReplaceMembers(invalid));
        Assert.Same(surface, first.StaticSurface);
        Assert.Same(surface, second.StaticSurface);
        Assert.Equal(version, first.ContactDependencyVersion);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContextResetOrDisposal_ShouldEndTheDeclarationLifetime(bool dispose)
    {
        using var context = Physics2DTestWorld.CreateContext();
        LSCollider2D member = Member(context);
        var surface = context.Physics2D.CreateStaticSurface(new[] { member });
        if (dispose) context.Dispose(); else context.Reset();
        Assert.Null(member.StaticSurface);
        Assert.Throws<ObjectDisposedException>(() => surface.ReplaceMembers(Array.Empty<LSCollider2D>()));
        surface.Dispose();
        surface.Dispose();
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void EndingPopulatedMembership_ShouldReleaseCachedCollidersIncludingInactiveCapacity(int ending)
    {
        using var context = Physics2DTestWorld.CreateContext();
        var left = new LSAABBoxCollider2D(new Vector2d(8, 2));
        var right = new LSAABBoxCollider2D(new Vector2d(8, 2));
        left.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(new Vector3d(-4, 0, -1), FixedQuaternion.Identity, Vector3d.One)));
        right.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(new Vector3d(4, 0, -1), FixedQuaternion.Identity, Vector3d.One)));
        using var surface = context.Physics2D.CreateStaticSurface(new LSCollider2D[] { left, right });
        var circle = new LSCircleCollider2D(Fixed64.Half);
        var body = new SolidBody2D(new TestMatterAgent(context), circle) { Mass = Fixed64.One };
        body.Initialize(new(Fixed64.Zero, Fixed64.Quarter));

        void Contact()
        {
            var contacts = new ContactManifold2D();
            Assert.True(CollisionDetection2D.TryCollide(new CollisionWorkItem2D(circle, left,
                ColliderSettings2D.GetCollisionType(circle.Shape, left.Shape)), contacts, 0));
            Assert.True(contacts.HasContact);
        }

        Contact();
        body.ResetPosition(new((Fixed64)(-3), Fixed64.Quarter));
        Contact();
        const BindingFlags fields = BindingFlags.Instance | BindingFlags.NonPublic;
        string[] buffers = { "_local", "_candidates", "_prepared" };
        foreach (string name in buffers)
        {
            var buffer = (SwiftList<LSCollider2D>)typeof(StaticSurface2D).GetField(name, fields)!.GetValue(surface)!;
            Assert.Single(buffer);
            Assert.Same(left, buffer[0]);
            // The real gather shrank without clearing capacity; this is the retained tail
            // that a count-only Clear would fail to release at the lifetime boundary.
            Assert.Contains(right, buffer.InnerArray.AsSpan(buffer.Count).ToArray());
        }
        Assert.Same(circle, typeof(StaticSurface2D).GetField("_moving", fields)!.GetValue(surface));
        Assert.Same(left, typeof(StaticSurface2D).GetField("_carrier", fields)!.GetValue(surface));

        if (ending == 0) surface.Dispose();
        else if (ending == 1) context.Reset();
        else if (ending == 2) context.Dispose();
        else surface.ReplaceMembers(Array.Empty<LSCollider2D>());

        Assert.Null(left.StaticSurface);
        Assert.Null(right.StaticSurface);
        foreach (string name in buffers)
        {
            var buffer = (SwiftList<LSCollider2D>)typeof(StaticSurface2D).GetField(name, fields)!.GetValue(surface)!;
            Assert.Empty(buffer);
            Assert.All(buffer.InnerArray, item => Assert.Null(item));
        }
        Assert.Null(typeof(StaticSurface2D).GetField("_moving", fields)!.GetValue(surface));
        Assert.Null(typeof(StaticSurface2D).GetField("_carrier", fields)!.GetValue(surface));
    }

    [Fact]
    public void MembershipMutation_ShouldRequireTheCompleteFixedStepBoundary()
    {
        using var context = Physics2DTestWorld.CreateContext();
        LSCollider2D member = Member(context);
        var surface = context.Physics2D.CreateStaticSurface(new[] { member });
        context.Simulate();
        Assert.Throws<InvalidOperationException>(() => surface.ReplaceMembers(Array.Empty<LSCollider2D>()));
        Assert.Throws<InvalidOperationException>(() => surface.Dispose());
        context.LateSimulate();
        surface.Dispose();
        Assert.Throws<ObjectDisposedException>(() => surface.ReplaceMembers(new[] { member }));
        using var replacement = context.Physics2D.CreateStaticSurface(new[] { member });
        Assert.Same(replacement, member.StaticSurface);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ResetRegistration_ShouldRejectAStaleShellEvenAfterIdReuse(bool reuseId)
    {
        using var context = Physics2DTestWorld.CreateContext();
        LSCollider2D stale = Member(context);
        context.Reset();
        if (reuseId) Member(context);
        Assert.Throws<ArgumentException>(() => context.Physics2D.CreateStaticSurface(new[] { stale }));
    }

    [Theory]
    [InlineData(BodyMotionType.Dynamic)]
    [InlineData(BodyMotionType.Kinematic)]
    public void LosingTheStaticRole_ShouldRemoveMembershipBeforeBodyReuse(BodyMotionType nextRole)
    {
        using var context = Physics2DTestWorld.CreateContext();
        var member = new LSAABBoxCollider2D(Vector2d.One);
        var body = new SolidBody2D(new TestMatterAgent(context), member);
        body.Initialize(Vector2d.Zero, motionType: BodyMotionType.Static);
        using var surface = context.Physics2D.CreateStaticSurface(new[] { member });
        body.SetMotionType(nextRole);
        Assert.Null(member.StaticSurface);
        body.SetMotionType(BodyMotionType.Static);
        Assert.Null(member.StaticSurface);
        surface.ReplaceMembers(new[] { member });
        Assert.Same(surface, member.StaticSurface);
        body.Deactivate();
        Assert.Null(member.StaticSurface);
    }

    [Fact]
    public void DisableAndDeactivate_ShouldRequireAnExplicitNewDeclaration()
    {
        using var context = Physics2DTestWorld.CreateContext();
        LSCollider2D member = Member(context);
        using var surface = context.Physics2D.CreateStaticSurface(new[] { member });
        member.IsActive = false;
        Assert.Null(member.StaticSurface);
        member.IsActive = true;
        Assert.Null(member.StaticSurface);
        surface.ReplaceMembers(new[] { member });
        member.Deactivate();
        Assert.Null(member.StaticSurface);
        surface.Remove(member);
        member.InitializeWithNoBody(new TestMatterAgent(context));
        Assert.Null(member.StaticSurface);
        surface.ReplaceMembers(new[] { member });
        Assert.Same(surface, member.StaticSurface);
    }

    [Fact]
    public void ExitCallback_ShouldRejectADeactivatingMemberBeforeItsRegistryEntryIsReleased()
    {
        using var context = Physics2DTestWorld.CreateContext();
        context.Environment.Gravity = Fixed64.Zero;
        LSCollider2D member = Member(context);
        using var surface = context.Physics2D.CreateStaticSurface(new[] { member });
        var mover = new SolidBody2D(new TestMatterAgent(context), new LSCircleCollider2D(Fixed64.Half))
        { Mass = Fixed64.One };
        mover.Initialize(new(Fixed64.Zero, Fixed64.One + Fixed64.Quarter));
        mover.UseManualGrounding();
        mover.SleepEnabled = false;
        context.Simulate(); context.LateSimulate();
        bool attempted = false;
        member.OnContactExit += _ =>
        {
            attempted = true;
            Assert.Throws<ArgumentException>(() => context.Physics2D.CreateStaticSurface(new[] { member }));
        };
        member.Deactivate();
        Assert.True(attempted);
        Assert.Null(member.StaticSurface);
        Assert.False(member.IsDeactivationInProgress);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    public void CommittedNeighborChanges_ShouldInvalidateDependentMembers(int change)
    {
        using var context = Physics2DTestWorld.CreateContext();
        LSCollider2D first = Member(context), neighbor = Member(context, true);
        using var surface = context.Physics2D.CreateStaticSurface(new[] { first, neighbor });
        uint before = first.ContactDependencyVersion;
        uint editedBefore = neighbor.ContactDependencyVersion;
        if (change == 0) neighbor.Material = PhysicsMaterial.Bouncy;
        if (change == 1) neighbor.Layer = new PhysicsLayer(1);
        if (change == 2) neighbor.IgnoredCollisionLayers = PhysicsLayerMask.All;
        if (change == 3) neighbor.IsTrigger = true;
        if (change == 4)
        {
            neighbor.LocalOffset = Vector2d.Right;
            neighbor.Simulate();
        }
        Assert.True(neighbor.ContactDependencyVersion > editedBefore);
        // Materials invalidate their actual borrowed rows, not every neighbor's geometry.
        if (change == 0) Assert.Equal(before, first.ContactDependencyVersion);
        else Assert.True(first.ContactDependencyVersion > before);
        uint unchanged = first.ContactDependencyVersion;
        neighbor.Material = neighbor.Material;
        neighbor.Layer = neighbor.Layer;
        neighbor.IgnoredCollisionLayers = neighbor.IgnoredCollisionLayers;
        neighbor.IsTrigger = neighbor.IsTrigger;
        Assert.Equal(unchanged, first.ContactDependencyVersion);
        Assert.Same(surface, neighbor.StaticSurface);
    }

    private static LSCollider2D Member(GravitasWorldContext context, bool polygon = false)
    {
        LSCollider2D collider = polygon
            ? new LSPolygonCollider2D(new(-1, -1), new(1, -1), new(1, 1), new(-1, 1))
            : new LSAABBoxCollider2D(new Vector2d(2, 2));
        collider.InitializeWithNoBody(new TestMatterAgent(context));
        return collider;
    }
}
