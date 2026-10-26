using Chronicler.Hashing;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Diagnostics;
using Gravitas.Materials;
using Gravitas.Support;
using Gravitas.Tests.Support;
using SwiftCollections;
using Chronicler.Diagnostics;
using System;
using System.Reflection;
using Xunit;

namespace Gravitas.Tests.Response;

public sealed partial class CollisionResponse2DManifoldTests
{
    [Fact]
    public void Resolve_ShouldSolveAllIndependentGroupsAndKeepTheirWarmStartsSeparate()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D moving = CreateBox(context, Vector2d.Zero);
        SolidBody2D wall = CreateBox(context, Vector2d.Right * (Fixed64)2, immovable: true);
        moving.FreezeAxes = BodyFreezeAxes2D.Rotation;
        moving.ApplyCollisionLinearVelocityDelta(new Vector2d((Fixed64)2, (Fixed64)3));
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        var material = new PhysicsMaterial(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);
        for (int g = 2; g >= 0; g--)
        {
            var key = new ContactGroupKey(0, 0, region: g);
            for (int sample = 0; sample < 2; sample++)
                pair.Manifold.AddContact(key, new ManifoldContact2D((ulong)(sample + 1),
                    Vector2d.Zero, Vector2d.Zero, Fixed64.Zero,
                    g == 1 ? Vector2d.Forward : Vector2d.Right,
                    hasMaterialOverride: true, materialA: material, materialB: material));
        }

        CollisionResponse2D.PrepareSolve(pair);
        CollisionResponse2D.Resolve(pair, applyCachedImpulse: false, applyPositionCorrection: false);

        moving.LinearVelocity.X.Abs().Should().BeLessThan(Fixed64.FromRaw(4));
        moving.LinearVelocity.Y.Abs().Should().BeLessThan(Fixed64.FromRaw(4));
        for (int g = 0; g < 3; g++)
        {
            var key = new ContactGroupKey(0, 0, region: g);
            pair.TryGetWarmStartImpulse(key, 1UL, out var impulse).Should().BeTrue();
            impulse.NormalImpulse.Should().Be(g == 0 ? Fixed64.Two
                : g == 1 ? (Fixed64)3 : Fixed64.Zero);
            pair.TryGetWarmStartImpulse(key, 2UL, out _).Should().BeTrue();
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Resolve_GroupedBodylessEndpoint_ShouldUseTheMovableParticipant(bool bodylessFirst)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var target = new LSAABBoxCollider2D(Vector2d.One);
        var targetAgent = new TestMatterAgent(context, new FixedTransform(
            new Vector3d(Fixed64.Two, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity, Vector3d.One));
        if (bodylessFirst) target.InitializeWithNoBody(targetAgent);
        SolidBody2D moving = CreateBox(context, Vector2d.Zero);
        if (!bodylessFirst) target.InitializeWithNoBody(targetAgent);
        moving.FreezeAxes = BodyFreezeAxes2D.Rotation;
        moving.ApplyCollisionLinearVelocityDelta(Vector2d.Right);
        var pair = new CollisionPair2D(moving.Collider, target);
        bool movingFirst = pair.ColliderA == moving.Collider;
        var material = new PhysicsMaterial(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);
        for (int g = 0; g < 2; g++)
            pair.Manifold.AddContact(new ContactGroupKey(0, 0, region: g,
                    sourceIdA: movingFirst ? 0 : target.Id + 1,
                    sourceIdB: movingFirst ? target.Id + 1 : 0),
                new ManifoldContact2D(1UL, Vector2d.Zero, Vector2d.Zero, Fixed64.Zero,
                    movingFirst ? Vector2d.Right : Vector2d.Left,
                    hasMaterialOverride: true, materialA: material, materialB: material));

        CollisionResponse2D.Resolve(pair);

        moving.LinearVelocity.Should().Be(Vector2d.Zero);
        moving.AngularVelocity.Should().Be(Fixed64.Zero);
        pair.ResponseState!.Impact.IsValid.Should().BeFalse();
    }

    [Fact]
    public void Resolve_JoinedSupplierNormal_ShouldNotBeFlippedTowardCarrierCenter()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D moving = CreateBox(context, Vector2d.Zero);
        SolidBody2D carrier = CreateBox(context, Vector2d.Right * (Fixed64)10, immovable: true);
        moving.FreezeAxes = BodyFreezeAxes2D.Rotation;
        moving.ApplyCollisionLinearVelocityDelta(Vector2d.Left);
        var pair = new CollisionPair2D(moving.Collider, carrier.Collider);
        var key = new ContactGroupKey(0, 0, sourceIdB: carrier.Collider.Id + 1);
        var material = new PhysicsMaterial(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);
        pair.Manifold.AddContact(key, new ManifoldContact2D(1UL,
            Vector2d.Zero, Vector2d.Zero, Fixed64.FromFraction(1, 4), Vector2d.Left,
            hasMaterialOverride: true, materialA: material, materialB: material));

        CollisionResponse2D.Resolve(pair);

        moving.Position.X.Should().Be(Fixed64.FromFraction(1, 4) - CollisionResponse2D.PenetrationSlop);
        moving.LinearVelocity.Should().Be(Vector2d.Zero);
        pair.TryGetWarmStartImpulse(key, 1UL, out var impulse).Should().BeTrue();
        impulse.NormalImpulse.Should().Be(Fixed64.One);
    }

    [Fact]
    public void Resolve_ParallelGroups_ShouldCorrectOnlyTheirDeepestDirectionAndKeepAllRows()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D moving = CreateBox(context, Vector2d.Zero);
        SolidBody2D wall = CreateBox(context, Vector2d.Right * (Fixed64)2, immovable: true);
        moving.FreezeAxes = BodyFreezeAxes2D.Rotation;
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        var material = new PhysicsMaterial(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);
        for (int g = 0; g < 4; g++)
        {
            var key = new ContactGroupKey(0, 0, region: g, sourceIdB: wall.Collider.Id + 1);
            for (int point = 0; point < 2; point++)
                pair.Manifold.AddContact(key, new ManifoldContact2D((ulong)(point + 1),
                    Vector2d.Zero, Vector2d.Zero, g == 1 ? Fixed64.Half : Fixed64.FromFraction(1, 4),
                    g == 3 ? Vector2d.Forward : Vector2d.Right,
                    hasMaterialOverride: true, materialA: material, materialB: material));
        }

        CollisionResponse2D.Resolve(pair);

        moving.Position.Should().Be(new Vector2d(
            -(Fixed64.Half - CollisionResponse2D.PenetrationSlop),
            -(Fixed64.FromFraction(1, 4) - CollisionResponse2D.PenetrationSlop)));
        pair.Manifold.Count.Should().Be(8);
        for (int g = 0; g < 4; g++)
        {
            var key = pair.Manifold.GetGroup(g).Key;
            pair.TryGetWarmStartImpulse(key, 1UL, out _).Should().BeTrue();
            pair.TryGetWarmStartImpulse(key, 2UL, out _).Should().BeTrue();
        }
    }

    [Fact]
    public void Resolve_ColdGroupedContact_ShouldCorrectPositionAndSolveVelocityWithoutApplyingWarmStarts()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D moving = CreateBox(context, Vector2d.Zero);
        SolidBody2D carrier = CreateBox(context, Vector2d.Right * (Fixed64)2, immovable: true);
        moving.FreezeAxes = BodyFreezeAxes2D.Rotation;
        moving.ApplyCollisionLinearVelocityDelta(Vector2d.One);
        var pair = new CollisionPair2D(moving.Collider, carrier.Collider);
        for (int group = 0; group < 2; group++)
            pair.Manifold.AddContact(new ContactGroupKey(0, 0, region: group, sourceIdB: carrier.Collider.Id + 1),
                new ManifoldContact2D(1UL, Vector2d.Zero, Vector2d.Zero,
                    group == 0 ? Fixed64.Quarter : Fixed64.Half,
                    group == 0 ? Vector2d.Right : Vector2d.Forward,
                    hasMaterialOverride: true, materialA: PhysicsMaterial.Frictionless,
                    materialB: PhysicsMaterial.Frictionless));
        CollisionResponse2D.PrepareSolve(pair);

        CollisionResponse2D.Resolve(pair, applyCachedImpulse: false, applyPositionCorrection: true);

        moving.Position.Should().Be(new Vector2d(
            -(Fixed64.Quarter - CollisionResponse2D.PenetrationSlop),
            -(Fixed64.Half - CollisionResponse2D.PenetrationSlop)));
        moving.LinearVelocity.Should().Be(Vector2d.Zero);
        moving.AngularVelocity.Should().Be(Fixed64.Zero);
        for (int group = 0; group < 2; group++)
        {
            pair.TryGetWarmStartImpulse(pair.Manifold.GetGroup(group).Key, 1UL, out var impulse).Should().BeTrue();
            impulse.NormalImpulse.Should().Be(Fixed64.One);
        }
    }

    [Fact]
    public void Simulate_LaterTriggerRemovingASupplier_ShouldRebuildAlreadyQueuedCarrierRows()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(frameRate: 60);
        context.Environment.Gravity = Fixed64.Zero;
        SolidBody2D floor = CreateBody(context, new LSAABBoxCollider2D(new Vector2d((Fixed64)4, Fixed64.One)),
            new Vector2d(Fixed64.Zero, -Fixed64.Half), immovable: true);
        SolidBody2D wall = CreateBody(context, new LSAABBoxCollider2D(new Vector2d(Fixed64.Half, (Fixed64)3)),
            new Vector2d(-Fixed64.FromFraction(3, 4), Fixed64.FromFraction(3, 2)), immovable: true);
        floor.Collider.Material = wall.Collider.Material = PhysicsMaterial.Frictionless;
        using var surface = context.Physics2D.CreateStaticSurface(new[] { floor.Collider, wall.Collider });
        SolidBody2D moving = CreateBody(context, new LSCircleCollider2D(Fixed64.Half),
            new Vector2d(-Fixed64.Quarter, Fixed64.Quarter));
        moving.Collider.Material = PhysicsMaterial.Frictionless;
        moving.FreezeAxes = BodyFreezeAxes2D.Rotation;
        moving.SleepEnabled = false;
        moving.UseManualGrounding();
        moving.ApplyCollisionLinearVelocityDelta(Vector2d.Left);
        int retiredSource = wall.Collider.Id + 1;
        var trigger = new LSCircleCollider2D(Fixed64.One) { IsTrigger = true };
        trigger.InitializeWithNoBody(new TestMatterAgent(context, new FixedTransform(
            new Vector3d(-Fixed64.Quarter, Fixed64.Zero, Fixed64.Quarter),
            FixedQuaternion.Identity, Vector3d.One)));
        bool removedAfterQueue = false;
        trigger.OnTriggerEnter += _ =>
        {
            floor.Collider.TryGetCollisionPair(moving.Collider.Id, out var carrier).Should().BeTrue();
            var queued = (SwiftList<CollisionPair2D>)typeof(GravitasPhysics2DService)
                .GetField("_discreteResponsePairs", BindingFlags.Instance | BindingFlags.NonPublic)!
                .GetValue(context.Physics2D)!;
            queued.Contains(carrier!).Should().BeTrue();
            bool retainedSupplier = false;
            for (int g = 0; g < carrier!.Manifold.GroupCount; g++)
            {
                ContactGroupKey key = carrier.Manifold.GetGroup(g).Key;
                retainedSupplier |= key.SourceIdA == retiredSource || key.SourceIdB == retiredSource;
            }
            retainedSupplier.Should().BeTrue();
            removedAfterQueue = true;
            wall.Deactivate();
        };

        context.Simulate();
        context.LateSimulate();

        removedAfterQueue.Should().BeTrue();
        wall.Active.Should().BeFalse();
        moving.LinearVelocity.X.Should().Be(-Fixed64.One);
        floor.Collider.TryGetCollisionPair(moving.Collider.Id, out var finalCarrier).Should().BeTrue();
        for (int g = 0; g < finalCarrier!.Manifold.GroupCount; g++)
        {
            ContactGroupKey key = finalCarrier.Manifold.GetGroup(g).Key;
            key.SourceIdA.Should().NotBe(retiredSource);
            key.SourceIdB.Should().NotBe(retiredSource);
        }
    }

    [Theory]
    [InlineData("removed")]
    [InlineData("replaced")]
    [InlineData("triggerA")]
    [InlineData("triggerB")]
    [InlineData("filter")]
    [InlineData("separated")]
    public void SolveDiscreteResponsePairs_ShouldRejectAQueuedContactThatIsNoLongerCurrent(string mutation)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var target = new LSAABBoxCollider2D(Vector2d.One);
        var agent = new TestMatterAgent(context, new FixedTransform(
            new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity, Vector3d.One));
        if (mutation == "triggerA") target.InitializeWithNoBody(agent);
        SolidBody2D moving = CreateBox(context, Vector2d.Zero);
        if (mutation != "triggerA") target.InitializeWithNoBody(agent);
        moving.FreezeAxes = BodyFreezeAxes2D.Rotation;
        moving.ApplyCollisionLinearVelocityDelta(Vector2d.Right);
        var pair = new CollisionPair2D(moving.Collider, target);
        pair.Manifold.SetContact(Vector2d.Zero, Vector2d.Zero, Fixed64.Zero,
            ReferenceEquals(pair.ColliderA, moving.Collider) ? Vector2d.Right : Vector2d.Left);
        SwiftList<CollisionPair2D> queued = QueueResponsePair(context, pair);
        var registered = (SwiftDictionary<ulong, CollisionPair2D>)typeof(GravitasPhysics2DService)
            .GetField("_pairs", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(context.Physics2D)!;
        ulong key = (ulong)typeof(GravitasPhysics2DService).GetMethod("CreatePairKey",
            BindingFlags.Static | BindingFlags.NonPublic)!.Invoke(null, new object[] { pair.Id1, pair.Id2 })!;
        switch (mutation)
        {
            case "removed": registered.Remove(key); break;
            case "replaced": registered[key] = new CollisionPair2D(moving.Collider, target); break;
            case "triggerA":
                ReferenceEquals(pair.ColliderA, target).Should().BeTrue();
                target.IsTrigger = true;
                break;
            case "triggerB":
                ReferenceEquals(pair.ColliderB, target).Should().BeTrue();
                target.IsTrigger = true;
                break;
            case "filter": moving.Collider.IgnoredCollisionLayers = PhysicsLayerMask.All; break;
            case "separated":
                target.LocalOffset = Vector2d.Right * (Fixed64)8;
                target.Simulate();
                pair.ContactDependenciesChanged.Should().BeTrue();
                break;
        }
        // Ordinary trigger and filter changes do not alter geometry stamps.
        // Queue validation must still check current eligibility and identity.
        if (mutation != "separated") pair.ContactDependenciesChanged.Should().BeFalse();

        typeof(GravitasPhysics2DService).GetMethod("SolveDiscreteResponsePairs",
            BindingFlags.Instance | BindingFlags.NonPublic)!.Invoke(context.Physics2D, null);

        queued.Should().BeEmpty();
        moving.LinearVelocity.Should().Be(Vector2d.Right);
        moving.Position.Should().Be(Vector2d.Zero);
        if (mutation == "removed") registered.Add(key, pair);
        if (mutation == "replaced") registered[key] = pair;
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Simulate_LaterTriggerInvalidatingTheCarrier_ShouldPromoteTheSurvivingOverlapInTheSameFrame(bool disableFilter)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(frameRate: 60);
        context.Environment.Gravity = Fixed64.Zero;
        // Keep all three candidates' canonical shared voxel equal so the
        // registered trigger runs after both lower-ID floor observations.
        Vector2d offset = new((Fixed64)4 + Fixed64.Half, (Fixed64)4 + Fixed64.Half);
        SolidBody2D left = CreateBody(context, new LSAABBoxCollider2D(new Vector2d(Fixed64.Two, Fixed64.One)),
            offset + new Vector2d(-Fixed64.One, -Fixed64.Half), immovable: true);
        SolidBody2D right = CreateBody(context, new LSAABBoxCollider2D(new Vector2d(Fixed64.Two, Fixed64.One)),
            offset + new Vector2d(Fixed64.One, -Fixed64.Half), immovable: true);
        left.Collider.Material = right.Collider.Material = PhysicsMaterial.Frictionless;
        using var surface = context.Physics2D.CreateStaticSurface(new[] { left.Collider, right.Collider });
        SolidBody2D moving = CreateBody(context, new LSCircleCollider2D(Fixed64.Half),
            offset + new Vector2d(Fixed64.Zero, Fixed64.Quarter));
        moving.Collider.Material = PhysicsMaterial.Frictionless;
        // Retiring the left piece exposes a real vertical rim; isolate the
        // carrier transfer from its legitimate horizontal position correction.
        moving.FreezeAxes = BodyFreezeAxes2D.Rotation | BodyFreezeAxes2D.PositionX;
        moving.SleepEnabled = false;
        moving.UseManualGrounding();
        moving.ApplyCollisionLinearVelocityDelta(-Vector2d.Forward);
        int leftEntered = 0, rightEntered = 0, rightContact = 0;
        left.Collider.OnContactEnter += _ => leftEntered++;
        right.Collider.OnContactEnter += _ => rightEntered++;
        right.Collider.OnContact += _ => rightContact++;
        var trigger = new LSCircleCollider2D(Fixed64.One) { IsTrigger = true };
        trigger.InitializeWithNoBody(new TestMatterAgent(context, new FixedTransform(
            new Vector3d(offset.X, Fixed64.Zero, offset.Y + Fixed64.Quarter),
            FixedQuaternion.Identity, Vector3d.One)));
        bool invalidatedAfterBothOverlaps = false;
        trigger.OnTriggerEnter += _ =>
        {
            leftEntered.Should().Be(1);
            rightEntered.Should().Be(1);
            left.Collider.TryGetCollisionPair(moving.Collider.Id, out var carrier).Should().BeTrue();
            right.Collider.TryGetCollisionPair(moving.Collider.Id, out var observation).Should().BeTrue();
            carrier!.Manifold.HasContact.Should().BeTrue();
            observation!.Manifold.HasContact.Should().BeFalse();
            invalidatedAfterBothOverlaps = true;
            if (disableFilter) left.Collider.IgnoredCollisionLayers = PhysicsLayerMask.All;
            else left.Deactivate();
        };

        context.Simulate();
        context.LateSimulate();

        invalidatedAfterBothOverlaps.Should().BeTrue();
        right.Collider.TryGetCollisionPair(moving.Collider.Id, out var surviving).Should().BeTrue();
        surviving!.IsColliding.Should().BeTrue();
        surviving.Manifold.HasContact.Should().BeTrue();
        moving.LinearVelocity.Should().Be(Vector2d.Zero);
        moving.Position.Y.Should().BeGreaterThanOrEqualTo(offset.Y + Fixed64.Half - CollisionResponse2D.PenetrationSlop);
        rightEntered.Should().Be(1);
        rightContact.Should().Be(1);

        moving.AddLinearImpulse(-Vector2d.Forward);
        context.Simulate();
        context.LateSimulate();

        rightEntered.Should().Be(1);
        rightContact.Should().Be(2, $"pose={moving.Position}, velocity={moving.LinearVelocity}, carrier={surviving.IsColliding}, rows={surviving.Manifold.Count}");
        moving.LinearVelocity.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void SolveDiscreteResponsePairs_ShouldOmitAnImmovablePairFromTheOtherMovableIsland()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D frozen = CreateBox(context, Vector2d.Zero);
        SolidBody2D wall = CreateBox(context, Vector2d.Right * (Fixed64)2, immovable: true);
        SolidBody2D moving = CreateBox(context, Vector2d.Forward * (Fixed64)8);
        SolidBody2D otherWall = CreateBox(context, new Vector2d((Fixed64)2, (Fixed64)8), immovable: true);
        frozen.Collider.Material = wall.Collider.Material = moving.Collider.Material
            = otherWall.Collider.Material = PhysicsMaterial.Frictionless;
        moving.FreezeAxes = BodyFreezeAxes2D.Rotation;
        moving.ApplyCollisionLinearVelocityDelta(Vector2d.Right);
        var frozenPair = new CollisionPair2D(frozen.Collider, wall.Collider);
        var movingPair = new CollisionPair2D(moving.Collider, otherWall.Collider);
        frozenPair.Manifold.SetContact(Vector2d.Zero, Vector2d.Zero, Fixed64.Half, Vector2d.Right);
        movingPair.Manifold.SetContact(moving.Position, otherWall.Position, Fixed64.Zero, Vector2d.Right);
        QueueResponsePair(context, frozenPair);
        QueueResponsePair(context, movingPair);
        frozen.FreezeAxes = BodyFreezeAxes2D.All;
        // The authored rows still describe the same pose after mobility changes.
        frozenPair.RefreshContactDependencies();

        typeof(GravitasPhysics2DService).GetMethod("SolveDiscreteResponsePairs",
            BindingFlags.Instance | BindingFlags.NonPublic)!.Invoke(context.Physics2D, null);

        frozen.Position.Should().Be(Vector2d.Zero);
        frozen.LinearVelocity.Should().Be(Vector2d.Zero);
        moving.LinearVelocity.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void ProcessCandidate_SleepingSurfaceNoncarrier_ShouldRetainOverlapWithoutSolvingAnEmptyManifold()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(frameRate: 60);
        context.Environment.Gravity = Fixed64.Zero;
        SolidBody2D left = CreateBody(context, new LSAABBoxCollider2D(new Vector2d(Fixed64.Two, Fixed64.One)),
            new Vector2d(-Fixed64.One, -Fixed64.Half), immovable: true);
        SolidBody2D right = CreateBody(context, new LSAABBoxCollider2D(new Vector2d(Fixed64.Two, Fixed64.One)),
            new Vector2d(Fixed64.One, -Fixed64.Half), immovable: true);
        using var surface = context.Physics2D.CreateStaticSurface(new[] { left.Collider, right.Collider });
        SolidBody2D moving = CreateBody(context, new LSAABBoxCollider2D(Vector2d.One),
            new Vector2d(Fixed64.Zero, Fixed64.Half));
        moving.FreezeAxes = BodyFreezeAxes2D.Rotation;
        moving.UseManualGrounding();
        context.Simulate();
        context.LateSimulate();
        right.Collider.TryGetCollisionPair(moving.Collider.Id, out var noncarrier).Should().BeTrue();
        noncarrier!.IsColliding.Should().BeTrue();
        noncarrier.Manifold.HasContact.Should().BeFalse();
        moving.Sleep();
        var queued = (SwiftList<CollisionPair2D>)typeof(GravitasPhysics2DService)
            .GetField("_discreteResponsePairs", BindingFlags.Instance | BindingFlags.NonPublic)!
            .GetValue(context.Physics2D)!;
        queued.FastClear();

        // An awake neighbor can make the partition revisit a sleeping pair.
        // Exercise that candidate owner with the real retained overlap pair.
        typeof(GravitasPhysics2DService).GetMethod("ProcessCandidate",
            BindingFlags.Instance | BindingFlags.NonPublic)!.Invoke(context.Physics2D,
                new object[] { moving.Collider, right.Collider, context.FrameCount });

        // Pending overlaps are filtered only after all callbacks can transfer the carrier.
        typeof(GravitasPhysics2DService).GetMethod("SolveDiscreteResponsePairs",
            BindingFlags.Instance | BindingFlags.NonPublic)!.Invoke(context.Physics2D, null);
        queued.Should().BeEmpty();
        noncarrier.IsColliding.Should().BeTrue();
        noncarrier.HasResponse.Should().BeFalse();
        moving.IsSleeping.Should().BeTrue();
        moving.Position.Should().Be(new Vector2d(Fixed64.Zero, Fixed64.Half));
        moving.LinearVelocity.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void Simulate_CompoundCrossingJoinedFloor_ShouldKeepNoncarrierOverlapWithoutARecoveryFailure()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(frameRate: 60);
        context.Environment.Gravity = Fixed64.Zero;
        SolidBody2D left = CreateBody(context, new LSAABBoxCollider2D(new Vector2d(Fixed64.Two, Fixed64.One)),
            new Vector2d(-Fixed64.One, -Fixed64.Half), immovable: true);
        SolidBody2D right = CreateBody(context, new LSAABBoxCollider2D(new Vector2d(Fixed64.Two, Fixed64.One)),
            new Vector2d(Fixed64.One, -Fixed64.Half), immovable: true);
        left.Collider.Material = right.Collider.Material = PhysicsMaterial.Frictionless;
        using var surface = context.Physics2D.CreateStaticSurface(new[] { left.Collider, right.Collider });
        var compound = new LSCompoundCollider2D(CompoundColliderPart2D.AABBox(Vector2d.One, Vector2d.Zero))
        { Material = PhysicsMaterial.Frictionless };
        SolidBody2D moving = CreateBody(context, compound,
            new Vector2d(-Fixed64.FromFraction(3, 4), Fixed64.FromFraction(49, 100)));
        moving.FreezeAxes = BodyFreezeAxes2D.Rotation;
        moving.SleepEnabled = false;
        moving.UseManualGrounding();
        moving.ApplyCollisionLinearVelocityDelta(Vector2d.Right);
        bool leftEntered = false, rightEntered = false;
        left.Collider.OnContactEnter += _ => leftEntered = true;
        right.Collider.OnContactEnter += _ => rightEntered = true;

        for (int frame = 0; frame < 64; frame++)
        {
            context.Simulate();
            context.LateSimulate();
        }

        (leftEntered && rightEntered).Should().BeTrue();
        moving.Position.X.Should().BeGreaterThan(Fixed64.Zero);
        (moving.Position.Y - Fixed64.Half).Abs().Should().BeLessThan(Fixed64.FromFraction(1, 50));
        moving.LinearVelocity.X.Should().Be(Fixed64.One);
        moving.AngularVelocity.Should().Be(Fixed64.Zero);
        // Compound priority makes it the pair owner; members retain holder IDs.
        compound.TryGetCollisionPair(right.Collider.Id, out var retained).Should().BeTrue();
        retained!.IsColliding.Should().BeTrue();
    }

    [Fact]
    public void JoinedUShape_ShouldNotBorrowAnExposedPlaneAcrossItsRealGap()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D left = CreateBody(context, new LSAABBoxCollider2D(new Vector2d(Fixed64.One, Fixed64.Two)),
            new Vector2d((Fixed64)(-2), Fixed64.Zero), immovable: true);
        SolidBody2D right = CreateBody(context, new LSAABBoxCollider2D(new Vector2d(Fixed64.One, Fixed64.Two)),
            new Vector2d(Fixed64.FromFraction(5, 2), Fixed64.Zero), immovable: true);
        SolidBody2D bottom = CreateBody(context, new LSAABBoxCollider2D(new Vector2d(Fixed64.FromFraction(11, 2), Fixed64.One)),
            new Vector2d(Fixed64.Quarter, -Fixed64.FromFraction(3, 2)), immovable: true);
        using var surface = context.Physics2D.CreateStaticSurface(new[] { left.Collider, right.Collider, bottom.Collider });
        SolidBody2D moving = CreateBody(context, new LSCircleCollider2D(Fixed64.FromFraction(17, 4)),
            new Vector2d((Fixed64)(-2), (Fixed64)(-3)));
        // The right arm's closest corner is offset(4,2): squared distance20
        // exceeds radius squared18.0625, despite overlapping broad-phase bounds.
        CollisionDetection2D.BoundsOverlap(moving.Collider, right.Collider).Should().BeTrue();
        CollisionDetection2D.TryCollide(moving.Collider, right.Collider, out _).Should().BeFalse();
        var pair = new CollisionPair2D(moving.Collider, left.Collider);

        CollisionDetection2D.TryCollide(pair, pair.Manifold, context.FrameCount).Should().BeTrue();

        pair.Manifold.HasContact.Should().BeTrue();
        for (int g = 0; g < pair.Manifold.GroupCount; g++)
        {
            ContactGroupKey key = pair.Manifold.GetGroup(g).Key;
            key.SourceIdA.Should().NotBe(right.Collider.Id + 1);
            key.SourceIdB.Should().NotBe(right.Collider.Id + 1);
        }
    }

    [Fact]
    public void CompoundJoinedCorner_ShouldPreserveEverySupplierGroupAndRefreshInheritedMaterial()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D floor = CreateBody(context, new LSAABBoxCollider2D(new Vector2d((Fixed64)4, Fixed64.One)),
            new Vector2d(Fixed64.Zero, -Fixed64.Half), immovable: true);
        SolidBody2D wall = CreateBody(context, new LSAABBoxCollider2D(new Vector2d(Fixed64.Half, (Fixed64)3)),
            new Vector2d(-Fixed64.FromFraction(3, 4), Fixed64.FromFraction(3, 2)), immovable: true);
        floor.Collider.Material = PhysicsMaterial.Frictionless;
        wall.Collider.Material = PhysicsMaterial.Bouncy;
        using var surface = context.Physics2D.CreateStaticSurface(new[] { floor.Collider, wall.Collider });
        var compound = new LSCompoundCollider2D(CompoundColliderPart2D.AABBox(Vector2d.One, Vector2d.Zero));
        SolidBody2D moving = CreateBody(context, compound, new Vector2d(-Fixed64.Quarter, Fixed64.Quarter));
        var pair = new CollisionPair2D(compound, floor.Collider);

        CollisionDetection2D.TryCollide(pair, pair.Manifold, context.FrameCount).Should().BeTrue();

        pair.Manifold.GroupCount.Should().Be(2);
        pair.Manifold.Count.Should().Be(4);
        bool floorFound = false, wallFound = false;
        for (int g = 0; g < pair.Manifold.GroupCount; g++)
        {
            ref ContactGroup2D group = ref pair.Manifold.GetGroup(g);
            group.Key.NamespaceA.Should().Be(1);
            group.Key.SourceIdB.Should().BeOneOf(floor.Collider.Id + 1, wall.Collider.Id + 1);
            floorFound |= group.Key.SourceIdB == floor.Collider.Id + 1;
            wallFound |= group.Key.SourceIdB == wall.Collider.Id + 1;
            group[0].Normal.Should().Be(group.Key.SourceIdB == floor.Collider.Id + 1
                ? -Vector2d.Forward : Vector2d.Left);
            group[0].MaterialB.Should().Be(group.Key.SourceIdB == floor.Collider.Id + 1
                ? floor.Collider.Material : wall.Collider.Material);
        }
        (floorFound && wallFound).Should().BeTrue();
        var changed = new PhysicsMaterial(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Half);
        compound.Material = changed;

        CollisionDetection2D.TryCollide(pair, pair.Manifold, context.FrameCount).Should().BeTrue();

        foreach (ManifoldContact2D contact in pair.Manifold)
            contact.MaterialA.Should().Be(changed);
        moving.Position.Should().Be(new Vector2d(-Fixed64.Quarter, Fixed64.Quarter));
    }

    [Fact]
    public void Resolve_GroupedRestitution_ShouldKeepExactAndCompactStickingFrictionEquivalent()
    {
        Vector2d? reference = null;
        foreach (Fixed64 coefficient in new[] { (Fixed64)10, Fixed64.MaxValue })
        {
            using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
            context.Settings.DiscreteSolverIterations = 1;
            context.Settings.RestitutionVelocityThreshold = Fixed64.Zero;
            SolidBody2D moving = CreateBox(context, Vector2d.Zero);
            SolidBody2D wall = CreateBox(context, Vector2d.Right * (Fixed64)2, immovable: true);
            moving.FreezeAxes = BodyFreezeAxes2D.Rotation;
            moving.ApplyCollisionLinearVelocityDelta(new Vector2d(Fixed64.One, Fixed64.Two));
            var pair = new CollisionPair2D(moving.Collider, wall.Collider);
            var material = new PhysicsMaterial(coefficient, coefficient, Fixed64.One);
            for (int g = 0; g < 2; g++)
                pair.Manifold.AddContact(new ContactGroupKey(0, 0, region: g,
                        sourceIdB: wall.Collider.Id + 1),
                    new ManifoldContact2D(1UL, Vector2d.Zero, Vector2d.Zero, Fixed64.Zero,
                        g == 0 ? Vector2d.Right : Vector2d.Forward,
                        hasMaterialOverride: true, materialA: material, materialB: material));

            CollisionResponse2D.Resolve(pair);

            moving.LinearVelocity.X.Abs().Should().BeLessThan(Fixed64.FromRaw(8));
            (moving.LinearVelocity.Y + Fixed64.Two).Abs().Should().BeLessThan(Fixed64.FromRaw(8));
            if (reference.HasValue) moving.LinearVelocity.Should().Be(reference.Value);
            reference = moving.LinearVelocity;
        }
    }

    [Theory]
    [InlineData(1, 1, false)]
    [InlineData(6, 1, false)]
    [InlineData(1, 2, false)]
    [InlineData(6, 2, false)]
    [InlineData(1, 1, true)]
    [InlineData(6, 1, true)]
    [InlineData(1, 2, true)]
    [InlineData(6, 2, true)]
    public void Resolve_ObliqueGroups_ShouldUseOneIncomingRestitutionReference(int iterations, int samples, bool island)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        context.Settings.DiscreteSolverIterations = iterations;
        context.Settings.RestitutionVelocityThreshold = Fixed64.Zero;
        SolidBody2D moving = CreateBox(context, Vector2d.Zero);
        SolidBody2D wall = CreateBox(context, Vector2d.Right * (Fixed64)2, immovable: true);
        moving.FreezeAxes = BodyFreezeAxes2D.Rotation;
        moving.ApplyCollisionLinearVelocityDelta(Vector2d.One);
        var pair = new CollisionPair2D(moving.Collider, wall.Collider);
        var material = new PhysicsMaterial(Fixed64.Zero, Fixed64.Zero, Fixed64.One);
        Vector2d oblique = new(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5));
        for (int g = 0; g < 2; g++)
            for (int sample = 0; sample < samples; sample++)
                pair.Manifold.AddContact(new ContactGroupKey(0, 0, region: g,
                        sourceIdB: wall.Collider.Id + 1),
                    new ManifoldContact2D((ulong)(sample + 1), Vector2d.Zero, Vector2d.Zero, Fixed64.Zero,
                        g == 0 ? Vector2d.Right : oblique,
                        hasMaterialOverride: true, materialA: material, materialB: material));

        if (island)
        {
            SolidBody2D otherWall = CreateBox(context, Vector2d.Forward * (Fixed64)8, immovable: true);
            var otherPair = new CollisionPair2D(moving.Collider, otherWall.Collider);
            var frictionless = new PhysicsMaterial(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);
            otherPair.Manifold.AddContact(new ContactAnchor2D(Vector2d.Zero, Vector2d.Zero),
                new ContactAnchor2D(Vector2d.Zero, Vector2d.Zero), Fixed64.Zero, Vector2d.Forward,
                frictionless, frictionless);
            // Register both contacts through the production owner so final
            // queue validation exercises real pair identity and dependencies.
            QueueResponsePair(context, pair);
            QueueResponsePair(context, otherPair);
            typeof(GravitasPhysics2DService).GetMethod("SolveDiscreteResponsePairs",
                BindingFlags.Instance | BindingFlags.NonPublic)!.Invoke(context.Physics2D, null);
        }
        else CollisionResponse2D.Resolve(pair);
        pair.ResponseState!.Impact.IsValid.Should().BeFalse();

        Fixed64 outgoing = -Vector2d.Dot(moving.LinearVelocity, oblique);
        (outgoing - Vector2d.Dot(Vector2d.One, oblique)).Abs()
            .Should().BeLessThan(Fixed64.FromRaw(64));
        moving.LinearVelocity.X.Should().BeLessThanOrEqualTo(-Fixed64.One + Fixed64.FromRaw(64));
        if (iterations > 1)
            (moving.LinearVelocity.X + Fixed64.One).Abs().Should().BeLessThan(Fixed64.FromFraction(1, 100));
    }

}
