using Chronicler;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Constraints;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using System.Linq;
using Xunit;

namespace Gravitas.Tests.Determinism;

public sealed class ReplayHashBranchCoverageTests
{
    [Fact]
    public void Joint2DReplayHash_AuthoritativeMode_ShouldIgnoreSolverCacheChanges()
    {
        using GravitasWorldContext context = CreateConstraint2DContext();
        Joint2D joint = CreateJoint2D(context);
        ChronicleHash authoritativeBefore = HashJoint2D(joint, GravitasReplayHashMode.Authoritative);
        ChronicleHash cacheBefore = HashJoint2D(joint, GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        joint.LastSolvedRowCount = 2;
        joint.AccumulatedImpulseMagnitude = Fixed64.FromFraction(3, 4);
        joint.LastSolveMetrics = new JointSolveMetrics2D(
            preparedRowCount: 2,
            linearAnchorErrorMagnitude: Fixed64.Half,
            angularErrorMagnitude: Fixed64.FromFraction(1, 4),
            limitErrorMagnitude: Fixed64.FromFraction(1, 8),
            accumulatedImpulseMagnitude: Fixed64.FromFraction(3, 4),
            incrementalImpulseMagnitude: Fixed64.FromFraction(5, 8),
            motorImpulseMagnitude: Fixed64.FromFraction(1, 3),
            motorErrorMagnitude: Fixed64.FromFraction(1, 6),
            clampedRowCount: 1);
        joint.SetCachedImpulse(0, Fixed64.Half);
        joint.SetCachedImpulse(1, Fixed64.FromFraction(3, 8));

        HashJoint2D(joint, GravitasReplayHashMode.Authoritative).Should().Be(authoritativeBefore);
        HashJoint2D(joint, GravitasReplayHashMode.AuthoritativeWithSolverCaches).Should().NotBe(cacheBefore);
    }

    [Fact]
    public void Joint2DReplayHash_ShouldDistinguishAuthoritativeMotorAndEnabledState()
    {
        using GravitasWorldContext context = CreateConstraint2DContext();
        Joint2D joint = CreateJoint2D(context);
        ChronicleHash initial = HashJoint2D(joint, GravitasReplayHashMode.Authoritative);

        joint.SetMotor(JointMotor2D.Angular(
            targetAngle: Fixed64.FromFraction(1, 4),
            driveStrength: Fixed64.One,
            damping: Fixed64.Half,
            maximumMotorImpulse: Fixed64.One));
        ChronicleHash motorHash = HashJoint2D(joint, GravitasReplayHashMode.Authoritative);
        joint.IsEnabled = false;

        motorHash.Should().NotBe(initial);
        HashJoint2D(joint, GravitasReplayHashMode.Authoritative).Should().NotBe(motorHash);
    }

    [Fact]
    public void Constraint2DReplayHash_ShouldEncodeRemovedJointSlotsAndRagdollActivation()
    {
        using GravitasWorldContext context = CreateConstraint2DContext();
        SolidBody2D first = CreateBody2D(context, Vector2d.Zero);
        SolidBody2D second = CreateBody2D(context, Vector2d.Right * (Fixed64)2);
        SolidBody2D third = CreateBody2D(context, Vector2d.Right * (Fixed64)4);
        Joint2D removed = context.Constraints2D.RegisterJoint(CreatePin2D(first, second));
        context.Constraints2D.RegisterJoint(CreatePin2D(second, third));
        ChronicleHash beforeRemoval = HashConstraints2D(context, GravitasReplayHashMode.Authoritative);

        context.Constraints2D.RemoveJoint(removed.Id).Should().BeTrue();
        ChronicleHash afterRemoval = HashConstraints2D(context, GravitasReplayHashMode.Authoritative);
        RagdollRuntime2D ragdoll = context.Constraints2D.RegisterRagdoll(new RagdollDefinition2D(
            new[]
            {
                new RagdollLinkDefinition2D(0, first),
                new RagdollLinkDefinition2D(1, third)
            },
            new[]
            {
                new RagdollJointDefinition2D(
                    0,
                    1,
                    JointType2D.Pin,
                    JointFrame2D.Identity,
                    JointFrame2D.Identity)
            },
            RagdollSelfCollisionPolicy.CollideAllLinks));
        ChronicleHash activeRagdoll = HashConstraints2D(context, GravitasReplayHashMode.Authoritative);

        ragdoll.DeactivateToKinematic();

        afterRemoval.Should().NotBe(beforeRemoval);
        activeRagdoll.Should().NotBe(afterRemoval);
        HashConstraints2D(context, GravitasReplayHashMode.Authoritative).Should().NotBe(activeRagdoll);
    }

    [Fact]
    public void CollisionPair2DReplayHash_ShouldEncodeSortedManifoldMaterialsAndWarmStartState()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        LSCircleCollider2D trigger = CreateBodylessCircle2D(context, Vector2d.Zero);
        SolidBody2D body = CreateBody2D(context, Vector2d.Right * Fixed64.Half);
        trigger.IsTrigger = true;
        context.Physics2D.PrepareReplayColliders();
        var first = new CollisionPair2D(trigger, body.Collider);
        var second = new CollisionPair2D(trigger, body.Collider);
        var materialPair = new CollisionPair2D(trigger, body.Collider);
        ChronicleHash empty = HashPair2D(first);

        AddTwo2DContacts(first, reversed: false, withMaterial: false);
        AddTwo2DContacts(second, reversed: true, withMaterial: false);
        AddTwo2DContacts(materialPair, reversed: false, withMaterial: true);
        first.MarkColliding(frame: 9);
        second.MarkColliding(frame: 9);
        materialPair.MarkColliding(frame: 9);
        ChronicleHash contactOnly = HashPair2D(first);
        ChronicleHash materialOverride = HashPair2D(materialPair);
        ulong contactId = first.Manifold[0].ContactId;
        first.StoreWarmStartImpulse(contactId, Fixed64.Half, Fixed64.FromFraction(1, 4));
        second.StoreWarmStartImpulse(contactId, Fixed64.Half, Fixed64.FromFraction(1, 4));
        ChronicleHash warmStarted = HashPair2D(first);

        contactOnly.Should().NotBe(empty);
        materialOverride.Should().NotBe(contactOnly);
        warmStarted.Should().NotBe(contactOnly);
        HashPair2D(second).Should().Be(warmStarted);
    }

    [Fact]
    public void CollisionPair3DReplayHash_ShouldEncodeMaterialAndWarmStartState()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * Fixed64.Half);
        scenario.Context.Physics.PrepareReplayColliders();
        CollisionPair pair = scenario.CreatePair(first.Collider, second.Collider);
        CollisionPair materialPair = scenario.CreatePair(first.Collider, second.Collider);
        ChronicleHash empty = HashPair3D(pair, GravitasReplayHashMode.Authoritative);

        Add3DContact(pair, withMaterial: false);
        Add3DContact(materialPair, withMaterial: true);
        ChronicleHash contactOnly = HashPair3D(pair, GravitasReplayHashMode.Authoritative);
        ChronicleHash materialOverride = HashPair3D(materialPair, GravitasReplayHashMode.Authoritative);
        ulong contactId = pair.Manifold[0].ContactId;
        pair.StoreWarmStartImpulse(
            contactId,
            Vector3d.Right,
            Fixed64.Half,
            Fixed64.FromFraction(1, 4),
            Fixed64.FromFraction(1, 8));
        ChronicleHash warmStarted = HashPair3D(pair, GravitasReplayHashMode.Authoritative);

        contactOnly.Should().NotBe(empty);
        materialOverride.Should().NotBe(contactOnly);
        warmStarted.Should().NotBe(contactOnly);
    }

    [Fact]
    public void CollisionPairMixedReplayHash_ShouldEncodeContactMaterialAndTriggerState()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSSphereCollider collider3D = CreateBodylessSphere3D(context, Vector3d.Zero);
        LSCircleCollider2D collider2D = CreateBodylessCircle2D(context, Vector2d.Right * Fixed64.Half);
        context.Physics.PrepareReplayColliders();
        context.Physics2D.PrepareReplayColliders();
        var nonTriggerPair = new CollisionPairMixed(collider3D, collider2D);
        ChronicleHash empty = HashMixedPair(nonTriggerPair);
        var contact = new MixedContact(
            Vector3d.Zero,
            new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right,
            Fixed64.FromFraction(1, 4));

        nonTriggerPair.MarkColliding(frame: 5, contact);
        ChronicleHash nonTriggerContact = HashMixedPair(nonTriggerPair);
        collider3D.IsTrigger = true;
        var triggerPair = new CollisionPairMixed(collider3D, collider2D);
        triggerPair.MarkColliding(frame: 5, contact);
        ChronicleHash triggerContact = HashMixedPair(triggerPair);
        triggerPair.MarkColliding(frame: 5, contact.WithMaterialOverride(
            new PhysicsMaterial(Fixed64.One, Fixed64.Half, Fixed64.Zero),
            new PhysicsMaterial(Fixed64.Half, Fixed64.Half, Fixed64.Half)));

        nonTriggerContact.Should().NotBe(empty);
        triggerContact.Should().NotBe(nonTriggerContact);
        HashMixedPair(triggerPair).Should().NotBe(triggerContact);
    }

    [Fact]
    public void CollisionPairMixedReplayHash_ShouldUseReplayOrdinalsInsteadOfRawColliderIds()
    {
        using GravitasWorldContext compact = CreateMixedContext();
        using GravitasWorldContext churned = CreateMixedContext();
        ChurnDeleted3DColliders(churned, 6);
        ChurnDeleted2DColliders(churned, 6);
        CollisionPairMixed compactPair = CreateReplayOrdinalMixedPair(compact);
        CollisionPairMixed churnedPair = CreateReplayOrdinalMixedPair(churned);

        HashMixedPair(churnedPair).Should().Be(HashMixedPair(compactPair));
    }

    [Fact]
    public void Body3DReplayHash_ShouldEncodePendingContinuousCollisionHandoff()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.SetFrameRate(1);
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        LSSphereCollider ignored = scenario.CreateStaticSphere(Vector3d.Right * (Fixed64)4);
        ChronicleHash authoritativeBefore = HashBody3D(body.Body, GravitasReplayHashMode.Authoritative);
        ChronicleHash cacheBefore = HashBody3D(body.Body, GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        body.Body.ApplyContinuousCollisionHandoff(
            Vector3d.Right * Fixed64.Half,
            Vector3d.Right,
            Fixed64.Half,
            ignoredCollider3D: ignored);

        HashBody3D(body.Body, GravitasReplayHashMode.Authoritative).Should().NotBe(authoritativeBefore);
        HashBody3D(body.Body, GravitasReplayHashMode.AuthoritativeWithSolverCaches).Should().NotBe(cacheBefore);
    }

    [Fact]
    public void Body2DReplayHash_ShouldEncodePendingContinuousCollisionHandoff()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(frameRate: 1);
        SolidBody2D body = CreateBody2D(context, Vector2d.Zero);
        LSCircleCollider2D ignored = CreateBodylessCircle2D(context, Vector2d.Right * (Fixed64)4);
        ChronicleHash authoritativeBefore = HashBody2D(body, GravitasReplayHashMode.Authoritative);
        ChronicleHash cacheBefore = HashBody2D(body, GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        body.ApplyContinuousCollisionHandoff(
            Vector2d.Right * Fixed64.Half,
            Vector2d.Right,
            Fixed64.Half,
            ignoredCollider2D: ignored);

        HashBody2D(body, GravitasReplayHashMode.Authoritative).Should().NotBe(authoritativeBefore);
        HashBody2D(body, GravitasReplayHashMode.AuthoritativeWithSolverCaches).Should().NotBe(cacheBefore);
    }

    [Fact]
    public void ComputeReplayHash_WithMixedHierarchy_ShouldUseReplayOrdinalsInsteadOfRawColliderIds()
    {
        using GravitasWorldContext compact = CreateMixedHierarchyContext(churnBeforeLive: 0);
        using GravitasWorldContext churned = CreateMixedHierarchyContext(churnBeforeLive: 6);

        churned.ComputeReplayHash().Should().Be(compact.ComputeReplayHash());
    }

    [Fact]
    public void ComputeReplayHash_WithCompound2DShapeDefinitions_ShouldEncodeAuthoredPartPayloads()
    {
        ChronicleHash baseline = HashCompound2DReplay(Compound2DReplayVariant.Baseline);

        HashCompound2DReplay(Compound2DReplayVariant.Baseline).Should().Be(baseline);
        HashCompound2DReplay(Compound2DReplayVariant.ShapeMaterial).Should().NotBe(baseline);
        HashCompound2DReplay(Compound2DReplayVariant.PartMaterial).Should().NotBe(baseline);
        HashCompound2DReplay(Compound2DReplayVariant.PartOffset).Should().NotBe(baseline);
        HashCompound2DReplay(Compound2DReplayVariant.PartRotation).Should().NotBe(baseline);
        HashCompound2DReplay(Compound2DReplayVariant.PartScale).Should().NotBe(baseline);
        HashCompound2DReplay(Compound2DReplayVariant.CapsuleHeight).Should().NotBe(baseline);
        HashCompound2DReplay(Compound2DReplayVariant.BoxSize).Should().NotBe(baseline);
        HashCompound2DReplay(Compound2DReplayVariant.PolygonVertex).Should().NotBe(baseline);
    }

    [Fact]
    public void ComputeReplayHash_WithCompound3DShapeDefinitions_ShouldEncodeAuthoredPartPayloads()
    {
        ChronicleHash baseline = HashCompound3DReplay(Compound3DReplayVariant.Baseline);

        HashCompound3DReplay(Compound3DReplayVariant.Baseline).Should().Be(baseline);
        HashCompound3DReplay(Compound3DReplayVariant.ShapeMaterial).Should().NotBe(baseline);
        HashCompound3DReplay(Compound3DReplayVariant.PartMaterial).Should().NotBe(baseline);
        HashCompound3DReplay(Compound3DReplayVariant.PartOffset).Should().NotBe(baseline);
        HashCompound3DReplay(Compound3DReplayVariant.PartRotation).Should().NotBe(baseline);
        HashCompound3DReplay(Compound3DReplayVariant.PartScale).Should().NotBe(baseline);
        HashCompound3DReplay(Compound3DReplayVariant.ConeHeight).Should().NotBe(baseline);
        HashCompound3DReplay(Compound3DReplayVariant.MeshVertex).Should().NotBe(baseline);
    }

    private static Joint2D CreateJoint2D(GravitasWorldContext context)
    {
        SolidBody2D first = CreateBody2D(context, Vector2d.Zero);
        SolidBody2D second = CreateBody2D(context, Vector2d.Right * (Fixed64)2);
        return context.Constraints2D.RegisterJoint(CreatePin2D(first, second));
    }

    private static JointDefinition2D CreatePin2D(SolidBody2D first, SolidBody2D second) =>
        new(
            first,
            second,
            new JointFrame2D(Vector2d.Right * Fixed64.Half, Fixed64.Zero),
            new JointFrame2D(-Vector2d.Right * Fixed64.Half, Fixed64.Zero),
            JointType2D.Pin,
            JointLimit2D.Unrestricted,
            JointMotor2D.Disabled,
            JointCollisionPolicy.SuppressLinked);

    private static void AddTwo2DContacts(CollisionPair2D pair, bool reversed, bool withMaterial)
    {
        pair.Manifold.BeginUpdate(9);
        if (reversed)
        {
            AddSecond2DContact(pair, withMaterial);
            AddFirst2DContact(pair, withMaterial);
            return;
        }

        AddFirst2DContact(pair, withMaterial);
        AddSecond2DContact(pair, withMaterial);
    }

    private static void AddFirst2DContact(CollisionPair2D pair, bool withMaterial)
    {
        var pointA = new Vector2d(Fixed64.Zero, Fixed64.Zero);
        var pointB = new Vector2d(Fixed64.Half, Fixed64.Zero);
        if (withMaterial)
        {
            pair.Manifold.AddContact(
                pointA,
                pointB,
                Fixed64.FromFraction(1, 4),
                Vector2d.Right,
                new PhysicsMaterial(Fixed64.One, Fixed64.Half, Fixed64.Zero),
                new PhysicsMaterial(Fixed64.Half, Fixed64.Half, Fixed64.Half));
            return;
        }

        pair.Manifold.AddContact(pointA, pointB, Fixed64.FromFraction(1, 4), Vector2d.Right);
    }

    private static void AddSecond2DContact(CollisionPair2D pair, bool withMaterial)
    {
        var pointA = new Vector2d(Fixed64.Zero, Fixed64.Half);
        var pointB = new Vector2d(Fixed64.Half, Fixed64.Half);
        if (withMaterial)
        {
            pair.Manifold.AddContact(
                pointA,
                pointB,
                Fixed64.FromFraction(1, 8),
                Vector2d.Right,
                new PhysicsMaterial(Fixed64.One, Fixed64.Half, Fixed64.Zero),
                new PhysicsMaterial(Fixed64.Half, Fixed64.Half, Fixed64.Half));
            return;
        }

        pair.Manifold.AddContact(pointA, pointB, Fixed64.FromFraction(1, 8), Vector2d.Right);
    }

    private static void Add3DContact(CollisionPair pair, bool withMaterial)
    {
        pair.Manifold.BeginUpdate(11);
        if (withMaterial)
        {
            pair.Manifold.AddContact(
                Vector3d.Zero,
                Vector3d.Right * Fixed64.Half,
                Fixed64.FromFraction(1, 4),
                Vector3d.Right,
                new PhysicsMaterial(Fixed64.One, Fixed64.Half, Fixed64.Zero),
                new PhysicsMaterial(Fixed64.Half, Fixed64.Half, Fixed64.Half));
            return;
        }

        pair.Manifold.AddContact(
            Vector3d.Zero,
            Vector3d.Right * Fixed64.Half,
            Fixed64.FromFraction(1, 4),
            Vector3d.Right);
    }

    private static GravitasWorldContext CreateConstraint2DContext()
    {
        GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        context.Settings.DiscreteSolverIterations = 8;
        return context;
    }

    private static GravitasWorldContext CreateMixedContext()
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.SetFrameRate(8);
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-16), (Fixed64)(-4), (Fixed64)(-16)),
                new Vector3d((Fixed64)16, (Fixed64)4, (Fixed64)16)),
            out _).Should().BeTrue();
        return context;
    }

    private static GravitasWorldContext CreateMixedHierarchyContext(int churnBeforeLive)
    {
        GravitasWorldContext context = CreateMixedContext();
        ChurnDeleted3DColliders(context, churnBeforeLive);
        ChurnDeleted2DColliders(context, churnBeforeLive);
        LSSphereCollider parent3D = CreateBodylessSphere3D(
            context,
            new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero));
        LSCircleCollider2D child2D = CreateBodylessCircle2D(
            context,
            new Vector2d((Fixed64)(-3), Fixed64.Zero));
        LSCircleCollider2D parent2D = CreateBodylessCircle2D(
            context,
            new Vector2d((Fixed64)3, Fixed64.Zero));
        LSSphereCollider child3D = CreateBodylessSphere3D(
            context,
            new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));

        child2D.SetParent(parent3D);
        child3D.SetParent(parent2D);
        return context;
    }

    private static CollisionPairMixed CreateReplayOrdinalMixedPair(GravitasWorldContext context)
    {
        LSSphereCollider collider3D = CreateBodylessSphere3D(context, Vector3d.Zero);
        LSCircleCollider2D collider2D = CreateBodylessCircle2D(context, Vector2d.Right * Fixed64.Half);
        context.Physics.PrepareReplayColliders();
        context.Physics2D.PrepareReplayColliders();
        var pair = new CollisionPairMixed(collider3D, collider2D);
        pair.MarkColliding(
            frame: 5,
            new MixedContact(
                Vector3d.Zero,
                new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero),
                Vector3d.Right,
                Fixed64.FromFraction(1, 4)));
        return pair;
    }

    private static ChronicleHash HashCompound2DReplay(Compound2DReplayVariant variant)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var compound = new LSCompoundCollider2D(CreateCompound2DParts(variant));
        compound.InitializeWithNoBody(new TestMatterAgent(context));
        return context.ComputeReplayHash();
    }

    private static CompoundColliderPart2D[] CreateCompound2DParts(Compound2DReplayVariant variant)
    {
        PhysicsMaterial shapeMaterial = PhysicsMaterialTestHelper.WithFrictionAndRestitution(
            variant == Compound2DReplayVariant.ShapeMaterial ? Fixed64.Half : Fixed64.FromFraction(3, 4),
            Fixed64.FromFraction(1, 4));
        PhysicsMaterial partMaterial = PhysicsMaterialTestHelper.WithFrictionAndRestitution(
            variant == Compound2DReplayVariant.PartMaterial ? Fixed64.Half : Fixed64.FromFraction(5, 8),
            Fixed64.FromFraction(1, 8));
        Vector2d capsuleOffset = variant == Compound2DReplayVariant.PartOffset
            ? Vector2d.Right
            : Vector2d.Zero;
        Fixed64 capsuleRotation = variant == Compound2DReplayVariant.PartRotation
            ? Fixed64.FromFraction(1, 8)
            : Fixed64.Zero;
        Vector2d capsuleScale = variant == Compound2DReplayVariant.PartScale
            ? new Vector2d(Fixed64.One, Fixed64.FromFraction(3, 2))
            : Vector2d.One;
        Fixed64 capsuleHeight = variant == Compound2DReplayVariant.CapsuleHeight ? (Fixed64)4 : (Fixed64)3;
        Vector2d boxSize = variant == Compound2DReplayVariant.BoxSize
            ? new Vector2d(Fixed64.One, (Fixed64)3)
            : new Vector2d(Fixed64.One, (Fixed64)2);
        Vector2d polygonPeak = variant == Compound2DReplayVariant.PolygonVertex
            ? new Vector2d(Fixed64.Zero, Fixed64.FromFraction(3, 2))
            : Vector2d.Forward;

        return new[]
        {
            new CompoundColliderPart2D(
                ColliderShapeDefinition2D.Circle(Fixed64.Half, shapeMaterial),
                new Vector2d((Fixed64)(-3), Fixed64.Zero)),
            new CompoundColliderPart2D(
                ColliderShapeDefinition2D.Capsule(Fixed64.Half, capsuleHeight),
                capsuleOffset,
                capsuleRotation,
                capsuleScale,
                partMaterial),
            new CompoundColliderPart2D(
                ColliderShapeDefinition2D.AABBox(boxSize),
                new Vector2d((Fixed64)3, Fixed64.Zero)),
            new CompoundColliderPart2D(
                ColliderShapeDefinition2D.ConvexPolygon(
                    new Vector2d(-Fixed64.Half, Fixed64.Zero),
                    new Vector2d(Fixed64.Half, Fixed64.Zero),
                    polygonPeak),
                new Vector2d(Fixed64.Zero, (Fixed64)3))
        };
    }

    private static ChronicleHash HashCompound3DReplay(Compound3DReplayVariant variant)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var compound = new LSCompoundCollider(CreateCompound3DParts(variant));
        scenario.InitializeStaticCollider(compound, Vector3d.Zero);
        return scenario.Context.ComputeReplayHash();
    }

    private static CompoundColliderPart[] CreateCompound3DParts(Compound3DReplayVariant variant)
    {
        PhysicsMaterial shapeMaterial = PhysicsMaterialTestHelper.WithFrictionAndRestitution(
            variant == Compound3DReplayVariant.ShapeMaterial ? Fixed64.Half : Fixed64.FromFraction(3, 4),
            Fixed64.FromFraction(1, 4));
        PhysicsMaterial partMaterial = PhysicsMaterialTestHelper.WithFrictionAndRestitution(
            variant == Compound3DReplayVariant.PartMaterial ? Fixed64.Half : Fixed64.FromFraction(5, 8),
            Fixed64.FromFraction(1, 8));
        Vector3d capsuleOffset = variant == Compound3DReplayVariant.PartOffset
            ? new Vector3d((Fixed64)(-2), Fixed64.Half, Fixed64.Zero)
            : new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero);
        FixedQuaternion capsuleRotation = variant == Compound3DReplayVariant.PartRotation
            ? FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)15, Fixed64.Zero)
            : FixedQuaternion.Identity;
        Vector3d capsuleScale = variant == Compound3DReplayVariant.PartScale
            ? new Vector3d(Fixed64.One, Fixed64.FromFraction(3, 2), Fixed64.One)
            : Vector3d.One;
        Fixed64 coneHeight = variant == Compound3DReplayVariant.ConeHeight ? (Fixed64)3 : (Fixed64)2;
        ColliderShapeDefinition meshDefinition = CreateConvexMeshReplayDefinition(variant, shapeMaterial);

        return new[]
        {
            new CompoundColliderPart(
                ColliderShapeDefinition.Sphere(Fixed64.Half, shapeMaterial),
                new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero)),
            new CompoundColliderPart(
                ColliderShapeDefinition.Capsule(Fixed64.Half, (Fixed64)2),
                capsuleOffset,
                capsuleRotation,
                capsuleScale,
                partMaterial),
            new CompoundColliderPart(
                ColliderShapeDefinition.Cuboid(Vector3d.One),
                Vector3d.Zero),
            new CompoundColliderPart(
                ColliderShapeDefinition.Cylinder(Fixed64.Half, (Fixed64)2),
                new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero)),
            new CompoundColliderPart(
                ColliderShapeDefinition.Cone(Fixed64.Half, coneHeight),
                new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero)),
            new CompoundColliderPart(
                meshDefinition,
                new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)3))
        };
    }

    private static ColliderShapeDefinition CreateConvexMeshReplayDefinition(
        Compound3DReplayVariant variant,
        PhysicsMaterial material)
    {
        LSMeshCollider mesh = MeshTestFixtures.CreateConvexCube();
        Vector3d[] vertices = mesh.Mesh.LocalVertices.ToArray();
        if (variant == Compound3DReplayVariant.MeshVertex)
            vertices[7] += new Vector3d(Fixed64.Zero, Fixed64.FromFraction(1, 4), Fixed64.Zero);

        return ColliderShapeDefinition.ConvexMesh(
            vertices,
            mesh.Mesh.Triangles.ToArray(),
            MeshInertiaPolicy.RequireClosedVolume,
            material);
    }

    private static SolidBody2D CreateBody2D(GravitasWorldContext context, Vector2d position)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var body = new SolidBody2D(
            new TestMatterAgent(context, transform),
            new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One
        };
        body.Initialize(position);
        return body;
    }

    private static LSCircleCollider2D CreateBodylessCircle2D(GravitasWorldContext context, Vector2d position)
    {
        var collider = new LSCircleCollider2D(Fixed64.Half);
        var transform = new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));
        return collider;
    }

    private static LSSphereCollider CreateBodylessSphere3D(GravitasWorldContext context, Vector3d position)
    {
        var collider = new LSSphereCollider();
        var transform = new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One);
        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));
        return collider;
    }

    private static void ChurnDeleted3DColliders(GravitasWorldContext context, int count)
    {
        for (int i = 0; i < count; i++)
        {
            LSSphereCollider collider = CreateBodylessSphere3D(
                context,
                new Vector3d((Fixed64)(8 + i), Fixed64.Zero, Fixed64.Zero));
            collider.Deactivate();
        }
    }

    private static void ChurnDeleted2DColliders(GravitasWorldContext context, int count)
    {
        for (int i = 0; i < count; i++)
        {
            LSCircleCollider2D collider = CreateBodylessCircle2D(
                context,
                new Vector2d((Fixed64)(8 + i), Fixed64.Zero));
            collider.Deactivate();
        }
    }

    private static ChronicleHash HashJoint2D(Joint2D joint, GravitasReplayHashMode mode) =>
        Hash((ref ChronicleHashWriter writer) => joint.ContributeReplayHash(ref writer, mode));

    private static ChronicleHash HashConstraints2D(GravitasWorldContext context, GravitasReplayHashMode mode) =>
        Hash((ref ChronicleHashWriter writer) => context.Constraints2D.ContributeReplayHash(ref writer, mode));

    private static ChronicleHash HashBody3D(SolidBody body, GravitasReplayHashMode mode) =>
        Hash((ref ChronicleHashWriter writer) => body.ContributeReplayHash(ref writer, mode));

    private static ChronicleHash HashBody2D(SolidBody2D body, GravitasReplayHashMode mode) =>
        Hash((ref ChronicleHashWriter writer) => body.ContributeReplayHash(ref writer, mode));

    private static ChronicleHash HashPair2D(CollisionPair2D pair) =>
        Hash((ref ChronicleHashWriter writer) =>
            pair.ContributeReplayHash(ref writer, GravitasReplayHashMode.Authoritative));

    private static ChronicleHash HashPair3D(CollisionPair pair, GravitasReplayHashMode mode) =>
        Hash((ref ChronicleHashWriter writer) => pair.ContributeReplayHash(ref writer, mode));

    private static ChronicleHash HashMixedPair(CollisionPairMixed pair) =>
        Hash((ref ChronicleHashWriter writer) =>
            pair.ContributeReplayHash(ref writer, GravitasReplayHashMode.Authoritative));

    private static ChronicleHash Hash(WriterAction action)
    {
        var writer = new ChronicleHashWriter();
        action(ref writer);
        return writer.ToHash();
    }

    private delegate void WriterAction(ref ChronicleHashWriter writer);

    private enum Compound2DReplayVariant
    {
        Baseline,
        ShapeMaterial,
        PartMaterial,
        PartOffset,
        PartRotation,
        PartScale,
        CapsuleHeight,
        BoxSize,
        PolygonVertex
    }

    private enum Compound3DReplayVariant
    {
        Baseline,
        ShapeMaterial,
        PartMaterial,
        PartOffset,
        PartRotation,
        PartScale,
        ConeHeight,
        MeshVertex
    }
}
