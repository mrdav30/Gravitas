using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

/// <content>Verifies initial-contact CCD admission for convex sources against two-sided mesh surfaces.</content>
public sealed partial class ContinuousCollisionDetectionTests
{
    public static TheoryData<MeshColliderMode, bool, bool, bool> InitialConvexMeshContactCases
    {
        get
        {
            var cases = new TheoryData<MeshColliderMode, bool, bool, bool>();
            foreach (bool reverseWinding in new[] { false, true })
            foreach (bool kinematic in new[] { false, true })
            {
                cases.Add(MeshColliderMode.Convex, reverseWinding, false, kinematic);
                cases.Add(MeshColliderMode.Concave, reverseWinding, false, kinematic);
                cases.Add(MeshColliderMode.Convex, reverseWinding, true, kinematic);
            }
            return cases;
        }
    }

    [Theory]
    [MemberData(nameof(InitialConvexMeshContactCases))]
    public void ContinuousCuboid_OnMeshSurface_ShouldAllowExitAndTangentMotionAndRejectClosing(
        MeshColliderMode mode, bool reverseWinding, bool compoundTarget, bool kinematic)
    {
        foreach (int side in new[] { -1, 1 })
        foreach (Fixed64 gap in new[] { Fixed64.Half, Fixed64.Quarter })
        foreach (int motion in new[] { 1, 0, -1 })
        {
            using PhysicsScenarioBuilder scenario = CreateCcdScenario();
            AddInitialConvexMeshTarget(scenario, mode, reverseWinding, compoundTarget);
            Vector3d normal = Vector3d.Up * (Fixed64)side;
            Vector3d start = normal * gap;
            SolidBody source = scenario.CreateBody(new LSCuboidCollider(), start, FixedQuaternion.Identity,
                preventAngularForces: true, isKinematic: kinematic).Body;
            source.Collider.Material = PhysicsMaterial.Frictionless;
            source.UseManualGrounding();
            source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            Vector3d displacement = motion == 0 ? Vector3d.Right : normal * (Fixed64)motion;
            if (kinematic)
                source.Agent.Transform.LocalPosition = start + displacement;
            else
                source.AddLinearImpulse(displacement * source.Mass);

            scenario.Context.Simulate();
            scenario.Context.LateSimulate();

            if (motion < 0)
            {
                (source.Position3d.Y * (Fixed64)side).Should().BeGreaterThanOrEqualTo(gap - Fixed64.Epsilon);
                source.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
                if (!kinematic)
                    source.LinearVelocity.Y.Should().Be(Fixed64.Zero);
            }
            else
            {
                if (motion == 0 && gap < Fixed64.Half && !kinematic)
                {
                    // Discrete penetration correction may change only the
                    // normal coordinate after CCD admits this tangential travel.
                    source.Position3d.X.Should().Be(displacement.X);
                    source.Position3d.Z.Should().Be(Fixed64.Zero);
                    (source.Position3d.Y * (Fixed64)side).Should().BeGreaterThanOrEqualTo(gap - Fixed64.Epsilon);
                }
                else
                    source.Position3d.Should().Be(start + displacement);
                source.LastContinuousCollisionToiIterationCount.Should().Be(0);
                if (!kinematic)
                    source.LinearVelocity.Should().Be(displacement);
            }
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ContinuousOffsetConvexSource_ShouldExitMeshUsingLeafGeometry(bool compoundSource, bool kinematic)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        AddInitialConvexMeshTarget(scenario, MeshColliderMode.Concave, false, false);
        LSCollider collider;
        if (compoundSource)
            collider = new LSCompoundCollider(new CompoundColliderPart(
                MeshTestFixtures.CreateConvexCubeDefinition(), Vector3d.Down * (Fixed64)2));
        else
        {
            LSMeshCollider cube = MeshTestFixtures.CreateConvexCube();
            Vector3d[] vertices = cube.Mesh.LocalVertices.ToArray();
            for (int i = 0; i < vertices.Length; i++)
                vertices[i] += Vector3d.Down * (Fixed64)2;
            collider = new LSMeshCollider(vertices, cube.Mesh.Triangles.ToArray());
        }
        Vector3d start = Vector3d.Up * Fixed64.FromFraction(3, 2);
        SolidBody source = scenario.CreateBody(collider, start, FixedQuaternion.Identity,
            preventAngularForces: true, isKinematic: kinematic).Body;
        source.Collider.Material = PhysicsMaterial.Frictionless;
        source.UseManualGrounding();
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        collider.BoundsMax.Y.Should().Be(Fixed64.Zero);
        if (kinematic)
            source.Agent.Transform.LocalPosition = start + Vector3d.Down;
        else
            source.AddLinearImpulse(Vector3d.Down * source.Mass);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        source.Position3d.Should().Be(start + Vector3d.Down);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
        collider.BoundsMax.Y.Should().Be(-Fixed64.One);
        if (!kinematic)
            source.LinearVelocity.Should().Be(Vector3d.Down);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContinuousCuboid_BelowRisingKinematicMesh_ShouldRetainSeparatingRelativeMotion(bool reverseWinding)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        FixedQuaternion floorRotation = reverseWinding
            ? new FixedQuaternion(Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero)
            : FixedQuaternion.Identity;
        SolidBody floor = scenario.CreateBody(MeshTestFixtures.CreateConvexQuadFloor(),
            Vector3d.Zero, floorRotation, isKinematic: true).Body;
        floor.Collider.Material = PhysicsMaterial.Frictionless;
        floor.UseManualGrounding();
        SolidBody source = scenario.CreateCuboid(Vector3d.Down * Fixed64.Half,
            preventAngularForces: true).Body;
        source.Collider.Material = PhysicsMaterial.Frictionless;
        source.UseManualGrounding();
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Collider.BoundsMax.Y.Should().Be(floor.Collider.BoundsMin.Y);

        floor.Agent.Transform.LocalPosition = Vector3d.Up;
        source.AddLinearImpulse(Vector3d.Down * source.Mass);
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        floor.Position3d.Should().Be(Vector3d.Up);
        floor.Rotation.Should().Be(floorRotation);
        floor.SampleContinuousCollisionLinearVelocity(Fixed64.Half).Should().Be(Vector3d.Up);
        source.Position3d.Should().Be(Vector3d.Down * Fixed64.FromFraction(3, 2));
        source.LinearVelocity.Should().Be(Vector3d.Down);
        source.AngularVelocity.Should().Be(Vector3d.Zero);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
        source.Collider.BoundsMax.Y.Should().Be(-Fixed64.One);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ContinuousYawingCuboid_OnMeshFloor_ShouldRetainTangentialTranslationAndRotation(
        bool reverseWinding, bool kinematic)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        AddInitialConvexMeshTarget(scenario, MeshColliderMode.Convex, reverseWinding, false);
        Vector3d start = Vector3d.Up * Fixed64.Half;
        SolidBody source = scenario.CreateBody(new LSCuboidCollider(), start,
            FixedQuaternion.Identity, isKinematic: kinematic).Body;
        source.Collider.Material = PhysicsMaterial.Frictionless;
        source.UseManualGrounding();
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Collider.BoundsMin.Y.Should().Be(Fixed64.Zero);
        Vector3d angularVelocity = Vector3d.Zero;
        FixedQuaternion expectedRotation;
        if (kinematic)
        {
            expectedRotation = PhysicsScenarioBuilder.Yaw(45);
            source.Agent.Transform.LocalPosition = start + Vector3d.Right;
            source.Agent.Transform.LocalRotation = expectedRotation;
        }
        else
        {
            // Yaw leaves the vertical contact extent unchanged throughout the
            // angular path. Every contact velocity is tangential to this floor.
            angularVelocity = Vector3d.Up * (Fixed64.Quarter * source.EffectiveInverseInertiaTensor.M22);
            source.AddLinearImpulse(Vector3d.Right * source.Mass);
            source.AddAngularImpulse(Vector3d.Up * Fixed64.Quarter);
            expectedRotation = new FixedQuaternion(Fixed64.Zero,
                angularVelocity.Y * Fixed64.Half, Fixed64.Zero, Fixed64.One).Normalized;
        }

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        source.Position3d.Should().Be(start + Vector3d.Right);
        source.Rotation.Should().Be(expectedRotation);
        source.AngularVelocity.Should().Be(angularVelocity);
        source.SampleContinuousCollisionLinearVelocity(Fixed64.Half).Should().Be(Vector3d.Right);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
        source.Collider.BoundsMin.Y.Should().Be(Fixed64.Zero);
        if (!kinematic)
            source.LinearVelocity.Should().Be(Vector3d.Right);
    }

    [Theory]
    [InlineData(0, -1, false)]
    [InlineData(0, -1, true)]
    [InlineData(0, 1, false)]
    [InlineData(0, 1, true)]
    [InlineData(2, -1, false)]
    [InlineData(2, -1, true)]
    [InlineData(2, 1, false)]
    [InlineData(2, 1, true)]
    public void ContinuousCardinalSpin_OnMeshPlane_ShouldRetainTangentialMotion(int axis, int side, bool kinematic)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        Vector3d unitAxis = axis == 0 ? Vector3d.Right : Vector3d.Forward;
        Vector3d normal = unitAxis * (Fixed64)side;
        Vector3d travel = axis == 0 ? Vector3d.Forward : Vector3d.Right;
        FixedQuaternion planeRotation = axis == 0
            ? new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, -Fixed64.One, Fixed64.One).Normalized
            : new FixedQuaternion(Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.One).Normalized;
        LSMeshCollider mesh = MeshTestFixtures.CreateConvexQuadFloor(
            axis == 0 ? MeshColliderMode.Concave : MeshColliderMode.Convex);
        LSCollider target = axis == 0 ? mesh : new LSCompoundCollider(
            new CompoundColliderPart(MeshTestFixtures.CreateDefinition(mesh), Vector3d.Zero));
        target.Material = PhysicsMaterial.Frictionless;
        target.InitializeWithNoBody(new TestMatterAgent(scenario.Context,
            new FixedTransform(Vector3d.Zero, planeRotation, Vector3d.One)));
        Vector3d start = normal * Fixed64.Half;
        SolidBody source = scenario.CreateBody(new LSCuboidCollider(), start,
            FixedQuaternion.Identity, isKinematic: kinematic).Body;
        source.Collider.Material = PhysicsMaterial.Frictionless;
        source.UseManualGrounding();
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Vector3d angularVelocity = Vector3d.Zero;
        FixedQuaternion expectedRotation;
        if (kinematic)
        {
            Vector3d imaginary = unitAxis * Fixed64.Quarter;
            expectedRotation = new FixedQuaternion(imaginary.X, imaginary.Y, imaginary.Z, Fixed64.One).Normalized;
            source.Agent.Transform.LocalPosition = start + travel;
            source.Agent.Transform.LocalRotation = expectedRotation;
        }
        else
        {
            Fixed64 inverseInertia = axis == 0 ? source.EffectiveInverseInertiaTensor.M11 : source.EffectiveInverseInertiaTensor.M33;
            angularVelocity = unitAxis * (Fixed64.Quarter * inverseInertia);
            source.AddLinearImpulse(travel * source.Mass);
            source.AddAngularImpulse(unitAxis * Fixed64.Quarter);
            expectedRotation = new FixedQuaternion(angularVelocity.X * Fixed64.Half,
                angularVelocity.Y * Fixed64.Half, angularVelocity.Z * Fixed64.Half, Fixed64.One).Normalized;
        }

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        source.Position3d.Should().Be(start + travel);
        source.Rotation.Should().Be(expectedRotation);
        source.AngularVelocity.Should().Be(angularVelocity);
        source.SampleContinuousCollisionLinearVelocity(Fixed64.Half).Should().Be(travel);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContinuousYawingCuboid_BelowMesh_ShouldRetainSeparatingMotion(bool kinematic)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        AddInitialConvexMeshTarget(scenario, MeshColliderMode.Concave, false, false);
        Vector3d start = Vector3d.Down * Fixed64.Half;
        SolidBody source = scenario.CreateBody(new LSCuboidCollider(), start,
            FixedQuaternion.Identity, isKinematic: kinematic).Body;
        source.Collider.Material = PhysicsMaterial.Frictionless;
        source.UseManualGrounding();
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Vector3d angularVelocity = Vector3d.Zero;
        FixedQuaternion expectedRotation;
        if (kinematic)
        {
            expectedRotation = PhysicsScenarioBuilder.Yaw(45);
            source.Agent.Transform.LocalPosition = start + Vector3d.Down;
            source.Agent.Transform.LocalRotation = expectedRotation;
        }
        else
        {
            angularVelocity = Vector3d.Up * (Fixed64.Quarter * source.EffectiveInverseInertiaTensor.M22);
            source.AddLinearImpulse(Vector3d.Down * source.Mass);
            source.AddAngularImpulse(Vector3d.Up * Fixed64.Quarter);
            expectedRotation = new FixedQuaternion(Fixed64.Zero, angularVelocity.Y * Fixed64.Half,
                Fixed64.Zero, Fixed64.One).Normalized;
        }

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        source.Position3d.Should().Be(Vector3d.Down * Fixed64.FromFraction(3, 2));
        source.Rotation.Should().Be(expectedRotation);
        source.AngularVelocity.Should().Be(angularVelocity);
        source.SampleContinuousCollisionLinearVelocity(Fixed64.Half).Should().Be(Vector3d.Down);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
        source.Collider.BoundsMax.Y.Should().Be(-Fixed64.One);
    }

    [Fact]
    public void ContinuousTiltingCuboid_OnMeshFloor_ShouldRetainAngularCrossingDetection()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        AddInitialConvexMeshTarget(scenario, MeshColliderMode.Convex, false, false);
        SolidBody source = scenario.CreateCuboid(Vector3d.Up * Fixed64.Half).Body;
        source.Collider.Material = PhysicsMaterial.Frictionless;
        source.UseManualGrounding();
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Fixed64 angularVelocity = Fixed64.Quarter * source.EffectiveInverseInertiaTensor.M11;
        FixedQuaternion proposedRotation = new FixedQuaternion(angularVelocity * Fixed64.Half,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One).Normalized;
        source.AddLinearImpulse(Vector3d.Right * source.Mass);
        source.AddAngularImpulse(Vector3d.Right * Fixed64.Quarter);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        source.Position3d.X.Should().BeLessThan(Fixed64.One);
        source.Position3d.Y.Should().Be(Fixed64.Half);
        source.Rotation.Should().NotBe(proposedRotation);
        source.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContinuousYawingCuboid_WithFloorAndWall_ShouldRetainSiblingBlockingGeometry(bool compoundTarget)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        LSMeshCollider floor = MeshTestFixtures.CreateConvexQuadFloor();
        LSCollider target;
        if (compoundTarget)
            target = new LSCompoundCollider(
                new CompoundColliderPart(MeshTestFixtures.CreateDefinition(floor), Vector3d.Zero),
                CompoundColliderPart.Cuboid(new Vector3d(Fixed64.Quarter, (Fixed64)4, (Fixed64)4),
                    Vector3d.Right * Fixed64.FromFraction(3, 2)));
        else
        {
            Vector3d[] vertices =
            {
                new(-2, 0, -2), new(2, 0, -2), new(-2, 0, 2), new(2, 0, 2),
                new(1, -2, -2), new(1, 2, -2), new(1, -2, 2), new(1, 2, 2)
            };
            target = new LSMeshCollider(vertices, new[] { 0, 2, 1, 1, 2, 3, 4, 5, 6, 5, 7, 6 },
                MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        }
        target.Material = PhysicsMaterial.Frictionless;
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        SolidBody source = scenario.CreateCuboid(Vector3d.Up * Fixed64.Half).Body;
        source.Collider.Material = PhysicsMaterial.Frictionless;
        source.UseManualGrounding();
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.AddLinearImpulse(Vector3d.Right * source.Mass);
        source.AddAngularImpulse(Vector3d.Up * Fixed64.Quarter);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        source.Position3d.X.Should().BeLessThan(Fixed64.One);
        source.Position3d.Y.Should().Be(Fixed64.Half);
        source.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void ContinuousYawingCuboid_OverlappingCurvedTarget_ShouldRetainConservativeDetection(int targetKind)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        LSCollider target = targetKind switch
        {
            0 => new LSSphereCollider { Radius = Fixed64.One },
            1 => new LSCompoundCollider(
                CompoundColliderPart.Sphere(Fixed64.One, Vector3d.Zero),
                new CompoundColliderPart(MeshTestFixtures.CreateConvexCubeDefinition(), Vector3d.Down * (Fixed64)2)),
            _ => new LSCompoundCollider(
                new CompoundColliderPart(MeshTestFixtures.CreateConvexCubeDefinition(), Vector3d.Down * (Fixed64)2),
                CompoundColliderPart.Sphere(Fixed64.One, Vector3d.Zero))
        };
        target.Material = PhysicsMaterial.Frictionless;
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        SolidBody source = scenario.CreateCuboid(Vector3d.Up).Body;
        source.Collider.Material = PhysicsMaterial.Frictionless;
        source.UseManualGrounding();
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        // The broad-phase sphere normal is Up, but the actual sphere rises
        // above the cuboid's bottom. A full-volume certificate cannot ignore it.
        source.Collider.BoundsMin.Y.Should().Be(Fixed64.Half);
        target.BoundsMax.Y.Should().Be(Fixed64.One);
        source.AddLinearImpulse(Vector3d.Right * source.Mass);
        source.AddAngularImpulse(Vector3d.Up * Fixed64.Quarter);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        source.Position3d.X.Should().BeLessThan(Fixed64.One);
        source.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void ContinuousCenteredSphere_WithNearbyRotation_ShouldRetainLaterStaticImpact()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        LSCuboidCollider wall = scenario.CreateCuboid(Vector3d.Right * (Fixed64)3, immovable: true).Collider;
        SolidBody blade = scenario.CreateBody(new LSCuboidCollider { Size = new Vector3d(8, 1, 1) },
            Vector3d.Up * (Fixed64)4, FixedQuaternion.Identity, isKinematic: true).Body;
        blade.UseManualGrounding();
        SolidBody source = scenario.CreateSphere(Vector3d.Zero, isKinematic: true).Body;
        source.UseManualGrounding();
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Vector3d travel = Vector3d.Right * (Fixed64)4;
        var hits = new SwiftList<Physics3DHit>();
        scenario.Context.Query3D.SweepSphereAll(Vector3d.Zero, travel, Fixed64.Half,
            PhysicsLayerMask.FromLayer(0), hits, source.Collider).Should().Be(1);
        hits[0].Collider.Should().BeSameAs(wall);
        hits[0].Distance.Should().Be((Fixed64)2);
        hits[0].Normal.Should().Be(Vector3d.Left);
        blade.Collider.BoundsMin.Y.Should().BeGreaterThan(Fixed64.Half);

        blade.Agent.Transform.LocalRotation = PhysicsScenarioBuilder.Yaw(45);
        source.Agent.Transform.LocalPosition = travel;
        source.Agent.Transform.LocalRotation = PhysicsScenarioBuilder.Yaw(30);
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        source.Position3d.X.Should().BeGreaterThan(Fixed64.One);
        source.Position3d.X.Should().BeLessThanOrEqualTo((Fixed64)2);
        source.Position3d.Y.Should().Be(Fixed64.Zero);
        source.Position3d.Z.Should().Be(Fixed64.Zero);
        source.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void ContinuousCenteredSphere_AtScalarFace_ShouldRetainUnmaterializableInitialNormal()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.World.TryAddGrid(new GridConfiguration(
            new Vector3d(Fixed64.MaxValue - (Fixed64)12, (Fixed64)(-4), (Fixed64)(-4)),
            new Vector3d(Fixed64.MaxValue, (Fixed64)8, (Fixed64)4)), out _).Should().BeTrue();
        Vector3d sourceStart = new(Fixed64.MaxValue - Fixed64.Quarter, Fixed64.Zero, Fixed64.Zero);
        var target = new LSSphereCollider { Radius = Fixed64.One };
        scenario.InitializeStaticCollider(target,
            new Vector3d(Fixed64.MaxValue - Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        SolidBody blade = scenario.CreateBody(new LSCuboidCollider
            { Size = new Vector3d((Fixed64)8, Fixed64.Quarter, Fixed64.Quarter) },
            new Vector3d(Fixed64.MaxValue - Fixed64.FromFraction(9, 2), (Fixed64)4, Fixed64.Zero),
            FixedQuaternion.Identity, isKinematic: true).Body;
        blade.UseManualGrounding();
        SolidBody source = scenario.CreateSphere(sourceStart, isKinematic: true).Body;
        source.UseManualGrounding();
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Vector3d travel = Vector3d.Up * (Fixed64)2;
        var hits = new SwiftList<Physics3DHit>();
        scenario.Context.Query3D.SweepSphereAll(sourceStart, sourceStart + travel, Fixed64.Half,
            PhysicsLayerMask.FromLayer(0), hits, source.Collider).Should().Be(1);
        hits[0].Collider.Should().BeSameAs(target);
        hits[0].Distance.Should().Be(Fixed64.Zero);
        hits[0].Normal.Should().Be(Vector3d.Right);
        // The selected target feature is at MaxValue + Half. Its rigid-frame
        // anchor and geometric normal remain valid even though Point cannot be read.
        hits[0].TryGetPoint(out _).Should().BeFalse();
        blade.Collider.BoundsMin.Y.Should().BeGreaterThan((Fixed64)3);

        blade.Agent.Transform.LocalRotation = PhysicsScenarioBuilder.Yaw(45);
        source.Agent.Transform.LocalPosition = sourceStart + travel;
        source.Agent.Transform.LocalRotation = PhysicsScenarioBuilder.Yaw(30);
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        source.Position3d.Should().Be(sourceStart + travel);
        source.SampleContinuousCollisionLinearVelocity(Fixed64.Half).Should().Be(travel);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }

    [Theory]
    [InlineData(MeshColliderMode.Convex, false)]
    [InlineData(MeshColliderMode.Convex, true)]
    [InlineData(MeshColliderMode.Concave, false)]
    [InlineData(MeshColliderMode.Concave, true)]
    public void ContinuousYawingMeshSource_OnPlane_ShouldRetainCompleteTangentialPose(
        MeshColliderMode sourceMode, bool bodyBackedFloor)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        LSMeshCollider floor = MeshTestFixtures.CreateConvexQuadFloor();
        floor.Material = PhysicsMaterial.Frictionless;
        if (bodyBackedFloor)
            scenario.CreateBody(floor, Vector3d.Zero, FixedQuaternion.Identity, immovable: true);
        else
            scenario.InitializeStaticCollider(floor, Vector3d.Zero);
        Vector3d start = Vector3d.Up * Fixed64.Half;
        SolidBody source = scenario.CreateBody(MeshTestFixtures.CreateConvexCube(sourceMode),
            start, FixedQuaternion.Identity).Body;
        source.Collider.Material = PhysicsMaterial.Frictionless;
        source.UseManualGrounding();
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Vector3d angularVelocity = Vector3d.Up * (Fixed64.Quarter * source.EffectiveInverseInertiaTensor.M22);
        FixedQuaternion expectedRotation = new FixedQuaternion(Fixed64.Zero,
            angularVelocity.Y * Fixed64.Half, Fixed64.Zero, Fixed64.One).Normalized;
        source.AddLinearImpulse(Vector3d.Right * source.Mass);
        source.AddAngularImpulse(Vector3d.Up * Fixed64.Quarter);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        source.Position3d.Should().Be(start + Vector3d.Right);
        source.Rotation.Should().Be(expectedRotation);
        source.LinearVelocity.Should().Be(Vector3d.Right);
        source.AngularVelocity.Should().Be(angularVelocity);
        source.Collider.BoundsMin.Y.Should().Be(Fixed64.Zero);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContinuousYawingCuboid_OnCompoundPlatforms_ShouldUseHighestFullTargetSupport(bool higherLater)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        ColliderShapeDefinition platform = MeshTestFixtures.CreateDefinition(MeshTestFixtures.CreateConvexQuadFloor());
        var target = new LSCompoundCollider(
            new CompoundColliderPart(platform, higherLater ? Vector3d.Down : Vector3d.Zero),
            new CompoundColliderPart(platform, higherLater ? Vector3d.Zero : Vector3d.Down));
        target.Material = PhysicsMaterial.Frictionless;
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        Vector3d start = Vector3d.Up * Fixed64.Half;
        SolidBody source = scenario.CreateCuboid(start).Body;
        source.Collider.Material = PhysicsMaterial.Frictionless;
        source.UseManualGrounding();
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Vector3d angularVelocity = Vector3d.Up * (Fixed64.Quarter * source.EffectiveInverseInertiaTensor.M22);
        FixedQuaternion expectedRotation = new FixedQuaternion(Fixed64.Zero,
            angularVelocity.Y * Fixed64.Half, Fixed64.Zero, Fixed64.One).Normalized;
        source.AddLinearImpulse(Vector3d.Right * source.Mass);
        source.AddAngularImpulse(Vector3d.Up * Fixed64.Quarter);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        source.Position3d.Should().Be(start + Vector3d.Right);
        source.Rotation.Should().Be(expectedRotation);
        source.LinearVelocity.Should().Be(Vector3d.Right);
        source.AngularVelocity.Should().Be(angularVelocity);
        source.Collider.BoundsMin.Y.Should().Be(target.BoundsMax.Y);
        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }

    [Fact]
    public void RotationalPlaneCertificate_WithoutGeometricNormal_ShouldRetainConservativeSearch()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        LSMeshCollider floor = MeshTestFixtures.CreateConvexQuadFloor();
        scenario.InitializeStaticCollider(floor, Vector3d.Zero);
        Vector3d start = Vector3d.Up * Fixed64.Half;
        SolidBody source = scenario.CreateCuboid(start).Body;
        Vector3d angularVelocity = Vector3d.Up * (Fixed64.Quarter * source.EffectiveInverseInertiaTensor.M22);
        source.AddAngularImpulse(Vector3d.Up * Fixed64.Quarter);
        // Registry-wide candidate gathering can retain identity without a
        // selected feature. No separating certificate may invent its normal.
        var hit = new Physics3DHit(floor, Vector3d.Zero, Vector3d.Zero, Fixed64.Zero, Vector3d.Right);

        source.CanExcludeInvariantTangentialContact(hit, start, Vector3d.Right,
            FixedQuaternion.Identity, PhysicsScenarioBuilder.Yaw(45), isKinematic: false).Should().BeFalse();

        source.Position3d.Should().Be(start);
        source.Rotation.Should().Be(FixedQuaternion.Identity);
        source.AngularVelocity.Should().Be(angularVelocity);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RotationalPlaneCertificate_WithMovingTarget_ShouldRetainConservativeSearch(bool kinematicTarget)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        LSMeshCollider floor = MeshTestFixtures.CreateConvexQuadFloor();
        SolidBody target = scenario.CreateBody(floor, Vector3d.Zero,
            FixedQuaternion.Identity, isKinematic: kinematicTarget).Body;
        Vector3d start = Vector3d.Up * Fixed64.Half;
        SolidBody source = scenario.CreateCuboid(start).Body;
        var worker = new ConvexSweepQueryWorker();
        worker.PreparePrimitiveSource(source.Collider, Vector3d.Right);
        worker.TrySweepPreparedSource(floor, out Physics3DHit hit).Should().BeTrue();
        hit.Normal.Should().Be(Vector3d.Up);
        if (kinematicTarget)
            target.Agent.Transform.LocalPosition = Vector3d.Up;
        else
            target.AddLinearImpulse(Vector3d.Up * target.Mass);

        // The source preserves its height, but a rising target can still cross
        // that supporting plane. The stationary-target certificate cannot apply.
        source.CanExcludeInvariantTangentialContact(hit, start, Vector3d.Right,
            FixedQuaternion.Identity, PhysicsScenarioBuilder.Yaw(45), isKinematic: false).Should().BeFalse();

        source.Position3d.Should().Be(start);
        source.Rotation.Should().Be(FixedQuaternion.Identity);
    }

    [Fact]
    public void RotationalPlaneCertificate_WithCompoundSource_ShouldRetainConservativeSearch()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        LSMeshCollider floor = MeshTestFixtures.CreateConvexQuadFloor();
        scenario.InitializeStaticCollider(floor, Vector3d.Zero);
        var collider = new LSCompoundCollider(
            CompoundColliderPart.Cuboid(Vector3d.One, Vector3d.Left),
            CompoundColliderPart.Cuboid(Vector3d.One, Vector3d.Right - Vector3d.Up * Fixed64.Half));
        Vector3d start = Vector3d.Up * Fixed64.Half;
        SolidBody source = scenario.CreateBody(collider, start, FixedQuaternion.Identity).Body;
        var worker = new ConvexSweepQueryWorker();
        worker.PrepareCompoundSource(collider, Vector3d.Right);
        worker.TrySweepPreparedSource(floor, out Physics3DHit hit).Should().BeTrue();
        hit.Normal.Should().Be(Vector3d.Up);
        collider.BoundsMin.Y.Should().BeLessThan(Fixed64.Zero);

        // The first leaf touches the floor, but its normal cannot certify the
        // second leaf's overlapping volume. Compound sources retain interval search.
        source.CanExcludeInvariantTangentialContact(hit, start, Vector3d.Right,
            FixedQuaternion.Identity, PhysicsScenarioBuilder.Yaw(45), isKinematic: false).Should().BeFalse();

        source.Position3d.Should().Be(start);
        source.Rotation.Should().Be(FixedQuaternion.Identity);
    }

    private static void AddInitialConvexMeshTarget(PhysicsScenarioBuilder scenario, MeshColliderMode mode,
        bool reverseWinding, bool compoundTarget)
    {
        LSMeshCollider mesh = MeshTestFixtures.CreateConvexQuadFloor(mode);
        FixedQuaternion rotation = reverseWinding
            ? new FixedQuaternion(Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero)
            : FixedQuaternion.Identity;
        LSCollider target = compoundTarget
            ? new LSCompoundCollider(new CompoundColliderPart(MeshTestFixtures.CreateDefinition(mesh), Vector3d.Zero, rotation))
            : mesh;
        target.Material = PhysicsMaterial.Frictionless;
        target.InitializeWithNoBody(new TestMatterAgent(scenario.Context,
            new FixedTransform(Vector3d.Zero, compoundTarget ? FixedQuaternion.Identity : rotation, Vector3d.One)));
    }
}
