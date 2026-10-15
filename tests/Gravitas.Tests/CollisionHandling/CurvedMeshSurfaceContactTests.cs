using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Serialization;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class CurvedMeshSurfaceContactTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void AngledCrossingCore_WithBothEndsBeyondThePatch_ShouldUseItsFiniteBoundary(bool reverse, bool subdivide)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider mesh = CreateQuad(reverse, subdivide);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        FixedQuaternion rotation = FixedQuaternion.LookRotation(new Vector3d(1, 0, 1).Normalized,
            new Vector3d(1, 1, -1).Normalized);
        var capsule = new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 9, 1) };
        scenario.CreateBody(capsule, Vector3d.Zero, rotation, immovable: true);

        CollisionPair pair = scenario.CreatePair(mesh, capsule);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.Count.Should().BeGreaterThan(0);
        // The complete plane section fits inside the quad. Its cylindrical
        // ray maximum is 2r / sqrt(1 - (axis dot plane-normal)^2), rather
        // than the whole convex quad's distinct minimum-translation depth.
        Fixed64 expected = capsule.ScaledRadius * 2
            / FixedMath.Sqrt(Fixed64.One - capsule.WorldAxis.Y * capsule.WorldAxis.Y);
        (pair.Manifold.PrimaryContact.Depth - expected).Abs().Should().BeLessThanOrEqualTo(Fixed64.FromRaw(64),
            "the primary sample must retain the admitted region's maximum ray");
        for (int i = 0; i < pair.Manifold.Count; i++)
        {
            ManifoldContact contact = pair.Manifold[i];
            contact.Normal.X.Should().Be(Fixed64.Zero);
            contact.Normal.Z.Should().Be(Fixed64.Zero);
            Vector3d localPoint = rotation.Inverse().Rotate(contact.PointB);
            (localPoint.X * localPoint.X + localPoint.Z * localPoint.Z
                - capsule.ScaledRadius * capsule.ScaledRadius).Abs()
                .Should().BeLessThanOrEqualTo(Fixed64.FromRaw(64),
                    "the paired anchor must lie on the actual cylindrical side");
            localPoint.Y.Abs().Should().BeLessThan(capsule.AxisLength / 2);
            contact.Depth.Should().BeLessThanOrEqualTo(expected + Fixed64.FromRaw(64));
            (contact.Depth - (contact.PointB.Y - contact.PointA.Y).Abs()).Abs()
                .Should().BeLessThanOrEqualTo(Fixed64.FromRaw(64),
                    "each finite coverage sample keeps its own matched boundary-ray depth");
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void CrossingCore_ShouldReportTheWholeCapsuleExit(bool reverse, bool subdivide)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider mesh = CreateQuad(reverse, subdivide);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        Vector3d center = new(Fixed64.FromFraction(3, 10), -Fixed64.Quarter, Fixed64.FromFraction(3, 10));
        LSCollider capsule = CreateShape(true);
        scenario.InitializeStaticCollider(capsule, center);
        CollisionPair pair = scenario.CreatePair(mesh, capsule);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.Count.Should().Be(1);
        pair.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.FromFraction(3, 4));
        for (int i = 0; i < pair.Manifold.Count; i++)
        {
            ManifoldContact contact = pair.Manifold[i];
            contact.Normal.Should().Be(Vector3d.Down);
            // The upper core end is at Y=1/4, and its cap reaches Y=3/4.
            contact.Depth.Should().BeLessThanOrEqualTo(Fixed64.FromFraction(3, 4));
            contact.PointA.Y.Should().Be(Fixed64.Zero);
            contact.PointB.Y.Should().Be(contact.Depth);
            Vector3d local = contact.PointB - center;
            Fixed64 axial = FixedMath.Clamp(local.Y, -Fixed64.Half, Fixed64.Half);
            Vector3d radial = local - Vector3d.Up * axial;
            (Vector3d.Dot(radial, radial) - Fixed64.Quarter).Abs()
                .Should().BeLessThanOrEqualTo(Fixed64.FromRaw(8));
            Vector3d.Cross(contact.PointA - contact.PointB, contact.Normal).Should().Be(Vector3d.Zero);
        }
        foreach (int raw in new[] { -1, 0, 1 })
        {
            LSCollider moved = CreateShape(true);
            scenario.InitializeStaticCollider(moved,
                center + Vector3d.Down * (Fixed64.FromFraction(3, 4) + Fixed64.FromRaw(raw)));
            CollisionPair movedPair = scenario.CreatePair(mesh, moved);
            CollisionDetection.DoCollisionCheck(movedPair).Should().Be(raw <= 0);
            if (raw <= 0) movedPair.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.FromRaw(-raw));
        }
    }

    [Theory]
    [InlineData(false, false, false)]
    [InlineData(false, true, false)]
    [InlineData(true, false, false)]
    [InlineData(true, true, false)]
    [InlineData(false, false, true)]
    [InlineData(true, true, true)]
    public void InteriorFootprint_ShouldRetainOnlySurfaceNormals(bool capsule, bool reverse, bool subdivide)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider mesh = CreateQuad(reverse, subdivide);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        LSCollider curved = CreateShape(capsule);
        scenario.InitializeStaticCollider(curved, new(Fixed64.FromFraction(3, 10), -Fixed64.Quarter, Fixed64.FromFraction(3, 10)));
        CollisionPair pair = scenario.CreatePair(mesh, curved);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.Count.Should().Be(1);
        for (int i = 0; i < pair.Manifold.Count; i++)
            pair.Manifold[i].Normal.Should().Be(Vector3d.Down);

        CollisionPair reversed = scenario.CreatePair(curved, mesh);
        CollisionDetection.DoCollisionCheck(reversed).Should().BeTrue();
        reversed.Manifold.Count.Should().Be(pair.Manifold.Count);
        for (int i = 0; i < pair.Manifold.Count; i++)
        {
            reversed.Manifold[i].Normal.Should().Be(pair.Manifold[i].Normal);
            reversed.Manifold[i].ContactId.Should().Be(pair.Manifold[i].ContactId);
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void FrictionlessFullLoop_ShouldPreserveMotionAcrossCoveredDiagonal(bool capsule, bool reverse)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        LSMeshCollider mesh = CreateQuad(reverse, false);
        mesh.Material = PhysicsMaterial.Frictionless;
        scenario.CreateBody(mesh, Vector3d.Zero, FixedQuaternion.Identity, immovable: true);
        LSCollider curved = CreateShape(capsule);
        curved.Material = PhysicsMaterial.Frictionless;
        var body = scenario.CreateBody(curved, new(Fixed64.FromFraction(3, 10), -Fixed64.Quarter, Fixed64.FromFraction(3, 10)),
            FixedQuaternion.Identity, preventAngularForces: true).Body;
        body.UseManualGrounding();
        body.AddLinearImpulse(new(-Fixed64.One, Fixed64.Half, -Fixed64.One));

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        body.LinearVelocity.X.Should().Be(-Fixed64.One);
        body.LinearVelocity.Z.Should().Be(-Fixed64.One);
        body.LinearVelocity.Y.Should().BeLessThanOrEqualTo(Fixed64.Zero);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void UniqueSupportFullLoop_ShouldPreserveFrictionlessSpinAndAvoidArtificialTorque(bool capsule, bool reverse)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        SolidBody body = CreateFrictionlessSupportedBody(scenario, capsule, reverse);
        if (!capsule)
        {
            // For radius 1/2, vX=-1/2 and omegaZ=1 describe rolling. A normal
            // through the sphere's center cannot slow its spin without friction.
            body.AddAngularImpulse(Vector3d.Forward / body.EffectiveInverseInertiaTensor.M33);
            body.AddLinearImpulse(Vector3d.Left * Fixed64.Half);
        }
        Vector3d initialAngular = body.AngularVelocity;
        body.AddLinearImpulse(Vector3d.Down * Fixed64.Half);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        body.AngularVelocity.Should().Be(initialAngular);
        body.LinearVelocity.X.Should().Be(capsule ? Fixed64.Zero : -Fixed64.Half);
        body.LinearVelocity.Y.Should().Be(Fixed64.Zero);
        body.LinearVelocity.Z.Should().Be(Fixed64.Zero);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ZeroCoreContactGeometry_ShouldBeIndependentOfTheSpinningBodyFrame(bool capsule, bool edge)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider mesh = CreateQuad(false, false);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        LSCollider curved = capsule
            ? new LSCapsuleCollider { Radius = Fixed64.Half, Size = Vector3d.One }
            : new LSSphereCollider { Radius = Fixed64.Half };
        Vector3d center = new(edge ? (Fixed64)2 + Fixed64.Quarter : Fixed64.Zero,
            Fixed64.Quarter, Fixed64.Zero);
        SolidBody body = scenario.CreateBody(curved, center, FixedQuaternion.Identity, immovable: true).Body;
        if (curved is LSCapsuleCollider zeroCore) zeroCore.AxisLength.Should().Be(Fixed64.Zero);
        CollisionPair pair = scenario.CreatePair(mesh, curved);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        var expected = new ManifoldContact[pair.Manifold.Count];
        for (int i = 0; i < expected.Length; i++) expected[i] = pair.Manifold[i];

        body.SetRotation(FixedQuaternion.FromAxisAngle(new Vector3d(1, 2, 3).Normalized, Fixed64.PiOver4));
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();

        pair.Manifold.Count.Should().Be(expected.Length);
        for (int i = 0; i < expected.Length; i++)
        {
            ManifoldContact actual = pair.Manifold[i];
            // A zero-core capsule and sphere have no geometric orientation.
            // Both the world witnesses and retained split anchors must remain
            // identical, so the solver sees the same lever and warm-start key.
            actual.PointA.Should().Be(expected[i].PointA);
            actual.PointB.Should().Be(expected[i].PointB);
            actual.AnchorA.Should().Be(expected[i].AnchorA);
            actual.AnchorB.Should().Be(expected[i].AnchorB);
            actual.Normal.Should().Be(expected[i].Normal);
            actual.Depth.Should().Be(expected[i].Depth);
            actual.ContactId.Should().Be(expected[i].ContactId);
        }
    }

    [Fact]
    public void FrictionlessRollingSupport_AfterSaveRestore_ShouldContinueWithTheSameMotion()
    {
        using PhysicsScenarioBuilder uninterrupted = PhysicsScenarioBuilder.Create();
        using PhysicsScenarioBuilder restored = PhysicsScenarioBuilder.Create();
        SolidBody first = CreateFrictionlessSupportedBody(uninterrupted, false, false);
        SolidBody second = CreateFrictionlessSupportedBody(restored, false, false);
        foreach (SolidBody body in new[] { first, second })
        {
            body.AddAngularImpulse(Vector3d.Forward / body.EffectiveInverseInertiaTensor.M33);
            body.AddLinearImpulse(new(-Fixed64.Half, -Fixed64.Half, Fixed64.Zero));
        }
        uninterrupted.Context.Simulate();
        uninterrupted.Context.LateSimulate();
        restored.Context.Simulate();
        restored.Context.LateSimulate();
        object payload = GravitasSerializationHarness.Serialize(first, GravitasSerializationTransport.Json);
        second.ResetPosition(new Vector3d(5, 5, 5), FixedQuaternion.Identity);
        GravitasSerializationHarness.Populate(second, payload, GravitasSerializationTransport.Json);

        for (int frame = 0; frame < 3; frame++)
        {
            // Force a renewed normal response, rather than comparing a free
            // trajectory that no longer consumes its rebuilt contact cache.
            first.AddLinearImpulse(Vector3d.Down * Fixed64.Quarter);
            second.AddLinearImpulse(Vector3d.Down * Fixed64.Quarter);
            uninterrupted.Context.Simulate();
            uninterrupted.Context.LateSimulate();
            restored.Context.Simulate();
            restored.Context.LateSimulate();
            second.Position3d.Should().Be(first.Position3d);
            second.Rotation.Should().Be(first.Rotation);
            second.LinearVelocity.Should().Be(first.LinearVelocity);
            second.AngularVelocity.Should().Be(first.AngularVelocity);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void UniqueSupportUnderGravity_ShouldSettleLikeTheEquivalentFlatCuboid(bool capsule)
    {
        using PhysicsScenarioBuilder meshScenario = PhysicsScenarioBuilder.Create();
        using PhysicsScenarioBuilder boxScenario = PhysicsScenarioBuilder.Create();
        SolidBody meshBody = CreateFrictionlessSupportedBody(meshScenario, capsule, false);
        SolidBody boxBody = CreateFrictionlessSupportedBody(boxScenario, capsule, false, boxFloor: true);
        Fixed64 restingHeight = capsule ? Fixed64.One : Fixed64.Half;
        meshBody.ResetPosition(Vector3d.Up * restingHeight, FixedQuaternion.Identity);
        boxBody.ResetPosition(Vector3d.Up * restingHeight, FixedQuaternion.Identity);
        meshScenario.Context.Environment.Gravity = Fixed64.One;
        boxScenario.Context.Environment.Gravity = Fixed64.One;
        Vector3d settledPosition = default;
        for (int frame = 0; frame < 96; frame++)
        {
            meshScenario.Context.Simulate();
            meshScenario.Context.LateSimulate();
            boxScenario.Context.Simulate();
            boxScenario.Context.LateSimulate();
            meshBody.Position3d.Should().Be(boxBody.Position3d);
            meshBody.LinearVelocity.Should().Be(boxBody.LinearVelocity);
            meshBody.AngularVelocity.Should().Be(Vector3d.Zero);
            meshBody.Rotation.Should().Be(FixedQuaternion.Identity);
            if (frame == 64) settledPosition = meshBody.Position3d;
            if (frame > 64) meshBody.Position3d.Should().Be(settledPosition);
        }
        meshBody.LinearVelocity.Should().Be(Vector3d.Zero);
        meshBody.Position3d.Y.Should().BeGreaterThan(Fixed64.Zero);
        meshBody.Position3d.Y.Should().BeLessThanOrEqualTo(restingHeight);
    }

    [Fact]
    public void FiniteFrictionSlidingSphere_ShouldReduceSlipAndProduceRollingTorque()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        var material = new PhysicsMaterial(Fixed64.Half, Fixed64.Half, Fixed64.Zero);
        LSMeshCollider mesh = CreateQuad(false, false);
        mesh.Material = material;
        scenario.CreateBody(mesh, Vector3d.Zero, FixedQuaternion.Identity, immovable: true);
        var sphere = new LSSphereCollider { Radius = Fixed64.Half, Material = material };
        SolidBody body = scenario.CreateBody(sphere, Vector3d.Up * Fixed64.Quarter, FixedQuaternion.Identity).Body;
        body.UseManualGrounding();
        body.SleepEnabled = false;
        body.AddLinearImpulse(Vector3d.Right + Vector3d.Down);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        body.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        body.LinearVelocity.X.Should().BeLessThan(Fixed64.One);
        body.AngularVelocity.Z.Should().BeLessThan(Fixed64.Zero,
            "leftward contact friction at the bottom of a rightward-sliding sphere produces clockwise rolling torque");
        body.AngularVelocity.X.Should().Be(Fixed64.Zero);
        body.AngularVelocity.Y.Should().Be(Fixed64.Zero);
        body.LinearVelocity.Y.Should().Be(Fixed64.Zero);
        body.LinearVelocity.Z.Should().Be(Fixed64.Zero);
        Fixed64 contactSlip = body.LinearVelocity.X + body.AngularVelocity.Z * sphere.ScaledRadius;
        contactSlip.Abs().Should().BeLessThan(Fixed64.One);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ExposedEdge_ShouldRetainRadialConstraint(bool capsule)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider mesh = CreateQuad(false, false);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        LSCollider curved = CreateShape(capsule);
        scenario.InitializeStaticCollider(curved, new((Fixed64)2 + Fixed64.Quarter, -Fixed64.Quarter, Fixed64.Zero));
        CollisionPair pair = scenario.CreatePair(mesh, curved);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        bool found = false;
        foreach (ManifoldContact contact in pair.Manifold)
            found |= contact.Normal.X > Fixed64.Zero;
        found.Should().BeTrue("the genuine edge keeps its independent radial constraint alongside face coverage");
    }

    [Fact]
    public void WorldChart_AtScalarFace_ShouldRejectCoveredSeam()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var mesh = new LSMeshCollider(new[] { new Vector3d(4, -2, -2), new Vector3d(4, 2, -2),
            new Vector3d(4, -2, 2), new Vector3d(4, 2, 2), new Vector3d(-4, -2, -2), new Vector3d(-4, 2, -2),
            new Vector3d(-4, -2, 2), new Vector3d(-4, 2, 2) }, new[] { 0, 2, 1, 1, 2, 3, 4, 6, 5, 5, 6, 7 }, MeshColliderMode.Concave);
        scenario.InitializeStaticCollider(mesh, new(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero));
        var curved = new LSSphereCollider { Radius = Fixed64.Half };
        scenario.InitializeStaticCollider(curved, new(Fixed64.MinValue + (Fixed64)4 + Fixed64.Quarter, Fixed64.FromFraction(3, 10), Fixed64.FromFraction(3, 10)));
        CollisionPair pair = scenario.CreatePair(mesh, curved);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        for (int i = 0; i < pair.Manifold.Count; i++) pair.Manifold[i].Normal.Should().Be(Vector3d.Right);
    }

    [Fact]
    public void WorldChart_WithUnrepresentableNeighborVertex_ShouldPreserveAdmittedContact()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var mesh = new LSMeshCollider(new[] { new Vector3d(0, 0, -2), new Vector3d(0, 2, -2),
            new Vector3d(0, 2, 2), new Vector3d(0, -2, 2) }, new[] { 0, 1, 2, 0, 2, 3 }, MeshColliderMode.Concave);
        scenario.InitializeStaticCollider(mesh, new(Fixed64.Zero, Fixed64.MinValue, Fixed64.Zero));
        var sphere = new LSSphereCollider { Radius = Fixed64.FromFraction(3, 2) };
        scenario.InitializeStaticCollider(sphere, new(Fixed64.Quarter, Fixed64.MinValue + Fixed64.FromFraction(9, 4), (Fixed64)3));
        CollisionPair pair = scenario.CreatePair(mesh, sphere);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.Count.Should().BeGreaterThan(0);
        bool found = false;
        foreach (ManifoldContact contact in pair.Manifold)
            found |= contact.Normal.Y > Fixed64.Zero;
        found.Should().BeTrue("the genuine exposed feature survives even when the independent face ray is deeper");
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void CompoundPart_ShouldRemapSurfaceNormalAndMaterials(bool capsule, bool reverseArguments)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider mesh = CreateQuad(false, true);
        mesh.Material = PhysicsMaterial.Frictionless;
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        CompoundColliderPart part = capsule
            ? CompoundColliderPart.Capsule(Fixed64.Half, (Fixed64)2, Vector3d.Zero, PhysicsMaterial.Bouncy)
            : CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero, PhysicsMaterial.Bouncy);
        var compound = new LSCompoundCollider(part);
        scenario.InitializeStaticCollider(compound, new(Fixed64.FromFraction(3, 10), -Fixed64.Quarter, Fixed64.FromFraction(3, 10)));
        CollisionPair pair = reverseArguments ? scenario.CreatePair(mesh, compound) : scenario.CreatePair(compound, mesh);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.ColliderA.Should().BeSameAs(compound);
        pair.Manifold.GroupCount.Should().Be(1);
        pair.Manifold.Count.Should().Be(1);
        for (int i = 0; i < pair.Manifold.Count; i++)
        {
            ManifoldContact contact = pair.Manifold[i];
            contact.Normal.Should().Be(Vector3d.Up);
            contact.FeatureNamespaceA.Should().Be(1);
            contact.FeatureNamespaceB.Should().Be(0);
            contact.MaterialA.Should().Be(PhysicsMaterial.Bouncy);
            contact.MaterialB.Should().Be(PhysicsMaterial.Frictionless);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RotatedCrossingCore_ShouldOrientNormalAndBoundaryAnchorTogether(bool reverse)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)180);
        Vector3d origin = new(4, 3, 2);
        LSMeshCollider mesh = CreateQuad(reverse, false);
        scenario.CreateBody(mesh, origin, rotation, immovable: true);
        LSCollider capsule = CreateShape(true);
        Vector3d localCenter = new(Fixed64.FromFraction(3, 10), -Fixed64.Quarter, Fixed64.FromFraction(3, 10));
        scenario.CreateBody(capsule, origin + rotation.Rotate(localCenter), rotation, immovable: true);
        CollisionPair pair = scenario.CreatePair(mesh, capsule);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.Count.Should().Be(1);
        pair.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.FromFraction(3, 4));
        for (int i = 0; i < pair.Manifold.Count; i++)
        {
            ManifoldContact contact = pair.Manifold[i];
            contact.Normal.Should().Be(Vector3d.Up);
            contact.PointA.Y.Should().Be((Fixed64)3);
            contact.PointB.Y.Should().Be((Fixed64)3 - contact.Depth);
            contact.Depth.Should().BeLessThanOrEqualTo(Fixed64.FromFraction(3, 4));
        }
    }

    [Theory]
    [InlineData(false, -1, -1)]
    [InlineData(false, -1, 1)]
    [InlineData(false, 1, -1)]
    [InlineData(false, 1, 1)]
    [InlineData(true, -1, -1)]
    [InlineData(true, -1, 1)]
    [InlineData(true, 1, -1)]
    [InlineData(true, 1, 1)]
    public void ExposedCorner_ShouldRetainBothRadialComponents(bool capsule, int x, int z)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var mesh = CreateQuad(false, false);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        LSCollider curved = CreateShape(capsule);
        scenario.InitializeStaticCollider(curved, new((Fixed64)x * ((Fixed64)2 + Fixed64.Quarter),
            -Fixed64.Quarter, (Fixed64)z * ((Fixed64)2 + Fixed64.Quarter)));
        CollisionPair pair = scenario.CreatePair(mesh, curved);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        bool found = false;
        foreach (ManifoldContact contact in pair.Manifold)
            found |= contact.Normal.X * (Fixed64)x > Fixed64.Zero
                && contact.Normal.Z * (Fixed64)z > Fixed64.Zero;
        found.Should().BeTrue("the genuine corner keeps both radial components in its own feature group");
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void WarmedSurfaceReduction_ShouldAllocateNothing(bool capsule)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var mesh = CreateQuad(false, true);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        LSCollider curved = CreateShape(capsule);
        scenario.InitializeStaticCollider(curved, new(Fixed64.FromFraction(3, 10), -Fixed64.Quarter, Fixed64.FromFraction(3, 10)));
        CollisionPair pair = scenario.CreatePair(mesh, curved);
        for (int i = 0; i < 32; i++) CollisionDetection.DoCollisionCheck(pair);
        long before = System.GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 128; i++) CollisionDetection.DoCollisionCheck(pair);
        (System.GC.GetAllocatedBytesForCurrentThread() - before).Should().Be(0);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void HoleBoundary_ShouldRemainExposed(bool capsule)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Vector3d[] vertices = { new(-2, 0, -2), new(2, 0, -2), new(2, 0, 2), new(-2, 0, 2),
            new(-Fixed64.Half, Fixed64.Zero, -Fixed64.Half), new(Fixed64.Half, Fixed64.Zero, -Fixed64.Half),
            new(Fixed64.Half, Fixed64.Zero, Fixed64.Half), new(-Fixed64.Half, Fixed64.Zero, Fixed64.Half) };
        var mesh = new LSMeshCollider(vertices, new[] { 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5,
            2, 3, 7, 2, 7, 6, 3, 0, 4, 3, 4, 7 }, MeshColliderMode.Concave);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        LSCollider curved = CreateShape(capsule);
        scenario.InitializeStaticCollider(curved, new(Fixed64.Quarter, -Fixed64.Quarter, Fixed64.Zero));
        CollisionPair pair = scenario.CreatePair(mesh, curved);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        bool left = false;
        for (int i = 0; i < pair.Manifold.Count; i++) left |= pair.Manifold[i].Normal.X < Fixed64.Zero;
        left.Should().BeTrue("the inner rim is a real boundary, not a covered seam");
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void IndependentSurfaces_ShouldRetainMoreThanFourConstraints(bool capsule)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Vector3d[] vertices = new Vector3d[18];
        int[] indices = new int[18];
        Vector3d[] normals = { Vector3d.Right, Vector3d.Left, Vector3d.Up, Vector3d.Down, Vector3d.Forward, -Vector3d.Forward };
        for (int face = 0; face < 6; face++)
        {
            Vector3d normal = normals[face];
            Vector3d tangent = face < 2 ? Vector3d.Up : Vector3d.Right;
            Vector3d other = Vector3d.Cross(normal, tangent);
            Vector3d center = normal * Fixed64.Quarter;
            vertices[face * 3] = center - tangent * (Fixed64)2 - other * (Fixed64)2;
            vertices[face * 3 + 1] = center + tangent * (Fixed64)2 - other * (Fixed64)2;
            vertices[face * 3 + 2] = center + other * (Fixed64)2;
            for (int i = 0; i < 3; i++) indices[face * 3 + i] = face * 3 + i;
        }
        var mesh = new LSMeshCollider(vertices, indices, MeshColliderMode.Concave);
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        LSCollider curved = CreateShape(capsule);
        scenario.InitializeStaticCollider(curved, Vector3d.Zero);
        CollisionPair pair = scenario.CreatePair(mesh, curved);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.GroupCount.Should().Be(6);
        foreach (Vector3d normal in normals)
        {
            bool found = false;
            for (int i = 0; i < pair.Manifold.Count; i++) found |= pair.Manifold[i].Normal == normal;
            found.Should().BeTrue("every independent face must survive reduction");
        }
    }

    private static SolidBody CreateFrictionlessSupportedBody(PhysicsScenarioBuilder scenario, bool capsule, bool reverse,
        bool boxFloor = false)
    {
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        LSCollider floor = boxFloor ? new LSCuboidCollider { Size = new(4, 1, 4) } : CreateQuad(reverse, false);
        floor.Material = PhysicsMaterial.Frictionless;
        scenario.CreateBody(floor, boxFloor ? Vector3d.Down * Fixed64.Half : Vector3d.Zero,
            FixedQuaternion.Identity, immovable: true);
        LSCollider curved = CreateShape(capsule);
        curved.Material = PhysicsMaterial.Frictionless;
        Fixed64 height = capsule ? Fixed64.Half + Fixed64.Quarter : Fixed64.Quarter;
        SolidBody body = scenario.CreateBody(curved, Vector3d.Up * height, FixedQuaternion.Identity).Body;
        body.SleepEnabled = false;
        body.UseManualGrounding();
        return body;
    }

    private static LSCollider CreateShape(bool capsule) => capsule
        ? new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) }
        : new LSSphereCollider { Radius = Fixed64.Half };

    private static LSMeshCollider CreateQuad(bool reverse, bool subdivide)
    {
        Vector3d[] vertices = { new(-2, 0, -2), new(2, 0, -2), new(-2, 0, 2), new(2, 0, 2), Vector3d.Zero };
        if (!subdivide) System.Array.Resize(ref vertices, 4);
        int[] triangles = subdivide ? new[] { 0, 2, 4, 2, 3, 4, 3, 1, 4, 1, 0, 4 } : new[] { 0, 2, 1, 1, 2, 3 };
        if (reverse)
            for (int i = 0; i < triangles.Length; i += 3)
                (triangles[i], triangles[i + 2]) = (triangles[i + 2], triangles[i]);
        return new(vertices, triangles, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
    }
}
