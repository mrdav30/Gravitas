using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class CompoundColliderTests
{
    public static TheoryData<string, FixedQuaternion> RotationAdmissionCases => new()
    {
        { "zero", FixedQuaternion.Zero },
        { "scaled", new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, (Fixed64)2) },
        { "saturated", new FixedQuaternion(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.MaxValue) },
        { "near-unit", new FixedQuaternion(Fixed64.Epsilon, Fixed64.Zero, Fixed64.Zero, Fixed64.One) }
    };

    [Theory]
    [MemberData(nameof(RotationAdmissionCases))]
    public void CompoundPartRotationAdmission_ShouldPublishOneNormalizedOrientation(
        string _,
        FixedQuaternion admittedRotation)
    {
        FixedQuaternion expected = admittedRotation.Normalized;
        CompoundColliderPart part = CompoundColliderPart.Cone(
            Fixed64.Half,
            (Fixed64)2,
            Vector3d.Zero,
            admittedRotation,
            Vector3d.One);
        var compound = new LSCompoundCollider(part);

        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCompoundCollider> body = scenario.CreateBody(
            compound,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            immovable: true);
        var runtimePart = (LSConeCollider)body.Collider.GetPartCollider(0);

        body.Collider.Parts[0].LocalRotation.Should().Be(expected);
        runtimePart.CompoundLocalRotation.Should().Be(expected);
        runtimePart.Rotation.Should().Be(expected);
        runtimePart.Axis.IsNormalized().Should().BeTrue();
    }

    [Fact]
    public void Initialize_WithUnderflowedPartWorldScale_ShouldRejectBeforeBinding()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var compound = new LSCompoundCollider(CompoundColliderPart.Sphere(
            Fixed64.Half,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.MinIncrement, Fixed64.One, Fixed64.One)));
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.One));

        Action initialize = () => compound.InitializeWithNoBody(new TestMatterAgent(context, transform));

        initialize.Should().Throw<ArgumentException>().WithParameterName("scale");
        compound.Id.Should().Be(-1);
        compound.HasHostBinding.Should().BeFalse();
        context.Physics.ColliderCount.Should().Be(0);
    }

    [Fact]
    public void BodyInitialize_WithUnderflowedPartWorldScale_ShouldRejectBeforeRuntimeMutation()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var compound = new LSCompoundCollider(CompoundColliderPart.Sphere(
            Fixed64.Half,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.MinIncrement, Fixed64.One, Fixed64.One)));
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.One));
        var body = new SolidBody(new TestMatterAgent(context, transform), compound);

        Action initialize = () => body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);

        initialize.Should().Throw<ArgumentException>().WithParameterName("scale");
        body.Active.Should().BeFalse();
        body.DynamicId.Should().Be(-1);
        compound.Id.Should().Be(-1);
        compound.Body.Should().BeNull();
        compound.HasHostBinding.Should().BeFalse();
        context.Physics.BodyCount.Should().Be(0);
        context.Physics.ColliderCount.Should().Be(0);
    }

    [Fact]
    public void BodyInitialize_WithCylinderPartWhoseScaledHalfHeightCollapses_ShouldRejectBeforeRuntimeMutation()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var compound = new LSCompoundCollider(CompoundColliderPart.Cylinder(
            Fixed64.Half,
            Fixed64.FromRaw(1),
            Vector3d.Zero));
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        var body = new SolidBody(new TestMatterAgent(context, transform), compound);

        Action initialize = () => body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);

        initialize.Should().Throw<ArgumentException>()
            .WithParameterName("Size")
            .WithMessage("*positive half-height*");
        body.Active.Should().BeFalse();
        body.DynamicId.Should().Be(-1);
        compound.Id.Should().Be(-1);
        compound.HasHostBinding.Should().BeFalse();
        context.Physics.BodyCount.Should().Be(0);
        context.Physics.ColliderCount.Should().Be(0);
    }

    [Fact]
    public void Initialize_ShouldRegisterOnlyOwningColliderAndAggregatePartBounds()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Zero)),
            CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero)));

        ScenarioBody<LSCompoundCollider> body = scenario.CreateBody(
            compound,
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity);

        body.Collider.Shape.Should().Be(ColliderType.Compound);
        body.Collider.PartCount.Should().Be(2);
        body.Collider.GetPartId(0).Should().Be(0);
        body.Collider.GetPartId(1).Should().Be(1);
        scenario.Context.Physics.ColliderCount.Should().Be(1);
        scenario.Context.Physics.TryGetColliderById(body.Collider.GetPartCollider(0).Id, out _).Should().BeFalse();
        scenario.Context.Physics.TryGetColliderById(body.Collider.GetPartCollider(1).Id, out _).Should().BeFalse();

        body.Collider.BoundsMin.Should().Be(new Vector3d(-Fixed64.FromFraction(3, 2), -Fixed64.Half, -Fixed64.Half));
        body.Collider.BoundsMax.Should().Be(new Vector3d(Fixed64.FromFraction(5, 2), Fixed64.Half, Fixed64.Half));
        body.Collider.Center.Should().Be(PhysicsScenarioBuilder.Vector(0, 0, 0));
    }

    [Fact]
    public void Constructor_ShouldRejectDefaultParts()
    {
        Action act = () => _ = new LSCompoundCollider(default(CompoundColliderPart));

        act.Should().Throw<ArgumentException>()
            .WithMessage("*default*");
    }

    [Fact]
    public void CompoundColliderPart_ShouldCaptureAuthoredShapeTransformScaleAndMaterial()
    {
        PhysicsMaterial material = PhysicsMaterial.Frictionless;
        Vector3d offset = new(Fixed64.One, (Fixed64)2, (Fixed64)3);
        Vector3d scale = new((Fixed64)2, (Fixed64)3, (Fixed64)4);
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            (Fixed64)10,
            (Fixed64)20,
            (Fixed64)30);
        Vector3d[] vertices =
        {
            new(Fixed64.One, Fixed64.One, Fixed64.One),
            new(-Fixed64.One, -Fixed64.One, Fixed64.One),
            new(-Fixed64.One, Fixed64.One, -Fixed64.One),
            new(Fixed64.One, -Fixed64.One, -Fixed64.One)
        };
        int[] triangles = { 0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3 };

        CompoundColliderPart shapeDefaults = new(ColliderShapeDefinition.Sphere(Fixed64.Half));
        CompoundColliderPart transformDefaults = new(
            ColliderShapeDefinition.Cuboid(Vector3d.One),
            offset,
            rotation);
        CompoundColliderPart sphere = CompoundColliderPart.Sphere(Fixed64.Half, offset, material);
        CompoundColliderPart transformedSphere = CompoundColliderPart.Sphere(Fixed64.Half, offset, rotation, scale);
        CompoundColliderPart materialCapsule = CompoundColliderPart.Capsule(Fixed64.Half, (Fixed64)3, offset, material);
        CompoundColliderPart capsule = CompoundColliderPart.Capsule(Fixed64.Half, (Fixed64)3, offset, rotation, scale);
        CompoundColliderPart cuboid = CompoundColliderPart.Cuboid(new Vector3d((Fixed64)2, (Fixed64)4, (Fixed64)6), offset);
        CompoundColliderPart cylinder = CompoundColliderPart.Cylinder(Fixed64.One, (Fixed64)5, offset, material);
        CompoundColliderPart transformedCylinder = CompoundColliderPart.Cylinder(Fixed64.One, (Fixed64)5, offset, rotation, scale);
        CompoundColliderPart materialCone = CompoundColliderPart.Cone(Fixed64.One, (Fixed64)6, offset, material);
        CompoundColliderPart cone = CompoundColliderPart.Cone(Fixed64.One, (Fixed64)6, offset, rotation, scale);
        CompoundColliderPart materialMesh = CompoundColliderPart.ConvexMesh(
            vertices,
            triangles,
            offset,
            material,
            MeshInertiaPolicy.SurfaceApproximation);
        CompoundColliderPart transformedMesh = CompoundColliderPart.ConvexMesh(
            vertices,
            triangles,
            offset,
            rotation,
            scale,
            MeshInertiaPolicy.SurfaceApproximation);

        shapeDefaults.Shape.Kind.Should().Be(ColliderShapeDefinitionKind.Sphere);
        shapeDefaults.LocalOffset.Should().Be(Vector3d.Zero);
        shapeDefaults.LocalRotation.Should().Be(FixedQuaternion.Identity);
        shapeDefaults.LocalScale.Should().Be(Vector3d.One);

        transformDefaults.Shape.Kind.Should().Be(ColliderShapeDefinitionKind.Cuboid);
        transformDefaults.LocalOffset.Should().Be(offset);
        transformDefaults.LocalRotation.Should().Be(rotation);
        transformDefaults.LocalScale.Should().Be(Vector3d.One);

        sphere.Shape.Kind.Should().Be(ColliderShapeDefinitionKind.Sphere);
        sphere.LocalOffset.Should().Be(offset);
        sphere.LocalScale.Should().Be(Vector3d.One);
        sphere.Material.Should().Be(material);
        transformedSphere.LocalRotation.Should().Be(rotation);
        transformedSphere.LocalScale.Should().Be(scale);

        materialCapsule.Material.Should().Be(material);
        capsule.Shape.Kind.Should().Be(ColliderShapeDefinitionKind.Capsule);
        capsule.LocalRotation.Should().Be(rotation);
        capsule.LocalScale.Should().Be(scale);
        capsule.Material.Should().Be(PhysicsMaterial.Default);

        cuboid.Shape.Kind.Should().Be(ColliderShapeDefinitionKind.Cuboid);
        cylinder.Shape.Kind.Should().Be(ColliderShapeDefinitionKind.Cylinder);
        cylinder.Material.Should().Be(material);
        transformedCylinder.LocalRotation.Should().Be(rotation);
        transformedCylinder.LocalScale.Should().Be(scale);
        materialCone.Material.Should().Be(material);
        cone.Shape.Kind.Should().Be(ColliderShapeDefinitionKind.Cone);
        cone.LocalRotation.Should().Be(rotation);
        cone.LocalScale.Should().Be(scale);
        materialMesh.Shape.Kind.Should().Be(ColliderShapeDefinitionKind.ConvexMesh);
        materialMesh.Shape.MeshInertiaPolicy.Should().Be(MeshInertiaPolicy.SurfaceApproximation);
        materialMesh.Material.Should().Be(material);
        transformedMesh.Shape.Kind.Should().Be(ColliderShapeDefinitionKind.ConvexMesh);
        transformedMesh.Shape.MeshInertiaPolicy.Should().Be(MeshInertiaPolicy.SurfaceApproximation);
        transformedMesh.LocalRotation.Should().Be(rotation);
        transformedMesh.LocalScale.Should().Be(scale);
    }

    [Fact]
    public void CompoundColliderPart_ShouldRejectNonPositiveScaleComponents()
    {
        ColliderShapeDefinition shape = ColliderShapeDefinition.Sphere(Fixed64.One);

        Action zeroX = () => _ = new CompoundColliderPart(
            shape,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.One));
        Action negativeY = () => _ = new CompoundColliderPart(
            shape,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.One, -Fixed64.One, Fixed64.One));

        zeroX.Should().Throw<ArgumentException>().WithParameterName("localScale");
        negativeY.Should().Throw<ArgumentException>().WithParameterName("localScale");
    }

    [Fact]
    public void Constructor_ShouldReservePartsForCompoundLifecycleOnly()
    {
        var compound = new LSCompoundCollider(CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero));
        LSCollider part = compound.GetPartCollider(0);

        Action act = part.Simulate;

        act.Should().Throw<InvalidOperationException>()
            .WithMessage("*standalone lifecycle*");
    }

    [Fact]
    public void PartShapeMutation_ShouldRefreshAggregateBounds()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var compound = new LSCompoundCollider(CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero));
        ScenarioBody<LSCompoundCollider> body = scenario.CreateBody(
            compound,
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity);
        var part = (LSSphereCollider)body.Collider.GetPartCollider(0);

        part.Radius = Fixed64.One;
        body.Collider.Simulate();

        body.Collider.BoundsMin.Should().Be(new Vector3d(-Fixed64.One, -Fixed64.One, -Fixed64.One));
        body.Collider.BoundsMax.Should().Be(new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One));
        body.Collider.RuntimeShapeVersion.Should().BeGreaterThan(1u);
    }
}
