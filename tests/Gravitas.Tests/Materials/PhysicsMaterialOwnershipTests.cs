using Chronicler;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Materials;

public sealed class PhysicsMaterialOwnershipTests
{
    [Fact]
    public void Standalone3DCollider_ShouldStoreAssignedMaterial()
    {
        var collider = new LSSphereCollider();
        PhysicsMaterial material = RoughMaterial();

        collider.Material = material;

        collider.Material.Should().Be(material);
    }

    [Fact]
    public void Standalone2DCollider_ShouldStoreAssignedMaterial()
    {
        var collider = new LSCircleCollider2D(Fixed64.Half);
        PhysicsMaterial material = SlickMaterial();

        collider.Material = material;

        collider.Material.Should().Be(material);
    }

    [Fact]
    public void ShapeDefinition3D_ShouldMaterializeColliderMaterial()
    {
        PhysicsMaterial material = RoughMaterial();
        ColliderShapeDefinition definition = ColliderShapeDefinition.Sphere(Fixed64.Half, material);

        LSCollider collider = definition.CreateCollider();

        collider.Material.Should().Be(material);
        definition.Material.Should().Be(material);
    }

    [Fact]
    public void ShapeDefinition2D_ShouldMaterializeColliderMaterial()
    {
        PhysicsMaterial material = SlickMaterial();
        ColliderShapeDefinition2D definition = ColliderShapeDefinition2D.Circle(Fixed64.Half, material);

        LSCollider2D collider = definition.CreateCollider();

        collider.Material.Should().Be(material);
        definition.Material.Should().Be(material);
    }

    [Fact]
    public void Compound3DParts_ShouldKeepDistinctMaterials()
    {
        PhysicsMaterial rough = RoughMaterial();
        PhysicsMaterial slick = SlickMaterial();
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Zero), rough),
            CompoundColliderPart.Cuboid(Vector3d.One, new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero), slick));

        compound.GetPartCollider(0).Material.Should().Be(rough);
        compound.GetPartCollider(1).Material.Should().Be(slick);
    }

    [Fact]
    public void Compound2DParts_ShouldKeepDistinctMaterials()
    {
        PhysicsMaterial rough = RoughMaterial();
        PhysicsMaterial slick = SlickMaterial();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(-Fixed64.One, Fixed64.Zero), rough),
            CompoundColliderPart2D.AABBox(Vector2d.One, new Vector2d(Fixed64.One, Fixed64.Zero), slick));

        compound.GetPartCollider(0).Material.Should().Be(rough);
        compound.GetPartCollider(1).Material.Should().Be(slick);
    }

    [Fact]
    public void Compound3DPartWithoutMaterial_ShouldUseOwnerMaterial()
    {
        PhysicsMaterial ownerMaterial = RoughMaterial();
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero));

        compound.Material = ownerMaterial;

        compound.GetPartCollider(0).Material.Should().Be(ownerMaterial);
    }

    [Fact]
    public void Compound2DPartWithoutMaterial_ShouldUseOwnerMaterial()
    {
        PhysicsMaterial ownerMaterial = SlickMaterial();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero));

        compound.Material = ownerMaterial;

        compound.GetPartCollider(0).Material.Should().Be(ownerMaterial);
    }

    [Fact]
    public void ReplayHash_ShouldIncludeStandaloneColliderMaterial()
    {
        using PhysicsScenarioBuilder roughScenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> roughBody = roughScenario.CreateSphere(Vector3d.Zero);
        roughBody.Collider.Material = RoughMaterial();

        using PhysicsScenarioBuilder slickScenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> slickBody = slickScenario.CreateSphere(Vector3d.Zero);
        slickBody.Collider.Material = SlickMaterial();

        roughScenario.Context.ComputeReplayHash()
            .Should()
            .NotBe(slickScenario.Context.ComputeReplayHash());
    }

    [Fact]
    public void ReplayHash_ShouldIncludeCompoundPartMaterial()
    {
        ChronicleHash roughHash = ComputeCompoundHash(
            CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero, RoughMaterial()));
        ChronicleHash slickHash = ComputeCompoundHash(
            CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero, SlickMaterial()));

        roughHash.Should().NotBe(slickHash);
    }

    [Fact]
    public void ReplayHash_ShouldIncludeShapeDefinitionMaterial()
    {
        ChronicleHash roughHash = ComputeCompoundHash(
            new CompoundColliderPart(
                ColliderShapeDefinition.Sphere(Fixed64.Half, RoughMaterial()),
                Vector3d.Zero));
        ChronicleHash slickHash = ComputeCompoundHash(
            new CompoundColliderPart(
                ColliderShapeDefinition.Sphere(Fixed64.Half, SlickMaterial()),
                Vector3d.Zero));

        roughHash.Should().NotBe(slickHash);
    }

    private static PhysicsMaterial RoughMaterial() =>
        new((Fixed64)2, Fixed64.One, Fixed64.Zero);

    private static PhysicsMaterial SlickMaterial() =>
        new(Fixed64.FromFraction(1, 4), Fixed64.FromFraction(1, 8), Fixed64.Half);

    private static ChronicleHash ComputeCompoundHash(CompoundColliderPart part)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        _ = scenario.CreateBody(
            new LSCompoundCollider(part),
            Vector3d.Zero,
            FixedQuaternion.Identity);

        return scenario.Context.ComputeReplayHash();
    }
}
