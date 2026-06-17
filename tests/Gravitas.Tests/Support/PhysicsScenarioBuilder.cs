using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Support;
using GridForge.Configuration;
using SwiftCollections;
using SwiftCollections.Diagnostics;
using System;
using System.Reflection;

namespace Gravitas.Tests.Support;

internal sealed class PhysicsScenarioBuilder : IDisposable
{
    private const int DefaultGridExtent = 16;

    private static readonly FieldInfo IsTriggerField =
        typeof(LSCollider).GetField("_isTrigger", BindingFlags.Instance | BindingFlags.NonPublic)
        ?? throw new InvalidOperationException("Unable to locate LSCollider trigger state.");

    private bool _disposed;

    private PhysicsScenarioBuilder()
    {
        Context = GravitasWorldContext.CreateOwned();
        EnsureGrid();
    }

    public GravitasWorldContext Context { get; }

    public static PhysicsScenarioBuilder Create() => new();

    public void EnsureGrid(int extent = DefaultGridExtent)
    {
        if (Context.World.ActiveGrids.Count > 0)
            return;

        var configuration = new GridConfiguration(
            new Vector3d(-extent, -extent, -extent),
            new Vector3d(extent, extent, extent));

        Context.World.TryAddGrid(configuration, out _).Should().BeTrue();
    }

    public ScenarioBody<TCollider> CreateBody<TCollider>(
        TCollider collider,
        Vector3d position,
        FixedQuaternion rotation,
        Fixed64? mass = null,
        bool immovable = false,
        bool preventAngularForces = false,
        bool isDynamic = true,
        bool isKinematic = false)
        where TCollider : LSCollider
    {
        var transform = new FixedTransform(position, rotation, Vector3d.One);
        var agent = new TestMatterAgent(Context, transform);
        var body = new StiffBody(agent, collider)
        {
            Mass = mass ?? Fixed64.One,
            Immovable = immovable,
            IsKinematic = isKinematic,
            PreventAngularForces = preventAngularForces
        };

        body.Initialize(position, rotation, isDynamic);
        return new ScenarioBody<TCollider>(body, collider);
    }

    public ScenarioBody<LSSphereCollider> CreateSphere(
        Vector3d position,
        FixedQuaternion? rotation = null,
        Fixed64? mass = null,
        bool immovable = false,
        bool preventAngularForces = false,
        bool isKinematic = false)
    {
        return CreateBody(
            new LSSphereCollider(),
            position,
            rotation ?? FixedQuaternion.Identity,
            mass,
            immovable,
            preventAngularForces,
            isKinematic: isKinematic);
    }

    public ScenarioBody<LSCapsuleCollider> CreateCapsule(
        Vector3d position,
        FixedQuaternion? rotation = null,
        Fixed64? mass = null,
        bool immovable = false,
        bool preventAngularForces = false)
    {
        return CreateBody(
            new LSCapsuleCollider(),
            position,
            rotation ?? FixedQuaternion.Identity,
            mass,
            immovable,
            preventAngularForces);
    }

    public ScenarioBody<LSCuboidCollider> CreateCuboid(
        Vector3d position,
        FixedQuaternion? rotation = null,
        Fixed64? mass = null,
        bool immovable = false,
        bool preventAngularForces = false)
    {
        return CreateBody(
            new LSCuboidCollider(),
            position,
            rotation ?? FixedQuaternion.Identity,
            mass,
            immovable,
            preventAngularForces);
    }

    public ScenarioBody<LSCylinderCollider> CreateCylinder(
        Vector3d position,
        FixedQuaternion? rotation = null,
        Fixed64? mass = null,
        bool immovable = false,
        bool preventAngularForces = false)
    {
        return CreateBody(
            new LSCylinderCollider(),
            position,
            rotation ?? FixedQuaternion.Identity,
            mass,
            immovable,
            preventAngularForces);
    }

    public LSSphereCollider CreateStaticSphere(Vector3d position)
    {
        var collider = new LSSphereCollider();
        InitializeStaticCollider(collider, position);
        return collider;
    }

    public void InitializeStaticCollider(LSCollider collider, Vector3d position)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        var transform = new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(Context, transform);
        collider.InitializeWithNoBody(agent);
    }

    public CollisionPair CreatePair(LSCollider colliderA, LSCollider colliderB)
    {
        return new CollisionPair(colliderA, colliderB);
    }

    public static void SetTrigger(LSCollider collider, bool isTrigger = true)
    {
        IsTriggerField.SetValue(collider, isTrigger);
    }

    public static FixedQuaternion Yaw(int degrees)
    {
        return FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            (Fixed64)degrees,
            Fixed64.Zero);
    }

    public static Vector3d Vector(int x, int y, int z) =>
        new((Fixed64)x, (Fixed64)y, (Fixed64)z);

    public static Vector3d Vector(Fixed64 x, Fixed64 y, Fixed64 z) =>
        new(x, y, z);

    public void Dispose()
    {
        if (_disposed)
            return;

        Context.Dispose();
        _disposed = true;
    }
}

internal readonly struct ScenarioBody<TCollider>
    where TCollider : LSCollider
{
    public ScenarioBody(StiffBody body, TCollider collider)
    {
        Body = body;
        Collider = collider;
    }

    public StiffBody Body { get; }

    public TCollider Collider { get; }
}
