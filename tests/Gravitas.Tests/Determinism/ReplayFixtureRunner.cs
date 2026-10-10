using Chronicler;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Constraints;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Support;
using Gravitas.Tests.Serialization;
using Gravitas.Tests.Support;
using System;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests.Determinism;

// Test-only execution of a finite command vocabulary, not a host replay API.
internal sealed class ReplayFixtureRunner : IDisposable
{
    private readonly ReplayFixture _fixture;
    private readonly PhysicsScenarioBuilder _scenario = PhysicsScenarioBuilder.Create();
    private readonly Dictionary<int, SolidBody> _bodies3D = new();
    private readonly Dictionary<int, SolidBody2D> _bodies2D = new();
    private readonly Dictionary<object, int> _hostIds = new(ReferenceEqualityComparer.Instance);
    private readonly List<string> _events = new();
    private readonly List<string> _queries = new();
    private readonly bool _skipRestore;
    private int? _retired3D;
    private int? _retired2D;

    public ReplayFixtureRunner(ReplayFixture fixture, bool skipRestore = false)
    {
        _fixture = fixture;
        _skipRestore = skipRestore;
        // All fixture actors use layer zero. Supply its matrix explicitly:
        // registered display names are ambient host state and must not change
        // this trace's hashed settings when another test registers a layer.
        Context.ApplySettings(new PhysicsSettings(fixture.FrameRate, new[,] { { true } })
        {
            RuntimeMode = fixture.RuntimeMode,
            DiscreteSolverIterations = fixture.SolverIterations
        });
        Context.Environment.Gravity = Fixed64.Zero;
        Context.Environment.AirDensity = Fixed64.Zero;
        Context.Environment.DampingFactor = Fixed64.Zero;
        Context.Environment.MinSpeed = Fixed64.Zero;
        Context.Environment.MaxSpeed = Fixed64.FromRaw(fixture.MaxSpeedRaw);
        Context.Environment.MaxFallSpeed = Context.Environment.MaxSpeed;
        foreach (ReplayActor actor in fixture.Actors)
            if (actor.StartActive)
                Spawn(actor);
    }

    public GravitasWorldContext Context => _scenario.Context;
    public List<Joint3D> Joints { get; } = new();
    public int ReusedSlots { get; private set; }

    public ReplayFrame[] Run()
    {
        var frames = new ReplayFrame[_fixture.FrameCount];
        int nextCommand = 0;
        for (int frame = 0; frame < frames.Length; frame++)
        {
            _events.Clear();
            _queries.Clear();
            // Array order is authoritative even for commands sharing a frame.
            while (nextCommand < _fixture.Commands.Length && _fixture.Commands[nextCommand].Frame == frame)
                Execute(_fixture.Commands[nextCommand++]);
            Context.Simulate();
            Context.LateSimulate();
            frames[frame] = Snapshot();
            if (frame == 0 && _fixture.Name == "three-d-surface-contacts-v1")
                AssertSurfaceConstraints();
        }
        return frames;
    }

    private void Execute(ReplayCommand command)
    {
        ReplayActor actor = Array.Find(_fixture.Actors, value => value.Id == command.Actor)!;
        if (command.Operation == "Spawn")
        {
            Spawn(actor);
            return;
        }
        if (command.Operation == "Query")
        {
            Query(actor);
            return;
        }

        bool is3D = actor.Dimension == "3D";
        Assert.True(is3D ? _bodies3D.ContainsKey(actor.Id) : _bodies2D.ContainsKey(actor.Id),
            $"{_fixture.Name}: {command.Operation} targets inactive actor {actor.Id}.");
        switch (command.Operation)
        {
            case "Impulse":
                if (is3D) _bodies3D[actor.Id].AddLinearImpulse(Vector(command.ValueRaw));
                else _bodies2D[actor.Id].AddLinearImpulse(Planar(command.ValueRaw));
                break;
            case "Sleep":
                if (is3D) _bodies3D[actor.Id].Sleep();
                else _bodies2D[actor.Id].Sleep();
                break;
            case "Wake":
                if (is3D) _bodies3D[actor.Id].Wake();
                else _bodies2D[actor.Id].Wake();
                break;
            case "Despawn":
                if (is3D)
                {
                    SolidBody body = _bodies3D[actor.Id];
                    _retired3D = body.Collider.Id;
                    body.Deactivate();
                    _bodies3D.Remove(actor.Id);
                    Assert.False(body.Active);
                }
                else
                {
                    SolidBody2D body = _bodies2D[actor.Id];
                    _retired2D = body.Collider.Id;
                    body.Deactivate();
                    _bodies2D.Remove(actor.Id);
                    Assert.False(body.Active);
                }
                break;
            case "Restore":
                Restore(actor);
                break;
            case "BallSocket":
                Assert.True(_bodies3D.ContainsKey(command.OtherActor), "BallSocket endpoint must be active.");
                var first = _bodies3D[actor.Id];
                var second = _bodies3D[command.OtherActor];
                Joints.Add(Context.Constraints3D.RegisterJoint(new JointDefinition3D(
                    first, second,
                    new FixedTransform(Vector3d.Right * Fixed64.Half, FixedQuaternion.Identity, Vector3d.One),
                    new FixedTransform(Vector3d.Left * Fixed64.Half, FixedQuaternion.Identity, Vector3d.One),
                    JointType3D.BallSocket, JointLimit3D.Unrestricted, JointMotor3D.Disabled,
                    JointCollisionPolicy.Collide)));
                break;
            default:
                throw new InvalidOperationException($"Unknown fixture operation {command.Operation}.");
        }
    }

    private void Spawn(ReplayActor actor)
    {
        Assert.False(_bodies3D.ContainsKey(actor.Id) || _bodies2D.ContainsKey(actor.Id), "Host actor already active.");
        Vector3d position = Vector(actor.PositionRaw);
        Fixed64 mass = Fixed64.FromRaw(actor.MassRaw);
        if (actor.Dimension == "3D")
        {
            LSCollider collider = actor.Shape switch
            {
                "Sphere" => new LSSphereCollider(),
                "Cone" => new LSConeCollider(),
                "Mesh" => new LSMeshCollider(Array.ConvertAll(actor.VerticesRaw, Vector), actor.Triangles,
                    MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation),
                _ => new LSCuboidCollider { Size = Vector(actor.SizeRaw) }
            };
            if (actor.Frictionless) collider.Material = PhysicsMaterial.Frictionless;
            SolidBody body = _scenario.CreateBody(collider, position, FixedQuaternion.Identity,
                mass: mass, immovable: actor.MotionType == BodyMotionType.Static,
                isKinematic: actor.MotionType == BodyMotionType.Kinematic,
                preventAngularForces: actor.PreventAngularForces).Body;
            // These zero-gravity fixtures exercise rigid-body contacts and CCD;
            // host-owned grounding keeps automatic support snaps out of the trace.
            body.UseManualGrounding();
            body.ContinuousCollisionMode = actor.Continuous ? ContinuousCollisionMode.Continuous : ContinuousCollisionMode.Discrete;
            _bodies3D.Add(actor.Id, body);
            Bind(actor.Id, body, collider);
            collider.OnContactEnter += other => Contact("enter", actor.Id, other);
            collider.OnContact += other => Contact("contact", actor.Id, other);
            collider.OnContactExit += other => Contact("exit", actor.Id, other);
            collider.OnMixedContactEnter += other => Contact("mixed-enter", actor.Id, other);
            collider.OnMixedContact += other => Contact("mixed-contact", actor.Id, other);
            collider.OnMixedContactExit += other => Contact("mixed-exit", actor.Id, other);
            if (_retired3D.HasValue)
            {
                Assert.Equal(_retired3D.Value, collider.Id);
                ReusedSlots++;
                _retired3D = null;
            }
        }
        else
        {
            LSCollider2D collider = actor.Shape == "Sphere"
                ? new LSCircleCollider2D(Fixed64.Half)
                : new LSAABBoxCollider2D(Planar(actor.SizeRaw));
            var transform = new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One);
            var body = new SolidBody2D(new TestMatterAgent(Context, transform), collider) { Mass = mass };
            body.Initialize(Planar(actor.PositionRaw), motionType: actor.MotionType);
            body.ContinuousCollisionMode = actor.Continuous ? ContinuousCollisionMode.Continuous : ContinuousCollisionMode.Discrete;
            _bodies2D.Add(actor.Id, body);
            Bind(actor.Id, body, collider);
            collider.OnContactEnter += other => Contact("enter", actor.Id, other);
            collider.OnContact += other => Contact("contact", actor.Id, other);
            collider.OnContactExit += other => Contact("exit", actor.Id, other);
            collider.OnMixedContactEnter += other => Contact("mixed-enter", actor.Id, other);
            collider.OnMixedContact += other => Contact("mixed-contact", actor.Id, other);
            collider.OnMixedContactExit += other => Contact("mixed-exit", actor.Id, other);
            if (_retired2D.HasValue)
            {
                Assert.Equal(_retired2D.Value, collider.Id);
                ReusedSlots++;
                _retired2D = null;
            }
        }
    }

    private void Bind(int id, object body, object collider)
    {
        _hostIds.Add(body, id);
        _hostIds.Add(collider, id);
    }

    private void AssertSurfaceConstraints()
    {
        // Inspect the first generated manifold, before later separation can retire
        // it. Hash equality alone must not bless the original missing-wall/seam bugs.
        CollisionPair walls = Context.Physics.GetCollisionPair(_bodies3D[80].Collider.Id, _bodies3D[81].Collider.Id)!;
        Assert.NotNull(walls);
        Assert.Contains(walls.Manifold, contact => contact.Normal == Vector3d.Right);
        Assert.Contains(walls.Manifold, contact => contact.Normal == Vector3d.Forward);
        Assert.True(walls.Manifold.GroupCount >= 2);
        CollisionPair tab = Context.Physics.GetCollisionPair(_bodies3D[90].Collider.Id, _bodies3D[91].Collider.Id)!;
        Assert.NotNull(tab);
        Assert.Contains(tab.Manifold, contact => contact.Normal == Vector3d.Down && contact.Depth > Fixed64.Zero);
        // Match the original reported square-plus-tab domain exactly.
        Fixed64 side = Fixed64.FromFraction(1, 5), halfTab = Fixed64.FromFraction(1, 20);
        for (int index = 0; index < tab.Manifold.GroupCount; index++)
        {
            ContactGroup group = tab.Manifold.GetGroup(index);
            for (int sample = 0; sample < group.Count; sample++)
            {
                if (group.Key.Region >= 0)
                    Assert.Equal(Vector3d.Down, group[sample].Normal);
                else
                {
                    Vector3d point = group[sample].PointA - Vector3d.Right * (Fixed64)8;
                    Assert.False(point.X == side && FixedMath.Abs(point.Z) < halfTab,
                        "The square/tab join is covered and cannot become an exposed boundary row.");
                }
            }
        }
    }

    private void Contact(string kind, int actor, object other) =>
        _events.Add($"{kind}:{actor}:{_hostIds[other]}");

    private void Query(ReplayActor actor)
    {
        // Probe live pose; after retirement probe the authored location to
        // verify that the removed actor is absent from the query service.
        Vector3d center = Vector(actor.PositionRaw);
        object? collider;
        if (actor.Dimension == "3D")
        {
            if (_bodies3D.TryGetValue(actor.Id, out SolidBody? body))
                center = body.Position3d;
            Context.Query3D.Raycast(center - Vector3d.Right * (Fixed64)2, Vector3d.Right,
                (Fixed64)4, out var hit, PhysicsLayerMask.All);
            collider = hit.Collider;
        }
        else
        {
            Vector2d planar = Planar(actor.PositionRaw);
            if (_bodies2D.TryGetValue(actor.Id, out SolidBody2D? body))
                planar = body.Position;
            Context.Query2D.Raycast(planar - Vector2d.Right * (Fixed64)2, planar + Vector2d.Right * (Fixed64)2,
                PhysicsLayerMask.All, out var hit);
            collider = hit.Collider;
        }
        string target = collider == null ? "none" : _hostIds[collider].ToString(System.Globalization.CultureInfo.InvariantCulture);
        _queries.Add($"query:{actor.Id}:{target}");
    }

    private void Restore(ReplayActor actor)
    {
        if (_skipRestore)
            return;
        // Only a free body's payload is restored; clocks, world graph and solver
        // caches are deliberately left with their existing owners.
        IRecordable body = actor.Dimension == "3D" ? _bodies3D[actor.Id] : _bodies2D[actor.Id];
        var before = Context.ComputeReplayHash();
        object payload = GravitasSerializationHarness.Serialize(body, GravitasSerializationTransport.Json);
        if (actor.Dimension == "3D") _bodies3D[actor.Id].AddForce(Vector3d.Up * (Fixed64)3);
        else _bodies2D[actor.Id].AddForce(new Vector2d(Fixed64.Zero, (Fixed64)3));
        Assert.NotEqual(before, Context.ComputeReplayHash());
        GravitasSerializationHarness.Populate(body, payload, GravitasSerializationTransport.Json);
        Assert.Equal(before, Context.ComputeReplayHash());
    }

    private ReplayFrame Snapshot()
    {
        var bodies = new List<ReplayBodyState>();
        // Host manifest order, never hash-table or recycled collider-ID order.
        foreach (ReplayActor actor in _fixture.Actors)
        {
            if (_bodies3D.TryGetValue(actor.Id, out SolidBody? body))
                bodies.Add(new ReplayBodyState
                {
                    Actor = actor.Id, PositionRaw = Raw(body.Position3d), VelocityRaw = Raw(body.LinearVelocity),
                    AngularVelocityRaw = Raw(body.AngularVelocity), Sleeping = body.IsSleeping,
                    RotationRaw = new[] { body.Rotation.X.m_rawValue, body.Rotation.Y.m_rawValue, body.Rotation.Z.m_rawValue, body.Rotation.W.m_rawValue },
                    ToiIterations = body.LastContinuousCollisionToiIterationCount
                });
            else if (_bodies2D.TryGetValue(actor.Id, out SolidBody2D? planar))
                bodies.Add(new ReplayBodyState
                {
                    Actor = actor.Id, PositionRaw = Raw(planar.Position), VelocityRaw = Raw(planar.LinearVelocity),
                    AngularVelocityRaw = new[] { planar.AngularVelocity.m_rawValue }, Sleeping = planar.IsSleeping,
                    RotationRaw = new[] { planar.Rotation.m_rawValue }, ToiIterations = planar.LastContinuousCollisionToiIterationCount
                });
        }
        return new ReplayFrame
        {
            FrameCount = Context.FrameCount, LateToken = Context.LateSimulateToken,
            Authoritative = Context.ComputeReplayHash().ToString(),
            SolverCaches = _fixture.Expected[0].SolverCaches == null ? null : Context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches).ToString(),
            Bodies = bodies.ToArray(), Events = _events.ToArray(), Queries = _queries.ToArray()
        };
    }

    private static Vector3d Vector(long[] raw) => new(Fixed64.FromRaw(raw[0]), Fixed64.FromRaw(raw[1]), Fixed64.FromRaw(raw[2]));
    private static Vector2d Planar(long[] raw) => new(Fixed64.FromRaw(raw[0]), Fixed64.FromRaw(raw[2]));
    private static long[] Raw(Vector3d value) => new[] { value.X.m_rawValue, value.Y.m_rawValue, value.Z.m_rawValue };
    private static long[] Raw(Vector2d value) => new[] { value.X.m_rawValue, value.Y.m_rawValue };

    public void Dispose() => _scenario.Dispose();
}
