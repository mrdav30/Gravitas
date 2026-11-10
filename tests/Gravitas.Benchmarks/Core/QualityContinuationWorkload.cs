using System;
using Chronicler;
using Chronicler.Hashing;
using Chronicler.Serialization;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using static Gravitas.Benchmarks.StrategyWorkload;

namespace Gravitas.Benchmarks;

internal static class QualityContinuationWorkload
{
    private const int Seconds = 8;
    private const int CheckpointSeconds = 4;
    private static readonly Fixed64 Gravity = Fixed64.FromFraction(49, 5);
    private static readonly PhysicsMaterial Material = new(Fixed64.Half, Fixed64.FromFraction(1, 3), Fixed64.Zero);

    internal static int Verify()
    {
        bool csv = Environment.GetEnvironmentVariable("GRAVITAS_QUALITY_FRAME_CSV") == "1";
        if (csv)
            Console.WriteLine("fixture,frame,uninterrupted_solver_depth,restored_solver_depth,uninterrupted_floor_residual,restored_floor_residual,position_delta,velocity_delta,rotation_delta_deg,hash_equal");
        foreach (int rate in new[] { 32, 60 })
        foreach (bool planar in new[] { true, false })
        foreach (bool supported in new[] { false, true })
            Verify(rate, planar, supported, csv);
        return 0;
    }

    private static void Verify(int rate, bool planar, bool supported, bool csv)
    {
        string id = $"continuation-{(planar ? "2d" : "3d")}-{(supported ? "support" : "free")}-{rate}hz-v1";
        using var uninterrupted = new Scene(rate, planar, supported);
        using var repeated = new Scene(rate, planar, supported);
        using var restored = new Scene(rate, planar, supported);
        using var cold = new Scene(rate, planar, supported);
        int checkpoint = CheckpointSeconds * rate;
        int frames = Seconds * rate;
        var warmTrace = new ChronicleHashWriter();
        var restoredTrace = new ChronicleHashWriter();
        warmTrace.WriteSection(id, 1);
        restoredTrace.WriteSection(id, 1);

        for (int step = 0; step < checkpoint; step++)
        {
            Sample warm = uninterrupted.Step(step);
            Require(warm == repeated.Step(step) && warm == restored.Step(step) && warm == cold.Step(step),
                step + 1, $"{id} prefix replay diverged");
            WriteHash(ref warmTrace, warm.Hash);
            WriteHash(ref restoredTrace, warm.Hash);
        }

        uninterrupted.QueueCheckpointForce();
        repeated.QueueCheckpointForce();
        cold.QueueCheckpointForce();
        Require(uninterrupted.Context.ElapsedTime == restored.Context.ElapsedTime
            && uninterrupted.Context.ElapsedTime == cold.Context.ElapsedTime,
            checkpoint, $"{id} checkpoint clocks differ");
        Sample before = uninterrupted.Read();
        Require(supported ? before.SupportRows > 0 && before.WarmRows > 0 : before.Sleeping && before.SupportRows == 0,
            checkpoint, $"{id} did not reach its authored save boundary");

        string primary = JsonRecordSerializer.Serialize(uninterrupted.Primary);
        string auxiliary = JsonRecordSerializer.Serialize(uninterrupted.Auxiliary);
        restored.Perturb();
        Require(JsonRecordSerializer.Serialize(restored.Primary) != primary
            && JsonRecordSerializer.Serialize(restored.Auxiliary) != auxiliary, checkpoint, $"{id} target perturbation had no effect");
        Require(!supported || restored.Read().WarmRows > 0, checkpoint, $"{id} perturbation cleared the target's warm rows");
        JsonRecordSerializer.Populate(restored.Primary, primary);
        JsonRecordSerializer.Populate(restored.Auxiliary, auxiliary);
        cold.InvalidateSupportForColdControl();
        Sample loaded = restored.Read();
        Sample reference = cold.Read();
        Require(loaded.WarmRows == 0 && reference.WarmRows == 0
            && loaded with { Hash = before.Hash, WarmRows = before.WarmRows } == before,
            checkpoint, $"{id} load changed the saved body/contact observation");
        Require(JsonRecordSerializer.Serialize(restored.Primary) == primary
            && JsonRecordSerializer.Serialize(restored.Auxiliary) == auxiliary,
            checkpoint, $"{id} body records changed on population");
        Require(loaded == reference && (supported ? loaded.Hash != before.Hash : loaded == before),
            checkpoint, $"{id} checkpoint did not match its cache-equivalent control");

        Fixed64 peakPosition = Fixed64.Zero, peakVelocity = Fixed64.Zero, peakRotation = Fixed64.Zero;
        Fixed64 finalPosition = Fixed64.Zero, finalVelocity = Fixed64.Zero, finalRotation = Fixed64.Zero;
        Fixed64 peakResidual = Fixed64.Zero, peakResidualDelta = Fixed64.Zero;
        int firstMismatch = 0, lastMismatch = 0;
        for (int step = checkpoint; step < frames; step++)
        {
            Sample warm = uninterrupted.Step(step);
            Sample applied = restored.Step(step);
            Require(warm == repeated.Step(step), step + 1, $"{id} uninterrupted replay diverged");
            Require(applied == cold.Step(step), step + 1, $"{id} restored replay diverged from its cache-equivalent control");
            if (!supported)
                Require(applied == warm, step + 1, $"{id} free continuation diverged from uninterrupted");

            finalPosition = (applied.Position - warm.Position).Magnitude;
            finalVelocity = (applied.Velocity - warm.Velocity).Magnitude;
            Fixed64 angle = applied.PlanarAngle - warm.PlanarAngle;
            finalRotation = planar ? FixedMath.RadToDeg(FixedMath.Atan2(FixedMath.Sin(angle), FixedMath.Cos(angle)).Abs())
                : FixedQuaternion.Angle(applied.Rotation, warm.Rotation);
            peakPosition = FixedMath.Max(peakPosition, finalPosition);
            peakVelocity = FixedMath.Max(peakVelocity, finalVelocity);
            peakRotation = FixedMath.Max(peakRotation, finalRotation);
            peakResidual = FixedMath.Max(peakResidual, applied.FloorResidual);
            peakResidualDelta = FixedMath.Max(peakResidualDelta, (applied.FloorResidual - warm.FloorResidual).Abs());
            if (applied.Hash != warm.Hash)
            {
                if (firstMismatch == 0) firstMismatch = step + 1;
                lastMismatch = step + 1;
            }
            WriteHash(ref warmTrace, warm.Hash);
            WriteHash(ref restoredTrace, applied.Hash);
            if (csv)
                Console.WriteLine(FormattableString.Invariant(
                    $"{id},{step + 1},{warm.SolverDepth:F9},{applied.SolverDepth:F9},{warm.FloorResidual:F9},{applied.FloorResidual:F9},{finalPosition:F9},{finalVelocity:F9},{finalRotation:F9},{applied.Hash == warm.Hash}"));
        }
        Require(restored.Spawns == 3 && restored.Despawns == 3 && restored.MassChanges == 3
            && (supported || restored.Sleeps == 4 && restored.Wakes == 4), frames, $"{id} did not execute the complete command schedule");
        Sample terminal = restored.Read();
        Require(terminal.Enters == (supported ? 1 : 0) && terminal.Exits == 0, frames, $"{id} published unexpected contact transitions");
        Console.WriteLine(FormattableString.Invariant(
            $"{id} frames {frames}, checkpoint {checkpoint}, warm/restored traces {warmTrace.ToHash()}/{restoredTrace.ToHash()}, positive normal warm rows before/after load {before.WarmRows}/{loaded.WarmRows}, exact cold checkpoint and {frames - checkpoint} suffix steps"));
        Console.WriteLine(FormattableString.Invariant(
            $"  Peak/final recovery vs uninterrupted position {peakPosition:F9}/{finalPosition:F9}, velocity {peakVelocity:F9}/{finalVelocity:F9}, rotation deg {peakRotation:F9}/{finalRotation:F9}, peak floor residual/delta {peakResidual:F9}/{peakResidualDelta:F9}, first/last unequal hashes {firstMismatch}/{lastMismatch}, spawns/despawns {restored.Spawns}/{restored.Despawns}, mass changes {restored.MassChanges}, sleep/wake commands {restored.Sleeps}/{restored.Wakes}, contact enter/exit {terminal.Enters}/{terminal.Exits}"));
    }

    private static void WriteHash(ref ChronicleHashWriter writer, ChronicleHash hash)
    {
        writer.WriteUInt64(hash.Low);
        writer.WriteUInt64(hash.High);
    }

    private sealed class Scene : IDisposable
    {
        private readonly int _rate;
        private readonly bool _planar;
        private readonly bool _supported;
        private readonly SolidBody2D _body2D;
        private readonly SolidBody _body3D;
        private readonly LSCollider2D _floor2D;
        private readonly LSCollider _floor3D;
        private SolidBody2D _aux2D;
        private SolidBody _aux3D;
        private int _enters, _exits;

        internal Scene(int rate, bool planar, bool supported)
        {
            _rate = rate;
            _planar = planar;
            _supported = supported;
            Context = BenchmarkPhysicsScene.CreateContext(16);
            Context.Settings.RuntimeMode = planar ? PhysicsRuntimeMode.TwoD : PhysicsRuntimeMode.ThreeD;
            Context.SetFrameRate(rate);
            Context.Environment.Gravity = supported ? Gravity : Fixed64.Zero;
            Context.Environment.AirDensity = Fixed64.Zero;
            Context.Environment.DampingFactor = Fixed64.Zero;

            Fixed64 height = supported ? Fixed64.FromFraction(17, 32) : Fixed64.Zero;
            if (planar)
            {
                if (supported)
                {
                    _floor2D = new LSAABBoxCollider2D(ColliderShapeDefinition2D.AABBox(new Vector2d(16, 1), Material));
                    _floor2D.InitializeWithNoBody(new BenchmarkMatterAgent(Context, new Vector3d(Fixed64.Zero, Fixed64.Zero, -Fixed64.Half)));
                }
                _body2D = Create2D(Fixed64.Zero, height, supported);
                (_floor2D ?? _body2D.Collider).OnContactEnter += _ => _enters++;
                (_floor2D ?? _body2D.Collider).OnContactExit += _ => _exits++;
            }
            else
            {
                if (supported)
                {
                    _floor3D = new LSCuboidCollider(ColliderShapeDefinition.Cuboid(new Vector3d(16, 1, 8), Material));
                    _floor3D.InitializeWithNoBody(new BenchmarkMatterAgent(Context, new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Zero)));
                }
                _body3D = Create3D(Fixed64.Zero, height, supported);
                (_floor3D ?? _body3D.Collider).OnContactEnter += _ => _enters++;
                (_floor3D ?? _body3D.Collider).OnContactExit += _ => _exits++;
            }
        }

        internal GravitasWorldContext Context { get; }
        internal IRecordable Primary => (IRecordable)_body2D ?? _body3D;
        internal IRecordable Auxiliary => (IRecordable)_aux2D ?? _aux3D;
        internal int Spawns { get; private set; }
        internal int Despawns { get; private set; }
        internal int MassChanges { get; private set; }
        internal int Sleeps { get; private set; }
        internal int Wakes { get; private set; }

        internal Sample Step(int step)
        {
            if (step % _rate == 0)
            {
                switch (step / _rate)
                {
                    case 0: Sleep(); break;
                    case 1: Spawn(); Pulse(); break;
                    case 2: Despawn(); SetMass((Fixed64)2); Sleep(); break;
                    case 3: Spawn(); Pulse(); break;
                    case 4: Pulse(); break;
                    case 5: Despawn(); SetMass((Fixed64)3); Sleep(); break;
                    case 6: Spawn(); Pulse(); break;
                    case 7: Despawn(); SetMass(Fixed64.One); break;
                }
            }
            if (step == 7 * _rate / 2)
                Sleep();

            Context.Simulate();
            Context.LateSimulate();
            Require(Context.FrameCount == step + 1 && Context.LateSimulateToken == step + 1, step + 1, "continuation clock diverged");
            int bodies = _planar ? Context.Physics2D.BodyCount : Context.Physics.BodyCount;
            int colliders = _planar ? Context.Physics2D.ColliderCount : Context.Physics.ColliderCount;
            Require(bodies == 1 + (Auxiliary == null ? 0 : 1) && colliders == bodies + (_supported ? 1 : 0), step + 1,
                "continuation lost a registered body or collider");
            Sample sample = Read();
            Require(_supported ? step < _rate || sample.SupportRows > 0 && sample.FloorResidual <= Fixed64.FromFraction(1, 16)
                : sample.SupportRows == 0 && sample.Enters == 0, step + 1, "continuation lost its support or free-motion boundary");
            return sample;
        }

        internal Sample Read()
        {
            int rows = 0, warm = 0;
            Fixed64 depth = Fixed64.Zero;
            if (TryGetSupport(out CollisionPair2D pair2D))
            {
                for (int g = 0; g < pair2D.Manifold.GroupCount; g++)
                {
                    ref ContactGroup2D group = ref pair2D.Manifold.GetGroup(g);
                    for (int i = 0; i < group.Count; i++)
                    {
                        var row = group[i];
                        if ((pair2D.ColliderB == _body2D.Collider ? row.Normal.Y : -row.Normal.Y) <= Fixed64.Zero)
                            continue;
                        Require(!row.DepthIsClamped, (int)Context.FrameCount, "planar support depth clamped");
                        rows++;
                        depth = FixedMath.Max(depth, row.Depth);
                        if (pair2D.TryGetWarmStartImpulse(group.Key, row.ContactId, out var impulse) && impulse.NormalImpulse > Fixed64.Zero)
                            warm++;
                    }
                }
            }
            if (TryGetSupport(out CollisionPair pair3D))
            {
                for (int g = 0; g < pair3D.Manifold.GroupCount; g++)
                {
                    ref ContactGroup group = ref pair3D.Manifold.GetGroup(g);
                    for (int i = 0; i < group.Count; i++)
                    {
                        var row = group[i];
                        if ((pair3D.ColliderB == _body3D.Collider ? row.Normal.Y : -row.Normal.Y) <= Fixed64.Zero)
                            continue;
                        Require(!row.DepthIsClamped, (int)Context.FrameCount, "spatial support depth clamped");
                        rows++;
                        depth = FixedMath.Max(depth, row.Depth);
                        if (pair3D.TryGetWarmStartImpulse(group.Key, row.ContactId, out var impulse) && impulse.NormalImpulse > Fixed64.Zero)
                            warm++;
                    }
                }
            }
            Fixed64 residual = Fixed64.Zero;
            if (_supported)
            {
                bool insideFloor = _planar
                    ? _body2D.Collider.MinX > _floor2D.MinX && _body2D.Collider.MaxX < _floor2D.MaxX
                    : _body3D.Collider.BoundsMin.X > _floor3D.BoundsMin.X && _body3D.Collider.BoundsMax.X < _floor3D.BoundsMax.X
                        && _body3D.Collider.BoundsMin.Z > _floor3D.BoundsMin.Z && _body3D.Collider.BoundsMax.Z < _floor3D.BoundsMax.Z;
                Require(insideFloor, (int)Context.FrameCount, "continuation left the floor footprint");
                residual = FixedMath.Max(Fixed64.Zero, -(_body2D?.Collider.MinY ?? _body3D.Collider.BoundsMin.Y));
            }
            return new Sample(Context.ComputeReplayHash(), _body2D?.Position.ToVector3d(Fixed64.Zero) ?? _body3D.Position3d,
                _body2D?.LinearVelocity.ToVector3d(Fixed64.Zero) ?? _body3D.LinearVelocity,
                _body2D?.Rotation ?? Fixed64.Zero, _body3D?.Rotation ?? FixedQuaternion.Identity,
                depth, residual, rows, warm, _body2D?.IsSleeping ?? _body3D.IsSleeping, _enters, _exits);
        }

        internal void QueueCheckpointForce()
        {
            if (_supported)
            {
                _body2D?.AddForce(Vector2d.Right);
                _body3D?.AddForce(Vector3d.Right);
            }
            _aux2D?.AddForce(Vector2d.Right);
            _aux3D?.AddForce(Vector3d.Right);
        }

        internal void Perturb()
        {
            if (_planar)
            {
                if (_supported) _body2D.AddForce(Vector2d.Left);
                else _body2D.Mass = (Fixed64)7;
                _aux2D.AddForce(Vector2d.Left);
            }
            else
            {
                if (_supported) _body3D.AddForce(Vector3d.Left);
                else _body3D.Mass = (Fixed64)7;
                _aux3D.AddForce(Vector3d.Left);
            }
        }

        internal void InvalidateSupportForColdControl()
        {
            // Populate republishes the authored shape and drops accumulated impulses. Reproduce only those effects in the control.
            if (TryGetSupport(out CollisionPair2D pair2D))
            {
                pair2D.ClearWarmStart();
                _body2D.Collider.MarkContactDependencyChanged();
            }
            if (TryGetSupport(out CollisionPair pair3D))
            {
                pair3D.ClearWarmStart();
                _body3D.Collider.MarkContactDependencyChanged();
            }
        }

        private bool TryGetSupport(out CollisionPair2D pair)
        {
            pair = null;
            return _floor2D != null && (_floor2D.TryGetCollisionPair(_body2D.Collider.Id, out pair)
                || _body2D.Collider.TryGetCollisionPair(_floor2D.Id, out pair));
        }

        private bool TryGetSupport(out CollisionPair pair)
        {
            pair = null;
            return _floor3D != null && (_floor3D.TryGetCollisionPair(_body3D.Collider.Id, out pair)
                || _body3D.Collider.TryGetCollisionPair(_floor3D.Id, out pair));
        }

        private SolidBody2D Create2D(Fixed64 x, Fixed64 height, bool support)
        {
            var position = new Vector2d(x, height);
            LSCollider2D collider = support
                ? new LSPolygonCollider2D(ColliderShapeDefinition2D.ConvexPolygon(Material,
                    new Vector2d(-Fixed64.One, -Fixed64.Half), new Vector2d(Fixed64.One, -Fixed64.Half),
                    new Vector2d(Fixed64.One, Fixed64.Half), new Vector2d(-Fixed64.One, Fixed64.Half)))
                : new LSCircleCollider2D(Fixed64.Half);
            var body = new SolidBody2D(new BenchmarkMatterAgent(Context, position.ToVector3d(Fixed64.Zero)), collider)
            {
                Mass = Fixed64.One,
                Gravity = support ? Vector2d.Backward * Gravity : Vector2d.Zero,
                SleepEnabled = !support
            };
            body.UseManualGrounding();
            body.Initialize(position);
            return body;
        }

        private SolidBody Create3D(Fixed64 x, Fixed64 height, bool support)
        {
            var position = new Vector3d(x, height, Fixed64.Zero);
            LSCollider collider = support
                ? new LSCuboidCollider(ColliderShapeDefinition.Cuboid(new Vector3d(2, 1, 2), Material))
                : new LSSphereCollider();
            var body = new SolidBody(new BenchmarkMatterAgent(Context, position), collider)
            {
                Mass = Fixed64.One,
                GravityScale = support ? Fixed64.One : Fixed64.Zero,
                SleepEnabled = !support,
                LinearDragCoefficient = Fixed64.Zero
            };
            body.UseManualGrounding();
            body.Initialize(position, FixedQuaternion.Identity);
            return body;
        }

        private void Spawn()
        {
            Require(Auxiliary == null, (int)Context.FrameCount, "continuation spawn slot is occupied");
            Spawns++;
            if (_planar) _aux2D = Create2D((Fixed64)12, Fixed64.Zero, support: false);
            else _aux3D = Create3D((Fixed64)12, Fixed64.Zero, support: false);
        }

        private void Despawn()
        {
            Require(Auxiliary != null, (int)Context.FrameCount, "continuation despawn slot is empty");
            Despawns++;
            _aux2D?.Deactivate();
            _aux3D?.Deactivate();
            _aux2D = null;
            _aux3D = null;
        }

        private void SetMass(Fixed64 mass)
        {
            Require((_body2D?.Mass ?? _body3D.Mass) != mass, (int)Context.FrameCount, "continuation mass change repeated the existing mass");
            if (_planar) _body2D.Mass = mass;
            else _body3D.Mass = mass;
            Require((_body2D?.Mass ?? _body3D.Mass) == mass, (int)Context.FrameCount, "continuation mass change did not take effect");
            MassChanges++;
        }

        private void Sleep()
        {
            if (_supported) return;
            Require(!(_body2D?.IsSleeping ?? _body3D.IsSleeping), (int)Context.FrameCount, "continuation body slept before the scheduled sleep");
            Sleeps++;
            _body2D?.Sleep();
            _body3D?.Sleep();
            Require(_body2D?.IsSleeping ?? _body3D.IsSleeping, (int)Context.FrameCount, "continuation sleep did not take effect");
        }

        private void Pulse()
        {
            bool wasSleeping = _body2D?.IsSleeping ?? _body3D.IsSleeping;
            _body2D?.AddLinearImpulse(Vector2d.Right * Fixed64.FromFraction(1, 8));
            _body3D?.AddLinearImpulse(Vector3d.Right * Fixed64.FromFraction(1, 8));
            if (wasSleeping && !(_body2D?.IsSleeping ?? _body3D.IsSleeping)) Wakes++;
        }

        public void Dispose() => Context.Dispose();
    }

    private readonly record struct Sample(ChronicleHash Hash, Vector3d Position, Vector3d Velocity, Fixed64 PlanarAngle,
        FixedQuaternion Rotation, Fixed64 SolverDepth, Fixed64 FloorResidual, int SupportRows, int WarmRows, bool Sleeping, int Enters, int Exits);
}
