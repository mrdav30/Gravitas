using System;
using Chronicler.Hashing;
using FixedMathSharp;
using FixedMathSharp.Chronicler;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Diagnostics;
using Gravitas.Materials;
using static Gravitas.Benchmarks.StrategyWorkload;

namespace Gravitas.Benchmarks;

internal static class MixedQualityWorkload
{
    private const int Frames = 16;
    private static readonly Fixed64 InitialDistance = Fixed64.FromFraction(7, 8);
    private static readonly Fixed64 VelocityTolerance = Fixed64.FromFraction(1, 128);
    private static readonly Fixed64 ResidualBound = Fixed64.FromFraction(1, 32);

    internal static int Verify()
    {
        foreach (bool vertical in new[] { false, true })
        {
            string name = vertical ? "vertical" : "planar";
            Result mixed = VerifyReplay(vertical, mixed: true);
            Result pure = VerifyReplay(vertical, mixed: false);
            Require((mixed.MoverSpeed - pure.MoverSpeed).Abs() <= VelocityTolerance
                && (mixed.TargetSpeed - pure.TargetSpeed).Abs() <= VelocityTolerance, 1,
                $"{name} mixed and pure response speeds diverged");
            Print(name, "mixed", mixed);
            Print(name, vertical ? "pure-3d" : "pure-2d", pure);
        }
        return 0;
    }

    private static Result VerifyReplay(bool vertical, bool mixed)
    {
        Result first = Run(vertical, mixed, observe: true);
        Result repeat = Run(vertical, mixed, observe: true);
        Result control = Run(vertical, mixed, observe: false);
        Require(first == repeat && first.Terminal == control.Terminal, Frames, "mixed quality replay diverged");
        return first;
    }

    private static Result Run(bool vertical, bool mixed, bool observe)
    {
        using GravitasWorldContext context = BenchmarkPhysicsScene.CreateMixedContext(8, 8);
        context.Settings.RuntimeMode = mixed ? PhysicsRuntimeMode.Mixed
            : vertical ? PhysicsRuntimeMode.ThreeD : PhysicsRuntimeMode.TwoD;
        context.SetFrameRate(60);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        if (observe && mixed)
            context.Diagnostics.Enable();

        Vector3d direction = vertical ? -Vector3d.Up : Vector3d.Right;
        Vector3d start = -direction * InitialDistance;
        SolidBody mover3D = null;
        SolidBody2D mover2D = null;
        SolidBody2D target2D = null;
        LSCuboidCollider target3D = null;
        if (mixed || vertical)
        {
            var sphere = new LSSphereCollider(ColliderShapeDefinition.Sphere(Fixed64.Half, PhysicsMaterial.Frictionless));
            mover3D = new SolidBody(new BenchmarkMatterAgent(context, start), sphere)
            {
                Mass = Fixed64.One,
                SleepEnabled = false
            };
            mover3D.UseManualGrounding();
            mover3D.Initialize(start, FixedQuaternion.Identity);
            mover3D.AddLinearImpulse(direction * Fixed64.Quarter);
        }
        else
        {
            mover2D = CreatePlanarBody(context, Circle(), start);
            mover2D.AddLinearImpulse(Vector2d.Right * Fixed64.Quarter);
        }

        if (mixed || !vertical)
        {
            LSCollider2D collider = vertical
                ? new LSAABBoxCollider2D(ColliderShapeDefinition2D.AABBox(new Vector2d(4, 4), PhysicsMaterial.Frictionless))
                : Circle();
            collider.MixedHalfThicknessOverride = Fixed64.Half;
            target2D = CreatePlanarBody(context, collider, Vector3d.Zero);
        }
        else
        {
            target3D = new LSCuboidCollider(ColliderShapeDefinition.Cuboid(new Vector3d(4, 1, 4), PhysicsMaterial.Frictionless));
            target3D.InitializeWithNoBody(new BenchmarkMatterAgent(context, Vector3d.Zero));
        }

        var trace = new ChronicleHashWriter();
        trace.WriteSection("quality-mixed-contact-v1", 1);
        Fixed64 moverSpeed = Fixed64.Zero, targetSpeed = Fixed64.Zero;
        Fixed64 solverDepth = Fixed64.Zero, peakResidual = Fixed64.Zero, residual = Fixed64.Zero;
        for (int frame = 1; frame <= Frames; frame++)
        {
            context.Simulate();
            context.LateSimulate();
            if (!observe)
                continue;

            Vector3d moverPosition = mover3D?.Position3d ?? Embed(mover2D.Position);
            Vector3d moverVelocity = mover3D?.LinearVelocity ?? Embed(mover2D.LinearVelocity);
            Vector3d targetPosition = target2D == null ? Vector3d.Zero : Embed(target2D.Position);
            Vector3d targetVelocity = target2D == null ? Vector3d.Zero : Embed(target2D.LinearVelocity);
            Require(moverPosition.Magnitude < Fixed64.Two && moverVelocity.Magnitude < Fixed64.Two
                && targetPosition.Magnitude < Fixed64.Two && targetVelocity.Magnitude < Fixed64.Two,
                frame, "impact left its bounded position or speed range");
            Require(moverPosition == direction * Vector3d.Dot(moverPosition, direction)
                && moverVelocity == direction * Vector3d.Dot(moverVelocity, direction)
                && targetPosition == direction * Vector3d.Dot(targetPosition, direction)
                && targetVelocity == direction * Vector3d.Dot(targetVelocity, direction), frame, "centered impact left its authored axis");
            Require((mover3D?.AngularVelocity ?? Vector3d.Zero) == Vector3d.Zero
                && (mover2D?.AngularVelocity ?? Fixed64.Zero) == Fixed64.Zero
                && (target2D?.AngularVelocity ?? Fixed64.Zero) == Fixed64.Zero
                && (mover3D?.Rotation ?? FixedQuaternion.Identity) == FixedQuaternion.Identity
                && (mover2D?.Rotation ?? Fixed64.Zero) == Fixed64.Zero
                && (target2D?.Rotation ?? Fixed64.Zero) == Fixed64.Zero, frame, "centered impact generated spin or rotation");
            if (target2D != null)
                Require(target2D.Agent.Transform.LocalPosition.Y == Fixed64.Zero && target2D.Collider.MixedSlabCenterY == Fixed64.Zero,
                    frame, "planar target left its embedding height");

            // Both authored centerline sections have unit combined reach along the impact axis.
            Fixed64 separation = Vector3d.Dot(targetPosition - moverPosition, direction);
            Require(separation > Fixed64.Zero && separation < Fixed64.Two, frame, "impact left the admitted centerline range");
            residual = FixedMath.Max(Fixed64.Zero, Fixed64.One - separation);
            peakResidual = FixedMath.Max(peakResidual, residual);
            if (frame == 1)
            {
                solverDepth = mixed ? ReadMixedImpact(context, mover3D.Collider, target2D.Collider, direction)
                    : vertical ? ReadSpatialImpact(mover3D.Collider, target3D, direction)
                    : ReadPlanarImpact(mover2D.Collider, target2D.Collider);
                if (mixed)
                    context.Diagnostics.Disable();
                moverSpeed = Vector3d.Dot(moverVelocity, direction);
                targetSpeed = Vector3d.Dot(targetVelocity, direction);
                Fixed64 expectedSpeed = vertical ? Fixed64.Zero : Fixed64.FromFraction(1, 8);
                Require((moverSpeed - expectedSpeed).Abs() <= VelocityTolerance
                    && (targetSpeed - expectedSpeed).Abs() <= VelocityTolerance, frame, "inelastic centerline speed missed its bound");
                Require(residual < Fixed64.One - InitialDistance, frame, "first step did not reduce the authored overlap");
            }

            ChronicleHash hash = context.ComputeReplayHash();
            trace.WriteUInt64(hash.Low);
            trace.WriteUInt64(hash.High);
            trace.WriteFixed64(residual);
        }

        if (observe)
            Require(residual <= ResidualBound, Frames, "terminal centerline overlap exceeded 1/32 of a unit");
        return new Result(trace.ToHash(), context.ComputeReplayHash(), moverSpeed, targetSpeed, solverDepth, peakResidual, residual);
    }

    private static Fixed64 ReadMixedImpact(GravitasWorldContext context, LSCollider mover, LSCollider2D target, Vector3d direction)
    {
        int contacts = 0, impulses = 0;
        Fixed64 depth = Fixed64.Zero;
        foreach (GravitasDiagnosticEvent diagnostic in context.Diagnostics.Events)
        {
            if (diagnostic.TryAsMixedContact(out GravitasMixedContactDiagnosticView contact))
            {
                Require(contact.Collider3DId == mover.Id && contact.Collider2DId == target.Id
                    && contact.Frame == 1 && contact.HasContact && !contact.DepthIsClamped
                    && contact.Normal3DTo2D == direction, 1, "unexpected mixed contact witness");
                depth = contact.Depth;
                contacts++;
            }
            if (diagnostic.TryAsMixedResponseImpulse(out GravitasMixedResponseImpulseDiagnosticView impulse))
            {
                Require(impulse.Collider3DId == mover.Id && impulse.Collider2DId == target.Id
                    && impulse.Frame == 1 && Vector3d.Dot(impulse.Impulse, direction) > Fixed64.Zero,
                    1, "mixed response impulse missed the authored contact axis");
                impulses++;
            }
        }
        Require(contacts == 1 && impulses == 1 && depth > Fixed64.Zero, 1, "mixed impact did not report one contact and one applied impulse");
        return depth;
    }

    private static Fixed64 ReadSpatialImpact(LSCollider mover, LSCollider target, Vector3d direction)
    {
        Require((mover.TryGetCollisionPair(target.Id, out CollisionPair pair) || target.TryGetCollisionPair(mover.Id, out pair))
            && pair.Manifold.Count == 1 && pair.Manifold.LastUpdatedFrame == 1, 1, "expected one fresh spatial contact row");
        ManifoldContact row = pair.Manifold.PrimaryContact;
        Require(!row.DepthIsClamped && row.Depth > Fixed64.Zero && (pair.ColliderA == mover ? row.Normal : -row.Normal) == direction,
            1, "unexpected spatial contact witness");
        return row.Depth;
    }

    private static Fixed64 ReadPlanarImpact(LSCollider2D mover, LSCollider2D target)
    {
        Require((mover.TryGetCollisionPair(target.Id, out CollisionPair2D pair) || target.TryGetCollisionPair(mover.Id, out pair))
            && pair.Manifold.Count == 1 && pair.Manifold.LastUpdatedFrame == 1, 1, "expected one fresh planar contact row");
        ManifoldContact2D row = pair.Manifold.PrimaryContact;
        Require(!row.DepthIsClamped && row.Depth > Fixed64.Zero && (pair.ColliderA == mover ? row.Normal : -row.Normal) == Vector2d.Right,
            1, "unexpected planar contact witness");
        return row.Depth;
    }

    private static LSCircleCollider2D Circle() => new(Fixed64.Half) { Material = PhysicsMaterial.Frictionless };

    private static SolidBody2D CreatePlanarBody(GravitasWorldContext context, LSCollider2D collider, Vector3d position)
    {
        var body = new SolidBody2D(new BenchmarkMatterAgent(context, position), collider)
        {
            Mass = Fixed64.One,
            SleepEnabled = false
        };
        body.UseManualGrounding();
        body.Initialize(position.ToVector2d());
        return body;
    }

    private static Vector3d Embed(Vector2d position) => new(position.X, Fixed64.Zero, position.Y);

    private static void Print(string name, string mode, Result result) => Console.WriteLine(FormattableString.Invariant(
        $"quality-{name}-{mode}-v1 trace {result.Trace}, terminal {result.Terminal}, mover/target normal speed {result.MoverSpeed:F6}/{result.TargetSpeed:F6}, initial solve depth {result.SolverDepth:F6}, peak/final residual {result.PeakResidual:F6}/{result.FinalResidual:F6}"));

    private readonly record struct Result(ChronicleHash Trace, ChronicleHash Terminal, Fixed64 MoverSpeed, Fixed64 TargetSpeed,
        Fixed64 SolverDepth, Fixed64 PeakResidual, Fixed64 FinalResidual);
}
