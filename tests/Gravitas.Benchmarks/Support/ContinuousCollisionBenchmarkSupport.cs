using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using Gravitas.Support;
using GridForge.Configuration;
using SwiftCollections;
using System;

namespace Gravitas.Benchmarks;

internal static class ContinuousCollisionBenchmarkSupport
{
    public static readonly Vector3d Force3D = Vector3d.Right * (Fixed64)2;
    public static readonly Vector2d Force2D = Vector2d.Right * (Fixed64)2;
    public static readonly Vector3d ShapeExactDynamicForce3D = Vector3d.Right * (Fixed64)10;
    public static readonly Vector2d ShapeExactDynamicForce2D = Vector2d.Right * (Fixed64)10;
    public static readonly Vector3d ToiIterationForce3D = new((Fixed64)4, Fixed64.Zero, (Fixed64)4);
    public static readonly Vector2d ToiIterationForce2D = new((Fixed64)4, (Fixed64)4);

    public static GravitasWorldContext CreateContext3D(int extentX, int extentZ)
    {
        GravitasWorldContext context = BenchmarkEnvironment.PrepareOwnedContext();
        ConfigureContext(context);
        AddGrid(context, extentX, extentZ);
        return context;
    }

    public static GravitasWorldContext CreateContext2D(int extentX, int extentZ)
    {
        GravitasWorldContext context = BenchmarkEnvironment.PrepareOwnedContext();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        ConfigureContext(context);
        AddGrid(context, extentX, extentZ);
        return context;
    }

    public static GravitasWorldContext CreateMixedContext(int extentX, int extentZ)
    {
        GravitasWorldContext context = BenchmarkEnvironment.PrepareOwnedContext();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        ConfigureContext(context);
        AddGrid(context, extentX, extentZ);
        return context;
    }

    public static SolidBody CreateSphere3D(GravitasWorldContext context, Vector3d position)
    {
        var agent = new BenchmarkMatterAgent(context, position);
        var body = new SolidBody(agent, new LSSphereCollider())
        {
            ContinuousCollisionMode = ContinuousCollisionMode.Continuous,
            GroundProbeMode = GroundProbeMode.Ray,
            Mass = Fixed64.One
        };

        body.Initialize(position, FixedQuaternion.Identity);
        return body;
    }

    public static SolidBody2D CreateCircle2D(GravitasWorldContext context, Vector2d position)
    {
        var agent = new BenchmarkMatterAgent(context, position.ToVector3d(Fixed64.Zero));
        var body = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            ContinuousCollisionMode = ContinuousCollisionMode.Continuous,
            Mass = Fixed64.One
        };

        body.Initialize(position);
        return body;
    }

    public static SolidBody CreateThinCuboid3D(GravitasWorldContext context, Vector3d position)
    {
        var agent = new BenchmarkMatterAgent(context, position);
        var body = new SolidBody(
            agent,
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.FromFraction(1, 5), Fixed64.FromFraction(1, 5))
            })
        {
            ContinuousCollisionMode = ContinuousCollisionMode.Continuous,
            Mass = Fixed64.One
        };

        body.Initialize(position, FixedQuaternion.Identity);
        return body;
    }

    public static SolidBody2D CreateThinPolygon2D(GravitasWorldContext context, Vector2d position)
    {
        var agent = new BenchmarkMatterAgent(context, position.ToVector3d(Fixed64.Zero));
        var collider = new LSPolygonCollider2D(
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(-1, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(1, 10)),
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(1, 10)));
        var body = new SolidBody2D(agent, collider)
        {
            ContinuousCollisionMode = ContinuousCollisionMode.Continuous,
            Mass = Fixed64.One
        };

        body.Initialize(position);
        return body;
    }

    public static void CreateStaticSphere3D(GravitasWorldContext context, Vector3d position)
    {
        var agent = new BenchmarkMatterAgent(context, position);
        var body = new SolidBody(agent, new LSSphereCollider { Radius = Fixed64.FromFraction(1, 4) })
        {
            Mass = Fixed64.One
        };

        body.Initialize(position, FixedQuaternion.Identity, BodyMotionType.Static);
    }

    public static void CreateStaticCuboid3D(GravitasWorldContext context, Vector3d position, Vector3d size)
    {
        var agent = new BenchmarkMatterAgent(context, position);
        var body = new SolidBody(agent, new LSCuboidCollider { Size = size })
        {
            Mass = Fixed64.One
        };

        body.Initialize(position, FixedQuaternion.Identity, BodyMotionType.Static);
    }

    public static void CreateStaticCircle2D(GravitasWorldContext context, Vector2d position)
    {
        var agent = new BenchmarkMatterAgent(context, position.ToVector3d(Fixed64.Zero));
        var body = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.FromFraction(1, 4)))
        {
            Mass = Fixed64.One
        };

        body.Initialize(position, motionType: BodyMotionType.Static);
    }

    public static void CreateStaticAabb2D(GravitasWorldContext context, Vector2d position, Vector2d size)
    {
        var agent = new BenchmarkMatterAgent(context, position.ToVector3d(Fixed64.Zero));
        var body = new SolidBody2D(agent, new LSAABBoxCollider2D(size))
        {
            Mass = Fixed64.One
        };

        body.Initialize(position, motionType: BodyMotionType.Static);
    }

    public static void Reset3DBodies(SwiftList<SolidBody> bodies, Vector3d[] positions, bool pairedDirections)
    {
        for (int i = 0; i < bodies.Count; i++)
        {
            SolidBody body = bodies[i];
            body.ResetPosition(positions[i], FixedQuaternion.Identity);
            body.AddForce(Get3DForce(i, pairedDirections));
        }
    }

    public static void Reset3DBodies(SwiftList<SolidBody> bodies, Vector3d[] positions, Vector3d force)
    {
        for (int i = 0; i < bodies.Count; i++)
        {
            SolidBody body = bodies[i];
            body.ResetPosition(positions[i], FixedQuaternion.Identity);
            body.AddForce(force);
        }
    }

    public static void Reset2DBodies(SwiftList<SolidBody2D> bodies, Vector2d[] positions, bool pairedDirections)
    {
        for (int i = 0; i < bodies.Count; i++)
        {
            SolidBody2D body = bodies[i];
            body.ResetPosition(positions[i]);
            body.AddForce(Get2DForce(i, pairedDirections));
        }
    }

    public static void Reset2DBodies(SwiftList<SolidBody2D> bodies, Vector2d[] positions, Vector2d force)
    {
        for (int i = 0; i < bodies.Count; i++)
        {
            SolidBody2D body = bodies[i];
            body.ResetPosition(positions[i]);
            body.AddForce(force);
        }
    }

    public static void Reset2DDynamicShapeExactBodies(SwiftList<SolidBody2D> bodies, Vector2d[] positions)
    {
        for (int i = 0; i < bodies.Count; i++)
        {
            SolidBody2D body = bodies[i];
            body.ResetPosition(positions[i]);
            if ((i & 1) == 0)
                body.AddForce(ShapeExactDynamicForce2D);
            else
                body.Sleep();
        }
    }

    public static void Reset3DDynamicShapeExactBodies(SwiftList<SolidBody> bodies, Vector3d[] positions)
    {
        for (int i = 0; i < bodies.Count; i++)
        {
            SolidBody body = bodies[i];
            body.ResetPosition(positions[i], FixedQuaternion.Identity);
            if ((i & 1) == 0)
                body.AddForce(ShapeExactDynamicForce3D);
            else
                body.Sleep();
        }
    }

    public static void Reset3DBodyPositions(SwiftList<SolidBody> bodies, Vector3d[] positions)
    {
        for (int i = 0; i < bodies.Count; i++)
            bodies[i].ResetPosition(positions[i], FixedQuaternion.Identity);
    }

    public static void Reset2DBodyPositions(SwiftList<SolidBody2D> bodies, Vector2d[] positions)
    {
        for (int i = 0; i < bodies.Count; i++)
        {
            bodies[i].ResetPosition(positions[i]);
        }
    }

    public static void Reset3DAngularBodies(SwiftList<SolidBody> bodies, Vector3d[] positions, bool angularMotion)
    {
        for (int i = 0; i < bodies.Count; i++)
        {
            SolidBody body = bodies[i];
            body.ResetPosition(positions[i], FixedQuaternion.Identity);
            if (angularMotion)
                body.AddAngularImpulse(Vector3d.Up);
        }
    }

    public static void Reset2DAngularBodies(SwiftList<SolidBody2D> bodies, Vector2d[] positions, bool angularMotion)
    {
        for (int i = 0; i < bodies.Count; i++)
        {
            SolidBody2D body = bodies[i];
            body.ResetPosition(positions[i]);
            if (angularMotion)
                body.AddAngularImpulse(Fixed64.One);
        }
    }

    public static Vector3d Get3DForce(int index, bool pairedDirections) =>
        pairedDirections && (index & 1) == 1 ? -Force3D : Force3D;

    public static Vector2d Get2DForce(int index, bool pairedDirections) =>
        pairedDirections && (index & 1) == 1 ? -Force2D : Force2D;

    public static Vector3d Sum3D(SwiftList<SolidBody> bodies)
    {
        Vector3d total = Vector3d.Zero;
        for (int i = 0; i < bodies.Count; i++)
            total += bodies[i].Position3d;

        return total;
    }

    public static Vector2d Sum2D(SwiftList<SolidBody2D> bodies)
    {
        Vector2d total = Vector2d.Zero;
        for (int i = 0; i < bodies.Count; i++)
            total += bodies[i].Position;

        return total;
    }

    public static int SweepMixedStatic2DQueries(
        GravitasWorldContext context,
        SwiftList<SolidBody> bodies,
        Vector3d[] positions,
        SwiftList<PhysicsMixedHit> hits)
    {
        int total = 0;
        context.AdvanceLateSimulateToken();
        for (int i = 0; i < bodies.Count; i++)
        {
            Vector3d start = positions[i];
            Vector3d end = start + Force3D;
            total += context.QueryMixed.SweepSphereAgainstStatic2DAll(
                start,
                end,
                Fixed64.Half,
                PhysicsLayerMask.All,
                hits,
                bodies[i].Collider,
                includeTriggers: false,
                cacheTargetPartitions: true);
            total += context.QueryMixed.LastQueryCandidateCount;
        }

        return total;
    }

    public static int SweepMixedStatic3DQueries(
        GravitasWorldContext context,
        SwiftList<SolidBody2D> bodies,
        Vector2d[] positions,
        SwiftList<PhysicsMixedHit> hits)
    {
        int total = 0;
        context.AdvanceLateSimulateToken();
        for (int i = 0; i < bodies.Count; i++)
        {
            Vector2d start = positions[i];
            Vector2d end = start + Force2D;
            total += context.QueryMixed.SweepCircleAgainstStatic3DAll(
                start,
                end,
                Fixed64.Half,
                bodies[i].Collider.MixedSlabCenterY,
                bodies[i].Collider.MixedHalfThickness,
                PhysicsLayerMask.All,
                hits,
                bodies[i].Collider,
                includeTriggers: false,
                cacheTargetPartitions: true);
            total += context.QueryMixed.LastQueryCandidateCount;
        }

        return total;
    }

    public static int SweepPure3DStaticQueries(
        GravitasWorldContext context,
        SwiftList<SolidBody> bodies,
        Vector3d[] positions,
        bool pairedDirections,
        SwiftList<Physics3DHit> hits)
    {
        int total = 0;
        for (int i = 0; i < bodies.Count; i++)
        {
            Vector3d start = positions[i];
            Vector3d end = start + Get3DForce(i, pairedDirections);
            total += context.Query3D.SweepSphereAgainstStaticAll(
                start,
                end,
                Fixed64.Half,
                PhysicsLayerMask.All,
                hits,
                bodies[i].Collider,
                includeTriggers: false);
            total += context.Query3D.LastQueryCandidateCount;
        }

        return total;
    }

    public static int SweepPure2DStaticQueries(
        GravitasWorldContext context,
        SwiftList<SolidBody2D> bodies,
        Vector2d[] positions,
        bool pairedDirections,
        SwiftList<Physics2DHit> hits)
    {
        int total = 0;
        for (int i = 0; i < bodies.Count; i++)
        {
            Vector2d start = positions[i];
            Vector2d end = start + Get2DForce(i, pairedDirections);
            total += context.Query2D.SweepCircleAgainstStaticAll(
                start,
                end,
                Fixed64.Half,
                PhysicsLayerMask.All,
                hits,
                bodies[i].Collider,
                includeTriggers: false);
            total += context.Query2D.LastQueryCandidateCount;
        }

        return total;
    }

    public static int QueryPure3DDynamicCandidates(GravitasWorldContext context, SwiftList<SolidBody> bodies)
    {
        int total = 0;
        context.Physics.PrepareContinuousCollisionFrame();
        for (int i = 0; i < bodies.Count; i++)
        {
            SolidBody body = bodies[i];
            SwiftList<int> candidates = context.Physics.QueryContinuousCollisionCandidates(
                DynamicCcdCandidateIndex.CreateSweptSphereBounds(
                    body.ContinuousCollisionFrameStart,
                    body.ContinuousCollisionFrameDisplacement,
                    body.ResolveContinuousCollisionProxyRadius()));
            total += candidates.Count;
        }

        return total;
    }

    public static int QueryPure2DDynamicCandidates(GravitasWorldContext context, SwiftList<SolidBody2D> bodies)
    {
        int total = 0;
        context.Physics2D.PrepareContinuousCollisionFrame();
        for (int i = 0; i < bodies.Count; i++)
        {
            SolidBody2D body = bodies[i];
            SwiftList<int> candidates = context.Physics2D.QueryPlanarContinuousCollisionCandidates(
                DynamicCcdCandidateIndex2D.CreateSweptCircleBounds(
                    body.ContinuousCollisionFrameStart,
                    body.ContinuousCollisionFrameDisplacement,
                    body.ResolveContinuousCollisionProxyRadius()));
            total += candidates.Count;
        }

        return total;
    }

    public static int SweepPure3DDynamicRelativeTargets(GravitasWorldContext context, SwiftList<SolidBody> bodies)
    {
        int total = 0;
        context.Physics.PrepareContinuousCollisionFrame();
        for (int i = 0; i < bodies.Count; i++)
        {
            SolidBody source = bodies[i];
            Fixed64 sourceRadius = source.ResolveContinuousCollisionProxyRadius();
            SwiftList<int> candidates = context.Physics.QueryContinuousCollisionCandidates(
                DynamicCcdCandidateIndex.CreateSweptSphereBounds(
                    source.ContinuousCollisionFrameStart,
                    source.ContinuousCollisionFrameDisplacement,
                    sourceRadius));

            for (int j = 0; j < candidates.Count; j++)
            {
                int dynamicId = candidates[j];
                SolidBody target = context.Physics.GetContinuousCollisionCandidate(dynamicId);
                if (ReferenceEquals(source, target))
                {
                    continue;
                }

                Fixed64 targetRadius = target.ResolveContinuousCollisionProxyRadius();
                if (ContinuousCollisionMath.TrySweepRelativeSpheres(
                        source.ContinuousCollisionFrameStart,
                        source.ContinuousCollisionFrameDisplacement,
                        sourceRadius,
                        target.ContinuousCollisionFrameStart,
                        target.ContinuousCollisionFrameDisplacement,
                        targetRadius,
                        out _,
                        out _,
                        out _))
                {
                    total++;
                }
            }
        }

        return total;
    }

    public static int SweepPure2DDynamicRelativeTargets(GravitasWorldContext context, SwiftList<SolidBody2D> bodies)
    {
        int total = 0;
        context.Physics2D.PrepareContinuousCollisionFrame();
        for (int i = 0; i < bodies.Count; i++)
        {
            SolidBody2D source = bodies[i];
            Fixed64 sourceRadius = source.ResolveContinuousCollisionProxyRadius();
            SwiftList<int> candidates = context.Physics2D.QueryPlanarContinuousCollisionCandidates(
                DynamicCcdCandidateIndex2D.CreateSweptCircleBounds(
                    source.ContinuousCollisionFrameStart,
                    source.ContinuousCollisionFrameDisplacement,
                    sourceRadius));

            for (int j = 0; j < candidates.Count; j++)
            {
                int dynamicId = candidates[j];
                SolidBody2D target = context.Physics2D.GetContinuousCollisionCandidate(dynamicId);
                if (ReferenceEquals(source, target))
                {
                    continue;
                }

                Fixed64 targetRadius = target.ResolveContinuousCollisionProxyRadius();
                if (ContinuousCollisionMath.TrySweepRelativeCircles(
                        source.ContinuousCollisionFrameStart,
                        source.ContinuousCollisionFrameDisplacement,
                        sourceRadius,
                        target.ContinuousCollisionFrameStart,
                        target.ContinuousCollisionFrameDisplacement,
                        targetRadius,
                        out _,
                        out _,
                        out _))
                {
                    total++;
                }
            }
        }

        return total;
    }

    private static void ConfigureContext(GravitasWorldContext context)
    {
        context.SetFrameRate(1);
        context.Settings.GroundCheckLayerMask = PhysicsLayerMask.None;
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.MinSpeed = Fixed64.Zero;
        context.Environment.MaxSpeed = (Fixed64)16;
        context.Environment.MaxFallSpeed = (Fixed64)16;
    }

    private static void AddGrid(GravitasWorldContext context, int extentX, int extentZ)
    {
        if (!context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-16), (Fixed64)(-8), (Fixed64)(-16)),
                new Vector3d((Fixed64)extentX, (Fixed64)8, (Fixed64)extentZ)),
            out _))
        {
            throw new InvalidOperationException("Unable to create dynamic CCD benchmark grid.");
        }
    }
}
