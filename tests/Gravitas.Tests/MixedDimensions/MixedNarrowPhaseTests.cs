using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using SwiftCollections;
using SwiftCollections.Query;
using System;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed class MixedNarrowPhaseTests
{
    public static TheoryData<string, Func<GravitasWorldContext, LSCollider>, Func<GravitasWorldContext, LSCollider2D>> RemainingPrimitivePairs =>
        new()
        {
            {
                "Cuboid_Circle",
                context => CreateCuboid3D(context, new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero).Collider
            },
            {
                "Cuboid_AABox",
                context => CreateCuboid3D(context, new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)), Vector2d.Zero).Collider
            },
            {
                "Cuboid_Capsule",
                context => CreateCuboid3D(context, new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3), Vector2d.Zero).Collider
            },
            {
                "Cuboid_ConvexPolygon",
                context => CreateCuboid3D(
                    context,
                    new Vector3d(Fixed64.FromFraction(13, 10), Fixed64.Zero, Fixed64.Zero),
                    FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)45, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, CreateSquarePolygon(), Vector2d.Zero, FixedMath.DegToRad((Fixed64)45)).Collider
            },
            {
                "Capsule_Circle",
                context => CreateCapsule3D(context, new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero).Collider
            },
            {
                "Capsule_AABox",
                context => CreateCapsule3D(
                    context,
                    new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.Zero),
                    FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90)).Collider,
                context => CreateBody2D(context, new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)), Vector2d.Zero).Collider
            },
            {
                "Capsule_Capsule2D",
                context => CreateCapsule3D(context, new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3), Vector2d.Zero).Collider
            },
            {
                "Capsule_ConvexPolygon",
                context => CreateCapsule3D(context, new Vector3d(Fixed64.FromFraction(13, 10), Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, CreateSquarePolygon(), Vector2d.Zero, FixedMath.DegToRad((Fixed64)45)).Collider
            },
            {
                "Cylinder_Circle",
                context => CreateCylinder3D(context, new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero).Collider
            },
            {
                "Cylinder_AABox",
                context => CreateCylinder3D(
                    context,
                    new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.Zero),
                    FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90)).Collider,
                context => CreateBody2D(context, new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)), Vector2d.Zero).Collider
            },
            {
                "Cylinder_Capsule2D",
                context => CreateCylinder3D(context, new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3), Vector2d.Zero).Collider
            },
            {
                "Cylinder_ConvexPolygon",
                context => CreateCylinder3D(context, new Vector3d(Fixed64.FromFraction(13, 10), Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, CreateSquarePolygon(), Vector2d.Zero, FixedMath.DegToRad((Fixed64)45)).Collider
            },
            {
                "Cone_Circle",
                context => CreateCone3D(context, new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero).Collider
            },
            {
                "Cone_AABox",
                context => CreateCone3D(context, new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)), Vector2d.Zero).Collider
            },
            {
                "Cone_Capsule2D",
                context => CreateCone3D(context, new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3), Vector2d.Zero).Collider
            },
            {
                "Cone_ConvexPolygon",
                context => CreateCone3D(context, new Vector3d(Fixed64.FromFraction(13, 10), Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, CreateSquarePolygon(), Vector2d.Zero, FixedMath.DegToRad((Fixed64)45)).Collider
            }
        };

    public static TheoryData<string, Func<GravitasWorldContext, LSCollider>, Func<GravitasWorldContext, LSCollider2D>> SeparatedPrimitivePrismPairs =>
        new()
        {
            {
                "Cuboid_AABox",
                context => CreateCuboid3D(context, new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)), Vector2d.Zero).Collider
            },
            {
                "Cuboid_Capsule",
                context => CreateCuboid3D(context, new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3), Vector2d.Zero).Collider
            },
            {
                "Cuboid_ConvexPolygon",
                context => CreateCuboid3D(
                    context,
                    new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero),
                    FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)45, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, CreateSquarePolygon(), Vector2d.Zero, FixedMath.DegToRad((Fixed64)45)).Collider
            },
            {
                "Capsule_AABox",
                context => CreateCapsule3D(
                    context,
                    new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero),
                    FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90)).Collider,
                context => CreateBody2D(context, new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)), Vector2d.Zero).Collider
            },
            {
                "Capsule_Capsule2D",
                context => CreateCapsule3D(context, new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3), Vector2d.Zero).Collider
            },
            {
                "Capsule_ConvexPolygon",
                context => CreateCapsule3D(context, new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, CreateSquarePolygon(), Vector2d.Zero, FixedMath.DegToRad((Fixed64)45)).Collider
            },
            {
                "Cylinder_AABox",
                context => CreateCylinder3D(
                    context,
                    new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero),
                    FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90)).Collider,
                context => CreateBody2D(context, new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)), Vector2d.Zero).Collider
            },
            {
                "Cylinder_Capsule2D",
                context => CreateCylinder3D(context, new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3), Vector2d.Zero).Collider
            },
            {
                "Cylinder_ConvexPolygon",
                context => CreateCylinder3D(context, new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, CreateSquarePolygon(), Vector2d.Zero, FixedMath.DegToRad((Fixed64)45)).Collider
            },
            {
                "Cone_AABox",
                context => CreateCone3D(context, new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)), Vector2d.Zero).Collider
            },
            {
                "Cone_Capsule2D",
                context => CreateCone3D(context, new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3), Vector2d.Zero).Collider
            },
            {
                "Cone_ConvexPolygon",
                context => CreateCone3D(context, new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, CreateSquarePolygon(), Vector2d.Zero, FixedMath.DegToRad((Fixed64)45)).Collider
            }
        };

    public static TheoryData<string, Func<GravitasWorldContext, LSCollider>, Func<GravitasWorldContext, LSCollider2D>> VerticallySeparatedPrimitivePrismPairs =>
        new()
        {
            {
                "Cuboid_AABox",
                context => CreateCuboid3D(context, new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)), Vector2d.Zero).Collider
            },
            {
                "Capsule_AABox",
                context => CreateCapsule3D(context, new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)), Vector2d.Zero).Collider
            },
            {
                "Cylinder_AABox",
                context => CreateCylinder3D(context, new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)), Vector2d.Zero).Collider
            },
            {
                "Cone_AABox",
                context => CreateCone3D(context, new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero)).Collider,
                context => CreateBody2D(context, new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)), Vector2d.Zero).Collider
            }
        };

    public static TheoryData<string, Func<GravitasWorldContext, LSCollider>> CurvedCircleSlabExactMissPairs =>
        new()
        {
            { "Capsule", context => CreateCapsule3D(context, Vector3d.Zero).Collider },
            { "Cylinder", context => CreateCylinder3D(context, Vector3d.Zero).Collider },
            { "Cone", context => CreateCone3D(context, Vector3d.Zero).Collider }
        };

    public static TheoryData<string, Func<GravitasWorldContext, LSCollider>> CurvedPrismSlabExactMissPairs =>
        new()
        {
            {
                "Capsule",
                context => CreateCapsule3D(
                    context,
                    new Vector3d(Fixed64.FromFraction(29, 20), Fixed64.Zero, Fixed64.FromFraction(29, 20))).Collider
            },
            {
                "Cylinder",
                context => CreateCylinder3D(
                    context,
                    new Vector3d(Fixed64.FromFraction(29, 20), Fixed64.Zero, Fixed64.FromFraction(29, 20))).Collider
            },
            {
                "Cone",
                context => CreateCone3D(
                    context,
                    new Vector3d(Fixed64.FromFraction(29, 20), Fixed64.Zero, Fixed64.FromFraction(29, 20))).Collider
            }
        };

    public static TheoryData<string, ColliderType, Vector3d, FixedQuaternion, ColliderType2D, Fixed64> PrimitivePrismExactMissPairs =>
        new()
        {
            {
                "Cuboid_AABox",
                ColliderType.AABox,
                new Vector3d(Fixed64.FromFraction(-3, 2), -Fixed64.One, Fixed64.FromFraction(-3, 2)),
                Euler(0, 0, 15),
                ColliderType2D.AABox,
                Fixed64.Zero
            },
            {
                "Cuboid_Capsule2D",
                ColliderType.AABox,
                new Vector3d(-Fixed64.One, -Fixed64.One, (Fixed64)(-2)),
                FixedQuaternion.Identity,
                ColliderType2D.Capsule,
                Fixed64.Zero
            },
            {
                "Cuboid_Capsule2D_EdgeAxis",
                ColliderType.AABox,
                new Vector3d(Fixed64.One, Fixed64.FromFraction(4, 5), Fixed64.Half),
                Euler(15, 0, 15),
                ColliderType2D.Capsule,
                FixedMath.DegToRad((Fixed64)25)
            },
            {
                "Cuboid_ConvexPolygon",
                ColliderType.AABox,
                new Vector3d(Fixed64.FromFraction(-3, 2), -Fixed64.One, Fixed64.FromFraction(-3, 2)),
                FixedQuaternion.Identity,
                ColliderType2D.ConvexPolygon,
                FixedMath.DegToRad((Fixed64)15)
            },
            {
                "Cuboid_Circle_ClosestFeatureAxis",
                ColliderType.AABox,
                new Vector3d(Fixed64.FromFraction(9, 10), Fixed64.Zero, Fixed64.FromFraction(9, 10)),
                FixedQuaternion.Identity,
                ColliderType2D.Circle,
                Fixed64.Zero
            },
            {
                "Cuboid_Circle_EdgeUpAxis",
                ColliderType.AABox,
                new Vector3d(Fixed64.FromFraction(9, 10), Fixed64.Zero, Fixed64.FromFraction(9, 10)),
                Euler(15, 0, 15),
                ColliderType2D.Circle,
                Fixed64.Zero
            },
            {
                "Cuboid_AABox_EdgeUp",
                ColliderType.AABox,
                new Vector3d(Fixed64.FromFraction(-3, 2), -Fixed64.Half, Fixed64.FromFraction(3, 2)),
                Euler(0, 15, 75),
                ColliderType2D.AABox,
                Fixed64.Zero
            },
            {
                "Cuboid_AABox_EdgePrism",
                ColliderType.AABox,
                new Vector3d(Fixed64.FromFraction(-3, 2), Fixed64.One, -Fixed64.One),
                Euler(0, 15, 15),
                ColliderType2D.AABox,
                Fixed64.Zero
            },
            {
                "Capsule_AABox",
                ColliderType.Capsule,
                new Vector3d(Fixed64.FromFraction(-3, 2), (Fixed64)(-2), Fixed64.FromFraction(-3, 2)),
                FixedQuaternion.Identity,
                ColliderType2D.AABox,
                Fixed64.Zero
            },
            {
                "Capsule_Capsule2D",
                ColliderType.Capsule,
                new Vector3d(-Fixed64.One, (Fixed64)(-2), (Fixed64)(-2)),
                FixedQuaternion.Identity,
                ColliderType2D.Capsule,
                Fixed64.Zero
            },
            {
                "Capsule_ConvexPolygon",
                ColliderType.Capsule,
                new Vector3d(Fixed64.FromFraction(-3, 2), (Fixed64)(-2), Fixed64.FromFraction(-3, 2)),
                FixedQuaternion.Identity,
                ColliderType2D.ConvexPolygon,
                Fixed64.Zero
            },
            {
                "Capsule_AABox_Line",
                ColliderType.Capsule,
                new Vector3d(Fixed64.FromFraction(-3, 2), (Fixed64)2, Fixed64.FromFraction(-3, 2)),
                Euler(0, 0, 15),
                ColliderType2D.AABox,
                Fixed64.Zero
            },
            {
                "Capsule_AABox_LineUp",
                ColliderType.Capsule,
                new Vector3d((Fixed64)(-2), Fixed64.FromFraction(-3, 2), Fixed64.FromFraction(-3, 2)),
                Euler(0, 0, 45),
                ColliderType2D.AABox,
                Fixed64.Zero
            },
            {
                "Capsule_AABox_PrismNormal",
                ColliderType.Capsule,
                new Vector3d((Fixed64)(-2), Fixed64.FromFraction(-3, 2), -Fixed64.One),
                Euler(0, 15, 30),
                ColliderType2D.AABox,
                Fixed64.Zero
            },
            {
                "Capsule_ConvexPolygon_PrismEdge",
                ColliderType.Capsule,
                new Vector3d(Fixed64.FromFraction(-3, 2), -Fixed64.Half, Fixed64.FromFraction(7, 5)),
                Euler(0, 0, 30),
                ColliderType2D.ConvexPolygon,
                FixedMath.DegToRad((Fixed64)30)
            },
            {
                "Capsule_Circle_LineAxis",
                ColliderType.Capsule,
                new Vector3d(-Fixed64.FromFraction(8, 5), Fixed64.FromFraction(8, 5), Fixed64.Zero),
                Euler(0, 0, 45),
                ColliderType2D.Circle,
                Fixed64.Zero
            },
            {
                "Capsule_Capsule2D_FirstAxis",
                ColliderType.Capsule,
                new Vector3d(-Fixed64.FromFraction(29, 20), Fixed64.Zero, Fixed64.FromFraction(29, 20)),
                FixedQuaternion.Identity,
                ColliderType2D.Capsule,
                FixedMath.DegToRad((Fixed64)45)
            },
            {
                "Cylinder_AABox",
                ColliderType.Cylinder,
                new Vector3d(Fixed64.FromFraction(-3, 2), -Fixed64.One, Fixed64.FromFraction(-3, 2)),
                FixedQuaternion.Identity,
                ColliderType2D.AABox,
                Fixed64.Zero
            },
            {
                "Cylinder_Capsule2D",
                ColliderType.Cylinder,
                new Vector3d(-Fixed64.One, -Fixed64.One, (Fixed64)(-2)),
                FixedQuaternion.Identity,
                ColliderType2D.Capsule,
                Fixed64.Zero
            },
            {
                "Cylinder_ConvexPolygon",
                ColliderType.Cylinder,
                new Vector3d(Fixed64.FromFraction(-3, 2), -Fixed64.One, Fixed64.FromFraction(-3, 2)),
                FixedQuaternion.Identity,
                ColliderType2D.ConvexPolygon,
                Fixed64.Zero
            },
            {
                "Cylinder_AABox_LineUp",
                ColliderType.Cylinder,
                new Vector3d(Fixed64.FromFraction(-3, 2), -Fixed64.One, Fixed64.FromFraction(-3, 2)),
                Euler(0, 0, 15),
                ColliderType2D.AABox,
                Fixed64.Zero
            },
            {
                "Cylinder_AABox_Line",
                ColliderType.Cylinder,
                new Vector3d(Fixed64.FromFraction(-3, 2), Fixed64.One, Fixed64.FromFraction(-3, 2)),
                Euler(0, 0, 15),
                ColliderType2D.AABox,
                Fixed64.Zero
            },
            {
                "Cylinder_AABox_EdgePrism",
                ColliderType.Cylinder,
                new Vector3d(Fixed64.FromFraction(-3, 2), -Fixed64.One, -Fixed64.One),
                Euler(0, 0, 15),
                ColliderType2D.AABox,
                Fixed64.Zero
            },
            {
                "Cylinder_ConvexPolygon_PrismEdge",
                ColliderType.Cylinder,
                new Vector3d(Fixed64.FromFraction(-3, 2), Fixed64.Half, Fixed64.FromFraction(7, 5)),
                Euler(0, 0, 30),
                ColliderType2D.ConvexPolygon,
                FixedMath.DegToRad((Fixed64)30)
            },
            {
                "Cylinder_Circle_LineAxis",
                ColliderType.Cylinder,
                new Vector3d(-Fixed64.FromFraction(19, 20), Fixed64.FromFraction(19, 20), Fixed64.Zero),
                Euler(0, 0, 45),
                ColliderType2D.Circle,
                Fixed64.Zero
            },
            {
                "Cylinder_Circle_LineUpAxis",
                ColliderType.Cylinder,
                new Vector3d(-Fixed64.FromFraction(9, 10), Fixed64.Zero, -Fixed64.FromFraction(13, 20)),
                Euler(45, 0, 45),
                ColliderType2D.Circle,
                Fixed64.Zero
            },
            {
                "Cylinder_Capsule2D_FirstAxis",
                ColliderType.Cylinder,
                new Vector3d(-Fixed64.FromFraction(29, 20), Fixed64.Zero, Fixed64.FromFraction(29, 20)),
                FixedQuaternion.Identity,
                ColliderType2D.Capsule,
                FixedMath.DegToRad((Fixed64)45)
            },
            {
                "Cone_AABox",
                ColliderType.Cone,
                new Vector3d(Fixed64.FromFraction(-3, 2), -Fixed64.One, Fixed64.FromFraction(-3, 2)),
                FixedQuaternion.Identity,
                ColliderType2D.AABox,
                Fixed64.Zero
            },
            {
                "Cone_Capsule2D",
                ColliderType.Cone,
                new Vector3d(-Fixed64.One, -Fixed64.One, (Fixed64)(-2)),
                FixedQuaternion.Identity,
                ColliderType2D.Capsule,
                Fixed64.Zero
            },
            {
                "Cone_ConvexPolygon",
                ColliderType.Cone,
                new Vector3d(Fixed64.FromFraction(-3, 2), -Fixed64.One, Fixed64.FromFraction(-3, 2)),
                FixedQuaternion.Identity,
                ColliderType2D.ConvexPolygon,
                Fixed64.Zero
            },
            {
                "Cone_AABox_Axis",
                ColliderType.Cone,
                new Vector3d(Fixed64.FromFraction(-3, 2), Fixed64.One, Fixed64.FromFraction(-3, 2)),
                Euler(0, 0, 15),
                ColliderType2D.AABox,
                Fixed64.Zero
            },
            {
                "Cone_ConvexPolygon_PrismEdge",
                ColliderType.Cone,
                new Vector3d(Fixed64.FromFraction(-3, 2), Fixed64.Half, Fixed64.FromFraction(7, 5)),
                Euler(0, 0, 30),
                ColliderType2D.ConvexPolygon,
                FixedMath.DegToRad((Fixed64)30)
            },
            {
                "Cone_ConvexPolygon_EdgeAxis",
                ColliderType.Cone,
                new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One),
                Euler(45, 0, 45),
                ColliderType2D.ConvexPolygon,
                FixedMath.DegToRad((Fixed64)45)
            },
            {
                "Cone_Circle_Axis",
                ColliderType.Cone,
                new Vector3d(-Fixed64.FromFraction(19, 20), Fixed64.FromFraction(19, 20), Fixed64.Zero),
                Euler(0, 0, 45),
                ColliderType2D.Circle,
                Fixed64.Zero
            },
            {
                "Cone_Circle_AxisUp",
                ColliderType.Cone,
                new Vector3d(-Fixed64.FromFraction(9, 10), Fixed64.Zero, -Fixed64.FromFraction(13, 20)),
                Euler(45, 0, 45),
                ColliderType2D.Circle,
                Fixed64.Zero
            },
            {
                "Cone_Capsule2D_FirstAxis",
                ColliderType.Cone,
                new Vector3d(-Fixed64.FromFraction(29, 20), Fixed64.Zero, Fixed64.FromFraction(29, 20)),
                FixedQuaternion.Identity,
                ColliderType2D.Capsule,
                FixedMath.DegToRad((Fixed64)45)
            },
            {
                "Cuboid_Capsule2D_FirstAxis",
                ColliderType.AABox,
                new Vector3d(-Fixed64.FromFraction(29, 20), Fixed64.Zero, Fixed64.FromFraction(29, 20)),
                Euler(0, 45, 0),
                ColliderType2D.Capsule,
                FixedMath.DegToRad((Fixed64)45)
            }
        };

    public static TheoryData<string, Vector3d, Vector3d, Vector3d, ColliderType2D, Fixed64, bool> MeshTriangleReducerCases =>
        new()
        {
            {
                "TriangleNormal_CircleReject",
                new Vector3d(Fixed64.FromFraction(2, 5), -Fixed64.FromFraction(1, 4), Fixed64.Half),
                new Vector3d(Fixed64.Half, Fixed64.FromFraction(1, 4), Fixed64.FromFraction(2, 5)),
                new Vector3d(Fixed64.Half, -Fixed64.FromFraction(1, 4), Fixed64.FromFraction(2, 5)),
                ColliderType2D.Circle,
                Fixed64.Zero,
                false
            },
            {
                "TriangleNormal_AABoxReject",
                new Vector3d(Fixed64.FromFraction(9, 10), -Fixed64.FromFraction(1, 4), Fixed64.FromFraction(6, 5)),
                new Vector3d(Fixed64.FromFraction(6, 5), Fixed64.FromFraction(1, 4), Fixed64.FromFraction(9, 10)),
                new Vector3d(Fixed64.FromFraction(13, 10), -Fixed64.FromFraction(1, 4), Fixed64.FromFraction(4, 5)),
                ColliderType2D.AABox,
                Fixed64.Zero,
                false
            },
            {
                "EmbeddedCapsuleAxisReject",
                new Vector3d(-Fixed64.FromFraction(11, 10), Fixed64.Zero, Fixed64.FromFraction(11, 10)),
                new Vector3d(-Fixed64.FromFraction(6, 5), Fixed64.Zero, Fixed64.FromFraction(11, 10)),
                new Vector3d(-Fixed64.FromFraction(11, 10), Fixed64.Zero, Fixed64.FromFraction(6, 5)),
                ColliderType2D.Capsule,
                FixedMath.DegToRad((Fixed64)45),
                false
            },
            {
                "TriangleEdgeCapsuleAxisReject",
                new Vector3d(Fixed64.FromFraction(2, 5), Fixed64.FromFraction(7, 10), -Fixed64.One),
                new Vector3d(Fixed64.FromFraction(7, 10), Fixed64.FromFraction(2, 5), Fixed64.Zero),
                new Vector3d(Fixed64.FromFraction(4, 5), Fixed64.FromFraction(6, 5), Fixed64.Zero),
                ColliderType2D.Capsule,
                Fixed64.Zero,
                false
            },
            {
                "ClosestFeatureCapsuleReject",
                new Vector3d(Fixed64.FromFraction(2, 5), Fixed64.Zero, Fixed64.FromFraction(7, 5)),
                new Vector3d(Fixed64.FromFraction(22, 5), Fixed64.Zero, Fixed64.FromFraction(29, 10)),
                new Vector3d(Fixed64.FromFraction(2, 5), Fixed64.Zero, Fixed64.FromFraction(39, 10)),
                ColliderType2D.Capsule,
                Fixed64.Zero,
                false
            },
            {
                "CircleSegmentPiercesTriangle",
                new Vector3d(-Fixed64.One, Fixed64.Zero, -Fixed64.One),
                new Vector3d(Fixed64.One, Fixed64.Zero, -Fixed64.One),
                new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.One),
                ColliderType2D.Circle,
                Fixed64.Zero,
                true
            },
            {
                "CircleSegmentEdgeImprovesClosestPair",
                new Vector3d(Fixed64.FromFraction(1, 10), Fixed64.Zero, Fixed64.FromFraction(1, 10)),
                new Vector3d(Fixed64.FromFraction(2, 5), Fixed64.Zero, Fixed64.FromFraction(1, 10)),
                new Vector3d(Fixed64.FromFraction(1, 10), Fixed64.Zero, Fixed64.FromFraction(2, 5)),
                ColliderType2D.Circle,
                Fixed64.Zero,
                true
            }
        };

    [Fact]
    public void SphereCircleSlab_WithPlanarOverlap_ShouldReportDeterministicContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(
            context,
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D circle = CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(sphere.Collider, circle.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.HasContact.Should().BeTrue();
        contact.Normal3DTo2D.Should().Be(-Vector3d.Right);
        contact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        contact.Point3D.Should().Be(new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero));
        contact.Point2D.Should().Be(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
    }

    [Fact]
    public void SphereCustom2DSlab_WithDegenerateClosestPoint_ShouldUseDeterministicFallbackNormal()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D custom = CreateBody2D(context, new UnsupportedTestCollider2D(), Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(sphere.Collider, custom.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.HasContact.Should().BeTrue();
        contact.Normal3DTo2D.Should().Be(Vector3d.Right);
        contact.Point3D.Should().Be(Vector3d.Right * Fixed64.Half);
        contact.Point2D.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void SphereCustom2DSlab_WithDegenerateClosestPointAndOffsetCenter_ShouldUseCenterFallbackNormal()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D custom = CreateBody2D(
            context,
            new UnsupportedTestCollider2D(),
            new Vector2d(Fixed64.Zero, Fixed64.One));

        bool collided = CollisionDetectionMixed.TryCollide(sphere.Collider, custom.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.HasContact.Should().BeTrue();
        contact.Normal3DTo2D.Should().Be(Vector3d.Forward);
        contact.Point3D.Should().Be(Vector3d.Forward * Fixed64.Half);
        contact.Point2D.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void SphereCustomContaining2DSlab_WithoutPlanarBoundary_ShouldRetainVerticalBoundary()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D custom = CreateBody2D(
            context,
            new UnsupportedTestCollider2D(containsPoints: true),
            Vector2d.Zero);

        custom.Collider.ContainsPoint(Vector2d.Zero).Should().BeTrue();
        MixedEmbedded2DGeometry.TryGetPlanarBoundaryPoint(
            custom.Collider,
            Vector2d.Zero,
            out _,
            out _).Should().BeFalse();

        CollisionDetectionMixed.TryCollide(
            sphere.Collider,
            custom.Collider,
            out MixedContact contact).Should().BeTrue();

        contact.Normal3DTo2D.Should().Be(-Vector3d.Up);
        contact.Point3D.Should().Be(new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Zero));
        contact.Point2D.Should().Be(new Vector3d(Fixed64.Zero, -Fixed64.Half, Fixed64.Zero));
        contact.Depth.Should().Be(Fixed64.One);
    }

    [Fact]
    public void Unsupported3DSlab_WithOverlappingBounds_ShouldReturnNoContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<UnsupportedTestCollider3D> unsupported = CreateBody3D(
            context,
            new UnsupportedTestCollider3D(),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        SolidBody2D circle = CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(unsupported.Collider, circle.Collider, out MixedContact contact);

        collided.Should().BeFalse();
        contact.HasContact.Should().BeFalse();
    }

    [Fact]
    public void SphereCircleSlab_WithSeparatedYSlab_ShouldNotCollide()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero));
        SolidBody2D circle = CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero);

        CollisionDetectionMixed.TryCollide(sphere.Collider, circle.Collider, out MixedContact contact).Should().BeFalse();
        contact.HasContact.Should().BeFalse();
    }

    [Fact]
    public void SphereCapsuleSlab_WithPlanarSideOverlap_ShouldReportContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(
            context,
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D capsule = CreateBody2D(context, new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3), Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(sphere.Collider, capsule.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.HasContact.Should().BeTrue();
        contact.Normal3DTo2D.Should().Be(-Vector3d.Right);
        contact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        contact.Point3D.X.Should().Be(Fixed64.FromFraction(1, 4));
        contact.Point2D.X.Should().Be(Fixed64.Half);
    }

    [Fact]
    public void SphereAABoxSlab_WithTouchingFace_ShouldReportZeroDepthContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(
            context,
            new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D box = CreateBody2D(context, new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)), Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(sphere.Collider, box.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.Zero);
        contact.Normal3DTo2D.Should().Be(-Vector3d.Right);
        contact.Point3D.Should().Be(new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero));
        contact.Point2D.Should().Be(new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero));
    }

    [Fact]
    public void SphereAABoxSlab_FromInsideWithPlanarSideCloserThanCap_ShouldUsePlanarNormal()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(
            context,
            new Vector3d(Fixed64.FromFraction(9, 10), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D box = CreateBody2D(context, new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)), Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(sphere.Collider, box.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.HasContact.Should().BeTrue();
        contact.Normal3DTo2D.Should().Be(Vector3d.Right);
        contact.Point2D.Should().Be(new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero));
        contact.Depth.Should().Be(Fixed64.FromFraction(3, 5));
    }

    [Fact]
    public void SphereAABoxSlab_FromInsideWithTopCapCloserThanPlanarSide_ShouldUseVerticalNormal()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.FromFraction(9, 20), Fixed64.Zero));
        SolidBody2D box = CreateBody2D(context, new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)), Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(sphere.Collider, box.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.HasContact.Should().BeTrue();
        contact.Normal3DTo2D.Should().Be(Vector3d.Up);
        contact.Point2D.Should().Be(new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Zero));
        contact.Depth.Should().Be(Fixed64.FromFraction(11, 20));
    }

    [Fact]
    public void SphereConvexPolygonSlab_WithCornerOverlap_ShouldReportContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(
            context,
            new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)2));
        var polygon = new LSPolygonCollider2D(
            new Vector2d(-Fixed64.One, -Fixed64.One),
            new Vector2d(Fixed64.One, -Fixed64.One),
            new Vector2d(Fixed64.One, Fixed64.One),
            new Vector2d(-Fixed64.One, Fixed64.One));
        SolidBody2D polygonBody = CreateBody2D(context, polygon, Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(sphere.Collider, polygonBody.Collider, out MixedContact contact);

        collided.Should().BeFalse();

        ScenarioBody<LSSphereCollider> overlappingSphere = CreateSphere3D(
            context,
            new Vector3d(Fixed64.FromFraction(13, 10), Fixed64.Zero, Fixed64.FromFraction(13, 10)));

        CollisionDetectionMixed.TryCollide(overlappingSphere.Collider, polygonBody.Collider, out contact).Should().BeTrue();
        contact.Depth.Should().BeGreaterThan(Fixed64.Zero);
        contact.Point2D.X.Should().Be(Fixed64.One);
        contact.Point2D.Z.Should().Be(Fixed64.One);
    }

    [Theory]
    [MemberData(nameof(RemainingPrimitivePairs))]
    public void RemainingPrimitivePairs_WithRepresentativeOverlap_ShouldReportContact(
        string caseName,
        Func<GravitasWorldContext, LSCollider> create3D,
        Func<GravitasWorldContext, LSCollider2D> create2D)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider collider3D = create3D(context);
        LSCollider2D collider2D = create2D(context);

        bool collided = CollisionDetectionMixed.TryCollide(collider3D, collider2D, out MixedContact contact);

        collided.Should().BeTrue(caseName);
        contact.HasContact.Should().BeTrue();
        contact.Depth.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        contact.Normal3DTo2D.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Theory]
    [MemberData(nameof(SeparatedPrimitivePrismPairs))]
    public void PrimitivePrismPairs_WithPlanarSeparation_ShouldNotReportContact(
        string caseName,
        Func<GravitasWorldContext, LSCollider> create3D,
        Func<GravitasWorldContext, LSCollider2D> create2D)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider collider3D = create3D(context);
        LSCollider2D collider2D = create2D(context);

        bool collided = CollisionDetectionMixed.TryCollide(collider3D, collider2D, out MixedContact contact);

        collided.Should().BeFalse(caseName);
        contact.HasContact.Should().BeFalse();
    }

    [Theory]
    [MemberData(nameof(VerticallySeparatedPrimitivePrismPairs))]
    public void PrimitivePrismPairs_WithVerticalSlabSeparation_ShouldNotReportContact(
        string caseName,
        Func<GravitasWorldContext, LSCollider> create3D,
        Func<GravitasWorldContext, LSCollider2D> create2D)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider collider3D = create3D(context);
        LSCollider2D collider2D = create2D(context);

        bool collided = CollisionDetectionMixed.TryCollide(collider3D, collider2D, out MixedContact contact);

        collided.Should().BeFalse(caseName);
        contact.HasContact.Should().BeFalse();
    }

    [Fact]
    public void CuboidConvexPolygonSlab_WithOverlappingBoundsButSeparatingObliqueAxis_ShouldNotReportContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCuboidCollider> cuboid = CreateCuboid3D(
            context,
            new Vector3d(Fixed64.FromFraction(29, 20), Fixed64.Zero, Fixed64.FromFraction(29, 20)),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)45, Fixed64.Zero));
        SolidBody2D polygon = CreateBody2D(context, CreateSquarePolygon(), Vector2d.Zero, FixedMath.DegToRad((Fixed64)45));

        bool collided = CollisionDetectionMixed.TryCollide(cuboid.Collider, polygon.Collider, out MixedContact contact);

        collided.Should().BeFalse();
        contact.HasContact.Should().BeFalse();
    }

    [Fact]
    public void CuboidAABoxSlab_WithOverlappingBoundsButRotatedCornerSeparation_ShouldNotReportContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCuboidCollider> cuboid = CreateCuboid3D(
            context,
            new Vector3d(Fixed64.FromFraction(33, 20), Fixed64.Zero, Fixed64.FromFraction(33, 20)),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)45, Fixed64.Zero));
        SolidBody2D box = CreateBody2D(
            context,
            new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)),
            Vector2d.Zero);

        cuboid.Collider.Bounds.Intersects(box.Collider.MixedBounds3D).Should().BeTrue();

        bool collided = CollisionDetectionMixed.TryCollide(cuboid.Collider, box.Collider, out MixedContact contact);

        collided.Should().BeFalse();
        contact.HasContact.Should().BeFalse();
    }

    [Fact]
    public void CuboidAABoxSlab_WithTouchingFace_ShouldReportZeroDepthContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCuboidCollider> cuboid = CreateCuboid3D(
            context,
            new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D box = CreateBody2D(context, new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)), Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(cuboid.Collider, box.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.Zero);
        contact.Normal3DTo2D.Should().Be(-Vector3d.Right);
        contact.Point3D.X.Should().Be(Fixed64.One);
        contact.Point2D.X.Should().Be(Fixed64.One);
    }

    [Fact]
    public void CylinderAABoxSlab_WithStartingOverlap_ShouldReturnStableContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCylinderCollider> cylinder = CreateCylinder3D(context, Vector3d.Zero);
        SolidBody2D box = CreateBody2D(context, new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)), Vector2d.Zero);

        bool firstCollided = CollisionDetectionMixed.TryCollide(cylinder.Collider, box.Collider, out MixedContact first);
        bool secondCollided = CollisionDetectionMixed.TryCollide(cylinder.Collider, box.Collider, out MixedContact second);

        firstCollided.Should().BeTrue();
        secondCollided.Should().BeTrue();
        first.HasContact.Should().BeTrue();
        first.Depth.Should().BeGreaterThan(Fixed64.Zero);
        first.Normal3DTo2D.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
        second.Point3D.Should().Be(first.Point3D);
        second.Point2D.Should().Be(first.Point2D);
        second.Normal3DTo2D.Should().Be(first.Normal3DTo2D);
        second.Depth.Should().Be(first.Depth);
    }

    [Fact]
    public void CylinderCircleSlab_WithPlanarRimOverlap_ShouldReportFiniteCylinderContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCylinderCollider> cylinder = CreateCylinder3D(
            context,
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D circle = CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(cylinder.Collider, circle.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.HasContact.Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        contact.Normal3DTo2D.Should().Be(-Vector3d.Right);
        contact.Point3D.X.Should().Be(Fixed64.FromFraction(1, 4));
        contact.Point2D.X.Should().Be(Fixed64.Half);
    }

    [Fact]
    public void ConeCircleSlab_WithBaseRimOverlap_ShouldReportFiniteConeContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSConeCollider> cone = CreateCone3D(
            context,
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D circle = CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(cone.Collider, circle.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.HasContact.Should().BeTrue();
        contact.Depth.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        contact.Normal3DTo2D.Should().Be(-Vector3d.Right);
        contact.Point3D.X.Should().BeLessThan(cone.Collider.Center.X);
        contact.Point2D.X.Should().Be(Fixed64.Half);
    }

    [Fact]
    public void ConeCircleSlab_WithSeparatedYSlab_ShouldNotCollide()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSConeCollider> cone = CreateCone3D(
            context,
            new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero));
        SolidBody2D circle = CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero);

        CollisionDetectionMixed.TryCollide(cone.Collider, circle.Collider, out MixedContact contact).Should().BeFalse();
        contact.HasContact.Should().BeFalse();
    }

    [Theory]
    [InlineData(ColliderType2D.AABox, 0)]
    [InlineData(ColliderType2D.Capsule, 0)]
    [InlineData(ColliderType2D.ConvexPolygon, 45)]
    public void ConePrismSlab_WithEmbeddedPrismOverlap_ShouldReportFiniteConeContact(
        ColliderType2D prismShape,
        int prismRotationDegrees)
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSConeCollider> cone = CreateCone3D(
            context,
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        LSCollider2D prism = CreatePrimitive2D(
            context,
            prismShape,
            FixedMath.DegToRad((Fixed64)prismRotationDegrees));

        bool collided = CollisionDetectionMixed.TryCollide(cone.Collider, prism, out MixedContact contact);

        collided.Should().BeTrue();
        contact.HasContact.Should().BeTrue();
        contact.Depth.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        contact.Normal3DTo2D.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Theory]
    [InlineData(ColliderType.AABox)]
    [InlineData(ColliderType.Capsule)]
    [InlineData(ColliderType.Cylinder)]
    [InlineData(ColliderType.Cone)]
    public void PrimitiveDegenerateCapsule2DSlab_WithRepresentativeOverlap_ShouldReportContact(ColliderType shape3D)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider collider3D = CreatePrimitive3D(
            context,
            shape3D,
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity);
        SolidBody2D capsule2D = CreateBody2D(
            context,
            new LSCapsuleCollider2D(Fixed64.Half, Fixed64.One),
            Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(collider3D, capsule2D.Collider, out MixedContact contact);

        collided.Should().BeTrue(shape3D.ToString());
        contact.HasContact.Should().BeTrue();
        contact.Depth.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        contact.Normal3DTo2D.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void CapsuleCircleSlab_WithPlanarSideOverlap_ShouldReportContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCapsuleCollider> capsule = CreateCapsule3D(
            context,
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D circle = CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(capsule.Collider, circle.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        contact.Normal3DTo2D.Should().Be(-Vector3d.Right);
        contact.Point3D.X.Should().Be(Fixed64.FromFraction(1, 4));
        contact.Point2D.X.Should().Be(Fixed64.Half);
    }

    [Fact]
    public void DegenerateCapsuleCircleSlab_WithPlanarOverlap_ShouldReportStableContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var collider = new LSCapsuleCollider
        {
            Size = Vector3d.One
        };
        ScenarioBody<LSCapsuleCollider> capsule = CreateBody3D(
            context,
            collider,
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity);
        SolidBody2D circle = CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(capsule.Collider, circle.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.HasContact.Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        contact.Normal3DTo2D.Should().Be(-Vector3d.Right);
    }

    [Theory]
    [MemberData(nameof(CurvedCircleSlabExactMissPairs))]
    public void CurvedCircleSlab_WithOverlappingBoundsButDiagonalSeparation_ShouldRejectExactContact(
        string _,
        Func<GravitasWorldContext, LSCollider> createCollider3D)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider collider3D = createCollider3D(context);
        SolidBody2D circle = CreateBody2D(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            new Vector2d(Fixed64.FromFraction(9, 10), Fixed64.FromFraction(9, 10)));

        collider3D.Bounds.Intersects(circle.Collider.MixedBounds3D).Should().BeTrue();

        CollisionDetectionMixed.TryCollide(collider3D, circle.Collider, out MixedContact contact).Should().BeFalse();
        contact.HasContact.Should().BeFalse();
    }

    [Theory]
    [MemberData(nameof(CurvedPrismSlabExactMissPairs))]
    public void CurvedAABoxSlab_WithOverlappingBoundsButCornerSeparation_ShouldRejectExactContact(
        string caseName,
        Func<GravitasWorldContext, LSCollider> createCollider3D)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider collider3D = createCollider3D(context);
        SolidBody2D box = CreateBody2D(
            context,
            new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)),
            Vector2d.Zero);

        collider3D.Bounds.Intersects(box.Collider.MixedBounds3D).Should().BeTrue(caseName);

        CollisionDetectionMixed.TryCollide(collider3D, box.Collider, out MixedContact contact).Should().BeFalse(caseName);
        contact.HasContact.Should().BeFalse();
    }

    [Theory]
    [MemberData(nameof(PrimitivePrismExactMissPairs))]
    public void PrimitivePrismPairs_WithOverlappingBoundsButExactAxisSeparation_ShouldRejectContact(
        string caseName,
        ColliderType shape3D,
        Vector3d position3D,
        FixedQuaternion rotation3D,
        ColliderType2D shape2D,
        Fixed64 rotation2D)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider collider3D = CreatePrimitive3D(context, shape3D, position3D, rotation3D);
        LSCollider2D collider2D = CreatePrimitive2D(context, shape2D, rotation2D);

        collider3D.Bounds.Intersects(collider2D.MixedBounds3D).Should().BeTrue(caseName);

        CollisionDetectionMixed.TryCollide(collider3D, collider2D, out MixedContact contact).Should().BeFalse(caseName);
        contact.HasContact.Should().BeFalse();
    }

    [Fact]
    public void PrimitivePrismPairs_WithRotatedOrientationMatrix_ShouldClassifyContactsDeterministically()
    {
        var cases = new[]
        {
            new MixedPrimitiveCase(
                "CuboidCircle_FaceOverlap",
                ColliderType.AABox,
                ColliderType2D.Circle,
                new Vector3d(Fixed64.FromFraction(7, 10), Fixed64.Zero, Fixed64.Zero),
                Euler(0, 35, 0),
                Fixed64.Zero,
                true),
            new MixedPrimitiveCase(
                "CuboidCircle_EdgeAxisSeparation",
                ColliderType.AABox,
                ColliderType2D.Circle,
                new Vector3d(Fixed64.FromFraction(31, 20), Fixed64.Zero, Fixed64.FromFraction(31, 20)),
                Euler(0, 45, 0),
                Fixed64.Zero,
                false),
            new MixedPrimitiveCase(
                "CapsuleCircle_SideOverlap",
                ColliderType.Capsule,
                ColliderType2D.Circle,
                new Vector3d(Fixed64.FromFraction(7, 10), Fixed64.Zero, Fixed64.Zero),
                Euler(0, 0, 35),
                Fixed64.Zero,
                true),
            new MixedPrimitiveCase(
                "CapsuleCircle_CrossAxisSeparation",
                ColliderType.Capsule,
                ColliderType2D.Circle,
                new Vector3d(Fixed64.FromFraction(8, 5), Fixed64.Zero, Fixed64.FromFraction(8, 5)),
                Euler(0, 0, 45),
                Fixed64.Zero,
                false),
            new MixedPrimitiveCase(
                "CylinderCircle_RimOverlap",
                ColliderType.Cylinder,
                ColliderType2D.Circle,
                new Vector3d(Fixed64.FromFraction(7, 10), Fixed64.Zero, Fixed64.Zero),
                Euler(0, 0, 20),
                Fixed64.Zero,
                true),
            new MixedPrimitiveCase(
                "CylinderCircle_CrossAxisSeparation",
                ColliderType.Cylinder,
                ColliderType2D.Circle,
                new Vector3d(Fixed64.FromFraction(8, 5), Fixed64.Zero, Fixed64.FromFraction(8, 5)),
                Euler(0, 0, 45),
                Fixed64.Zero,
                false),
            new MixedPrimitiveCase(
                "ConeCircle_BaseOverlap",
                ColliderType.Cone,
                ColliderType2D.Circle,
                new Vector3d(Fixed64.FromFraction(7, 10), Fixed64.Zero, Fixed64.Zero),
                Euler(0, 0, 20),
                Fixed64.Zero,
                true),
            new MixedPrimitiveCase(
                "ConeCircle_CrossAxisSeparation",
                ColliderType.Cone,
                ColliderType2D.Circle,
                new Vector3d(Fixed64.FromFraction(8, 5), Fixed64.Zero, Fixed64.FromFraction(8, 5)),
                Euler(0, 0, 45),
                Fixed64.Zero,
                false),
            new MixedPrimitiveCase(
                "CuboidCapsule_EdgeOverlap",
                ColliderType.AABox,
                ColliderType2D.Capsule,
                new Vector3d(Fixed64.FromFraction(7, 10), Fixed64.Zero, Fixed64.FromFraction(3, 10)),
                Euler(0, 35, 10),
                FixedMath.DegToRad((Fixed64)25),
                true),
            new MixedPrimitiveCase(
                "CuboidCapsule_EdgeAxisSeparation",
                ColliderType.AABox,
                ColliderType2D.Capsule,
                new Vector3d(Fixed64.FromFraction(33, 20), Fixed64.Zero, Fixed64.FromFraction(33, 20)),
                Euler(0, 45, 10),
                FixedMath.DegToRad((Fixed64)25),
                false),
            new MixedPrimitiveCase(
                "CapsuleCapsule_EdgeOverlap",
                ColliderType.Capsule,
                ColliderType2D.Capsule,
                new Vector3d(Fixed64.FromFraction(7, 10), Fixed64.Zero, Fixed64.Zero),
                Euler(0, 0, 35),
                FixedMath.DegToRad((Fixed64)25),
                true),
            new MixedPrimitiveCase(
                "CapsuleCapsule_EdgeAxisSeparation",
                ColliderType.Capsule,
                ColliderType2D.Capsule,
                new Vector3d(Fixed64.FromFraction(8, 5), Fixed64.Zero, Fixed64.FromFraction(8, 5)),
                Euler(0, 0, 45),
                FixedMath.DegToRad((Fixed64)25),
                false),
            new MixedPrimitiveCase(
                "CylinderCapsule_EdgeOverlap",
                ColliderType.Cylinder,
                ColliderType2D.Capsule,
                new Vector3d(Fixed64.FromFraction(7, 10), Fixed64.Zero, Fixed64.Zero),
                Euler(0, 0, 35),
                FixedMath.DegToRad((Fixed64)25),
                true),
            new MixedPrimitiveCase(
                "CylinderCapsule_EdgeAxisSeparation",
                ColliderType.Cylinder,
                ColliderType2D.Capsule,
                new Vector3d(Fixed64.FromFraction(8, 5), Fixed64.Zero, Fixed64.FromFraction(8, 5)),
                Euler(0, 0, 45),
                FixedMath.DegToRad((Fixed64)25),
                false),
            new MixedPrimitiveCase(
                "ConeCapsule_EdgeOverlap",
                ColliderType.Cone,
                ColliderType2D.Capsule,
                new Vector3d(Fixed64.FromFraction(7, 10), Fixed64.Zero, Fixed64.Zero),
                Euler(0, 0, 35),
                FixedMath.DegToRad((Fixed64)25),
                true),
            new MixedPrimitiveCase(
                "ConeCapsule_EdgeAxisSeparation",
                ColliderType.Cone,
                ColliderType2D.Capsule,
                new Vector3d(Fixed64.FromFraction(8, 5), Fixed64.Zero, Fixed64.FromFraction(8, 5)),
                Euler(0, 0, 45),
                FixedMath.DegToRad((Fixed64)25),
                false)
        };

        foreach (MixedPrimitiveCase testCase in cases)
        {
            using GravitasWorldContext context = CreateMixedContext();
            LSCollider collider3D = CreatePrimitive3D(context, testCase.Shape3D, testCase.Position3D, testCase.Rotation3D);
            LSCollider2D collider2D = testCase.Shape2D == ColliderType2D.Circle
                ? CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, testCase.Rotation2D).Collider
                : CreatePrimitive2D(context, testCase.Shape2D, testCase.Rotation2D);

            bool collided = CollisionDetectionMixed.TryCollide(collider3D, collider2D, out MixedContact contact);

            collided.Should().Be(testCase.ExpectedContact, testCase.Name);
            contact.HasContact.Should().Be(testCase.ExpectedContact, testCase.Name);
            if (testCase.ExpectedContact)
            {
                contact.Depth.Should().BeGreaterThanOrEqualTo(Fixed64.Zero, testCase.Name);
                contact.Normal3DTo2D.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero, testCase.Name);
            }
        }
    }

    [Fact]
    public void PrimitivePrismPairs_WithDeterministicPoseSweep_ShouldRemainStableAndProduceValidContacts()
    {
        ColliderType[] shapes3D =
        {
            ColliderType.AABox,
            ColliderType.Capsule,
            ColliderType.Cylinder,
            ColliderType.Cone
        };
        ColliderType2D[] shapes2D =
        {
            ColliderType2D.Circle,
            ColliderType2D.AABox,
            ColliderType2D.Capsule,
            ColliderType2D.ConvexPolygon
        };
        Vector3d[] positions =
        {
            new(Fixed64.FromFraction(3, 5), Fixed64.Zero, Fixed64.Zero),
            new(Fixed64.FromFraction(7, 5), Fixed64.Zero, Fixed64.Zero),
            new(Fixed64.FromFraction(8, 5), Fixed64.Zero, Fixed64.FromFraction(8, 5)),
            new(Fixed64.Zero, Fixed64.FromFraction(7, 10), Fixed64.FromFraction(7, 10)),
            new(Fixed64.FromFraction(7, 10), Fixed64.FromFraction(7, 10), Fixed64.Zero),
            new(Fixed64.FromFraction(9, 4), Fixed64.Zero, Fixed64.Zero),
            new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromFraction(9, 4)),
            new(Fixed64.Zero, Fixed64.FromFraction(5, 4), Fixed64.Zero),
            new(Fixed64.FromFraction(5, 4), Fixed64.Zero, -Fixed64.FromFraction(5, 4)),
            new(-Fixed64.FromFraction(5, 4), Fixed64.Zero, Fixed64.FromFraction(5, 4))
        };
        FixedQuaternion[] rotations3D =
        {
            FixedQuaternion.Identity,
            Euler(0, 35, 15),
            Euler(20, 0, 65)
        };
        Fixed64[] rotations2D =
        {
            Fixed64.Zero,
            FixedMath.DegToRad((Fixed64)25),
            FixedMath.DegToRad((Fixed64)60)
        };

        int hits = 0;
        int misses = 0;
        foreach (ColliderType shape3D in shapes3D)
        {
            foreach (ColliderType2D shape2D in shapes2D)
            {
                foreach (Vector3d position in positions)
                {
                    foreach (FixedQuaternion rotation3D in rotations3D)
                    {
                        foreach (Fixed64 rotation2D in rotations2D)
                        {
                            using GravitasWorldContext context = CreateMixedContext();
                            LSCollider collider3D = CreatePrimitive3D(context, shape3D, position, rotation3D);
                            LSCollider2D collider2D = shape2D == ColliderType2D.Circle
                                ? CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, rotation2D).Collider
                                : CreatePrimitive2D(context, shape2D, rotation2D);

                            bool first = CollisionDetectionMixed.TryCollide(collider3D, collider2D, out MixedContact firstContact);
                            bool second = CollisionDetectionMixed.TryCollide(collider3D, collider2D, out MixedContact secondContact);

                            second.Should().Be(first);
                            secondContact.HasContact.Should().Be(firstContact.HasContact);
                            if (first)
                            {
                                hits++;
                                firstContact.HasContact.Should().BeTrue();
                                firstContact.Depth.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
                                firstContact.Normal3DTo2D.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
                                secondContact.Depth.Should().Be(firstContact.Depth);
                                secondContact.Normal3DTo2D.Should().Be(firstContact.Normal3DTo2D);
                            }
                            else
                            {
                                misses++;
                                firstContact.HasContact.Should().BeFalse();
                                secondContact.HasContact.Should().BeFalse();
                            }
                        }
                    }
                }
            }
        }

        hits.Should().BeGreaterThan(0);
        misses.Should().BeGreaterThan(0);
    }

    [Fact]
    public void CylinderConvexPolygonSlab_WithSeparatedYSlab_ShouldNotCollide()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCylinderCollider> cylinder = CreateCylinder3D(
            context,
            new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero));
        SolidBody2D polygon = CreateBody2D(context, CreateSquarePolygon(), Vector2d.Zero, FixedMath.DegToRad((Fixed64)45));

        CollisionDetectionMixed.TryCollide(cylinder.Collider, polygon.Collider, out MixedContact contact).Should().BeFalse();
        contact.HasContact.Should().BeFalse();
    }

    [Fact]
    public void CompoundCircleSlab_WithSeparatedFirstPart_ShouldUseLaterPartContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCompoundCollider> compound = CreateCompound3D(context, Vector3d.Zero);
        SolidBody2D circle = CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(compound.Collider, circle.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.HasContact.Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        contact.Normal3DTo2D.Should().Be(-Vector3d.Right);
        contact.Point3D.X.Should().Be(Fixed64.FromFraction(1, 4));
        contact.Point2D.X.Should().Be(Fixed64.Half);
    }

    [Fact]
    public void CompoundCircleSlab_WithShallowerFirstOverlaps_ShouldRetainFirstPartContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var compoundCollider = new LSCompoundCollider(
            CompoundColliderPart.Sphere(
                Fixed64.Half,
                new Vector3d(Fixed64.FromFraction(9, 10), Fixed64.Zero, Fixed64.Zero)),
            CompoundColliderPart.Sphere(
                Fixed64.Half,
                new Vector3d(Fixed64.FromFraction(3, 5), Fixed64.Zero, Fixed64.Zero)));
        ScenarioBody<LSCompoundCollider> compound = CreateBody3D(
            context,
            compoundCollider,
            Vector3d.Zero,
            FixedQuaternion.Identity);
        SolidBody2D circle = CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero);

        CollisionDetectionMixed.TryCollide(
            compound.Collider.GetPartCollider(0),
            circle.Collider,
            out MixedContact shallow).Should().BeTrue();
        CollisionDetectionMixed.TryCollide(
            compound.Collider.GetPartCollider(1),
            circle.Collider,
            out MixedContact deep).Should().BeTrue();

        shallow.Depth.Should().Be(Fixed64.FromFraction(1, 10));
        deep.Depth.Should().Be(Fixed64.FromFraction(2, 5));
        shallow.Point3D.Should().NotBe(deep.Point3D);

        CollisionDetectionMixed.TryCollide(
            compound.Collider,
            circle.Collider,
            out MixedContact contact).Should().BeTrue();

        contact.Depth.Should().Be(shallow.Depth);
        contact.Normal3DTo2D.Should().Be(shallow.Normal3DTo2D);
        contact.Point3D.Should().Be(shallow.Point3D);
    }

    [Fact]
    public void CompoundCircleSlab_WithOnlyExactMissParts_ShouldReturnNoContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var compoundCollider = new LSCompoundCollider(
            CompoundColliderPart.Sphere(
                Fixed64.Half,
                new Vector3d(Fixed64.FromFraction(9, 10), Fixed64.Zero, Fixed64.FromFraction(9, 10))),
            CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero)));
        ScenarioBody<LSCompoundCollider> compound = CreateBody3D(
            context,
            compoundCollider,
            Vector3d.Zero,
            FixedQuaternion.Identity);
        SolidBody2D circle = CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero);

        compound.Collider.GetPartCollider(0).Bounds.Intersects(circle.Collider.MixedBounds3D).Should().BeTrue();
        compound.Collider.GetPartCollider(1).Bounds.Intersects(circle.Collider.MixedBounds3D).Should().BeFalse();

        bool collided = CollisionDetectionMixed.TryCollide(compound.Collider, circle.Collider, out MixedContact contact);

        collided.Should().BeFalse();
        contact.HasContact.Should().BeFalse();
    }

    [Fact]
    public void SphereCompound2DSlab_WithSeparatedFirstPart_ShouldUseLaterPartContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(
            context,
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        var compound2D = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)(-4), Fixed64.Zero)),
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero));
        SolidBody2D body2D = CreateBody2D(context, compound2D, Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(sphere.Collider, body2D.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.HasContact.Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        contact.Normal3DTo2D.Should().Be(-Vector3d.Right);
        body2D.Collider.Should().BeSameAs(compound2D);
    }

    [Fact]
    public void SphereCompound2DSlab_WithOnlyExactMissParts_ShouldReturnNoContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(context, Vector3d.Zero);
        var compound2D = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(
                Fixed64.Half,
                new Vector2d(Fixed64.FromFraction(9, 10), Fixed64.FromFraction(9, 10))),
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)4, Fixed64.Zero)));
        SolidBody2D body2D = CreateBody2D(context, compound2D, Vector2d.Zero);

        sphere.Collider.Bounds.Intersects(compound2D.GetPartCollider(0).MixedBounds3D).Should().BeTrue();
        sphere.Collider.Bounds.Intersects(compound2D.GetPartCollider(1).MixedBounds3D).Should().BeFalse();

        bool collided = CollisionDetectionMixed.TryCollide(sphere.Collider, body2D.Collider, out MixedContact contact);

        collided.Should().BeFalse();
        contact.HasContact.Should().BeFalse();
    }

    [Fact]
    public void SphereCompound2DSlab_WithMultipleOverlaps_ShouldUseShallowestPartContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(
            context,
            new Vector3d(Fixed64.FromFraction(9, 10), Fixed64.Zero, Fixed64.Zero));
        var compound2D = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.FromFraction(3, 10), Fixed64.Zero)),
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero));
        SolidBody2D body2D = CreateBody2D(context, compound2D, Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(sphere.Collider, body2D.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.HasContact.Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.FromFraction(1, 10));
        contact.Normal3DTo2D.Should().Be(-Vector3d.Right);
        contact.Point2D.X.Should().Be(Fixed64.Half);
    }

    [Fact]
    public void SphereCompound2DSlab_WithShallowerFirstOverlaps_ShouldRetainFirstPartContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(
            context,
            new Vector3d(Fixed64.FromFraction(9, 10), Fixed64.Zero, Fixed64.Zero));
        var compound2D = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero),
            CompoundColliderPart2D.Circle(
                Fixed64.Half,
                new Vector2d(Fixed64.FromFraction(3, 10), Fixed64.Zero)));
        SolidBody2D body2D = CreateBody2D(context, compound2D, Vector2d.Zero);

        CollisionDetectionMixed.TryCollide(
            sphere.Collider,
            compound2D.GetPartCollider(0),
            out MixedContact shallow).Should().BeTrue();
        CollisionDetectionMixed.TryCollide(
            sphere.Collider,
            compound2D.GetPartCollider(1),
            out MixedContact deep).Should().BeTrue();

        shallow.Depth.Should().Be(Fixed64.FromFraction(1, 10));
        deep.Depth.Should().Be(Fixed64.FromFraction(2, 5));
        shallow.Point2D.Should().NotBe(deep.Point2D);

        CollisionDetectionMixed.TryCollide(
            sphere.Collider,
            body2D.Collider,
            out MixedContact contact).Should().BeTrue();

        contact.Depth.Should().Be(shallow.Depth);
        contact.Normal3DTo2D.Should().Be(shallow.Normal3DTo2D);
        contact.Point2D.Should().Be(shallow.Point2D);
    }

    [Fact]
    public void TryGetPlanarBoundaryPoint_WithCompoundContainingPoint_ShouldUseContainingPartBoundary()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)(-3), Fixed64.Zero)),
            CompoundColliderPart2D.AABBox(new Vector2d((Fixed64)2, (Fixed64)2), Vector2d.Zero));
        SolidBody2D body = CreateBody2D(context, compound, Vector2d.Zero);

        bool found = MixedEmbedded2DGeometry.TryGetPlanarBoundaryPoint(
            body.Collider,
            new Vector2d(Fixed64.FromFraction(1, 4), Fixed64.Zero),
            out Vector2d boundary,
            out Fixed64 distance);

        found.Should().BeTrue();
        boundary.Should().Be(new Vector2d(Fixed64.One, Fixed64.Zero));
        distance.Should().Be(Fixed64.FromFraction(3, 4));
    }

    [Fact]
    public void TryGetPlanarBoundaryPoint_WithCompoundSeparatedPoint_ShouldFallbackToNearestPartBoundary()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)(-3), Fixed64.Zero)),
            CompoundColliderPart2D.AABBox(new Vector2d((Fixed64)2, (Fixed64)2), new Vector2d((Fixed64)3, Fixed64.Zero)));
        SolidBody2D body = CreateBody2D(context, compound, Vector2d.Zero);

        bool found = MixedEmbedded2DGeometry.TryGetPlanarBoundaryPoint(
            body.Collider,
            new Vector2d(Fixed64.FromFraction(3, 2), Fixed64.Zero),
            out Vector2d boundary,
            out Fixed64 distance);

        found.Should().BeTrue();
        boundary.Should().Be(new Vector2d((Fixed64)2, Fixed64.Zero));
        distance.Should().Be(Fixed64.Half);
    }

    [Theory]
    [MemberData(nameof(MeshTriangleReducerCases))]
    public void MeshTriangleReducerCases_ShouldClassifyExactContact(
        string caseName,
        Vector3d first,
        Vector3d second,
        Vector3d third,
        ColliderType2D embeddedShape,
        Fixed64 embeddedRotation,
        bool expectedContact)
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSMeshCollider> mesh = CreateMesh3D(
            context,
            CreateTriangleMesh(first, second, third),
            Vector3d.Zero);
        LSCollider2D embedded = CreatePrimitive2D(context, embeddedShape, embeddedRotation);

        mesh.Collider.Bounds.Intersects(embedded.MixedBounds3D).Should().BeTrue(caseName);

        bool collided = CollisionDetectionMixed.TryCollide(mesh.Collider, embedded, out MixedContact contact);

        collided.Should().Be(expectedContact, caseName);
        contact.HasContact.Should().Be(expectedContact, caseName);
        if (expectedContact)
            contact.Normal3DTo2D.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero, caseName);
    }

    [Fact]
    public void ConvexMeshCircleSlab_WithTriangleCandidateOverlap_ShouldReportContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSMeshCollider> mesh = CreateMesh3D(
            context,
            MeshTestFixtures.CreateConvexCube(),
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D circle = CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(mesh.Collider, circle.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.HasContact.Should().BeTrue();
        contact.Depth.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        contact.Normal3DTo2D.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void ConvexMeshCapsuleSlab_WithTriangleCandidateOverlap_ShouldReportContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSMeshCollider> mesh = CreateMesh3D(
            context,
            MeshTestFixtures.CreateConvexCube(),
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D capsule = CreateBody2D(context, new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3), Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(mesh.Collider, capsule.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.HasContact.Should().BeTrue();
        contact.Depth.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        contact.Normal3DTo2D.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void ConvexMeshUnsupported2DSlab_WithOverlappingBounds_ShouldReturnNoContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSMeshCollider> mesh = CreateMesh3D(
            context,
            MeshTestFixtures.CreateConvexCube(),
            Vector3d.Zero);
        SolidBody2D unsupported = CreateBody2D(context, new UnsupportedTestCollider2D(), Vector2d.Zero);

        mesh.Collider.Bounds.Intersects(unsupported.Collider.MixedBounds3D).Should().BeTrue();

        bool collided = CollisionDetectionMixed.TryCollide(mesh.Collider, unsupported.Collider, out MixedContact contact);

        collided.Should().BeFalse();
        contact.HasContact.Should().BeFalse();
    }

    [Fact]
    public void RotatedMesh_WithConservativeTriangleCandidatesOutsideWorldBounds_ShouldReturnNoContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        Fixed64 center = Fixed64.FromFraction(6, 5);
        Fixed64 halfSize = Fixed64.FromFraction(1, 20);
        Vector3d[] vertices =
        {
            new(center - halfSize, Fixed64.Zero, center - halfSize),
            new(center + halfSize, Fixed64.Zero, center - halfSize),
            new(center, Fixed64.Zero, center + halfSize),
            new(-center - halfSize, Fixed64.Zero, -center - halfSize),
            new(-center + halfSize, Fixed64.Zero, -center - halfSize),
            new(-center, Fixed64.Zero, -center + halfSize)
        };
        var meshCollider = new LSMeshCollider(
            vertices,
            new[] { 0, 1, 2, 3, 4, 5 },
            MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);
        ScenarioBody<LSMeshCollider> mesh = CreateMesh3D(
            context,
            meshCollider,
            Vector3d.Zero,
            Euler(0, 45, 0));
        SolidBody2D box = CreateBody2D(
            context,
            new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)),
            Vector2d.Zero);
        var embeddedBounds = new FixedBoundVolume(box.Collider.MixedBounds3D.Min, box.Collider.MixedBounds3D.Max);
        var candidates = new SwiftList<int>();

        mesh.Collider.Bounds.Intersects(box.Collider.MixedBounds3D).Should().BeTrue();
        mesh.Collider.Mesh.GetTrianglesInWorldBounds(embeddedBounds, candidates);
        candidates.Should().HaveCount(2);
        for (int i = 0; i < candidates.Count; i++)
        {
            mesh.Collider.Mesh.GetTriangleVertices(
                candidates[i],
                out Vector3d first,
                out Vector3d second,
                out Vector3d third);
            var triangleBounds = new FixedBoundVolume(
                Vector3d.Min(Vector3d.Min(first, second), third),
                Vector3d.Max(Vector3d.Max(first, second), third));
            triangleBounds.Intersects(embeddedBounds).Should().BeFalse();
        }

        CollisionDetectionMixed.TryCollide(
            mesh.Collider,
            box.Collider,
            out MixedContact contact).Should().BeFalse();
        contact.HasContact.Should().BeFalse();
    }

    [Fact]
    public void MeshAABoxSlab_WithOverlappingBoundsButPlanarEdgeSeparation_ShouldRejectContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSMeshCollider> mesh = CreateMesh3D(
            context,
            CreateTriangleMesh(
                new Vector3d(Fixed64.FromFraction(9, 10), Fixed64.Zero, Fixed64.FromFraction(6, 5)),
                new Vector3d(Fixed64.FromFraction(6, 5), Fixed64.Zero, Fixed64.FromFraction(9, 10)),
                new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.FromFraction(3, 2))),
            Vector3d.Zero);
        SolidBody2D box = CreateBody2D(
            context,
            new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)),
            Vector2d.Zero);

        mesh.Collider.Bounds.Intersects(box.Collider.MixedBounds3D).Should().BeTrue();

        bool collided = CollisionDetectionMixed.TryCollide(mesh.Collider, box.Collider, out MixedContact contact);

        collided.Should().BeFalse();
        contact.HasContact.Should().BeFalse();
    }

    [Fact]
    public void MeshCapsuleSlab_WithOverlappingBoundsButRoundedEndSeparation_ShouldRejectContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSMeshCollider> mesh = CreateMesh3D(
            context,
            CreateTriangleMesh(
                new Vector3d(Fixed64.FromFraction(2, 5), Fixed64.Zero, Fixed64.FromFraction(29, 20)),
                new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.One),
                new Vector3d(Fixed64.FromFraction(6, 5), Fixed64.Zero, Fixed64.FromFraction(8, 5))),
            Vector3d.Zero);
        SolidBody2D capsule = CreateBody2D(
            context,
            new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3),
            Vector2d.Zero);

        mesh.Collider.Bounds.Intersects(capsule.Collider.MixedBounds3D).Should().BeTrue();

        bool collided = CollisionDetectionMixed.TryCollide(mesh.Collider, capsule.Collider, out MixedContact contact);

        collided.Should().BeFalse();
        contact.HasContact.Should().BeFalse();
    }

    [Fact]
    public void MeshConvexPolygonSlab_WithOverlappingBoundsButClosestFeatureSeparation_ShouldRejectContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSMeshCollider> mesh = CreateMesh3D(
            context,
            CreateTriangleMesh(
                new Vector3d(Fixed64.FromFraction(11, 10), Fixed64.Zero, Fixed64.FromFraction(11, 10)),
                new Vector3d(Fixed64.FromFraction(8, 5), Fixed64.Zero, Fixed64.FromFraction(11, 10)),
                new Vector3d(Fixed64.FromFraction(11, 10), Fixed64.Zero, Fixed64.FromFraction(8, 5))),
            Vector3d.Zero);
        SolidBody2D polygon = CreateBody2D(
            context,
            CreateSquarePolygon(),
            Vector2d.Zero,
            FixedMath.DegToRad((Fixed64)45));

        mesh.Collider.Bounds.Intersects(polygon.Collider.MixedBounds3D).Should().BeTrue();

        bool collided = CollisionDetectionMixed.TryCollide(mesh.Collider, polygon.Collider, out MixedContact contact);

        collided.Should().BeFalse();
        contact.HasContact.Should().BeFalse();
    }

    [Fact]
    public void ConcaveMeshAABoxSlab_WithInsideCornerFeature_ShouldReportStableContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSMeshCollider> mesh = CreateMesh3D(
            context,
            MeshTestFixtures.CreateInsideCorner(inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero);
        SolidBody2D box = CreateBody2D(
            context,
            new LSAABBoxCollider2D(new Vector2d(Fixed64.Half, Fixed64.Half)),
            new Vector2d(Fixed64.FromFraction(1, 4), Fixed64.FromFraction(1, 4)));

        bool firstCollided = CollisionDetectionMixed.TryCollide(mesh.Collider, box.Collider, out MixedContact first);
        bool secondCollided = CollisionDetectionMixed.TryCollide(mesh.Collider, box.Collider, out MixedContact second);

        firstCollided.Should().BeTrue();
        secondCollided.Should().BeTrue();
        first.HasContact.Should().BeTrue();
        first.Depth.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        first.Normal3DTo2D.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
        second.Point3D.Should().Be(first.Point3D);
        second.Point2D.Should().Be(first.Point2D);
        second.Normal3DTo2D.Should().Be(first.Normal3DTo2D);
        second.Depth.Should().Be(first.Depth);
    }

    [Fact]
    public void ConcaveMeshConvexPolygonSlab_WithRotatedPolygonFeature_ShouldReportContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSMeshCollider> mesh = CreateMesh3D(
            context,
            MeshTestFixtures.CreateInsideCorner(inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero);
        SolidBody2D polygon = CreateBody2D(
            context,
            CreateSquarePolygon(),
            new Vector2d(Fixed64.FromFraction(1, 4), Fixed64.FromFraction(1, 4)),
            FixedMath.DegToRad((Fixed64)45));

        bool collided = CollisionDetectionMixed.TryCollide(mesh.Collider, polygon.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.HasContact.Should().BeTrue();
        contact.Depth.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        contact.Normal3DTo2D.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void ConcaveMeshCircleSlab_InOpenChannelGap_ShouldNotCollideWithHullOnly()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSMeshCollider> mesh = CreateMesh3D(
            context,
            MeshTestFixtures.CreateUChannel(inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero);
        SolidBody2D circle = CreateBody2D(context, new LSCircleCollider2D(Fixed64.FromFraction(1, 4)), new Vector2d(Fixed64.Zero, (Fixed64)2));

        CollisionDetectionMixed.TryCollide(mesh.Collider, circle.Collider, out MixedContact contact).Should().BeFalse();
        contact.HasContact.Should().BeFalse();
    }

    private readonly record struct MixedPrimitiveCase(
        string Name,
        ColliderType Shape3D,
        ColliderType2D Shape2D,
        Vector3d Position3D,
        FixedQuaternion Rotation3D,
        Fixed64 Rotation2D,
        bool ExpectedContact);

    private static GravitasWorldContext CreateMixedContext()
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-8), (Fixed64)(-4), (Fixed64)(-8)),
                new Vector3d((Fixed64)8, (Fixed64)4, (Fixed64)8)),
            out _).Should().BeTrue();
        return context;
    }

    private static LSCollider CreatePrimitive3D(
        GravitasWorldContext context,
        ColliderType shape,
        Vector3d position,
        FixedQuaternion rotation)
    {
        return shape switch
        {
            ColliderType.AABox => CreateCuboid3D(context, position, rotation).Collider,
            ColliderType.Capsule => CreateCapsule3D(context, position, rotation).Collider,
            ColliderType.Cylinder => CreateCylinder3D(context, position, rotation).Collider,
            ColliderType.Cone => CreateCone3D(context, position, rotation).Collider,
            _ => throw new ArgumentOutOfRangeException(nameof(shape), shape, null)
        };
    }

    private static LSCollider2D CreatePrimitive2D(
        GravitasWorldContext context,
        ColliderType2D shape,
        Fixed64 rotation)
    {
        return shape switch
        {
            ColliderType2D.Circle => CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero, rotation).Collider,
            ColliderType2D.AABox => CreateBody2D(context, new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)), Vector2d.Zero, rotation).Collider,
            ColliderType2D.Capsule => CreateBody2D(context, new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3), Vector2d.Zero, rotation).Collider,
            ColliderType2D.ConvexPolygon => CreateBody2D(context, CreateSquarePolygon(), Vector2d.Zero, rotation).Collider,
            _ => throw new ArgumentOutOfRangeException(nameof(shape), shape, null)
        };
    }

    private static FixedQuaternion Euler(int x, int y, int z) =>
        FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)x, (Fixed64)y, (Fixed64)z);

    private static ScenarioBody<LSSphereCollider> CreateSphere3D(GravitasWorldContext context, Vector3d position)
    {
        return CreateBody3D(context, new LSSphereCollider(), position, FixedQuaternion.Identity);
    }

    private static ScenarioBody<LSCuboidCollider> CreateCuboid3D(
        GravitasWorldContext context,
        Vector3d position,
        FixedQuaternion? rotation = null)
    {
        return CreateBody3D(context, new LSCuboidCollider(), position, rotation ?? FixedQuaternion.Identity);
    }

    private static ScenarioBody<LSCapsuleCollider> CreateCapsule3D(
        GravitasWorldContext context,
        Vector3d position,
        FixedQuaternion? rotation = null)
    {
        var collider = new LSCapsuleCollider
        {
            Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One)
        };
        return CreateBody3D(context, collider, position, rotation ?? FixedQuaternion.Identity);
    }

    private static ScenarioBody<LSCylinderCollider> CreateCylinder3D(
        GravitasWorldContext context,
        Vector3d position,
        FixedQuaternion? rotation = null)
    {
        var collider = new LSCylinderCollider
        {
            Size = new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One)
        };
        return CreateBody3D(context, collider, position, rotation ?? FixedQuaternion.Identity);
    }

    private static ScenarioBody<LSConeCollider> CreateCone3D(
        GravitasWorldContext context,
        Vector3d position,
        FixedQuaternion? rotation = null)
    {
        var collider = new LSConeCollider
        {
            Size = new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One)
        };
        return CreateBody3D(context, collider, position, rotation ?? FixedQuaternion.Identity);
    }

    private static ScenarioBody<LSCompoundCollider> CreateCompound3D(GravitasWorldContext context, Vector3d position)
    {
        var collider = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero)),
            CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero)));
        return CreateBody3D(context, collider, position, FixedQuaternion.Identity);
    }

    private static ScenarioBody<LSMeshCollider> CreateMesh3D(
        GravitasWorldContext context,
        LSMeshCollider collider,
        Vector3d position,
        FixedQuaternion? rotation = null)
    {
        return CreateBody3D(context, collider, position, rotation ?? FixedQuaternion.Identity);
    }

    private static ScenarioBody<TCollider> CreateBody3D<TCollider>(
        GravitasWorldContext context,
        TCollider collider,
        Vector3d position,
        FixedQuaternion rotation)
        where TCollider : LSCollider
    {
        var agent = new TestMatterAgent(context, new FixedTransform(position, rotation, Vector3d.One));
        var body = new SolidBody(agent, collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(position, rotation);
        return new ScenarioBody<TCollider>(body, collider);
    }

    private static SolidBody2D CreateBody2D(
        GravitasWorldContext context,
        LSCollider2D collider,
        Vector2d position,
        Fixed64 rotation = default)
    {
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One));
        var body = new SolidBody2D(agent, collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(position, rotation);
        return body;
    }

    private static LSPolygonCollider2D CreateSquarePolygon() =>
        new(
            new Vector2d(-Fixed64.One, -Fixed64.One),
            new Vector2d(Fixed64.One, -Fixed64.One),
            new Vector2d(Fixed64.One, Fixed64.One),
            new Vector2d(-Fixed64.One, Fixed64.One));

    private static LSMeshCollider CreateTriangleMesh(Vector3d first, Vector3d second, Vector3d third) =>
        new(
            new[] { first, second, third },
            new[] { 0, 1, 2 },
            MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);
}
