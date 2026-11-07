//=======================================================================
// SolverContact2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Materials;

namespace Gravitas.CollisionHandling;



/// <summary>
/// Solver-ready scalar pure 2D contact data.
/// </summary>
internal readonly struct SolverContact2D
{
    public SolverContact2D(
        int groupIndex,
        int pointIndex,
        ulong contactId,
        ResponseBody2D bodyA,
        ResponseBody2D bodyB,
        ContactLever2D relativeA,
        ContactLever2D relativeB,
        Fixed64 depth,
        Vector2d normal,
        PhysicsMaterial materialA,
        PhysicsMaterial materialB,
        Fixed64 cachedNormalImpulse,
        Fixed64 cachedTangentImpulse,
        ContactGroupKey group = default)
    {
        Group = group;
        GroupIndex = groupIndex;
        PointIndex = pointIndex;
        ContactId = contactId;
        A = bodyA;
        B = bodyB;
        RelativeA = relativeA;
        RelativeB = relativeB;
        Depth = depth;
        Normal = normal;
        MaterialA = materialA;
        MaterialB = materialB;
        PhysicsMaterial.CombineFriction(materialA, materialB, out Fixed64 staticFriction, out Fixed64 dynamicFriction);
        StaticFriction = staticFriction;
        DynamicFriction = dynamicFriction;
        Restitution = PhysicsMaterial.CombineRestitution(materialA, materialB);
        Tangent = normal.RightHandNormal;
        CachedNormalImpulse = cachedNormalImpulse;
        CachedTangentImpulse = cachedTangentImpulse;
    }

    // Cache provenance beside the row; repeated manifold lookups cost more than this small copy.
    internal ContactGroupKey Group { get; }

    public int GroupIndex { get; }

    public int PointIndex { get; }

    public ulong ContactId { get; }

    public ResponseBody2D A { get; }

    public ResponseBody2D B { get; }

    public ContactLever2D RelativeA { get; }

    public ContactLever2D RelativeB { get; }

    public Fixed64 Depth { get; }

    public Vector2d Normal { get; }

    public PhysicsMaterial MaterialA { get; }

    public PhysicsMaterial MaterialB { get; }

    public Fixed64 StaticFriction { get; }

    public Fixed64 DynamicFriction { get; }

    public Fixed64 Restitution { get; }

    public Vector2d Tangent { get; }

    public Fixed64 CachedNormalImpulse { get; }

    public Fixed64 CachedTangentImpulse { get; }

}
