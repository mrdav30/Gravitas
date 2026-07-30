//=======================================================================
// SolverContact.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Materials;

namespace Gravitas.CollisionHandling;

internal readonly struct ContactLever3D
{
    private ContactLever3D(Vector3d vector, bool isExact)
    {
        Vector = vector;
        IsExact = isExact;
    }

    internal bool IsExact { get; }

    internal Vector3d Vector { get; }

    internal static ContactLever3D Create(
        in ContactAnchor point,
        in ContactAnchor origin)
    {
        if (point.TryGetOffsetFrom(origin, out Vector3d vector))
            return new ContactLever3D(vector, isExact: false);

        return new ContactLever3D(default, isExact: true);
    }
}

/// <summary>
/// Solver-ready 3D contact data, including the deterministic tangent frame and cached impulses.
/// </summary>
internal readonly struct SolverContact
{
    public SolverContact(
        int manifoldIndex,
        ulong contactId,
        ResponseBody bodyA,
        ResponseBody bodyB,
        ContactLever3D relativeA,
        ContactLever3D relativeB,
        Fixed64 depth,
        Vector3d normal,
        PhysicsMaterial materialA,
        PhysicsMaterial materialB,
        Fixed64 cachedNormalImpulse,
        Fixed64 cachedTangentImpulse,
        Fixed64 cachedSecondaryTangentImpulse)
    {
        ManifoldIndex = manifoldIndex;
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
        Tangent = CreateTangent(normal);
        SecondaryTangent = Vector3d.Cross(normal, Tangent).Normalized;
        CachedNormalImpulse = cachedNormalImpulse;
        CachedTangentImpulse = cachedTangentImpulse;
        CachedSecondaryTangentImpulse = cachedSecondaryTangentImpulse;
    }

    public int ManifoldIndex { get; }

    public ulong ContactId { get; }

    public ResponseBody A { get; }

    public ResponseBody B { get; }

    public ContactLever3D RelativeA { get; }

    public ContactLever3D RelativeB { get; }

    public Fixed64 Depth { get; }

    public Vector3d Normal { get; }

    public PhysicsMaterial MaterialA { get; }

    public PhysicsMaterial MaterialB { get; }

    public Fixed64 StaticFriction { get; }

    public Fixed64 DynamicFriction { get; }

    public Fixed64 Restitution { get; }

    public Vector3d Tangent { get; }

    public Vector3d SecondaryTangent { get; }

    public Fixed64 CachedNormalImpulse { get; }

    public Fixed64 CachedTangentImpulse { get; }

    public Fixed64 CachedSecondaryTangentImpulse { get; }

    internal static Vector3d CreateTangent(Vector3d normal)
    {
        Vector3d absolute = Vector3d.Abs(normal);
        Vector3d reference = absolute.X <= absolute.Y && absolute.X <= absolute.Z
            ? Vector3d.Right
            : absolute.Y <= absolute.Z
                ? Vector3d.Up
                : Vector3d.Forward;

        return Vector3d.Cross(reference, normal).Normalized;
    }
}
