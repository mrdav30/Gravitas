//=======================================================================
// PhysicsMaterial.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using MemoryPack;
using System;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;

namespace Gravitas.Materials;

/// <summary>
/// Deterministic surface material coefficients used by contact response.
/// </summary>
/// <remarks>
/// Static and dynamic friction are non-negative Coulomb coefficients.
/// Restitution is the bounce coefficient in the inclusive range [0, 1].
/// Dynamic friction cannot exceed static friction.
/// </remarks>
[Serializable]
[MemoryPackable]
public readonly partial struct PhysicsMaterial : IEquatable<PhysicsMaterial>
{
    /// <summary>
    /// The release default surface: unit friction, half restitution, geometric
    /// friction combine, and minimum restitution combine.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public static PhysicsMaterial Default =>
        new(Fixed64.One, Fixed64.One, Fixed64.Half);

    /// <summary>
    /// A surface with no contact friction and no restitution.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public static PhysicsMaterial Frictionless =>
        new(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);

    /// <summary>
    /// A high-restitution surface that keeps the default friction values.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public static PhysicsMaterial Bouncy =>
        new(Fixed64.One, Fixed64.One, Fixed64.One);

    /// <summary>
    /// Creates a deterministic surface material.
    /// </summary>
    [JsonConstructor]
    public PhysicsMaterial(
        Fixed64 staticFriction,
        Fixed64 dynamicFriction,
        Fixed64 restitution,
        PhysicsMaterialCombine frictionCombine = PhysicsMaterialCombine.GeometricMean,
        PhysicsMaterialCombine restitutionCombine = PhysicsMaterialCombine.Minimum)
    {
        ValidateFriction(staticFriction, nameof(staticFriction));
        ValidateFriction(dynamicFriction, nameof(dynamicFriction));
        if (dynamicFriction > staticFriction)
            throw new ArgumentOutOfRangeException(
                nameof(dynamicFriction),
                dynamicFriction,
                "Dynamic friction cannot exceed static friction.");
        if (restitution < Fixed64.Zero || restitution > Fixed64.One)
            throw new ArgumentOutOfRangeException(
                nameof(restitution),
                restitution,
                "Restitution must be between zero and one inclusive.");
        ValidateCombine(frictionCombine, nameof(frictionCombine));
        ValidateCombine(restitutionCombine, nameof(restitutionCombine));

        StaticFriction = staticFriction;
        DynamicFriction = dynamicFriction;
        Restitution = restitution;
        FrictionCombine = frictionCombine;
        RestitutionCombine = restitutionCombine;
    }

    /// <summary>
    /// Gets the friction coefficient used while tangential contact motion can be
    /// fully resisted by Coulomb static friction.
    /// </summary>
    public Fixed64 StaticFriction { get; }

    /// <summary>
    /// Gets the friction coefficient used after tangential motion exceeds the
    /// static friction limit.
    /// </summary>
    public Fixed64 DynamicFriction { get; }

    /// <summary>
    /// Gets the restitution coefficient. Zero removes closing velocity without
    /// bounce; one is fully elastic before the context restitution threshold is
    /// applied.
    /// </summary>
    public Fixed64 Restitution { get; }

    /// <summary>
    /// Gets this surface's preferred combine policy for static and dynamic
    /// friction coefficients.
    /// </summary>
    public PhysicsMaterialCombine FrictionCombine { get; }

    /// <summary>
    /// Gets this surface's preferred combine policy for restitution.
    /// </summary>
    public PhysicsMaterialCombine RestitutionCombine { get; }

    /// <summary>
    /// Resolves effective static and dynamic friction coefficients for a pair of
    /// contacting surfaces.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void CombineFriction(
        PhysicsMaterial left,
        PhysicsMaterial right,
        out Fixed64 staticFriction,
        out Fixed64 dynamicFriction)
    {
        PhysicsMaterialCombine policy = ResolveDominantPolicy(left.FrictionCombine, right.FrictionCombine);
        staticFriction = CombineScalar(left.StaticFriction, right.StaticFriction, policy);
        dynamicFriction = CombineScalar(left.DynamicFriction, right.DynamicFriction, policy);
        if (dynamicFriction > staticFriction)
            dynamicFriction = staticFriction;
    }

    /// <summary>
    /// Resolves the effective restitution coefficient for a pair of contacting
    /// surfaces.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 CombineRestitution(PhysicsMaterial left, PhysicsMaterial right)
    {
        PhysicsMaterialCombine policy = ResolveDominantPolicy(left.RestitutionCombine, right.RestitutionCombine);
        return FixedMath.Clamp(
            CombineScalar(left.Restitution, right.Restitution, policy),
            Fixed64.Zero,
            Fixed64.One);
    }

    /// <summary>
    /// Combines two scalar coefficients with one explicit deterministic policy.
    /// </summary>
    public static Fixed64 CombineScalar(
        Fixed64 left,
        Fixed64 right,
        PhysicsMaterialCombine policy)
    {
        ValidateCombine(policy, nameof(policy));
        return policy switch
        {
            PhysicsMaterialCombine.Minimum => FixedMath.Min(left, right),
            PhysicsMaterialCombine.Maximum => FixedMath.Max(left, right),
            PhysicsMaterialCombine.Average => (left + right) * Fixed64.Half,
            PhysicsMaterialCombine.Multiply => left * right,
            PhysicsMaterialCombine.GeometricMean => left > Fixed64.Zero && right > Fixed64.Zero
                ? FixedMath.Sqrt(left * right)
                : Fixed64.Zero,
            _ => throw new ArgumentOutOfRangeException(nameof(policy))
        };
    }

    /// <summary>
    /// Resolves the contact policy when two surfaces specify different combine
    /// policies. The order is deterministic and independent of collider order.
    /// </summary>
    public static PhysicsMaterialCombine ResolveDominantPolicy(
        PhysicsMaterialCombine left,
        PhysicsMaterialCombine right)
    {
        ValidateCombine(left, nameof(left));
        ValidateCombine(right, nameof(right));
        return GetPolicyPriority(left) >= GetPolicyPriority(right) ? left : right;
    }

    public bool Equals(PhysicsMaterial other) =>
        StaticFriction == other.StaticFriction
        && DynamicFriction == other.DynamicFriction
        && Restitution == other.Restitution
        && FrictionCombine == other.FrictionCombine
        && RestitutionCombine == other.RestitutionCombine;

    public override bool Equals(object? obj) =>
        obj is PhysicsMaterial other && Equals(other);

    public override int GetHashCode()
    {
        unchecked
        {
            int hash = StaticFriction.GetHashCode();
            hash = (hash * 397) ^ DynamicFriction.GetHashCode();
            hash = (hash * 397) ^ Restitution.GetHashCode();
            hash = (hash * 397) ^ FrictionCombine.GetHashCode();
            hash = (hash * 397) ^ RestitutionCombine.GetHashCode();
            return hash;
        }
    }

    public static bool operator ==(PhysicsMaterial left, PhysicsMaterial right) =>
        left.Equals(right);

    public static bool operator !=(PhysicsMaterial left, PhysicsMaterial right) =>
        !left.Equals(right);

    private static void ValidateFriction(Fixed64 value, string paramName)
    {
        if (value < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(paramName, value, "Friction cannot be negative.");
    }

    private static void ValidateCombine(PhysicsMaterialCombine value, string paramName)
    {
        if (value < PhysicsMaterialCombine.Minimum || value > PhysicsMaterialCombine.GeometricMean)
            throw new ArgumentOutOfRangeException(
                paramName,
                value,
                "Unsupported physics material combine policy.");
    }

    private static int GetPolicyPriority(PhysicsMaterialCombine policy) =>
        policy switch
        {
            PhysicsMaterialCombine.Average => 0,
            PhysicsMaterialCombine.Minimum => 1,
            PhysicsMaterialCombine.GeometricMean => 2,
            PhysicsMaterialCombine.Multiply => 3,
            PhysicsMaterialCombine.Maximum => 4,
            _ => throw new ArgumentOutOfRangeException(nameof(policy))
        };
}
