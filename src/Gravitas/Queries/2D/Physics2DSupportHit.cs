namespace Gravitas.Queries;

/// <summary>A sampled support surface and its owning collider registration lifetime.</summary>
public readonly struct Physics2DSupportHit
{
    private readonly long _lifetime;

    internal Physics2DSupportHit(Physics2DHit hit, long frame)
    {
        Hit = hit;
        SampledFrame = frame;
        _lifetime = hit.Collider.LifetimeVersion;
    }

    /// <summary>Gets the selected target-surface witness, not the probe center.</summary>
    public Physics2DHit Hit { get; }
    /// <summary>Gets the context frame on which the query ran.</summary>
    public long SampledFrame { get; }
    /// <summary>Whether the same active collider registration still exists; this does not certify geometric freshness.</summary>
    public bool IsCurrentLifetime => Hit.Collider != null
        && Hit.Collider.IsActive && Hit.Collider.LifetimeVersion == _lifetime
        && !Hit.Collider.Context.IsDisposed
        && Hit.Collider.Context.Physics2D.TryGetColliderById(Hit.Collider.Id, out var registered)
        && ReferenceEquals(registered, Hit.Collider);
}
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
