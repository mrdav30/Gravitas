//=======================================================================
// PhysicsQueryMeter.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Queries;

internal struct PhysicsQueryMeter
{
    private readonly PhysicsQueryBudget _budget;
    private ulong _discovery;
    private ulong _candidates;
    private ulong _geometry;

    internal PhysicsQueryMeter(in PhysicsQueryBudget budget) => _budget = budget;
    internal readonly PhysicsQueryWork Work => new(_discovery, _candidates, _geometry);
    internal readonly ulong DiscoveryRemaining => _budget.DiscoveryLimit - _discovery;

    internal void AddDiscovery(ulong work) => _discovery += work;

    internal bool ChargeCandidate() => ChargeCandidates(1);

    internal bool ChargeCandidates(ulong count) => TryCharge(ref _candidates, _budget.CandidateLimit, count);

    internal bool ChargeGeometry(ulong count) => TryCharge(ref _geometry, _budget.GeometryLimit, count);

    private static bool TryCharge(ref ulong used, ulong limit, ulong count)
    {
        if (count > limit - used)
            return false;
        used += count;
        return true;
    }
}
