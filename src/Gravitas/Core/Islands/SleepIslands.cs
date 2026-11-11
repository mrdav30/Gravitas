//=======================================================================
// SleepIslands.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace Gravitas;

/// <summary>
/// Tracks automatic sleep eligibility for connected body slots after response.
/// </summary>
internal sealed class SleepIslands
{
    private const int Ready = -1;
    private const int Blocked = -2;
    private int[] _parents = Array.Empty<int>();

    internal void Reserve(int slotCount)
    {
        if (slotCount > _parents.Length)
            Array.Resize(ref _parents, Math.Max(slotCount, _parents.Length * 2));
    }

    internal void Begin(int slotCount) => _parents.AsSpan(0, slotCount).Fill(Ready);

    internal void Join(int first, int second)
    {
        if (first < 0 || second < 0)
            return;

        first = FindRoot(first);
        second = FindRoot(second);
        if (first != second)
            _parents[Math.Max(first, second)] = Math.Min(first, second);
    }

    internal void Block(int slot) => _parents[FindRoot(slot)] = Blocked;

    internal bool CanSleep(int slot) => _parents[FindRoot(slot)] == Ready;

    private int FindRoot(int slot)
    {
        while (_parents[slot] >= 0)
        {
            int parent = _parents[slot];
            if (_parents[parent] >= 0)
                _parents[slot] = _parents[parent];
            slot = parent;
        }

        return slot;
    }
}
