//=======================================================================
// SuppressedColliderPairs.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using FixedMathSharp;
using SwiftCollections;

namespace Gravitas.Constraints;

/// <summary>Tracks the live collider pairs suppressed by joints and ragdolls.</summary>
internal sealed class SuppressedColliderPairs
{
    private readonly SwiftList<Pair> _pairs = new();

    // Each binary search halves the live range. The bit length is its comparison bound.
    internal ulong LookupProbeLimit => (ulong)(64 - Fixed64.CountLeadingZeroes((ulong)_pairs.Count));

    internal bool Contains(int colliderA, int colliderB) =>
        _pairs.Count != 0 && colliderA >= 0 && colliderB >= 0 && colliderA != colliderB
        && Find(CreateKey(colliderA, colliderB)) >= 0;

    internal void Add(int colliderA, int colliderB)
    {
        ulong key = CreateKey(colliderA, colliderB);
        int index = Find(key);
        if (index >= 0)
            _pairs[index] = new Pair(key, _pairs[index].Count + 1);
        else
            _pairs.Insert(~index, new Pair(key, 1));
    }

    internal void Remove(int colliderA, int colliderB)
    {
        ulong key = CreateKey(colliderA, colliderB);
        int index = Find(key);
        if (index < 0)
            return;

        int count = _pairs[index].Count;
        if (count == 1)
            _pairs.RemoveAt(index);
        else
            _pairs[index] = new Pair(key, count - 1);
    }

    internal void Clear() => _pairs.FastClear();

    private int Find(ulong key) =>
        _pairs.AsReadOnlySpan().BinarySearch<Pair, Pair>(new Pair(key, 0));

    private static ulong CreateKey(int a, int b) =>
        ((ulong)(uint)Math.Min(a, b) << 32) | (uint)Math.Max(a, b);

    private readonly struct Pair : IComparable<Pair>
    {
        internal Pair(ulong key, int count)
        {
            Key = key;
            Count = count;
        }

        private ulong Key { get; }
        internal int Count { get; }

        public int CompareTo(Pair other) => Key.CompareTo(other.Key);
    }
}
