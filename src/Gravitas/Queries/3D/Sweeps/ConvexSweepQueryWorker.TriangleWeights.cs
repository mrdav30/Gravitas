//=======================================================================
// ConvexSweepQueryWorker.TriangleWeights.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.Queries;

/// <content>Defines the nested TriangleWeights type.</content>
internal sealed partial class ConvexSweepQueryWorker
{
    internal readonly struct TriangleWeights
    {
        public TriangleWeights(Fixed64 a, Fixed64 b, Fixed64 c)
        {
            A = a;
            B = b;
            C = c;
        }

        public Fixed64 A { get; }

        public Fixed64 B { get; }

        public Fixed64 C { get; }
    }
}
