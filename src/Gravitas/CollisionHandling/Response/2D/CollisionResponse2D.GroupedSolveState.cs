//=======================================================================
// CollisionResponse2D.GroupedSolveState.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.CollisionHandling;

/// <content>Defines the nested GroupedSolveState type.</content>
public static partial class CollisionResponse2D
{
    // Ordinary pairs retain only a null reference. Grouped pairs reuse their
    // phase-local incoming motion and exact lever frames across island iterations.
    internal sealed class GroupedSolveState
    {
        internal ContactResponseSnapshot Impact;
        internal ContactAnchor2D CenterA, CenterB;
    }
}
