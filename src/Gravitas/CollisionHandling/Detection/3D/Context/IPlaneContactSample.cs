//=======================================================================
// IPlaneContactSample.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.CollisionHandling;

internal interface IPlaneContactSample
{
    Vector3d Point { get; }
}
