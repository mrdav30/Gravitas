//=======================================================================
// ContinuousSurfaceTriangleAdmission.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Geometry;

namespace Gravitas.Queries;

/// <summary>
/// Filters a mesh triangle sweep hit against the continuous surface contact rules.
/// </summary>
internal delegate bool ContinuousSurfaceTriangleAdmission(int triangle, ref Physics3DHit hit, WideDirection3d? exactNormal);
