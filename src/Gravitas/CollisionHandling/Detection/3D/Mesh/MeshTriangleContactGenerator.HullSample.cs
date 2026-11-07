//=======================================================================
// MeshTriangleContactGenerator.HullSample.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;

namespace Gravitas.CollisionHandling;

/// <content>Defines the nested HullSample type.</content>
internal static partial class MeshTriangleContactGenerator
{
    internal readonly struct HullSample : IPlaneContactSample
    {
        internal HullSample(FixedContactAnchors contact) => Contact = contact;
        internal readonly FixedContactAnchors Contact;
        public Vector3d Point => Contact.FirstAnchor.LocalPoint;
    }
}
