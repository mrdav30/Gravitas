//=======================================================================
// CollisionDetection.CuboidFaces.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using FixedMathSharp;
using FixedMathSharp.Geometry;

namespace Gravitas.CollisionHandling;

/// <content>Builds finite witness patches for cuboid face contacts.</content>
public static partial class CollisionDetection
{
    private readonly struct CuboidFaceVertex
    {
        internal CuboidFaceVertex(Vector3d reference, Vector3d incident, int boundaries)
        {
            Reference = reference;
            Incident = incident;
            Boundaries = boundaries;
        }

        internal readonly Vector3d Reference, Incident;
        internal readonly int Boundaries;
    }

    private static bool TryBuildCuboidFaceContacts(
        ContactManifold manifold,
        FixedOrientedBox first,
        FixedOrientedBox second,
        Vector3d normal,
        bool reversed)
    {
        Vector3d localFirst = first.Orientation.Conjugate().Rotate(normal);
        Vector3d localSecond = second.Orientation.Conjugate().Rotate(-normal);
        int firstAxis = GetCuboidFaceAxis(localFirst);
        int secondAxis = GetCuboidFaceAxis(localSecond);
        Fixed64 firstSign = localFirst[firstAxis] < Fixed64.Zero ? -Fixed64.One : Fixed64.One;
        Fixed64 secondSign = localSecond[secondAxis] < Fixed64.Zero ? -Fixed64.One : Fixed64.One;

        // Compare with the same materialized basis direction as the SAT, without
        // accepting a near-parallel edge axis as a face normal.
        bool secondIsReference = !IsCuboidFaceNormal(first.Orientation, firstAxis, firstSign, normal);
        if (secondIsReference && !IsCuboidFaceNormal(second.Orientation, secondAxis, secondSign, -normal))
            return false;

        FixedOrientedBox reference = secondIsReference ? second : first;
        FixedOrientedBox incident = secondIsReference ? first : second;
        int referenceAxis = secondIsReference ? secondAxis : firstAxis;
        int incidentAxis = secondIsReference ? firstAxis : secondAxis;
        Fixed64 referenceSign = secondIsReference ? secondSign : firstSign;
        Fixed64 incidentSign = secondIsReference ? firstSign : secondSign;
        Vector3d referenceNormal = secondIsReference ? -normal : normal;
        int referenceU = (referenceAxis + 1) % 3;
        int referenceV = (referenceAxis + 2) % 3;
        int incidentU = (incidentAxis + 1) % 3;
        int incidentV = (incidentAxis + 2) % 3;
        Vector3d referenceHalf = reference.HalfExtents;
        Vector3d incidentHalf = incident.HalfExtents;

        // Rounded crossings need not preserve exact convexity. Any sequence of n vertices
        // emits at most n + floor(n/2) samples on one plane, so five clips grow 4 to at most 28.
        Span<CuboidFaceVertex> vertices = stackalloc CuboidFaceVertex[28];
        Span<CuboidFaceVertex> scratch = stackalloc CuboidFaceVertex[28];
        for (int i = 0; i < 4; i++)
        {
            Vector3d incidentPoint = default;
            incidentPoint[incidentAxis] = incidentSign * incidentHalf[incidentAxis];
            incidentPoint[incidentU] = i is 0 or 3 ? -incidentHalf[incidentU] : incidentHalf[incidentU];
            incidentPoint[incidentV] = i < 2 ? -incidentHalf[incidentV] : incidentHalf[incidentV];
            var anchor = new FixedPointAnchor(incident.Center, incident.Orientation, incidentPoint);
            // SAT overlap bounds center separation by two admitted radii. Exact
            // rigid rotations preserve length, so a corner fits within 3 * 4096.
            _ = anchor.TryGetLocalPointIn(reference.Center, reference.Orientation, out Vector3d referencePoint);
            vertices[i] = new CuboidFaceVertex(referencePoint, incidentPoint, (1 << i) | (1 << ((i + 3) % 4)));
        }

        int count = 4;
        for (int plane = 0; plane < 5; plane++)
        {
            int axis = plane < 2 ? referenceU : plane < 4 ? referenceV : referenceAxis;
            Fixed64 sign = plane == 4 ? referenceSign : (plane & 1) == 0 ? Fixed64.One : -Fixed64.One;
            count = ClipCuboidFace(vertices[..count], scratch, axis, referenceHalf[axis], sign, 1 << (4 + plane));
            if (count == 0)
                return false;
            Span<CuboidFaceVertex> temporary = vertices;
            vertices = scratch;
            scratch = temporary;
        }

        bool referenceIsPairFirst = secondIsReference == reversed;
        Vector3d pairNormal = reversed ? -normal : normal;
        // Face and clipping-boundary identities survive changes in penetration.
        int faces = secondIsReference ? 1 : 0;
        faces = faces * 6 + referenceAxis * 2 + (referenceSign < Fixed64.Zero ? 1 : 0);
        faces = faces * 6 + incidentAxis * 2 + (incidentSign < Fixed64.Zero ? 1 : 0);
        foreach (CuboidFaceVertex sample in vertices[..count])
        {
            Vector3d referencePoint = sample.Reference;
            referencePoint[referenceAxis] = referenceHalf[referenceAxis] * referenceSign;
            var referenceAnchor = new ContactAnchor(reference.Center, reference.Orientation, referencePoint);
            var incidentAnchor = new ContactAnchor(incident.Center, incident.Orientation, sample.Incident);
            // Each witness stays within its box. Their offset is bounded by
            // 4 * 4096, and the dot with a unit normal also fits in Fixed64.
            _ = referenceAnchor.TryGetOffsetFrom(incidentAnchor, out Vector3d difference);
            _ = Vector3d.TryDot(difference, referenceNormal, out Fixed64 depth);
            if (depth < Fixed64.Zero)
                continue;

            ulong id = 0xC0B0000000000000UL | ((ulong)faces << 9) | (uint)sample.Boundaries;
            manifold.AddContact(default, new ManifoldContact(id,
                referenceIsPairFirst ? referenceAnchor : incidentAnchor,
                referenceIsPairFirst ? incidentAnchor : referenceAnchor, depth, pairNormal));
        }
        return manifold.HasContact;
    }

    private static int GetCuboidFaceAxis(Vector3d localNormal)
    {
        int axis = localNormal.X.Abs() >= localNormal.Y.Abs() ? 0 : 1;
        return localNormal[axis].Abs() >= localNormal.Z.Abs() ? axis : 2;
    }

    private static bool IsCuboidFaceNormal(FixedQuaternion orientation, int axis, Fixed64 sign, Vector3d normal)
    {
        var basis = new WideRationalBasis3d(orientation);
        WideAxis3 direction = basis.GetAxis(axis);
        return WideNormalization.GetNormalized(Signed576.ExtendValue(direction.X),
            Signed576.ExtendValue(direction.Y), Signed576.ExtendValue(direction.Z)) * sign == normal;
    }

    private static int ClipCuboidFace(
        ReadOnlySpan<CuboidFaceVertex> input,
        Span<CuboidFaceVertex> output,
        int axis,
        Fixed64 limit,
        Fixed64 sign,
        int boundary)
    {
        int count = 0;
        CuboidFaceVertex previous = input[^1];
        Fixed64 previousDistance = limit - previous.Reference[axis] * sign;
        foreach (CuboidFaceVertex current in input)
        {
            Fixed64 distance = limit - current.Reference[axis] * sign;
            if ((previousDistance < Fixed64.Zero && distance > Fixed64.Zero)
                || (previousDistance > Fixed64.Zero && distance < Fixed64.Zero))
            {
                Fixed64 firstWeight = distance.Abs();
                Fixed64 secondWeight = previousDistance.Abs();
                Vector3d clipped = InterpolateCuboidFace(previous.Reference, current.Reference, firstWeight, secondWeight);
                clipped[axis] = limit * sign;
                output[count++] = new CuboidFaceVertex(clipped,
                    InterpolateCuboidFace(previous.Incident, current.Incident, firstWeight, secondWeight),
                    (previous.Boundaries & current.Boundaries) | boundary);
            }
            if (distance >= Fixed64.Zero)
            {
                output[count++] = distance == Fixed64.Zero
                    ? new CuboidFaceVertex(current.Reference, current.Incident, current.Boundaries | boundary)
                    : current;
            }
            previous = current;
            previousDistance = distance;
        }
        return count;
    }

    private static Vector3d InterpolateCuboidFace(
        Vector3d first, Vector3d second, Fixed64 firstWeight, Fixed64 secondWeight)
    {
        Signed192 denominator = Signed192.Raw(firstWeight + secondWeight);
        return new Vector3d(
            InterpolateCuboidFaceCoordinate(first.X, second.X, firstWeight, secondWeight, denominator),
            InterpolateCuboidFaceCoordinate(first.Y, second.Y, firstWeight, secondWeight, denominator),
            InterpolateCuboidFaceCoordinate(first.Z, second.Z, firstWeight, secondWeight, denominator));
    }

    private static Fixed64 InterpolateCuboidFaceCoordinate(
        Fixed64 first, Fixed64 second, Fixed64 firstWeight, Fixed64 secondWeight, Signed192 denominator)
    {
        // Both weights are positive. The result stays in the bounded endpoint interval.
        Signed192 numerator = WideArithmetic.AddSigned192(Fixed64.GetExactRawProduct(first, firstWeight),
            Fixed64.GetExactRawProduct(second, secondWeight));
        _ = Fixed64.TryGetSignedRawRatio(numerator, denominator, out Fixed64 coordinate);
        return coordinate;
    }
}
