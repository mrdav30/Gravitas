//=======================================================================
// ConvexSweepQueryWorker.InitialContacts.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.CollisionHandling;

namespace Gravitas.Queries;

/// <content>Preserves geometric initial mesh contact normals before compound or triangle reduction.</content>
internal sealed partial class ConvexSweepQueryWorker
{
    internal static bool TryResolveInitialMeshContact(
        ConvexShape sourceShape, ConvexShape targetShape, GjkResult result,
        ref ContactAnchor anchor, ref Vector3d normal)
    {
        if (!result.Intersects)
        {
            // Disjoint contacts inside the query tolerance retain the actual
            // closest-feature separation axis, including edges and vertices.
            normal = result.Normal;
            return true;
        }

        if (anchor.TryGetWorldPoint(out Vector3d point)
            && targetShape.TryGetPlanarSurfaceNormal(point, out Vector3d planeNormal))
        {
            planeNormal = targetShape.GetInitialExitNormal(sourceShape, planeNormal);
            FixedPointAnchor targetSupport = targetShape.GetSupportAnchor(planeNormal);
            FixedPointAnchor sourceSupport = sourceShape.GetSupportAnchor(-planeNormal);
            if (WidePointAnchor3d.CompareProjectedOffsets(
                    targetSupport, sourceSupport, ZeroAnchor, ZeroAnchor, planeNormal) <= 0)
            {
                // A supporting plane with no positive overlap certifies
                // touching/separation even at an apex or triangle seam. Keep
                // that plane instead of an equally touching oblique edge axis
                // which could mistake planar tangent motion for approach.
                normal = planeNormal;
                return true;
            }
        }

        if (targetShape.TryGetInitialMeshContactNormal(sourceShape,
                out FixedPointAnchor targetAnchor, out Vector3d contactNormal))
        {
            anchor = new ContactAnchor(targetAnchor);
            normal = contactNormal;
            return true;
        }

        Vector3d witnessNormal = WidePointAnchor3d.GetDirection(result.PointA, result.PointB);
        if (witnessNormal != Vector3d.Zero
            && WidePointAnchor3d.CompareProjectedOffsets(
                targetShape.GetSupportAnchor(witnessNormal),
                sourceShape.GetSupportAnchor(-witnessNormal),
                ZeroAnchor, ZeroAnchor, witnessNormal) <= 0)
        {
            // GJK's bounded working scale can collapse a one-raw gap. Its
            // retained anchors still provide a separating feature direction;
            // admit it only when the full support intervals certify that axis.
            normal = witnessNormal;
            return true;
        }

        // A bounded GJK intersection alone does not establish a valid initial
        // normal. Do not fabricate a contact when neither complete geometry
        // nor the retained support witnesses can certify it.
        return false;
    }
}
