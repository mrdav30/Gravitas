//=======================================================================
// PhysicsMesh.GeometryFrame.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using SwiftCollections;
using SwiftCollections.Query;
using System;

namespace Gravitas.Colliders;

/// <content>Retains authored geometry independently of rounded collider center and inspection vertices.</content>
public partial class PhysicsMesh
{
    private ScaledRigidFrame3d _sourceFrame, _preparedSourceFrame;
    private ScaledRigidFrame3d _geometryFrame, _preparedGeometryFrame;
    private bool _geometryUsesSource, _preparedGeometryUsesSource;
    private bool _scaleIsExact, _preparedScaleIsExact;

    internal ref readonly ScaledRigidFrame3d GeometryFrame => ref _geometryFrame;

    internal ReadOnlySpan<Vector3d> GeometryVertices =>
        _geometryUsesSource ? _localVertices : _scaledLocalVertices;

    internal FixedBoundBox GeometryLocalBounds =>
        _geometryUsesSource ? _localBounds : _scaledLocalBounds;

    internal void GetGeometryTriangleVertices(int index,
        out Vector3d first, out Vector3d second, out Vector3d third)
    {
        ChronicleGuard.ThrowIfArrayIndexInvalid(index, _triangleCount, nameof(index));
        ReadOnlySpan<Vector3d> vertices = GeometryVertices;
        int offset = index * 3;
        first = vertices[_triangles[offset]];
        second = vertices[_triangles[offset + 1]];
        third = vertices[_triangles[offset + 2]];
    }

    internal bool TryConvertWorldToRigidGeometry(Vector3d worldPoint, out Vector3d point) =>
        new FixedPointAnchor(worldPoint, FixedQuaternion.Identity, Vector3d.Zero)
            .TryGetLocalPointIn(_geometryFrame.RigidOrigin, _geometryFrame.Rotation, out point);

    internal void GetTrianglesInGeometryBounds(FixedBoundVolume bounds, SwiftList<int> result)
    {
        result.FastClear();
        TriangleBVH.Query(bounds, result);
    }

    private void SelectPreparedGeometryFrame(Vector3d center, FixedQuaternion rotation)
    {
        // With zero placement, the supplied scalar center is the exact pose
        // origin. Exact centered scale products then retain the compact map.
        _preparedGeometryUsesSource = _preparedSourceFrame.HasLocalPlacement || !_preparedScaleIsExact;
        _preparedGeometryFrame = _preparedGeometryUsesSource
            ? _preparedSourceFrame
            : ScaledRigidFrame3d.Rigid(center, rotation);
    }
}
