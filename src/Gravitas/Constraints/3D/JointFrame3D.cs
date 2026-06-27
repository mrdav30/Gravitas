//=======================================================================
// JointFrame3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using SwiftCollections;

namespace Gravitas.Constraints;

/// <summary>
/// Immutable local anchor frame used by a 3D joint.
/// </summary>
public readonly struct JointFrame3D
{
    /// <summary>
    /// Creates a local joint frame.
    /// </summary>
    public JointFrame3D(Vector3d position, FixedQuaternion rotation)
    {
        Position = position;
        Rotation = rotation.Normalized;
    }

    /// <summary>
    /// Gets the local anchor position relative to the linked body.
    /// </summary>
    public Vector3d Position { get; }

    /// <summary>
    /// Gets the local anchor orientation relative to the linked body.
    /// </summary>
    public FixedQuaternion Rotation { get; }

    internal static JointFrame3D FromTransform(FixedTransform transform, string parameterName)
    {
        SwiftThrowHelper.ThrowIfNull(transform, parameterName);
        return new JointFrame3D(transform.Position, transform.Rotation);
    }
}
