//=======================================================================
// MixedContact.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas;

/// <summary>
/// Contact generated between one 3D collider and one embedded 2D collider.
/// </summary>
public readonly struct MixedContact
{
    public MixedContact(Vector3d point3D, Vector3d point2D, Vector3d normal3DTo2D, Fixed64 depth)
    {
        Point3D = point3D;
        Point2D = point2D;
        Normal3DTo2D = normal3DTo2D;
        Depth = depth;
        HasContact = true;
    }

    public bool HasContact { get; }

    public Vector3d Point3D { get; }

    public Vector3d Point2D { get; }

    /// <summary>
    /// Contact normal pointing from the 3D collider toward the embedded 2D collider volume.
    /// </summary>
    public Vector3d Normal3DTo2D { get; }

    public Fixed64 Depth { get; }
}
