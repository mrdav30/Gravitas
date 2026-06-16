using FixedMathSharp;

namespace Gravitas.Support;

/// <summary>
/// A struct that represents a fixed transform, which is a combination of a position and rotation. 
/// This is used for static colliders that don't have a body, so we can store their position and rotation separately.
/// </summary>
public class FixedTransform
{
    private Fixed4x4 _matrix;

    private FixedTransform? _parent;

    public FixedTransform(
        Vector3d position,
        FixedQuaternion rotation,
        Vector3d scale,
        FixedTransform? parent = null)
    {
        _matrix = Fixed4x4.CreateTransform(position, rotation, scale);
        _parent = parent;
    }

    public FixedTransform(Fixed4x4 matrix, FixedTransform? parent = null)
    {
        _matrix = matrix;
        _parent = parent;
    }

    public Vector3d Position
    {
        get => _matrix.Translation;
        set => _matrix.SetTranslation(value);
    }

    public FixedQuaternion Rotation
    {
        get => _matrix.Rotation;
        set => _matrix.SetRotation(value);
    }

    public Vector3d Scale
    {
        get => _matrix.Scale;
        set => _matrix.SetGlobalScale(value);
    }

    public Vector3d LossyScale => _matrix.Scale;

    public Vector3d EulerAngles
    {
        get => Rotation.EulerAngles;
        set => Rotation = FixedQuaternion.FromEulerAnglesInDegrees(value.X, value.Y, value.Z);
    }

    public FixedTransform? Parent
    {
        get => _parent;
        set => _parent = value;
    }
}
