using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections;
using SwiftCollections.Pool;
using SwiftCollections.Query;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

public static partial class CollisionDetection
{
    #region Support

    private readonly struct AxisPenetration
    {
        public AxisPenetration(Vector3d axis, Fixed64 depth)
        {
            Axis = axis;
            Depth = depth;
            HasValue = true;
        }

        public Vector3d Axis { get; }

        public Fixed64 Depth { get; }

        public bool HasValue { get; }
    }


    #endregion
}
