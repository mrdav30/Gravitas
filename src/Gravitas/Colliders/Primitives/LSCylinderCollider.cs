using FixedMathSharp;
using Gravitas.Raycasting;
using SwiftCollections;

namespace Gravitas.Colliders;

public class LSCylinderCollider : LSCollider
{
    public override ColliderType Shape => ColliderType.Cylinder;
    public override int Priority => ColliderSettings.GetPriority(Shape);

    protected override void OnInitialize()
    {
        _radius = FixedMath.Sqrt(LocalScale.y * LocalScale.y / 4 + LocalScale.x * LocalScale.z);
        base.OnInitialize();
    }

    protected override void GenerateShape()
    {
        throw new System.NotImplementedException();
    }

    public override Fixed3x3 CalculateInertiaTensor(Fixed64 mass)
    {
        throw new System.NotImplementedException();
    }

    public override bool ColliderOverlapsRay(RaycastSegmentWorker worker, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        throw new System.NotImplementedException();
    }

    public override Vector3d ClosestPointOnSurface(Vector3d other)
    {
        throw new System.NotImplementedException();
    }

    public override Vector3d GetNormalAtPoint(Vector3d point)
    {
        throw new System.NotImplementedException();
    }

    protected override void BuildShape()
    {
        throw new System.NotImplementedException();
    }
}
