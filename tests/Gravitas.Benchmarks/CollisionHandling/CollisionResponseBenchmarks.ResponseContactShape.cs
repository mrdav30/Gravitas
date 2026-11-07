namespace Gravitas.Benchmarks;

/// <content>Defines the nested ResponseContactShape type.</content>
public partial class CollisionResponseBenchmarks
{
    public enum ResponseContactShape : byte
    {
        SingleContact,
        FaceManifold,
        RestingFaceManifold,
        CylinderContact,
        MeshContact,
        CompoundPartContact,
        GroupSingle,
        GroupRedundant,
        GroupIndependent
    }
}
