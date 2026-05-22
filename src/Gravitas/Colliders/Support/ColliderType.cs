namespace Gravitas.Colliders
{
    public enum ColliderType : byte
    {
        None,
        Sphere, // gets bounds based on set radius & height (more of a cylinder...)
        AABox, // Axis Aligned Bounding Box - Gets bounds based on set width/height/length (good for static objects)
        OBBox, // Oriented Bounding Box Gets bounds based on mesh, uses scale to set width/height
        Capsule,
        Cylinder, // not fully implemented
        Mesh
    }
}
