//=======================================================================
// GravitasDebugDrawCommandVisitor.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Diagnostics;

/// <summary>
/// Adapter-friendly visitor for typed debug draw command payloads.
/// </summary>
public abstract class GravitasDebugDrawCommandVisitor
{
    /// <summary>Visits a line draw command.</summary>
    public virtual void VisitLine(in GravitasLineDebugDrawView view) { }

    /// <summary>Visits a ray draw command.</summary>
    public virtual void VisitRay(in GravitasRayDebugDrawView view) { }

    /// <summary>Visits a point draw command.</summary>
    public virtual void VisitPoint(in GravitasPointDebugDrawView view) { }

    /// <summary>Visits a wire-sphere draw command.</summary>
    public virtual void VisitWireSphere(in GravitasWireSphereDebugDrawView view) { }

    /// <summary>Visits a wire-box draw command.</summary>
    public virtual void VisitWireBox(in GravitasWireBoxDebugDrawView view) { }

    /// <summary>Visits a wire-capsule draw command.</summary>
    public virtual void VisitWireCapsule(in GravitasWireCapsuleDebugDrawView view) { }

    /// <summary>Visits a wire-cylinder draw command.</summary>
    public virtual void VisitWireCylinder(in GravitasWireCylinderDebugDrawView view) { }

    /// <summary>Visits a wire-triangle draw command.</summary>
    public virtual void VisitWireTriangle(in GravitasWireTriangleDebugDrawView view) { }

    /// <summary>Visits a wire-cone draw command.</summary>
    public virtual void VisitWireCone(in GravitasWireConeDebugDrawView view) { }

    /// <summary>Visits a command whose kind is not recognized.</summary>
    public virtual void VisitUnknown(in GravitasDebugDrawCommand command) { }
}
