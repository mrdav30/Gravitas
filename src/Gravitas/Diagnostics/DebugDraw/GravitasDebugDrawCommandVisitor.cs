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
    public virtual void VisitLine(in GravitasLineDebugDrawView view) { }

    public virtual void VisitRay(in GravitasRayDebugDrawView view) { }

    public virtual void VisitPoint(in GravitasPointDebugDrawView view) { }

    public virtual void VisitWireSphere(in GravitasWireSphereDebugDrawView view) { }

    public virtual void VisitWireBox(in GravitasWireBoxDebugDrawView view) { }

    public virtual void VisitWireCapsule(in GravitasWireCapsuleDebugDrawView view) { }

    public virtual void VisitWireCylinder(in GravitasWireCylinderDebugDrawView view) { }

    public virtual void VisitWireTriangle(in GravitasWireTriangleDebugDrawView view) { }

    public virtual void VisitUnknown(in GravitasDebugDrawCommand command) { }
}
