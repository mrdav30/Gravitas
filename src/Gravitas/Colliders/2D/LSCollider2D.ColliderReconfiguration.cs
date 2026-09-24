// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.

using FixedMathSharp;

namespace Gravitas.Colliders;

/// <content>Stages and publishes native primitive reconfiguration without changing runtime identity.</content>
public abstract partial class LSCollider2D
{
    internal void PrepareDetachedBodyReconfigurationCandidate(
        SolidBody2D body, Vector2d localOffset, Vector2d position)
    {
        _body = body;
        _agent = body.Agent;
        _context = body.Context;
        _localOffset = localOffset;
        _mixedHalfThicknessOverride = body.Collider._mixedHalfThicknessOverride;
        PrepareStandaloneInitialization(body.Agent, useRequestedPose: true, position, body.Rotation);
        PublishPreparedShape();
    }

    internal bool MatchesCommittedReconfigurationCandidate(LSCollider2D candidate)
    {
        bool sameDimensions = this is LSCircleCollider2D circle
            ? circle.Radius == ((LSCircleCollider2D)candidate).Radius
            : ((LSCapsuleCollider2D)this).Radius == ((LSCapsuleCollider2D)candidate).Radius
                && ((LSCapsuleCollider2D)this).Height == ((LSCapsuleCollider2D)candidate).Height;
        return sameDimensions && _localOffset == candidate._localOffset
            && !_runtimeShapeState.ShouldRebuild(CreateReconfigurationSnapshot(candidate, _shapeVersion));
    }

    internal void PublishBodyReconfigurationCandidate(LSCollider2D candidate)
    {
        if (this is LSCircleCollider2D circle)
            circle.CopyPreparedReconfiguration((LSCircleCollider2D)candidate);
        else
            ((LSCapsuleCollider2D)this).CopyPreparedReconfiguration((LSCapsuleCollider2D)candidate);

        _localOffset = candidate._localOffset;
        _shapeVersion++;
        _preparedSnapshot = CreateReconfigurationSnapshot(candidate, _shapeVersion);
        _preparedBounds = candidate._preparedBounds;
        _preparedMixedBounds = candidate._preparedMixedBounds;
        PublishPreparedShape();
    }

    private static ColliderShapeSnapshot2D CreateReconfigurationSnapshot(LSCollider2D candidate, uint shapeVersion)
    {
        ColliderShapeSnapshot2D snapshot = candidate._preparedSnapshot;
        return new ColliderShapeSnapshot2D(snapshot.Center, snapshot.Rotation,
            snapshot.OwnerScale, snapshot.PartScale, snapshot.LocalOffset, shapeVersion,
            snapshot.MixedSlabCenterY, snapshot.MixedHalfThickness);
    }
}
