//=======================================================================
// ConstraintEndpointJointIndex.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using SwiftCollections;
using System;

namespace Gravitas.Constraints;

/// <summary>
/// Tracks registered joint IDs in stable endpoint order without scanning a context's peak joint range.
/// </summary>
internal sealed class ConstraintEndpointJointIndex<TBody>
    where TBody : class
{
    private const int DefaultJointCapacity = 64;

    private readonly SwiftDictionary<TBody, EndpointChain> _chainsByBody = new();
    private JointEndpoints[] _endpointsByJoint = new JointEndpoints[DefaultJointCapacity];

    internal void Add(TBody bodyA, TBody bodyB, int jointId)
    {
        SwiftThrowHelper.ThrowIfTrue(
            ReferenceEquals(bodyA, bodyB),
            nameof(bodyB),
            "A joint endpoint index requires two distinct bodies.");
        EnsureJointCapacity(jointId + 1);
        ref JointEndpoints endpoints = ref _endpointsByJoint[jointId];
        SwiftThrowHelper.ThrowIfTrue(
            endpoints.IsRegistered,
            nameof(jointId),
            "Joint endpoint ownership is already registered.");

        endpoints = new JointEndpoints(bodyA, bodyB);
        Append(bodyA, jointId, ref endpoints.EndpointA);
        Append(bodyB, jointId, ref endpoints.EndpointB);
        endpoints.IsRegistered = true;
    }

    internal void Remove(int jointId)
    {
        ref JointEndpoints endpoints = ref _endpointsByJoint[jointId];
        RemoveEndpoint(jointId, ref endpoints.EndpointA);
        RemoveEndpoint(jointId, ref endpoints.EndpointB);
        endpoints = default;
    }

    internal bool TryGetLast(TBody body, out int jointId)
    {
        if (_chainsByBody.TryGetValue(body, out EndpointChain chain))
        {
            jointId = chain.LastJointId;
            return true;
        }

        jointId = -1;
        return false;
    }

    internal bool TryGetPrevious(TBody body, int jointId, out int previousJointId)
    {
        previousJointId = GetEndpoint(jointId, body).PreviousJointId;
        return previousJointId != 0;
    }

    internal void Clear()
    {
        _chainsByBody.Clear();
        Array.Clear(_endpointsByJoint, 0, _endpointsByJoint.Length);
    }

    private void Append(TBody body, int jointId, ref EndpointLink endpoint)
    {
        endpoint.Body = body;
        if (!_chainsByBody.TryGetValue(body, out EndpointChain chain))
        {
            _chainsByBody.Add(body, new EndpointChain(jointId, jointId, 1));
            return;
        }

        endpoint.PreviousJointId = chain.LastJointId;
        ref EndpointLink previous = ref GetEndpoint(chain.LastJointId, body);
        previous.NextJointId = jointId;
        chain.LastJointId = jointId;
        chain.Count++;
        _chainsByBody[body] = chain;
    }

    private void RemoveEndpoint(int jointId, ref EndpointLink endpoint)
    {
        TBody body = endpoint.Body!;
        SwiftThrowHelper.ThrowIfTrue(
            !_chainsByBody.TryGetValue(body, out EndpointChain chain),
            nameof(jointId),
            "Joint endpoint chain is missing its body.");

        if (endpoint.PreviousJointId == 0)
            chain.FirstJointId = endpoint.NextJointId;
        else
            GetEndpoint(endpoint.PreviousJointId, body).NextJointId = endpoint.NextJointId;

        if (endpoint.NextJointId == 0)
            chain.LastJointId = endpoint.PreviousJointId;
        else
            GetEndpoint(endpoint.NextJointId, body).PreviousJointId = endpoint.PreviousJointId;

        chain.Count--;
        if (chain.Count == 0)
            _chainsByBody.Remove(body);
        else
            _chainsByBody[body] = chain;
    }

    private ref EndpointLink GetEndpoint(int jointId, TBody body)
    {
        ref JointEndpoints endpoints = ref _endpointsByJoint[jointId];
        if (ReferenceEquals(endpoints.EndpointA.Body, body))
            return ref endpoints.EndpointA;

        SwiftThrowHelper.ThrowIfTrue(
            !ReferenceEquals(endpoints.EndpointB.Body, body),
            nameof(body),
            "Joint endpoint chain references an unrelated body.");
        return ref endpoints.EndpointB;
    }

    private void EnsureJointCapacity(int required)
    {
        if (required <= _endpointsByJoint.Length)
            return;

        int newSize = _endpointsByJoint.Length;
        while (newSize < required)
            newSize *= 2;

        Array.Resize(ref _endpointsByJoint, newSize);
    }

    private struct EndpointChain
    {
        internal EndpointChain(int firstJointId, int lastJointId, int count)
        {
            FirstJointId = firstJointId;
            LastJointId = lastJointId;
            Count = count;
        }

        internal int FirstJointId;
        internal int LastJointId;
        internal int Count;
    }

    private struct JointEndpoints
    {
        internal JointEndpoints(TBody bodyA, TBody bodyB)
        {
            EndpointA = new EndpointLink(bodyA);
            EndpointB = new EndpointLink(bodyB);
            IsRegistered = false;
        }

        internal EndpointLink EndpointA;
        internal EndpointLink EndpointB;
        internal bool IsRegistered;
    }

    private struct EndpointLink
    {
        internal EndpointLink(TBody body)
        {
            Body = body;
            PreviousJointId = 0;
            NextJointId = 0;
        }

        internal TBody? Body;
        internal int PreviousJointId;
        internal int NextJointId;
    }
}
