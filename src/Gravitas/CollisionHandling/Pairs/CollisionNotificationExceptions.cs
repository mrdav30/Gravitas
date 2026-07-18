//=======================================================================
// CollisionNotificationExceptions.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using SwiftCollections;
using System;
using System.Runtime.CompilerServices;
using System.Runtime.ExceptionServices;

namespace Gravitas.CollisionHandling;

internal static class CollisionNotificationExceptions
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Capture(ref SwiftList<Exception>? exceptions, Exception exception) =>
        (exceptions ??= new SwiftList<Exception>(2)).Add(exception);

    [MethodImpl(MethodImplOptions.NoInlining)]
    internal static void ThrowIfAny(SwiftList<Exception>? exceptions)
    {
        if (exceptions == null)
            return;

        if (exceptions.Count == 1)
            ExceptionDispatchInfo.Capture(exceptions[0]).Throw();

        throw new AggregateException(exceptions);
    }
}
