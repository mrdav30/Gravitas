//=======================================================================
// ContactManifold2D.Enumerator.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Collections;
using System.Collections.Generic;

namespace Gravitas.CollisionHandling;

/// <content>Defines the nested Enumerator type.</content>
public sealed partial class ContactManifold2D
{
    /// <summary>Enumerates the active contacts in deterministic order.</summary>
    public struct Enumerator : IEnumerator<ManifoldContact2D>
    {
        private readonly ContactManifold2D _manifold;
        private int _group, _point;
        private int _index;

        internal Enumerator(ContactManifold2D manifold)
        {
            _manifold = manifold;
            _group = _point = 0;
            _index = -1;
        }

        /// <summary>Gets the current contact.</summary>
        public ManifoldContact2D Current
        {
            get
            {
                ChronicleGuard.ThrowIfListIndexInvalid(_index, _manifold.Count);
                return _manifold.GetGroup(_group)[_point];
            }
        }

        object IEnumerator.Current => Current;

        /// <summary>Advances to the next active contact.</summary>
        public bool MoveNext()
        {
            int next = _index + 1;
            if (next >= _manifold._count)
                return false;

            if (_index >= 0 && ++_point >= _manifold.GetGroup(_group).Count)
            {
                _group++;
                _point = 0;
            }
            _index = next;
            return true;
        }

        /// <summary>Resets the enumerator to its initial position.</summary>
        public void Reset()
        {
            _group = _point = 0;
            _index = -1;
        }

        /// <summary>Releases enumerator resources.</summary>
        public void Dispose() { }
    }
}
