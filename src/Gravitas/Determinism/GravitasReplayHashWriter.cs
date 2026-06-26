//=======================================================================
// GravitasReplayHashWriter.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Support;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// Allocation-free fixed-width writer for deterministic replay hash payloads.
/// </summary>
internal struct GravitasReplayHashWriter
{
    private const ulong FnvPrime = 1099511628211UL;
    private const ulong LowOffset = 14695981039346656037UL;
    private const ulong HighOffset = 7809847782465536322UL;

    private ulong _low;
    private ulong _high;

    public GravitasReplayHashWriter()
    {
        _low = LowOffset;
        _high = HighOffset;
    }

    public void WriteSection(string tag, int version)
    {
        SwiftThrowHelper.ThrowIfNull(tag, nameof(tag));
        WriteInt32(tag.Length);
        for (int i = 0; i < tag.Length; i++)
        {
            char value = tag[i];
            SwiftThrowHelper.ThrowIfArgument(
                value > 0x7f,
                nameof(tag),
                "Replay hash section tags must be stable ASCII.");
            WriteByte((byte)value);
        }

        WriteInt32(version);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void WriteBool(bool value) => WriteByte(value ? (byte)1 : (byte)0);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void WriteByte(byte value)
    {
        unchecked
        {
            _low ^= value;
            _low *= FnvPrime;

            _high ^= value + 0x9e3779b97f4a7c15UL + (_low << 6) + (_low >> 2);
            _high *= FnvPrime;
        }
    }

    public void WriteInt32(int value) => WriteUInt32(unchecked((uint)value));

    public void WriteUInt32(uint value)
    {
        WriteByte((byte)value);
        WriteByte((byte)(value >> 8));
        WriteByte((byte)(value >> 16));
        WriteByte((byte)(value >> 24));
    }

    public void WriteInt64(long value) => WriteUInt64(unchecked((ulong)value));

    public void WriteUInt64(ulong value)
    {
        WriteByte((byte)value);
        WriteByte((byte)(value >> 8));
        WriteByte((byte)(value >> 16));
        WriteByte((byte)(value >> 24));
        WriteByte((byte)(value >> 32));
        WriteByte((byte)(value >> 40));
        WriteByte((byte)(value >> 48));
        WriteByte((byte)(value >> 56));
    }

    public void WriteEnum<TEnum>(TEnum value) where TEnum : struct, Enum
    {
        switch (Unsafe.SizeOf<TEnum>())
        {
            case 1:
                WriteByte(Unsafe.As<TEnum, byte>(ref value));
                break;
            case 2:
                WriteUInt16(Unsafe.As<TEnum, ushort>(ref value));
                break;
            case 4:
                WriteUInt32(Unsafe.As<TEnum, uint>(ref value));
                break;
            case 8:
                WriteUInt64(Unsafe.As<TEnum, ulong>(ref value));
                break;
            default:
                throw new InvalidOperationException("Unsupported enum width.");
        }
    }

    public void WriteFixed64(Fixed64 value) => WriteInt64(value.m_rawValue);

    public void WriteVector2d(Vector2d value)
    {
        WriteFixed64(value.X);
        WriteFixed64(value.Y);
    }

    public void WriteVector3d(Vector3d value)
    {
        WriteFixed64(value.X);
        WriteFixed64(value.Y);
        WriteFixed64(value.Z);
    }

    public void WriteVector4d(Vector4d value)
    {
        WriteFixed64(value.X);
        WriteFixed64(value.Y);
        WriteFixed64(value.Z);
        WriteFixed64(value.W);
    }

    public void WriteQuaternion(FixedQuaternion value)
    {
        WriteFixed64(value.X);
        WriteFixed64(value.Y);
        WriteFixed64(value.Z);
        WriteFixed64(value.W);
    }

    public void WriteTransform(FixedTransform value)
    {
        SwiftThrowHelper.ThrowIfNull(value, nameof(value));
        WriteVector3d(value.Position);
        WriteQuaternion(value.Rotation);
        WriteVector3d(value.LossyScale);
    }

    public void WriteFixed3x3(Fixed3x3 value)
    {
        WriteFixed64(value.M11);
        WriteFixed64(value.M12);
        WriteFixed64(value.M13);
        WriteFixed64(value.M21);
        WriteFixed64(value.M22);
        WriteFixed64(value.M23);
        WriteFixed64(value.M31);
        WriteFixed64(value.M32);
        WriteFixed64(value.M33);
    }

    public void WritePhysicsLayer(PhysicsLayer value) => WriteInt32(value.Index);

    public void WritePhysicsLayerMask(PhysicsLayerMask value) => WriteInt32(value.Bits);

    public GravitasReplayHash ToHash()
    {
        ulong low = FinalizeLane(_low);
        ulong high = FinalizeLane(_high ^ low);
        return new GravitasReplayHash(low, high);
    }

    private void WriteUInt16(ushort value)
    {
        WriteByte((byte)value);
        WriteByte((byte)(value >> 8));
    }

    private static ulong FinalizeLane(ulong value)
    {
        unchecked
        {
            value ^= value >> 33;
            value *= 0xff51afd7ed558ccdUL;
            value ^= value >> 33;
            value *= 0xc4ceb9fe1a85ec53UL;
            value ^= value >> 33;
            return value;
        }
    }
}
