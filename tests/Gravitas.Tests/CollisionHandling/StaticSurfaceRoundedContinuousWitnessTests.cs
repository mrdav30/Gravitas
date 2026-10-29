using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Materials;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using System.Numerics;
using Xunit;

namespace Gravitas.Tests;

public sealed class StaticSurfaceRoundedContinuousWitnessTests
{
    [Theory]
    [InlineData("Sphere")] [InlineData("ZeroCore")] [InlineData("Capsule")]
    public void RoundedClosestWitnessAtRotatedSeam_ShouldNotExcludeTheRealFace(string shape)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Environment.Gravity = scene.Context.Environment.AirDensity = scene.Context.Environment.DampingFactor = Fixed64.Zero;
        scene.Context.Environment.MinSpeed = Fixed64.Zero;
        scene.Context.Environment.MaxSpeed = scene.Context.Environment.MaxFallSpeed = (Fixed64)1024;
        scene.Context.SetFrameRate(16);
        long k = (1L << 32) / 5;
        var authoredRotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(3 * k), Fixed64.FromRaw(4 * k));
        LSMeshCollider left = Sheet(scene, -4, 0, authoredRotation), right = Sheet(scene, 0, 4, authoredRotation);
        FixedQuaternion actual = left.Body!.Rotation;
        Assert.Equal(actual, right.Body!.Rotation);
        Assert.Equal(Fixed64.Zero, actual.X); Assert.Equal(Fixed64.Zero, actual.Y);
        using var declaration = scene.Context.Physics.CreateStaticSurface(new[] { left, right });
        Vector3d center = FindNearNormalWithSubRawInteriorProjection(actual);
        Vector3d normal = actual.Rotate(Vector3d.Up), inward = -normal;
        Assert.True(scene.Context.Query3D.SweepSphere(center, Fixed64.One, inward, (Fixed64)4,
            out Physics3DHit geometric, PhysicsLayerMask.All));
        Assert.Equal(Fixed64.Zero, geometric.Distance);
        LSCollider collider = shape == "Sphere" ? new LSSphereCollider { Radius = Fixed64.One, Material = PhysicsMaterial.Frictionless }
            : new LSCapsuleCollider { Radius = Fixed64.One, Size = new(2, shape == "Capsule" ? 3 : 2, 2), Material = PhysicsMaterial.Frictionless };
        FixedQuaternion sourceRotation = shape == "Capsule" ? new FixedQuaternion(Fixed64.FromRaw(3037000500), Fixed64.Zero,
            Fixed64.Zero, Fixed64.FromRaw(3037000500)) : FixedQuaternion.Identity;
        SolidBody body = scene.CreateBody(collider, center, sourceRotation).Body;
        body.FreezeAxes = BodyFreezeAxes3D.Rotation; body.SleepEnabled = false; body.UseManualGrounding();
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        body.AddLinearImpulse(inward * (Fixed64)64);
        scene.Context.Simulate(); scene.Context.LateSimulate();
        Assert.True(Vector3d.Dot(body.Position3d, normal) > Fixed64.Half,
            $"{shape}: the real initial face was excluded after rounding its witness: {body.Position3d}, velocity {body.LinearVelocity}.");
        Assert.True(body.LastContinuousCollisionToiIterationCount > 0);
    }

    private static LSMeshCollider Sheet(PhysicsScenarioBuilder scene, int left, int right, FixedQuaternion rotation)
    {
        var mesh = new LSMeshCollider(new[] { new Vector3d(left, 0, -2), new(left, 0, 2), new(right, 0, -2), new(right, 0, 2) },
            new[] { 0, 1, 2, 2, 1, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)
        { Material = PhysicsMaterial.Frictionless };
        scene.CreateBody(mesh, Vector3d.Zero, rotation, immovable: true);
        return mesh;
    }

    private static Vector3d FindNearNormalWithSubRawInteriorProjection(FixedQuaternion rotation)
    {
        BigInteger z = rotation.Z.m_rawValue, w = rotation.W.m_rawValue;
        BigInteger tx = w * w - z * z, ty = 2 * z * w, denominator = w * w + z * z;
        BigInteger q = BigInteger.One << 32, length = q - 32;
        BigInteger x = Round(-ty * length, denominator), y = Round(tx * length, denominator);
        // Nine nearest lattice candidates are enough for this bounded fixture;
        // select using exact public-rotation geometry rather than assuming the
        // authored quaternion survived body normalization unchanged.
        for (int dx = -1; dx <= 1; dx++)
        for (int dy = -1; dy <= 1; dy++)
        {
            BigInteger cx = x + dx, cy = y + dy, tangent = tx * cx + ty * cy;
            BigInteger divisor = denominator * denominator;
            if (tangent <= 0 || 2 * BigInteger.Abs(tx * tangent) > divisor
                || 2 * BigInteger.Abs(ty * tangent) > divisor) continue;
            BigInteger outward = -ty * cx + tx * cy;
            if (outward <= 0 || outward >= q * denominator) continue;
            Assert.Equal(BigInteger.Zero, Round(tx * tangent, divisor));
            Assert.Equal(BigInteger.Zero, Round(ty * tangent, divisor));
            return new(Fixed64.FromRaw((long)cx), Fixed64.FromRaw((long)cy), Fixed64.Zero);
        }
        Assert.Fail("The bounded rotated-seam oracle did not find its required sub-raw projection.");
        return default;
    }

    private static BigInteger Round(BigInteger numerator, BigInteger denominator)
    {
        BigInteger result = BigInteger.DivRem(BigInteger.Abs(numerator), denominator, out BigInteger remainder);
        if (2 * remainder > denominator || 2 * remainder == denominator && !result.IsEven) result++;
        return numerator.Sign < 0 ? -result : result;
    }
}
