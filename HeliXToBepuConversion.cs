using BepuPhysics;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

namespace WpfCollision
{
    /// <summary>
    /// Bepuphysics2 quaternions are System.Numerics ; Helix Toolkit quaternions are System.Windows.Media.Media3D
    /// Convert from Z-up (Helix) to Y-up (Bepu)
    /// </summary>
    public static class HelixToBepuConversion
    {
        public static RigidPose MatrixTransform3DToRigidPose(MatrixTransform3D transform)
        {
            Matrix3D m = transform.Matrix;

            // Extract position
            var helixPosition = new Vector3D(m.OffsetX, m.OffsetY, m.OffsetZ);
            var bepuPosition = SwapYandZ(helixPosition);

            // Extract rotation
            System.Numerics.Quaternion helixRotation = QuaternionFromMatrix(m);
            System.Numerics.Quaternion bepuRotation = HelixToBepuQuaternion(helixRotation);

            return new RigidPose(bepuPosition, bepuRotation);
        }

        private static Vector3 SwapYandZ(Vector3D v)
        {
            // Convert from Z-up (Helix) to Y-up (Bepu)
            return new Vector3((float)v.X, (float)v.Z, (float)v.Y);
        }

        public static System.Numerics.Quaternion HelixToBepuQuaternion(System.Numerics.Quaternion helixQuat)
        {
            // Convert from Z-up to Y-up
            return new System.Numerics.Quaternion(helixQuat.X, helixQuat.Z, helixQuat.Y, -helixQuat.W);
        }

        public static System.Numerics.Quaternion QuaternionFromMatrix(Matrix3D m)
        {
            Vector3D x = new Vector3D(m.M11, m.M12, m.M13);
            Vector3D y = new Vector3D(m.M21, m.M22, m.M23);
            Vector3D z = new Vector3D(m.M31, m.M32, m.M33);

            x.Normalize(); y.Normalize(); z.Normalize();

            var rot = new Matrix3D(
                x.X, x.Y, x.Z, 0,
                y.X, y.Y, y.Z, 0,
                z.X, z.Y, z.Z, 0,
                0, 0, 0, 1);

            return rot.ToQuaternion();
        }

        public static System.Numerics.Quaternion ToQuaternion(this Matrix3D m)
        {
            double trace = m.M11 + m.M22 + m.M33;
            double w, x, y, z;

            if (trace > 0)
            {
                double s = Math.Sqrt(trace + 1.0) * 2;
                w = 0.25 * s;
                x = (m.M23 - m.M32) / s;
                y = (m.M31 - m.M13) / s;
                z = (m.M12 - m.M21) / s;
            }
            else if ((m.M11 > m.M22) && (m.M11 > m.M33))
            {
                double s = Math.Sqrt(1.0 + m.M11 - m.M22 - m.M33) * 2;
                w = (m.M23 - m.M32) / s;
                x = 0.25 * s;
                y = (m.M21 + m.M12) / s;
                z = (m.M31 + m.M13) / s;
            }
            else if (m.M22 > m.M33)
            {
                double s = Math.Sqrt(1.0 + m.M22 - m.M11 - m.M33) * 2;
                w = (m.M31 - m.M13) / s;
                x = (m.M21 + m.M12) / s;
                y = 0.25 * s;
                z = (m.M32 + m.M23) / s;
            }
            else
            {
                double s = Math.Sqrt(1.0 + m.M33 - m.M11 - m.M22) * 2;
                w = (m.M12 - m.M21) / s;
                x = (m.M31 + m.M13) / s;
                y = (m.M32 + m.M23) / s;
                z = 0.25 * s;
            }

            return new System.Numerics.Quaternion((float)x, (float)y, (float)z, (float)w);
        }

        /// <summary>
        /// Converts Euler angles (in radians) from Helix Toolkit (Z-up) to a quaternion suitable for BepuPhysics (Y-up).
        /// </summary>
        /// <param name="angleXRad">Rotation around X axis in Helix (remains X in Bepu)</param>
        /// <param name="angleYRad">Rotation around Y axis in Helix (becomes Z in Bepu)</param>
        /// <param name="angleZRad">Rotation around Z axis in Helix (becomes Y in Bepu)</param>
        public static System.Numerics.Quaternion CreateBepuQuaternionFromHelixEulerAngles(
            float angleXRad,
            float angleYRad,
            float angleZRad)
        {
            // Map Helix axes to Bepu axes:
            // Helix X - Bepu X
            // Helix Y - Bepu Z
            // Helix Z - Bepu Y

            // Create individual axis-angle quaternions with remapped axes
            var qx = System.Numerics.Quaternion.CreateFromAxisAngle(Vector3.UnitX, angleXRad);         // X stays X
            var qz = System.Numerics.Quaternion.CreateFromAxisAngle(Vector3.UnitY, angleZRad);         // Z - Y
            var qy = System.Numerics.Quaternion.CreateFromAxisAngle(Vector3.UnitZ, angleYRad);         // Y - Z

            // Multiply in desired order: X - Y - Z (Helix logic); Bepu order is right to left
            var result = System.Numerics.Quaternion.Concatenate(
                             System.Numerics.Quaternion.Concatenate(qx, qy), qz);

            return result;
        }

    }
}
