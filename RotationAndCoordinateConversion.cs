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
    /// converts between coordinates of helix toolkit and bepuphysics2. 
    /// converts between quaternions of helix toolkit and bepuphysics2.
    /// Bepuphysics2 quaternions are System.Numerics ; Helix Toolkit quaternions are System.Windows.Media.Media3D
    /// </summary>
    internal class RotationAndCoordinateConversion
    {

        /// <summary>
        /// Converts a Helix Toolkit Quaternion (Z-up) to BepuPhysics Quaternion (Y-up).
        /// </summary>
        public static System.Numerics.Quaternion HelixToBepu(System.Windows.Media.Media3D.Quaternion helixQ)
        {
            // Convert to rotation matrix
            var rotationMatrix = new Matrix3D();
            rotationMatrix.Rotate(helixQ);

            // Swap Y and Z axes in the matrix (Z-up to Y-up)
            var swapped = new Matrix3D(
                rotationMatrix.M11, rotationMatrix.M13, rotationMatrix.M12, 0,
                rotationMatrix.M31, rotationMatrix.M33, rotationMatrix.M32, 0,
                rotationMatrix.M21, rotationMatrix.M23, rotationMatrix.M22, 0,
                0, 0, 0, 1
            );

            var bepuQ = ToBepuQuaternion(QuaternionFromMatrix(swapped));
            return bepuQ;
        }

        /// <summary>
        /// Converts a BepuPhysics Quaternion (Y-up) to Helix Toolkit Quaternion (Z-up).
        /// </summary>
        public static System.Windows.Media.Media3D.Quaternion BepuToHelix(System.Numerics.Quaternion bepuQ)
        {
            var matrix = MatrixFromQuaternion(ToMediaQuaternion(bepuQ));

            // Swap Y and Z axes in the matrix (Y-up to Z-up)
            var swapped = new Matrix3D(
                matrix.M11, matrix.M13, matrix.M12, 0,
                matrix.M31, matrix.M33, matrix.M32, 0,
                matrix.M21, matrix.M23, matrix.M22, 0,
                0, 0, 0, 1
            );

            return QuaternionFromMatrix(swapped);
        }

        private static System.Numerics.Quaternion ToBepuQuaternion(System.Windows.Media.Media3D.Quaternion q)
        {
            return new System.Numerics.Quaternion((float)q.X, (float)q.Y, (float)q.Z, (float)q.W);
        }

        private static System.Windows.Media.Media3D.Quaternion ToMediaQuaternion(System.Numerics.Quaternion q)
        {
            return new System.Windows.Media.Media3D.Quaternion(q.X, q.Y, q.Z, q.W);
        }

        public static System.Windows.Media.Media3D.Quaternion QuaternionFromMatrix(Matrix3D m)
        {
            // This uses WPF Quaternion from rotation matrix
            return new System.Windows.Media.Media3D.Quaternion(m.M11, m.M21, m.M31, 0); // will be corrected below
        }

        private static Matrix3D MatrixFromQuaternion(System.Windows.Media.Media3D.Quaternion q)
        {
            var m = new Matrix3D();
            m.Rotate(q);
            return m;
        }
        // ===== Coordinate conversion

        /// <summary>
        /// translate from HelixToolkit vector3 to bepu toolkit vector3 by flipping Y and Z,
        /// because bepuphysics2 works with Y facing up and HelixToolkit works with Z facing up
        /// </summary>
        /// <param name="helixVector">vector3 of Helix Toolkit coordinates</param>
        /// <returns></returns>
        public static System.Numerics.Vector3 Vector3_ToBepu(Vector3 helixVector)
        {
            return new System.Numerics.Vector3(
                (float)helixVector.X,
                (float)helixVector.Z, // Z becomes Y
                (float)helixVector.Y  // Y becomes Z
            );
        }
        /// <summary>
        /// translate from BepuPhysics coordinates to HelixToolkit coordinates
        /// </summary>
        /// <param name="bepuVector"></param>
        /// <returns></returns>
        public static Vector3 Vector3_ToHelix(System.Numerics.Vector3 bepuVector)
        {
            return new Vector3(
                bepuVector.X,
                bepuVector.Z, // Z from Y
                bepuVector.Y  // Y from Z
            );
        }

    }
}
