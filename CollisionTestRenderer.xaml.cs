using HelixToolkit.Wpf;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection.Metadata;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Threading;

namespace WpfCollision
{
    public delegate void CoordinateChanged_Delegate(double x, double y, double Z);
    public delegate void SimulationComplete_Delegate();
    /// <summary>
    /// Interaction logic for CollisionTestRenderer.xaml
    /// Inspired by ProfileAndToolUnderAngleRenderer
    /// Graphical component should not depend on physical collision detection component but they both exist in mainwindow
    /// </summary>
    public partial class CollisionTestRenderer : UserControl
    {
        public CollisionTestRenderer()
        {
            InitializeComponent();

            // Set up timer for animation
            _timer = new DispatcherTimer();
            // target 60 Frames Per Second, 1000ms/16 = 62.5
            _timer.Interval = TimeSpan.FromMilliseconds(16);
            _timer.Tick += OnTimerTick;
        }
        public event CoordinateChanged_Delegate OnCoordinateChanged;
        /// <summary>
        /// movement simulation has been finished
        /// </summary>
        public event SimulationComplete_Delegate OnSimulationComplete;
        
        public bool IsCylinderMoving { get; private set; } = false;
        public bool IsCylinderAllocatingDisabled { get; private set; } = false;

        private double _speed = 0.1; // Speed units per tick
        private DispatcherTimer _timer;

        private const string boxName = "StaticBox";
        private const String cylinderName = "KynematicCylinder";
        private ModelVisual3D modelVisualCylindric;
        private GeometryModel3D modelCylindric;

        public MatrixTransform3D cylinderTransform { get; private set; }
        public double StartCylinderX
        {
            get;  set;
        }
        public double StartCylinderY
        {
            get;  set;
        }
        public double StartCylinderZ
        {
            get;  set;
        }
        public double EndCylinderX
        {
            get;  set;
        }
        public double EndCylinderY
        {
            get;  set;
        }
        public double EndCylinderZ
        {
            get;  set;
        }
        public double CylinderAngleX { get; set; }
        public double CylinderAngleY { get; set; }
        public double CylinderAngleZ { get; set; }
        /// <summary>
        /// get transform matrix for initial positioning
        /// </summary>
        /// <param name="angle"></param>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="z"></param>
        /// <returns></returns>
        private Matrix3D getTransformMatrix( double x, double y, double z)
        {
            Matrix3D mm = new Matrix3D();
            //then position
            mm.Translate(new Vector3D(x, y, z));
            return mm;
        }
        private void OnTimerTick(object? sender, EventArgs e)
        {
            // Move in the direction of target
            MoveTowardsTarget(EndCylinderX, EndCylinderY, EndCylinderZ);
            // step here collision simulation too
            //...
        }

        private void timerStop()
        {
            _timer.Stop();
            IsCylinderMoving = false;
        }
        private void timerStart()
        {
            _timer.Start();
            IsCylinderMoving = true;
        }
        public void StartMovement()
        {
            if (IsCylinderMoving) return;
            // Current position
            double currentX = cylinderTransform.Matrix.OffsetX;
            double currentY = cylinderTransform.Matrix.OffsetY;
            double currentZ = cylinderTransform.Matrix.OffsetZ;
            double targetX = StartCylinderX;
            double targetY = StartCylinderY;
            double targetZ = StartCylinderZ;
            Vector3D offst = new Vector3D(targetX-currentX, targetY-currentY, targetZ-currentZ);
            if (offst.X != 0 || offst.Y != 0 || offst.Z != 0)
            {
                Matrix3D mmEdited = cylinderTransform.Matrix;
                mmEdited.Translate(offst);
                cylinderTransform.Matrix = mmEdited;
            }

            timerStart();
        }
        private void MoveTowardsTarget(double TargetX, double TargetY, double TargetZ)
        {

            // Current position
            double currentX = cylinderTransform.Matrix.OffsetX;
            double currentY = cylinderTransform.Matrix.OffsetY;
            double currentZ = cylinderTransform.Matrix.OffsetZ;
            Point3D _currentPosition = new Point3D(currentX, currentY, currentZ);
            Point3D _targetPosition = new Point3D(TargetX, TargetY, TargetZ);
            // current translation matrix
            Matrix3D mmEdited = cylinderTransform.Matrix;
            // Calculate the vector towards the target
            Vector3D direction = _targetPosition - _currentPosition;
            double distance = direction.Length;

            if (distance < _speed)
            {
                // If close enough, snap to target and stop moving
                mmEdited.Translate(new Vector3D(_targetPosition.X - _currentPosition.X, _targetPosition.Y - _currentPosition.Y, _targetPosition.Z - _currentPosition.Z));
                cylinderTransform.Matrix = mmEdited;
                OnCoordinateChanged?.Invoke(_targetPosition.X, _targetPosition.Y, _targetPosition.Z);
                // Stop movement when there is nowhere to go
                    timerStop();
                OnSimulationComplete?.Invoke();
                return;
            }

            // Normalize direction and scale by _speed
            direction.Normalize();
            Vector3D step = direction * _speed;

            // Move by the step vector
            mmEdited.Translate(step);
            cylinderTransform.Matrix = mmEdited;
            OnCoordinateChanged?.Invoke(mmEdited.OffsetX, mmEdited.OffsetY, mmEdited.OffsetZ);

        }
        public void RemoveFromSceneByName(string itemName)
        {
            for (int i = 0; i < this.viewPort3d.Children.Count; i++)
            {
                if (this.viewPort3d.Children[i].GetName() == itemName)
                {
                    viewPort3d.Children.RemoveAt(i); break;
                }
            }
        }

        public void LoadStaticShapeInViewport(double XDim, double YDim, double ZDim)
        {
            RemoveFromSceneByName(boxName);
            // Create the 3D box geometry
            var builder = new MeshBuilder(false, false);
            builder.AddBox(center: new Point3D(0, 0, 0),
                           xlength: XDim, ylength: YDim, zlength: ZDim);

            var mesh = builder.ToMesh();

            // Create the material
            var material = new DiffuseMaterial(new SolidColorBrush(Colors.LightGray));

            // Create the model
            var model = new GeometryModel3D  {
                Geometry = mesh,
                Material = material,
                BackMaterial = material
            };

            // Create a visual for the model
            var modelVisual = new ModelVisual3D
            {
                Content = model
            };
            modelVisual.SetName(boxName);
            // Add the visual to the viewport
            viewPort3d.Children.Add(modelVisual);
        }
        public void LoadKynematicCylinderInViewport(double radius, double height)
        {
            RemoveFromSceneByName(cylinderName);
            // Create the 3D cylinder geometry
            var builder = new MeshBuilder(false, false);
            builder.AddCylinder(
                p1: new Point3D(0, 0, -height / 2),  // Bottom center
                p2: new Point3D(0, 0, height / 2),   // Top center
                radius: radius,
                thetaDiv: 36); // Smoothness

            var mesh = builder.ToMesh();

            // Create the material
            var material = new DiffuseMaterial(new SolidColorBrush(Colors.Blue));

            // Create the model
             modelCylindric = new GeometryModel3D
            {
                Geometry = mesh,
                Material = material,
                BackMaterial = material
            };

            // Create a visual for the model
            modelVisualCylindric = new ModelVisual3D
            {
                Content = modelCylindric
            };

            cylinderTransform = new MatrixTransform3D(Matrix3D.Identity);
            modelVisualCylindric.Transform = cylinderTransform;

            modelVisualCylindric.SetName(cylinderName);
            // Add the visual to the viewport
            viewPort3d.Children.Add(modelVisualCylindric);

        }
        /// <summary>
        /// assign positions to cylindric. rotation is not used here
        /// </summary>
        /// <param name="startCoordinates"></param>
        /// <param name="endCoordinates"></param>
        public void repositionCylindricOnScene(Point3D startCoordinates, Point3D endCoordinates)
        {
            IsCylinderAllocatingDisabled = true;
            StartCylinderX = startCoordinates.X; StartCylinderY = startCoordinates.Y; StartCylinderZ = startCoordinates.Z;
            EndCylinderX = endCoordinates.X; EndCylinderY = endCoordinates.Y; EndCylinderZ = endCoordinates.Z;
            Matrix3D mm = getTransformMatrix(StartCylinderX, StartCylinderY, StartCylinderZ);
            cylinderTransform.Matrix = mm;

            IsCylinderAllocatingDisabled = false;
        }
        /// <summary>
        /// Rotate previously created graphical representation of cylindric in space, 
        /// supply angleX, angleY, and angleZ in degrees. 
        /// </summary>
        /// <param name="angleX">rotation angle in degrees along X</param>
        /// <param name="angleY">rotation angle in degrees along Y</param>
        /// <param name="angleZ">rotation angle in degrees along Z</param>
        public void rotateCylindricOnScene2(double angleX, double angleY, double angleZ)
        {
            // Current position
            double currentX = cylinderTransform.Matrix.OffsetX;
            double currentY = cylinderTransform.Matrix.OffsetY;
            double currentZ = cylinderTransform.Matrix.OffsetZ;

            var rotationX = new AxisAngleRotation3D(new Vector3D(1, 0, 0), angleX);
            var rotationY = new AxisAngleRotation3D(new Vector3D(0, 1, 0), angleY);
            var rotationZ = new AxisAngleRotation3D(new Vector3D(0, 0, 1), angleZ);
            var rotateTransform = new Transform3DGroup();
            rotateTransform.Children.Add(new RotateTransform3D(rotationX));
            rotateTransform.Children.Add(new RotateTransform3D(rotationY));
            rotateTransform.Children.Add(new RotateTransform3D(rotationZ));
            var rotationMatrix = rotateTransform.Value;

            rotationMatrix.Translate(new Vector3D(currentX, currentY, currentZ));
            cylinderTransform.Matrix = rotationMatrix;
        }
    }
}
