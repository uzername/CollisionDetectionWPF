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
    /// <summary>
    /// Interaction logic for CollisionTestRenderer.xaml
    /// Inspired by ProfileAndToolUnderAngleRenderer from MP
    /// Graphical component should not depend on physical component but they both exist in mainwindow
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
        public bool IsCylinderMoving { get; private set; } = false;
        public bool IsCylinderAllocatingDisabled { get; private set; } = false;

        private double _speed = 0.1; // Speed units per tick
        private DispatcherTimer _timer;

        private const string boxName = "StaticBox";
        private const String cylinderName = "KynematicCylinder";
        private ModelVisual3D modelVisualCylindric;
        private GeometryModel3D modelCylindric;

        private MatrixTransform3D cylinderTransform;
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
        private Matrix3D getTransformMatrix(double angle, double x, double y, double z)
        {
            Matrix3D mm = new Matrix3D();
            //rotate first
            mm.Rotate(new Quaternion(new Vector3D(1, 0, 0), angle));
            //then position
            mm.Translate(new Vector3D(x, y, z));
            return mm;
        }
        private void OnTimerTick(object? sender, EventArgs e)
        {
            // Move in the direction of target
            MoveTowardsTarget(10, 10, 10);
            // step here collision simulation too
            //...
        }
        private void MoveTowardsTarget(double TargetX, double TargetY, double TargetZ)
        {

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
            modelVisualCylindric.SetName(cylinderName);
            // Add the visual to the viewport
            viewPort3d.Children.Add(modelVisualCylindric);

        }
        /// <summary>
        /// assign positions to cylindric. rotation is ignored by now
        /// </summary>
        /// <param name="startCoordinates"></param>
        /// <param name="endCoordinates"></param>
        public void repositionCylindricOnScene(Point3D startCoordinates, Point3D endCoordinates)
        {
            IsCylinderAllocatingDisabled = true;
            StartCylinderX = startCoordinates.X; StartCylinderY = startCoordinates.Y; StartCylinderZ = startCoordinates.Z;
            EndCylinderX = endCoordinates.X; EndCylinderY = endCoordinates.Y; EndCylinderZ = endCoordinates.Z;
            Matrix3D mm = getTransformMatrix(0, StartCylinderX, StartCylinderY, StartCylinderZ);

            IsCylinderAllocatingDisabled = false;
        }
    }
}
