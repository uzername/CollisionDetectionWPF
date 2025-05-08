using System.Reflection.Metadata;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using WpfCollision.ViewModel;

namespace WpfCollision
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        CollisionDetectionHandler physicsHandler = new CollisionDetectionHandler();
        public MainWindow()
        {
            InitializeComponent();
            this.DataContext = new MainWindowVM();
            rendererInstance.LoadStaticShapeInViewport(4,1,1);
            rendererInstance.LoadKynematicCylinderInViewport(0.5, 2);
            rendererInstance.repositionCylindricOnScene(new System.Windows.Media.Media3D.Point3D(0, 0, 10), new System.Windows.Media.Media3D.Point3D(0, 0, -10));
            physicsHandler.OnCollisionRegistered += PhysicsHandler_OnCollisionRegistered;
            physicsHandler.initializeSimulation();
            physicsHandler.InitStaticBoxShape(4f, 1f, 1f);
            physicsHandler.InitCylindricShape(0.5f, 2f);
            physicsHandler.simulateMoveCylindrikSingleStep(new System.Numerics.Vector3(0, 0, 10));
            rendererInstance.OnCoordinateChanged += RendererInstance_OnCoordinateChanged;
        }

        private void RendererInstance_OnCoordinateChanged(double x, double y, double Z)
        {
            ((this.DataContext) as MainWindowVM).TextboxDatasource.AppendLine($"Coordinate of cylinder: [ {x} ; {y} ; {Z}]");
            physicsHandler.simulateMoveCylindrikSingleStep(new System.Numerics.Vector3((float)x, (float)y, (float)Z));
        }

        private void ButtonSimulate_Click(object sender, RoutedEventArgs e)
        {
            
            rendererInstance.StartMovement();
        }

        private void PhysicsHandler_OnCollisionRegistered(string parameter)
        {
            ((this.DataContext) as MainWindowVM).TextboxDatasource.AppendLine(parameter);
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            physicsHandler.FinalizeSimulation();
        }
    }
}