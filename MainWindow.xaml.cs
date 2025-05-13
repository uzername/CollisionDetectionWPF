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
        MainWindowVM vm;
        public MainWindow()
        {
            InitializeComponent();
            vm = new MainWindowVM();
            vm.ParametersDatasource = new ParametersViewVM {
                BoxXDim = 4, BoxYDim = 1, BoxZDim = 1,
                CylRadius = 2.0, CylHeight = 1,
                CylStartX = 0, CylStartY = 0, CylStartZ = 10, CylEndX = 0, CylEndY = 0, CylEndZ = -10 };
            this.DataContext = vm;
            physicsHandler.OnCollisionRegistered += PhysicsHandler_OnCollisionRegistered;
            physicsHandler.initializeSimulation();
            prepareGraphicalRepresentation();
            rendererInstance.OnCoordinateChanged += RendererInstance_OnCoordinateChanged;

            physicsHandler.InitStaticBoxShape((float)vm.ParametersDatasource.BoxXDim, (float)vm.ParametersDatasource.BoxYDim, (float)vm.ParametersDatasource.BoxZDim);
            physicsHandler.InitCylindricShape((float)vm.ParametersDatasource.CylRadius, (float)vm.ParametersDatasource.CylHeight);
            physicsHandler.simulateMoveCylindrikSingleStep(new System.Numerics.Vector3((float)vm.ParametersDatasource.CylStartX, (float)vm.ParametersDatasource.CylStartY, (float)vm.ParametersDatasource.CylStartZ));
            
        }
        private void prepareGraphicalRepresentation()
        {
            rendererInstance.LoadStaticShapeInViewport(vm.ParametersDatasource.BoxXDim, vm.ParametersDatasource.BoxYDim, vm.ParametersDatasource.BoxZDim);
            rendererInstance.LoadKynematicCylinderInViewport(vm.ParametersDatasource.CylRadius, vm.ParametersDatasource.CylHeight);
            rendererInstance.repositionCylindricOnScene(new System.Windows.Media.Media3D.Point3D(vm.ParametersDatasource.CylStartX, vm.ParametersDatasource.CylStartY, vm.ParametersDatasource.CylStartZ), new System.Windows.Media.Media3D.Point3D(vm.ParametersDatasource.CylEndX, vm.ParametersDatasource.CylEndY, vm.ParametersDatasource.CylEndZ));
        }
        private void RendererInstance_OnCoordinateChanged(double x, double y, double Z)
        {
            ((this.DataContext) as MainWindowVM).TextboxDatasource.AppendLine($"Coordinate of cylinder: [ {x} ; {y} ; {Z}]");
            // in bepuphysics2 y goes to top , in Helix Toolkit Z goes to top, so I need to swap
            // probably use Vector3_ToBepu from RotationAndCoordinateConversion
            physicsHandler.simulateMoveCylindrikSingleStep(new System.Numerics.Vector3((float)x, (float)Z, (float)y));
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
        private void ButtonApply_Click(object sender, RoutedEventArgs e)
        {
            prepareGraphicalRepresentation();
            physicsHandler.AssignRadiusLengthToCylindric((float)vm.ParametersDatasource.CylRadius, (float)vm.ParametersDatasource.CylHeight);
            physicsHandler.AssignDimensionsToBoxShape((float)vm.ParametersDatasource.BoxXDim, (float)vm.ParametersDatasource.BoxYDim, (float)vm.ParametersDatasource.BoxZDim);

        }
    }
}