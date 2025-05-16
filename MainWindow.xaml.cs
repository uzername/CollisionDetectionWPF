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
            vm.AllowEditing = true;
            vm.ParametersDatasource = new ParametersViewVM {
                BoxXDim = 4, BoxYDim = 1, BoxZDim = 1,
                CylRadius = 2.0, CylHeight = 1,
                CylStartX = 0, CylStartY = 0, CylStartZ = 10, CylEndX = 0, CylEndY = 0, CylEndZ = -10 };
            this.DataContext = vm;
            physicsHandler.OnCollisionRegistered += PhysicsHandler_OnCollisionRegistered;
            physicsHandler.initializeSimulation();
            prepareGraphicalRepresentation();
            rendererInstance.OnCoordinateChanged += RendererInstance_OnCoordinateChanged;
            rendererInstance.OnSimulationComplete += RendererInstance_OnSimulationComplete;

            physicsHandler.InitStaticBoxShape((float)vm.ParametersDatasource.BoxXDim, (float)vm.ParametersDatasource.BoxYDim, (float)vm.ParametersDatasource.BoxZDim);
            physicsHandler.InitCylindricShape((float)vm.ParametersDatasource.CylRadius, (float)vm.ParametersDatasource.CylHeight);
            physicsHandler.simulateMoveCylindrikSingleStep(new System.Numerics.Vector3((float)vm.ParametersDatasource.CylStartX, (float)vm.ParametersDatasource.CylStartY, (float)vm.ParametersDatasource.CylStartZ));
            
        }

        private void RendererInstance_OnSimulationComplete()
        {
            vm.AllowEditing = true;
        }

        private void prepareGraphicalRepresentation()
        {
            rendererInstance.LoadStaticShapeInViewport(vm.ParametersDatasource.BoxXDim, vm.ParametersDatasource.BoxYDim, vm.ParametersDatasource.BoxZDim);
            rendererInstance.LoadKynematicCylinderInViewport(vm.ParametersDatasource.CylRadius, vm.ParametersDatasource.CylHeight);
            rendererInstance.repositionCylindricOnScene(new System.Windows.Media.Media3D.Point3D(vm.ParametersDatasource.CylStartX, vm.ParametersDatasource.CylStartY, vm.ParametersDatasource.CylStartZ), new System.Windows.Media.Media3D.Point3D(vm.ParametersDatasource.CylEndX, vm.ParametersDatasource.CylEndY, vm.ParametersDatasource.CylEndZ));
            rendererInstance.rotateCylindricOnScene2(vm.ParametersDatasource.CylAngleX, vm.ParametersDatasource.CylAngleY, vm.ParametersDatasource.CylAngleZ);
        }
        private void RendererInstance_OnCoordinateChanged(double x, double y, double Z)
        {
            ((this.DataContext) as MainWindowVM).TextboxDatasource.AppendLine($"+++ Coordinate of cylinder: [ {x} ; {y} ; {Z}]");
            // in bepuphysics2 y goes to top , in Helix Toolkit Z goes to top, so I need to swap
            // probably use Vector3_ToBepu from RotationAndCoordinateConversion
            physicsHandler.simulateMoveCylindrikSingleStep(new System.Numerics.Vector3((float)x, (float)Z, (float)y));
        }

        private void ButtonSimulate_Click(object sender, RoutedEventArgs e)
        {
            ((this.DataContext) as MainWindowVM).TextboxDatasource.AppendLine($"=== SIMULATION STARTED ===");
            vm.AllowEditing = false;
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
            /*
            physicsHandler.AssignRadiusLengthAndRotationToCylindric((float)vm.ParametersDatasource.CylRadius, (float)vm.ParametersDatasource.CylHeight, 
                RotationAndCoordinateConversion.HelixToBepu( RotationAndCoordinateConversion.QuaternionFromMatrix( rendererInstance.cylinderTransform.Matrix) ) );
            */
            //physicsHandler.AssignRadiusLengthAndRotationToCylindric((float)vm.ParametersDatasource.CylRadius, (float)vm.ParametersDatasource.CylHeight, rendererInstance.cylinderTransform);
            physicsHandler.AssignRadiusLengthAndRotationToCylindric((float)vm.ParametersDatasource.CylRadius, (float)vm.ParametersDatasource.CylHeight, 
                
                    Single.DegreesToRadians( (float)vm.ParametersDatasource.CylAngleX),
                    Single.DegreesToRadians( (float)vm.ParametersDatasource.CylAngleY),
                    Single.DegreesToRadians( (float)vm.ParametersDatasource.CylAngleZ) 
                    
                );
            /*
            physicsHandler.AssignRadiusLengthAndRotationToCylindric((float)vm.ParametersDatasource.CylRadius, (float)vm.ParametersDatasource.CylHeight,
            HelixToBepuConversion.CreateBepuQuaternionFromHelixEulerAngles(
                Single.DegreesToRadians((float)vm.ParametersDatasource.CylAngleX),
                Single.DegreesToRadians((float)vm.ParametersDatasource.CylAngleY),
                Single.DegreesToRadians((float)vm.ParametersDatasource.CylAngleZ)
                    ));
            */
            physicsHandler.AssignDimensionsToBoxShape((float)vm.ParametersDatasource.BoxXDim, (float)vm.ParametersDatasource.BoxZDim, (float)vm.ParametersDatasource.BoxYDim);
            physicsHandler.simulateMoveCylindrikSingleStep(new System.Numerics.Vector3((float)vm.ParametersDatasource.CylStartX, (float)vm.ParametersDatasource.CylStartY, (float)vm.ParametersDatasource.CylStartZ));
        }
    }
}