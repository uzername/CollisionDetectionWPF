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
        public MainWindow()
        {
            InitializeComponent();
            this.DataContext = new MainWindowVM();
            rendererInstance.LoadStaticShapeInViewport(4,1,1);
            rendererInstance.LoadKynematicCylinderInViewport(0.5, 2);
        }

        private void ButtonSimulate_Click(object sender, RoutedEventArgs e)
        {
            CollisionDetectionHandler currentHandler = new CollisionDetectionHandler();
            currentHandler.OnCollisionRegistered += CurrentHandler_OnCollisionRegistered;
            currentHandler.SimulationSimple();
            //SimpleSelfContainedDemo.Run();
        }

        private void CurrentHandler_OnCollisionRegistered(string parameter)
        {
            ((this.DataContext) as MainWindowVM).TextboxDatasource.AppendLine(parameter);
        }
    }
}