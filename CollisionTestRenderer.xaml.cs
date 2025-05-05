using System;
using System.Collections.Generic;
using System.Linq;
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

namespace WpfCollision
{
    /// <summary>
    /// Interaction logic for CollisionTestRenderer.xaml
    /// Inspired by ProfileAndToolUnderAngleRenderer from MP
    /// 
    /// </summary>
    public partial class CollisionTestRenderer : UserControl
    {
        public CollisionTestRenderer()
        {
            InitializeComponent();
        }
        public bool IsCylinderMoving { get; private set; } = false;
        public bool IsCylinderAllocatingDisabled { get; private set; } = false;

        private MatrixTransform3D cylinderTransform;
        public double StartCylinderX
        {
            get { return (double)GetValue(StartCylinderXProperty); }
            set
            {
                SetValue(StartCylinderXProperty, value);

            }
        }
        public double StartCylinderY
        {
            get { return (double)GetValue(StartCylinderYProperty); }
            set
            {
                SetValue(StartCylinderYProperty, value);
            }
        }
        public double StartCylinderZ
        {
            get { return (double)GetValue(StartCylinderZProperty); }
            set
            {
                SetValue(StartCylinderZProperty, value);
            }
        }
        public double StartCylinderAngleX
        {
            get { return (double)GetValue(StartCylinderAngleXProperty); }
            set
            {
                SetValue(StartCylinderAngleXProperty, value);
            }
        }
        // ==== DEPENDENCY PROPERTIES FOR CYLINDER POSITION ====
        // https://metanit.com/sharp/wpf/13.php
        public static readonly DependencyProperty StartCylinderXProperty = DependencyProperty.Register(
                    "StartCylinderX",
                    typeof(double),
                    typeof(CollisionTestRenderer),
                    new FrameworkPropertyMetadata(
                        0.0,
                        FrameworkPropertyMetadataOptions.None,
                        new PropertyChangedCallback(OnStartCylinderXChanged),
                        new CoerceValueCallback(CoerceStartCylinderX)));

        private static object CoerceStartCylinderX(DependencyObject d, object baseValue)
        {
            return baseValue;
        }

        private static void OnStartCylinderXChanged(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            if (d is CollisionTestRenderer control)
            {
                // Call an instance method to handle the change
                control.OnStartCylinderXChangedInstance((double)e.OldValue, (double)e.NewValue);
            }
        }

        private void OnStartCylinderXChangedInstance(double oldValue, double newValue)
        {
            if (IsCylinderMoving || IsCylinderAllocatingDisabled) return;
            Matrix3D mm = getTransformMatrix(StartCylinderAngleX, StartCylinderX, StartCylinderY, StartCylinderZ);
            cylinderTransform.Matrix = mm;
        }

        public static readonly DependencyProperty StartCylinderYProperty = DependencyProperty.Register(
                    "StartCylinderY",
                    typeof(double),
                    typeof(CollisionTestRenderer),
                    new FrameworkPropertyMetadata(
                        0.0,
                        FrameworkPropertyMetadataOptions.None,
                        new PropertyChangedCallback(OnStartCylinderYChanged),
                        new CoerceValueCallback(CoerceStartCylinderY)));

        private static object CoerceStartCylinderY(DependencyObject d, object baseValue)
        {
            return baseValue;
        }

        private static void OnStartCylinderYChanged(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            if (d is CollisionTestRenderer control)
            {
                // Call an instance method to handle the change
                control.OnStartCylinderYChangedInstance((double)e.OldValue, (double)e.NewValue);
            }
        }

        private void OnStartCylinderYChangedInstance(double oldValue, double newValue)
        {
            if (IsCylinderMoving || IsCylinderAllocatingDisabled) return;
            Matrix3D mm = getTransformMatrix(StartCylinderAngleX, StartCylinderX, StartCylinderY, StartCylinderZ);
            cylinderTransform.Matrix = mm;
        }

        public static readonly DependencyProperty StartCylinderZProperty = DependencyProperty.Register(
                    "StartCylinderZ",
                    typeof(double),
                    typeof(CollisionTestRenderer),
                    new FrameworkPropertyMetadata(
                        0.0,
                        FrameworkPropertyMetadataOptions.None,
                        new PropertyChangedCallback(OnStartCylinderZChanged),
                        new CoerceValueCallback(CoerceStartCylinderZ)));

        private static object CoerceStartCylinderZ(DependencyObject d, object baseValue)
        {
            return baseValue;
        }

        private static void OnStartCylinderZChanged(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            if (d is CollisionTestRenderer control)
            {
                // Call an instance method to handle the change
                control.OnStartCylinderZChangedInstance((double)e.OldValue, (double)e.NewValue);
            }
        }

        private void OnStartCylinderZChangedInstance(double oldValue, double newValue)
        {
            if (IsCylinderMoving || IsCylinderAllocatingDisabled) return;
            Matrix3D mm = getTransformMatrix(StartCylinderAngleX, StartCylinderX, StartCylinderY, StartCylinderZ);
            cylinderTransform.Matrix = mm;
        }

        public static readonly DependencyProperty StartCylinderAngleXProperty = DependencyProperty.Register(
                    "StartCylinderAngleX",
                    typeof(double),
                    typeof(CollisionTestRenderer),
                    new FrameworkPropertyMetadata(
                        0.0,
                        FrameworkPropertyMetadataOptions.None,
                        new PropertyChangedCallback(OnStartCylinderAngleXChanged),
                        new CoerceValueCallback(CoerceStartCylinderXAngle)));

        private static object CoerceStartCylinderXAngle(DependencyObject d, object baseValue)
        {
            return baseValue;
        }

        private static void OnStartCylinderAngleXChanged(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            if (d is CollisionTestRenderer control)
            {
                // Call an instance method to handle the change
                control.OnStartCylinderAngleXChangedInstance((double)e.OldValue, (double)e.NewValue);
            }
        }
        private void OnStartCylinderAngleXChangedInstance(double oldValue, double newValue)
        {
            if (IsCylinderMoving || IsCylinderAllocatingDisabled) return;
            Matrix3D mm = getTransformMatrix(StartCylinderAngleX, StartCylinderX, StartCylinderY, StartCylinderZ);
            cylinderTransform.Matrix = mm;
        }
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

        public void LoadStaticShapeInViewport(double XDim, double YDim, double ZDim)
        {

        }
    }
}
