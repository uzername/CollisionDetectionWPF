using CommunityToolkit.Mvvm.ComponentModel;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace WpfCollision.ViewModel
{
    public class ParametersViewVM: ObservableObject  {
        private double _BoxXDim;
        private double _BoxYDim;
        private double _BoxZDim;
        private double _CylRadius;
        private double _CylHeight;

        private double _CylStartX;
        private double _CylStartY;
        private double _CylStartZ;
        private double _CylEndX;
        private double _CylEndY;
        private double _CylEndZ;
        private double _CylAngleX;
        private double _CylAngleY;
        private double _CylAngleZ;

        public double BoxXDim
        {
            get => _BoxXDim;
            set => SetProperty(ref _BoxXDim, value);
        }
        public double BoxYDim
        {
            get => _BoxYDim;
            set => SetProperty(ref _BoxYDim, value);
        }
        public double BoxZDim
        {
            get => _BoxZDim;
            set => SetProperty(ref _BoxZDim, value);
        }
        public double CylRadius
        {
            get => _CylRadius;
            set => SetProperty(ref _CylRadius, value);
        }
        public double CylHeight
        {
            get => _CylHeight;
            set => SetProperty(ref _CylHeight, value);
        }
        public double CylStartX
        {
            get => _CylStartX;
            set => SetProperty(ref _CylStartX, value);
        }
        public double CylStartY
        {
            get => _CylStartY;
            set => SetProperty(ref _CylStartY, value);
        }
        public double CylStartZ
        {
            get => _CylStartZ;
            set => SetProperty(ref _CylStartZ, value);
        }
        public double CylEndX
        {
            get => _CylEndX;
            set => SetProperty(ref _CylEndX, value);
        }
        public double CylEndY
        {
            get => _CylEndY;
            set => SetProperty(ref _CylEndY, value);
        }
        public double CylEndZ
        {
            get => _CylEndZ;
            set => SetProperty(ref _CylEndZ, value);
        }
        public double CylAngleX
        {
            get => _CylAngleX;
            set => SetProperty(ref _CylAngleX, value);
        }
        public double CylAngleY
        {
            get => _CylAngleY;
            set => SetProperty(ref _CylAngleY, value);
        }
        public double CylAngleZ
        {
            get => _CylAngleZ;
            set => SetProperty(ref _CylAngleZ, value);
        }
    }
}
