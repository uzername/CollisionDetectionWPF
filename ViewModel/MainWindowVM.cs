using CommunityToolkit.Mvvm.ComponentModel;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace WpfCollision.ViewModel
{
    public class MainWindowVM: ObservableObject
    {
        public MainWindowVM() {
            TextboxDatasource = new TextBoxLog();
        }
        private bool _allowEditing;
        public bool AllowEditing
        {
            get { return _allowEditing; }
            set { _allowEditing = value; OnPropertyChanged(nameof(AllowEditing)); }
        }
        private bool _pauseOnCollision;
        public bool PauseOnCollision
        {
            get { return _pauseOnCollision; }
            set { _pauseOnCollision = value; OnPropertyChanged(nameof(PauseOnCollision)); }
        }
        private TextBoxLog _textboxDatasource;
        public TextBoxLog TextboxDatasource
        {
            get => _textboxDatasource;
            set { _textboxDatasource = value; OnPropertyChanged(nameof(TextboxDatasource)); }
        }
        private ParametersViewVM _parametersDatasource;
        public ParametersViewVM ParametersDatasource
        {
            get => _parametersDatasource;
            set { _parametersDatasource = value; OnPropertyChanged(nameof(_parametersDatasource)); }
        }
    }
}
