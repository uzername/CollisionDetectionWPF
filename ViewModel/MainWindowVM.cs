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
