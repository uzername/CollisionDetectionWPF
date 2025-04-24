using CommunityToolkit.Mvvm.ComponentModel;

namespace WpfCollision.ViewModel
{
    /// <summary>
    /// viewmodel for textbox
    /// </summary>
    public class TextBoxLog: ObservableObject   {
        private string _text;
        public string TextLog { 
            get => _text; 
            set { _text = value; OnPropertyChanged(nameof(TextLog)); } 
        }
        public void AppendLine(string inLineToAppend)
        {
            TextLog = TextLog+inLineToAppend+Environment.NewLine;
        }
        public void CleanUp()
        {
            TextLog = "";
        }
    }
}
