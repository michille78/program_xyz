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
using System.Windows.Navigation;
using System.Windows.Shapes;

using PN_BVHDataReaderWraper;
using System.Runtime.InteropServices;

namespace WpfApplication1
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        IntPtr connectorRef;
        int counter;
        float[] valuesBuffer = new float[354];

        SocketStatusChanged socketStatusChangeded;
        FrameDataReceived frameDataReceived;

        public MainWindow()
        {
            InitializeComponent();

            socketStatusChangeded = new SocketStatusChanged(onSocketStatusChangeded);
            PN_DataReader.BRRegisterConnectionStatusCallback(IntPtr.Zero, socketStatusChangeded);

            frameDataReceived = new FrameDataReceived(onFrameDataReceived);
            PN_DataReader.BRRegisterFrameDataCallback(IntPtr.Zero, frameDataReceived);
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            PN_DataReader.BRRegisterConnectionStatusCallback(IntPtr.Zero, null);
            PN_DataReader.BRRegisterFrameDataCallback(IntPtr.Zero, null);
        }

        void onSocketStatusChangeded(IntPtr customObject, IntPtr sockRef, SocketConnectionStatus status, string msg)
        {
            txtSockStatus.Dispatcher.Invoke(new Action(delegate()
            {
                txtSockStatus.Text = msg;
            }));
        }

        void onFrameDataReceived(IntPtr customObject, IntPtr sockRef, IntPtr bvheader, IntPtr data)
        {
            BvhDataHeader header = (BvhDataHeader)Marshal.PtrToStructure(bvheader, typeof(BvhDataHeader));

            if (header.DataCount != valuesBuffer.Length)
            {
                valuesBuffer = new float[header.DataCount];
            }
            Marshal.Copy(data, valuesBuffer, 0, (int)header.DataCount);

            txtReceivingCounter.Dispatcher.Invoke(new Action(delegate()
            {
                counter++;
                txtReceivingCounter.Text = counter.ToString();
            }));
        }

        private void btnConnect_Click(object sender, RoutedEventArgs e)
        {
            if (connectorRef == IntPtr.Zero)
            { 
                connectorRef = PN_DataReader.BRConnectTo(txtIP.Text, int.Parse(txtPort.Text));
                if (connectorRef == IntPtr.Zero)
                {
                    btnConnect.Content = "Connect";
                }
                else
                {
                    btnConnect.Content = "Disconnect";

                    counter = 0;
                }
            }
            else
            {
                if (connectorRef != IntPtr.Zero)
                {
                    PN_DataReader.BRDisconnect(connectorRef);
                    connectorRef = IntPtr.Zero;
                    btnConnect.Content = "Connect";
                }
            }
        }
    }
}
