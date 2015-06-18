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

using System.Runtime.InteropServices;

using NeuronDataReaderWraper;

namespace demo_cs
{
    /// <summary>
    /// MainWindow.xaml 的交互逻辑
    /// </summary>
    public partial class MainWindow : Window
    {
        int                    _frameCount;
        FrameDataReceived      _DataReceived;
        BvhDataHeader          _bvhHeader;
        private float[]        _valuesBuffer = new float[354];
        SocketStatusChanged    _SocketStatusChanged;

        CmdId                  _currentCommandId;
        CommandDataReceived    _CmdDataReceived;
        CommandPack            _cmdDataHeader;
        private CmdResponseBoneSize[] _CmdDataBuffer = new CmdResponseBoneSize[59];
        string _CmdBvhInheritance;

        OptDataReceived _optDataReceived;
        OptDataHeader _optDataHeader;
        private float[] _optDataBuffer = new float[5 * 3 * sizeof(float)];

        IntPtr sockTCPRef = IntPtr.Zero;
        IntPtr sockUDPRef = IntPtr.Zero;


        int                    _frameCount1;

        public MainWindow()
        {
            InitializeComponent();
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            _DataReceived = new FrameDataReceived(bvhDataReceived);
            NeuronDataReader.BRRegisterFrameDataCallback(IntPtr.Zero, _DataReceived);

            _CmdDataReceived = new CommandDataReceived(cmdDataReceived);
            NeuronDataReader.BRRegisterCommandDataCallback(IntPtr.Zero, _CmdDataReceived);

            _SocketStatusChanged = new SocketStatusChanged(socketStatusChanged);
            NeuronDataReader.BRRegisterSocketStatusCallback(IntPtr.Zero, _SocketStatusChanged);

            _optDataReceived = new OptDataReceived(optDataReceived);
            NeuronDataReader.BRRegisterOptDataCallback(IntPtr.Zero, _optDataReceived);
        }

        private void Window_Closed(object sender, EventArgs e)
        {
            if (sockTCPRef != IntPtr.Zero)
            {
                NeuronDataReader.BRCloseSocket(sockTCPRef);
                sockTCPRef = IntPtr.Zero;
            }

            if (sockTCPRef1 != IntPtr.Zero)
            {
                NeuronDataReader.BRCloseSocket(sockTCPRef1);
                sockTCPRef1 = IntPtr.Zero;
            }

            if (sockUDPRef != IntPtr.Zero)
            {
                NeuronDataReader.BRCloseSocket(sockUDPRef);
                sockUDPRef = IntPtr.Zero;
            }
        }

        private void ButtonConnect_Click(object sender, RoutedEventArgs e)
        {
            if (NeuronDataReader.BRGetSocketStatus(sockTCPRef) == SocketStatus.CS_Running)
            {
                NeuronDataReader.BRCloseSocket(sockTCPRef);
                sockTCPRef = IntPtr.Zero;

                btnConnect.Content = "Connect";
            }
            else
            {
                sockTCPRef = NeuronDataReader.BRConnectTo(txtIP.Text, int.Parse(txtPort.Text));
                if (sockTCPRef == IntPtr.Zero)
                {
                    string msg = NeuronDataReader.strBRGetLastErrorMessage();
                    MessageBox.Show(msg, "Connection error", MessageBoxButton.OK, MessageBoxImage.Error);
                    return;
                }
                NeuronDataReader.BRRegisterAutoSyncParmeter(sockTCPRef, CmdId.Cmd_AvatarCount);
                btnConnect.Content = "Disconnect";

                _frameCount = 0;
            }
        }

        IntPtr sockTCPRef1;
        private void btnConnect1_Click(object sender, RoutedEventArgs e)
        {
            if (NeuronDataReader.BRGetSocketStatus(sockTCPRef1) == SocketStatus.CS_Running)
            {
                NeuronDataReader.BRCloseSocket(sockTCPRef1);
                sockTCPRef1 = IntPtr.Zero;

                btnConnect1.Content = "Connect";
            }
            else
            {
                sockTCPRef1 = NeuronDataReader.BRConnectTo(txtIP1.Text, int.Parse(txtPort1.Text));
                if (sockTCPRef1 == IntPtr.Zero)
                {
                    string msg = NeuronDataReader.strBRGetLastErrorMessage();
                    MessageBox.Show(msg, "Connection error", MessageBoxButton.OK, MessageBoxImage.Error);
                    return;
                }
                btnConnect1.Content = "Disconnect";

                _frameCount1 = 0;
            }

        }

        private void bvhDataReceived(IntPtr customObject, IntPtr sockRef, IntPtr header, IntPtr data)
        {
            _bvhHeader = (BvhDataHeader)Marshal.PtrToStructure(header, typeof(BvhDataHeader));

            // Change the buffer length if necessory
            if (_bvhHeader.DataCount != _valuesBuffer.Length)
            {
                _valuesBuffer = new float[_bvhHeader.DataCount];
            }

            Marshal.Copy(data, _valuesBuffer, 0, (int)_bvhHeader.DataCount);
            //_valuesBuffer = (float[])Marshal.PtrToStructure(data, typeof(float[_bvhHeader.DataCount]));

            if (sockRef == this.sockTCPRef)
            {
                _frameCount++;

                this.Dispatcher.Invoke(new Action(delegate()
                {
                    cbWithDisp.IsChecked = _bvhHeader.bWithDisp != 0;
                    cbWithPrefix.IsChecked = _bvhHeader.bWithReference != 0;

                    txtLog.Text = _frameCount.ToString();
                }));
            }

            if (sockRef == this.sockTCPRef1)
            {
                _frameCount1++;

                this.Dispatcher.Invoke(new Action(delegate()
                {
                    cbWithDisp.IsChecked = _bvhHeader.bWithDisp != 0;
                    cbWithPrefix.IsChecked = _bvhHeader.bWithReference != 0;

                    txtLog1.Text = _frameCount1.ToString();
                }));
            }

            if (sockRef == this.sockUDPRef)
            {
                _frameCount++;

                this.Dispatcher.Invoke(new Action(delegate()
                {
                    cbWithDisp.IsChecked = _bvhHeader.bWithDisp != 0;
                    cbWithPrefix.IsChecked = _bvhHeader.bWithReference != 0;

                    txtLog.Text = _frameCount.ToString();
                }));
            }

        }

         private void optDataReceived(IntPtr customObject, IntPtr sockRef, IntPtr header, IntPtr data)
         {
             _optDataHeader = (OptDataHeader)Marshal.PtrToStructure(header, typeof(OptDataHeader));

             // Change the buffer length if necessory
             if (_optDataHeader.DataCount != _optDataBuffer.Length)
             {
                 _optDataBuffer = new float[_optDataHeader.DataCount];
             }

             Marshal.Copy(data, _optDataBuffer, 0, (int)_optDataHeader.DataCount);
             //_valuesBuffer = (float[])Marshal.PtrToStructure(data, typeof(float[_bvhHeader.DataCount]));

             if (sockRef == this.sockTCPRef)
             {
                 _frameCount++;

                 this.Dispatcher.Invoke(new Action(delegate()
                 {
                     txtLog.Text = _frameCount.ToString();
                 }));
             }

             if (sockRef == this.sockTCPRef1)
             {
                 _frameCount1++;

                 this.Dispatcher.Invoke(new Action(delegate()
                 {
                     txtLog1.Text = _frameCount1.ToString();
                 }));
             }

             if (sockRef == this.sockUDPRef)
             {
                 _frameCount++;

                 this.Dispatcher.Invoke(new Action(delegate()
                 {
                     txtLog.Text = _frameCount.ToString();
                 }));
             }

         }
        

        private void cmdDataReceived(IntPtr customObject, IntPtr sockRef, IntPtr header, IntPtr data)
        {
            _cmdDataHeader = (CommandPack)Marshal.PtrToStructure(header, typeof(CommandPack));
            
            switch (_cmdDataHeader.CommandId)
            {
                case CmdId.Cmd_BoneSize:
                    {
                        // Change the buffer length if necessory
                        if (_cmdDataHeader.DataCount != _CmdDataBuffer.Length)
                        {
                            _CmdDataBuffer = new CmdResponseBoneSize[_cmdDataHeader.DataCount];
                        }

                        int offset = 0;
                        int structLen = Marshal.SizeOf(_CmdDataBuffer[0]);

                        for (int i = 0; i < (int)_cmdDataHeader.DataCount; i++)
                        {
                            _CmdDataBuffer[i] = (CmdResponseBoneSize)Marshal.PtrToStructure(data + offset, typeof(CmdResponseBoneSize));
                            offset += structLen;
                        }

                        txtCommandLog.Dispatcher.Invoke(new Action(delegate()
                        {
                            string log = "";
                            foreach (var item in _CmdDataBuffer)
                            {
                                log += item.BoneName + " : ";
                                log += item.BoneLength;
                                log += "\n";
                            }
                            txtCommandLog.Text = log;
                        }));
                    }
                    break;
                case CmdId.Cmd_AvatarName:
                    break;
                case CmdId.Cmd_FaceDirection:
                    break;
                case CmdId.Cmd_DataFrequency:
                    {
                        int avatarIndex = BitConverter.ToInt32(_cmdDataHeader.CmdParaments, 0);
                        int datafreq = BitConverter.ToInt32(_cmdDataHeader.CmdParaments, 4);

                        txtCommandLog.Dispatcher.Invoke(new Action(delegate()
                        {
                            txtCommandLog.Text = "avatarIndex : " + avatarIndex.ToString() + "\n";
                            txtCommandLog.Text += "dataFreq : " + datafreq.ToString() + "\n";
                        }));
                    }
                    break;
                case CmdId.Cmd_BvhInheritance:
                    {
                        if (_cmdDataHeader.DataCount <= 0) return;
                        byte[] bvhText = new byte[_cmdDataHeader.DataCount];

                        Marshal.Copy(data, bvhText, 0, (int)_cmdDataHeader.DataCount);

                        _CmdBvhInheritance = System.Text.Encoding.Default.GetString(bvhText);

                        txtCommandLog.Dispatcher.Invoke(new Action(delegate()
                        {
                            txtCommandLog.Text = _CmdBvhInheritance;
                        }));
                    }
                    break;
                case CmdId.Cmd_AvatarCount:
                    {
                        int avatarCount = BitConverter.ToInt32(_cmdDataHeader.CmdParaments, 0);
                        txtCommandLog.Dispatcher.Invoke(new Action(delegate()
                        {
                            txtCommandLog.Text = "AvatarCount : " + avatarCount.ToString();
                        }));
                    }
                    break;
                case CmdId.Cmd_CombinationMode:
                    {
                        int avatarIndex = BitConverter.ToInt32(_cmdDataHeader.CmdParaments, 0);
                        SensorCombinationModes combMode = (SensorCombinationModes)BitConverter.ToInt32(_cmdDataHeader.CmdParaments, Marshal.SizeOf(avatarIndex));
                        txtCommandLog.Dispatcher.Invoke(new Action(delegate()
                        {
                            switch (combMode)
                            { 
                                case SensorCombinationModes.SC_ArmOnly:
                                    txtCommandLog.Text = "Combination mode : " + "SC_ArmOnly";
                                    break;
                                case SensorCombinationModes.SC_UpperBody:
                                    txtCommandLog.Text = "Combination mode : " + "SC_UpperBody";
                                    break;
                                case SensorCombinationModes.SC_FullBody:
                                    txtCommandLog.Text = "Combination mode : " + "SC_FullBody";
                                    break;
                            }                            
                        }));
                    }
                    break;
            }
        }

        private void socketStatusChanged(IntPtr customObject, IntPtr sockRef, SocketStatus status, [MarshalAs(UnmanagedType.LPStr)]string msg)
        {
            this.Dispatcher.Invoke(new Action(delegate()
            {
                txtSockLog.Text = msg;
            }));
        }

        private void btnStartUDPService_Click(object sender, RoutedEventArgs e)
        {
            int nPort = 0;
            try 
	        {	        
		        nPort = int.Parse(txtUDPPort.Text);
	        }
	        catch (Exception)
	        {
                MessageBox.Show("Wrong port number.", "UDP Service", MessageBoxButton.OK, MessageBoxImage.Error);
		        return;
	        }

            if (btnStartUDPService.Content.ToString() == "Start")
            {
                sockUDPRef = NeuronDataReader.BRStartUDPServiceAt(nPort);
                if (sockUDPRef!=IntPtr.Zero)
                {
                    btnStartUDPService.Content = "Stop";
                }
                else
                {
                    btnStartUDPService.Content = "Start";
                }
            }
            else
            {
                NeuronDataReader.BRCloseSocket(sockUDPRef);
                btnStartUDPService.Content = "Start";
            }
        }

        private void btnCmdFetches_Click(object sender, RoutedEventArgs e)
        {
            if (sockTCPRef != IntPtr.Zero)
            {
                if (_currentCommandId == CmdId.Cmd_AvatarCount)
                {
                    NeuronDataReader.BRCommandFetchDataFromServer(sockTCPRef, _currentCommandId);
                }
                else 
                {
                    NeuronDataReader.BRCommandFetchAvatarDataFromServer(sockTCPRef, 0, _currentCommandId);
                }
            }
        }

        private void cbFetchCommand_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            _currentCommandId = (CmdId)cbFetchCommand.SelectedIndex;
        }
    }
}
