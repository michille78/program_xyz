using System;
using System.Collections;
using System.Threading;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Collections.Generic;
using System.IO;
using System.Runtime.InteropServices;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Formatters.Binary;

namespace PN_BVHDataReader
{
    public class SocketClient
    {
        //Socket客户端对象
        private Socket _clientSocket;
        private string _serverIp = "";
        private int _port;

        private bool _isDisconnecting = false;

        // Receiving data buffer
        private byte[] _recevieDataBuffer;

        /// <summary>
        /// Received data event
        /// </summary>
        public event SocketReceivedDataEvent ReceivedData;

        /// <summary>
        /// Status changed event
        /// </summary>
        public event SocketStatusChangedEvent StatusChanged;

        private int _timeOut;
        /// <summary>
        /// Set/Get time out prama. ms
        /// </summary>
        /// <value>The time out.</value>
        public int TimeOut
        {
            get
            {
                return _timeOut;
            }
            set
            {
                _timeOut = value;
            }
        }

        private int _receiveBufferLen = 154936; // system default buffer length is 154936...
        /// <summary>
        /// Set/Get buffer length used to recevie data
        /// </summary>
        /// <value>The time out.</value>
        public int ReceiveBufferLen
        {
            get
            {
                return _receiveBufferLen;
            }
            set
            {
                if (_receiveBufferLen != value)
                {
                    _receiveBufferLen = value;
                    _recevieDataBuffer = new byte[_receiveBufferLen];
                }
            }
        }

        /// <summary>
        /// Get connection status.
        /// </summary>
        /// <value><c>true</c> if this instance is connected; otherwise, <c>false</c>.</value>
        public bool IsConnected
        {
            get
            {
                if (_clientSocket == null) return false;
                return _clientSocket.Connected;
            }
        }

        private IDataParseDelegate _dataParser;
        /// <summary>
        /// Data parse delegate
        /// </summary>
        /// <value>The data parser object.</value>
        public IDataParseDelegate DataParser
        {
            get
            {
                return _dataParser;
            }
            set
            {
                _dataParser = value;
            }
        }

        // 构造函数
        public SocketClient()
        {
            // Init server information
            _serverIp = "127.0.0.1";
            _port = 7001;

            // Init time out
            _timeOut = 0;

            // Init receive buffer
            _recevieDataBuffer = new byte[_receiveBufferLen];
        }

        // 构造函数
        public SocketClient(IDataParseDelegate dataParser)
        {
            _dataParser = dataParser;

            // Init server information
            _serverIp = "127.0.0.1";
            _port = 7001;

            // Init time out
            _timeOut = 0;

            // Init receive buffer
            _recevieDataBuffer = new byte[_receiveBufferLen];
        }

        public bool Connect(string serverIp, int port)
        {
            _serverIp = serverIp;
            _port = port;

            _isDisconnecting = false;

            //服务器IP地址
            IPAddress ipAddress = IPAddress.Parse(_serverIp);
            //服务器端口
            IPEndPoint ipEndpoint = new IPEndPoint(ipAddress, _port);

            try
            {
                // tcp stream
                _clientSocket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
                _clientSocket.ReceiveTimeout = _timeOut;
                _clientSocket.SendTimeout = _timeOut;
                // connect
                _clientSocket.Connect(ipEndpoint);
                // check result
                if (_clientSocket.Connected)
                {
                    // change the default buffer size
                    _clientSocket.ReceiveBufferSize = _receiveBufferLen;

                    //与socket建立连接成功，开启线程接受服务端数据。
                    Thread thread = new Thread(new ThreadStart(fnReceivedData));
                    //thread.IsBackground = true;
                    thread.Start();

                    if (StatusChanged != null)
                    {
                        StatusChanged(ConnectionState.Connected, "Successfully connected to " + serverIp + ":" + port.ToString() + ".");
                    }
                }
                else
                {
                    if (StatusChanged != null)
                    {
                        StatusChanged(ConnectionState.Disconnected, "Failed to connected to " + serverIp + ":" + port.ToString() + ".");
                    }
                }
            }
            catch (Exception ex)
            {
                if (StatusChanged != null)
                {
                    StatusChanged(ConnectionState.Disconnected, "Failed to connected to " + serverIp + ":" + port.ToString() + ": " + ex.Message);
                }
            }

            return _clientSocket.Connected;
        }

        //关闭Socket
        public void Disconnect()
        {
            _isDisconnecting = true;

            if (_clientSocket != null && _clientSocket.Connected)
            {
                _clientSocket.Shutdown(SocketShutdown.Both);
                _clientSocket.Close();
            }
            _clientSocket = null;
        }

        private void fnReceivedData()
        {
        repeatReading:
            try
            {
                //在这个线程中接受服务器返回的数据
                while (_clientSocket.Connected)
                {
                    //Receive方法中会一直等待服务端回发消息
                    //如果没有回发会一直在这里等着。
                    int len = _clientSocket.Receive(_recevieDataBuffer, _receiveBufferLen, SocketFlags.None);
                    if (len <= 0)
                    {
                        if (StatusChanged != null)
                        {
                            StatusChanged(ConnectionState.Disconnected, "Disconnected from server.");
                        }
                        _clientSocket.Close();
                        break;
                    }

                    if (ReceivedData != null)
                    {
                        ReceivedData(_recevieDataBuffer, len);
                    }

                    if (_dataParser != null)
                    {
                        _dataParser.Parse(_recevieDataBuffer, len);
                    }
                }

                //与服务器断开连接跳出循环	
                if (StatusChanged != null)
                {
                    StatusChanged(ConnectionState.Disconnected, "Disconnected from server.");
                }
                _clientSocket.Close();
            }
            catch (Exception e)
            {
                // Error occured, bug do not interrupt reading
                if (_clientSocket != null && _clientSocket.Connected && !_isDisconnecting)
                {
                    if (StatusChanged != null)
                    {
                        StatusChanged(ConnectionState.Connected, "Reading error: " + e.Message);
                    }
                    goto repeatReading;
                }

                // Exit reading
                if (StatusChanged != null)
                {
                    if (_isDisconnecting)
                    {
                        _isDisconnecting = false;
                        StatusChanged(ConnectionState.Disconnected, "Disconnected successfully.");
                    }
                    else
                    {
                        ConnectionState state = _clientSocket.Connected ? ConnectionState.Connected : ConnectionState.Disconnected;
                        StatusChanged(state, "Failed to read data: " + e.Message);
                    }
                }
            }
        }

        /// <summary>
        /// Send string data to server
        /// </summary>
        /// <param name="strData">String content.</param>
        public void SendMessage(string strData)
        {
            byte[] msg = Encoding.UTF8.GetBytes(strData);

            if (!_clientSocket.Connected)
            {
                _clientSocket.Close();
                return;
            }
            try
            {
                //int i = clientSocket.Send(msg);
                IAsyncResult asyncSend = _clientSocket.BeginSend(msg, 0, msg.Length, SocketFlags.None, new AsyncCallback(sendCallback), _clientSocket);
                bool success = asyncSend.AsyncWaitHandle.WaitOne(_timeOut, true);
                if (!success)
                {
                    _clientSocket.Close();
                    if (StatusChanged != null)
                    {
                        StatusChanged(_clientSocket.Connected ? ConnectionState.Connected : ConnectionState.Disconnected, "Failed to send message to server. ");
                    }
                }
            }
            catch (Exception e)
            {
                if (StatusChanged != null)
                {
                    StatusChanged(_clientSocket.Connected ? ConnectionState.Connected : ConnectionState.Disconnected, "Failed to send message to server: " + e.Message);
                }
            }
        }

        /// <summary>
        /// Send block data to server
        /// </summary>
        /// <param name="obj">Object.</param>
        public void SendMessage(object obj)
        {
            if (!_clientSocket.Connected)
            {
                _clientSocket.Close();
                return;
            }
            try
            {
                //先得到数据包的长度
                short size = (short)Marshal.SizeOf(obj);
                //把数据包的长度写入byte数组中
                byte[] head = BitConverter.GetBytes(size);
                //把结构体对象转换成数据包，也就是字节数组
                byte[] data = StructToBytes(obj);

                //此时就有了两个字节数组，一个是标记数据包的长度字节数组， 一个是数据包字节数组，
                //同时把这两个字节数组合并成一个字节数组

                byte[] newByte = new byte[head.Length + data.Length];
                Array.Copy(head, 0, newByte, 0, head.Length);
                Array.Copy(data, 0, newByte, head.Length, data.Length);

                //计算出新的字节数组的长度
                int length = Marshal.SizeOf(size) + Marshal.SizeOf(obj);

                //向服务端异步发送这个字节数组
                IAsyncResult asyncSend = _clientSocket.BeginSend(newByte, 0, length, SocketFlags.None, new AsyncCallback(sendCallback), _clientSocket);
                //监测超时
                bool success = asyncSend.AsyncWaitHandle.WaitOne(_timeOut, true);
                if (!success)
                {
                    _clientSocket.Close();
                    if (StatusChanged != null)
                    {
                        StatusChanged(_clientSocket.Connected ? ConnectionState.Connected : ConnectionState.Disconnected, "Time out while sending data!");
                    }
                }

            }
            catch (Exception e)
            {
                if (StatusChanged != null)
                {
                    StatusChanged(_clientSocket.Connected ? ConnectionState.Connected : ConnectionState.Disconnected, "Send message error: " + e.Message);
                }
            }
        }

        private void sendCallback(IAsyncResult asyncSend)
        {

        }

        //结构体转字节数组
        public byte[] StructToBytes(object structObj)
        {
            int size = Marshal.SizeOf(structObj);
            IntPtr buffer = Marshal.AllocHGlobal(size);
            try
            {
                Marshal.StructureToPtr(structObj, buffer, false);
                byte[] bytes = new byte[size];
                Marshal.Copy(buffer, bytes, 0, size);
                return bytes;
            }
            finally
            {
                Marshal.FreeHGlobal(buffer);
            }
        }
        //字节数组转结构体
        public object BytesToStruct(byte[] bytes, Type strcutType)
        {
            int size = Marshal.SizeOf(strcutType);
            IntPtr buffer = Marshal.AllocHGlobal(size);
            try
            {
                Marshal.Copy(bytes, 0, buffer, size);
                return Marshal.PtrToStructure(buffer, strcutType);
            }
            finally
            {
                Marshal.FreeHGlobal(buffer);
            }
        }
    }
}