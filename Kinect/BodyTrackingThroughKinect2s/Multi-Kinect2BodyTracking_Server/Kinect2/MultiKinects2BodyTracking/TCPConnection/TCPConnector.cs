using System;
using System.Text;
using System.Threading;
using System.Windows;
    // Using TCP/IP functions
using System.Net.Sockets;

namespace Kinect2.MultiKinects2BodyTracking.TCPConnection {

    /// <summary>
    /// Construct the connection between the MultiKinect2BodyTracking client and server
    /// </summary>
    public class TCPConnector
    {

        #region Members

        /// <summary>
        /// The sockets constructed from .Net framework API
        /// </summary>
        private System.Net.Sockets.TcpClient readingSocket;
        private System.Net.Sockets.TcpClient sendingSocket;

        /// <summary>
        /// Indicate which type of the client is
        /// </summary>
        public int clientType = (int) clientTypes.UNKNOWN;

        #endregion // Members

        #region Methods

        /// <summary>
        /// Constructor
        /// </summary>
        public TCPConnector() {
            clientType = (int) clientTypes.UNKNOWN;
        }

        /// <summary>
        /// Constructor used to initialize the client type
        /// </summary>
        /// <param name="_clientType"></param>
        public TCPConnector(clientTypes _clientType) {
            clientType = (int) _clientType;
        }

        /// <summary>
        /// Initialize the sockets to connect with other program
        /// </summary>
        public void SetupSockets() {
            readingSocket = new System.Net.Sockets.TcpClient();
            sendingSocket = new System.Net.Sockets.TcpClient();
        }

        /// <summary>
        /// Connect to server function
        /// </summary>
        /// <param name="_serverIP"></param>
        public void ConnectToServer(string _serverIP) {
            this.sendingSocket.Connect(_serverIP, 8888);
            this.readingSocket.Connect(_serverIP, 8889);
        }

        /// <summary>
        /// Send data to server
        /// </summary>
        /// <param name="dataToSend"></param>
        public void SendData(string dataToSend) {
            if (string.IsNullOrEmpty(dataToSend))
                return;

            try {
                if (dataToSend == "image data") {
                    //isRecordingImage = true;
                } else {
                    NetworkStream serverStream = sendingSocket.GetStream();
                    serverStream.Flush();

                        // Transfer data string to Bytes[]
                    Byte[] sendBytes = Encoding.ASCII.GetBytes(dataToSend);

                        // Get data length
                    int dataLength = sendBytes.Length;
                    Byte[] dataLengthBytes = System.BitConverter.GetBytes(dataLength);

                        // Send data length
                    serverStream.Write(dataLengthBytes, 0, dataLengthBytes.Length);
                    serverStream.Flush();

                        // Send data body
                    serverStream.Write(sendBytes, 0, sendBytes.Length);
                    serverStream.Flush();
                }

                // Send image data
                //if (imageAvaliable) {
                //    imageAvaliable = false;

                //    int action = (int) UploadCommands.Update_raw_data;
                //    dataToSend = "u " + action.ToString() + " ";

                //    byte[] depthBytes = new byte[Depth.Length * sizeof(short)];
                //    Buffer.BlockCopy(Depth, 0, depthBytes, 0, Depth.Length * sizeof(short));

                //    byte[] points3DBytes = new byte[Depth.Length * 3 * sizeof(float)];
                //    Buffer.BlockCopy(points3D, 0, points3DBytes, 0, Depth.Length * 3 * sizeof(float));

                //        //append RGB data, depth data, and 3D point data to dataToSend string 
                //    StringCompressor s = new StringCompressor();
                //    dataToSend += s.CompressByteArray(RGB) + "*" + s.CompressByteArray(depthBytes) + "*" + s.CompressByteArray(points3DBytes);

                //    NetworkStream serverStream = sendingSocket.GetStream();
                //    serverStream.Flush();

                //    Byte[] sendBytes = Encoding.ASCII.GetBytes(dataToSend);

                //    int dataLength = sendBytes.Length;
                //    Byte[] dataLengthBytes = System.BitConverter.GetBytes(dataLength);

                //        //send data length
                //    serverStream.Write(dataLengthBytes, 0, dataLengthBytes.Length);
                //    serverStream.Flush();

                //    Thread.Sleep(1000);
                //    //send data body
                //    serverStream.Write(sendBytes, 0, sendBytes.Length);
                //    serverStream.Flush();
                //}
            } catch (Exception ex)  {
                MessageBox.Show(ex.ToString());
            }
        }

        /// <summary>
        /// Receive data from server with a maximum waiting time
        /// </summary>
        /// <returns></returns>
        public string ReceiveData_wait()
        {
            string receivedData = "";
            try {
                NetworkStream serverStream = readingSocket.GetStream();
                serverStream.ReadTimeout = 10;

                //the loop should continue until no data available to read and message string is filled.
                //if data is not available and message is empty then the loop should continue, until
                //data is available and message is filled.
                DateTime time_start = DateTime.Now;
                while ((DateTime.Now - time_start).TotalMilliseconds < serverStream.ReadTimeout) {
                    if (serverStream.DataAvailable) {
                            // The size of byte array storing data length = 4
                        int headerSize = 4;
                        Byte[] bb = new Byte[headerSize];
                        int read = serverStream.Read(bb, 0, headerSize);

                        int dataLength = 0;
                        try {
                            dataLength = System.BitConverter.ToInt32(bb, 0);
                        } catch {
                            break;
                        }

                        //dataLength == 0 means no data
                        if (dataLength == 0) continue;

                        bb = new Byte[dataLength];
                        read = serverStream.Read(bb, 0, dataLength);

                        if (read > 0)
                            receivedData = receivedData + Encoding.Default.GetString(bb, 0, read);
                    }
                    else if (receivedData.Length > 0)
                        break;
                }
            }
            catch {
                ////cannot receive data
                //this.isUpdating = false;
                //this.isPrinting = false;
                //this.isAudioReading = false;
            }

            return receivedData;
        }

        /// <summary>
        /// Disconnect from the server
        /// </summary>
        public string CloseConnection() {
            /* To send disconnect message */
            string s = "";

            SendData(((int)OtherCommands.Disconnect_from_server_server_hangs_and_wait_for_this_client_to_close_connection).ToString());
            SendData(((int)OtherCommands.Disconnect_from_server_server_hangs_and_wait_for_this_client_to_close_connection).ToString());
            SendData(((int)OtherCommands.Disconnect_from_server_server_hangs_and_wait_for_this_client_to_close_connection).ToString());
            s = ReceiveData_wait();

                // Close sockets
            readingSocket.Close();
            sendingSocket.Close();
            
            return s;
        }

        #endregion // Methods
    }
}
