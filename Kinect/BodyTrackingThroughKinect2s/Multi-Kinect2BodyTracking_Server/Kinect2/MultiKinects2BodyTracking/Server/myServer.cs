using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Windows.Controls;
using System.Windows;
using System.Windows.Media.Imaging;
using System.Windows.Media;
using System.IO;

using MathNet.Numerics.LinearAlgebra.Double;

using Kinect2.MultiKinects2BodyTracking.DataStructure;
using Kinect2.MultiKinects2BodyTracking.TCPConnection;

namespace Kinect2.MultiKinects2BodyTracking.Server {

    public class myServer {

        #region Members

        /* GUI Components */
        private const int WIDTH = 640;
        private const int HEIGHT = 480;
        GUIComponents guiComp;

        /* TCP Components */
        private static TcpListener tcpReadListener;
        private static TcpListener tcpSendListener;
        public List<myClient> clientList = new List<myClient>();
        public int[] ClientTypeCount = new int[] { 0, 0, 0, 0 }; // 4 client types defined
        string serverIP;
        TextBlock serverMsgTxtBlk;

        // public string fusedKinectParametersString = "";
        // string messageToPrint;

        public KinectData fusedKinectParameter = new KinectData();

        //string prevString;
        int msgLineCount = 0;
        
        #endregion // Members

        #region Methods

        public myServer(MainWindow mw, GUIComponents inGuiComp) {
            serverIP = getLocalIPAddress();
            mw.Title = "Kinect Admin GUI - " + serverIP + ":8888/8889 (read/send)";
            guiComp = inGuiComp;
            serverMsgTxtBlk = guiComp.serverMsgTxtBlock;
            StartServer();
        }

        public string getLocalIPAddress() {
            IPHostEntry host;
            string localIP = "";
            host = Dns.GetHostEntry(Dns.GetHostName());
            foreach (IPAddress ip in host.AddressList)
            {
                if (ip.AddressFamily == AddressFamily.InterNetwork)
                {
                    localIP = ip.ToString();
                }
            }
            return localIP;
        }

        public void StartServer()
        {
            try
            {
                // string strHostName = Dns.GetHostName();
                System.Net.IPAddress localIPAddress = System.Net.IPAddress.Parse(serverIP);
                IPEndPoint ipLocal = new IPEndPoint(localIPAddress, 8888);
                tcpReadListener = new TcpListener(ipLocal);
                tcpReadListener.Start();
                IPEndPoint ipLocal2 = new IPEndPoint(localIPAddress, 8889);
                tcpSendListener = new TcpListener(ipLocal2);
                tcpSendListener.Start();
                WaitForClientConnect();
            }
            catch (Exception ex)
            {
                string exx = ex.ToString();

            }
        }

        private void WaitForClientConnect()
        {
            tcpReadListener.BeginAcceptTcpClient(new System.AsyncCallback(OnReadClientConnect), new object());
            tcpSendListener.BeginAcceptTcpClient(new System.AsyncCallback(OnSendClientConnect), new object());
        }

        TcpClient readClientSocket = null, sendClientSocket = null;
        private void OnReadClientConnect(IAsyncResult asyn)
        {
            try
            {
                readClientSocket = default(TcpClient);
                readClientSocket = tcpReadListener.EndAcceptTcpClient(asyn);

                if (readClientSocket != null && sendClientSocket != null)
                {
                    myClient clientReq = new myClient(this, readClientSocket, sendClientSocket, guiComp);
                    clientList.Add(clientReq);
                    clientReq.StartClient();
                    updateServerMsg("New Client Connected!");
                    readClientSocket = null;
                    sendClientSocket = null;

                }
            }
            catch (Exception se)
            {
                MessageBox.Show("Client connection error :" + se.ToString());
            }

            WaitForClientConnect();
        }

        private void OnSendClientConnect(IAsyncResult asyn)
        {
            try
            {
                sendClientSocket = default(TcpClient);
                sendClientSocket = tcpSendListener.EndAcceptTcpClient(asyn);

                if (readClientSocket != null && sendClientSocket != null)
                {
                    myClient clientReq = new myClient(this, readClientSocket, sendClientSocket, guiComp);
                    clientList.Add(clientReq);
                    clientReq.StartClient();
                    updateServerMsg("New Client Connected!");
                    readClientSocket = null;
                    sendClientSocket = null;

                }
            }
            catch (Exception se)
            {
                MessageBox.Show("Client connection error :" + se.ToString());
            }

            WaitForClientConnect();
        }

        /// <summary>
        /// Delegates and callbacks for multi-threading
        /// </summary>
        /// <param name="text"></param>
        delegate void updateServerMsgCallback(string text);
        public void updateServerMsg(string text)
        {
            if (!MainWindow.getUpdateGUI()) return;
            if (this.serverMsgTxtBlk.Dispatcher.Thread != Thread.CurrentThread)
            {
                this.serverMsgTxtBlk.Dispatcher.Invoke(System.Windows.Threading.DispatcherPriority.Normal, new updateServerMsgCallback(this.updateServerMsg), text);
            }
            else
            {
                //restrict the line number of GUI textbox to reduce memory cost
                if (msgLineCount < 15)
                {
                    msgLineCount++; ;
                    serverMsgTxtBlk.Text = text + Environment.NewLine + serverMsgTxtBlk.Text;
                }
                else
                {
                    msgLineCount = 0;
                    serverMsgTxtBlk.Text = text;
                }
            }
        }

        #endregion // Methods

        /// <summary>
        /// This class define a client which can be one of the types defined in clientTypes
        /// </summary>
        public class myClient {

            #region Members

            public TcpClient readClientSocket;
            public TcpClient sendClientSocket;
            NetworkStream readNetworkStream = null;
            NetworkStream sendNetworkStream = null;
            public int clientID;
            public int clientType;
            public bool isWorking = false;
            public KinectData kinectParameter = new KinectData();

            int currentBufferSize = 0;
            public GUIComponents parentGUI;// kinectTxtBlk;
            myServer server;

            #endregion // Members

            #region Mehtods

            public myClient(myServer inServer, TcpClient readClientConnected, TcpClient sendClientConnected, GUIComponents inParentGUI)
            {
                this.sendClientSocket = sendClientConnected;
                this.readClientSocket = readClientConnected;
                parentGUI = inParentGUI;
                server = inServer;
            }

            public void StartClient()
            {
                try
                {
                    readNetworkStream = readClientSocket.GetStream();
                    server.updateServerMsg("Got read stream!");
                    sendNetworkStream = sendClientSocket.GetStream();
                    server.updateServerMsg("Got send stream!");
                    

                    sendData("Welcome! Please send client type to get your client ID: ");
                    //updateKinectInfo("Connecting....");
                    byte[] buffer = new byte[readClientSocket.ReceiveBufferSize];
                    //WaitForRequest();// 
                    readNetworkStream.BeginRead(buffer, 0, buffer.Length, ReadDataLengthCallback, buffer);

                }
                catch (Exception ex)
                {
                    updateKinectInfo("Connection failed!");
                    MessageBox.Show(ex.ToString());
                }
            }

            public void WaitForRequest()
            {
                try
                {
                    if (isWorking)
                    {
                        byte[] buffer = new byte[4];
                        readNetworkStream.BeginRead(buffer, 0, buffer.Length, ReadDataLengthCallback, buffer);
                    }

                }
                catch { }
            }

            public void ReadDataLengthCallback(IAsyncResult result)
            {
                try
                {
                    int read = readNetworkStream.EndRead(result);
                    if (read == 0)
                    {
                        readNetworkStream.Close();
                        readClientSocket.Close();

                        sendNetworkStream.Close();
                        sendClientSocket.Close();
                        return;
                    }

                    byte[] buffer = result.AsyncState as byte[];
                    currentBufferSize = System.BitConverter.ToInt32(buffer, 0);// int.Parse(data);

                    if (currentBufferSize != 0)
                    {
                        buffer = new byte[currentBufferSize];
                        if (initial)
                            readNetworkStream.BeginRead(buffer, 0, buffer.Length, SendInitialMessage, buffer);
                        else
                            readNetworkStream.BeginRead(buffer, 0, buffer.Length, ReadCallback, buffer);
                    }
                    else
                    {

                        MessageBox.Show(currentBufferSize.ToString());
                    }
                }
                catch (Exception e)
                {
                    string sss = e.ToString();

                    byte[] buffer = new byte[4];
                    readNetworkStream.BeginRead(buffer, 0, buffer.Length, ReadDataLengthCallback, buffer);

                }


            }

            private void SendInitialMessage(IAsyncResult result)
            {
                int read = readNetworkStream.EndRead(result);
                if (read == 0)
                {
                    readNetworkStream.Close();
                    readClientSocket.Close();

                    sendNetworkStream.Close();
                    sendClientSocket.Close();
                    return;
                }
                try
                {
                    byte[] buffer = result.AsyncState as byte[];
                    string data = Encoding.Default.GetString(buffer, 0, read);
                    data = data.TrimEnd('\0');
                    server.updateServerMsg("Received initial message:"+data);

                    clientType = Int32.Parse(data);
                    server.ClientTypeCount[clientType]++;
                    clientID = server.ClientTypeCount[clientType] - 1;
                    server.updateServerMsg("New Client " + getTypeString(clientType) + clientID + " is connected!!");

                    sendData("Your " + getTypeString(clientType) + "ID:  " + clientID);
                    kinectParameter.kinectID = clientID;
                    isWorking = true;
                    initial = false;



                    WaitForRequest();
                }

                catch (Exception ex)
                {
                    string ss = ex.ToString();
                    MessageBox.Show(ss);
                    // Byte[] sb = Encoding.ASCII.GetBytes("Please send client type to get your client ID");
                    // _networkStream.Write(sb, 0, sb.Length);
                    // _networkStream.Flush();
                    // SendInitialMessage(result);

                    WaitForRequest();
                }
            }

            bool initial = true;
            public static int readCallbackCount = 0;
            private void ReadCallback(IAsyncResult result)
            {
                /*       if (initial)
                       {
                           SendInitialMessage(result);
                           return;
                       }*/
                try
                {
                    int read = readNetworkStream.EndRead(result);
                    if (read == 0)
                    {
                        readNetworkStream.Close();
                        readClientSocket.Close();
                        sendNetworkStream.Close();
                        sendClientSocket.Close();
                        return;
                    }

                    byte[] buffer = result.AsyncState as byte[];
                    string data = Encoding.Default.GetString(buffer, 0, read);
                    data = data.TrimEnd('\0');
                    // string[] command = data.Split('#');
                    // int action = Int32.Parse(inputdatas[0]);
                    // sendRequestData(action, inputdatas[1]);

                    if (!execute_command(data))
                        sendData("Wrong Action!");

                }
                catch (Exception ex)
                {

                    string ss = ex.ToString();
                    ;
                }

                this.WaitForRequest();
            }

            bool SocketConnected(Socket s)
            {
                bool part1 = s.Poll(1000, SelectMode.SelectRead);
                bool part2 = (s.Available == 0);
                if (part1 & part2)
                    return false;
                else
                    return true;
            }

            public string getTypeString(int type)
            {
                switch (type)
                {
                    case (int)clientTypes.KINECT:
                        return "KINECT";
                    case (int)clientTypes.DATA_PROCESSOR:
                        return "DATA_PROCESSOR";
                    case (int)clientTypes.ROBOT:
                        return "ROBOT";
                    default:
                        return "UNKNOWN TYPE";
                }
            }

            public bool execute_command(string command)
            {

               server.updateServerMsg("Received data from " + getTypeString(clientType) + clientID);// + " : " + command.ToString());
                try
                {
                    string[] ss = command.Split(' ');
                    if (ss[0] == "U" || ss[0] == "u")
                        return execute_upload_command(Int32.Parse(ss[1]), ss[2]);
                    else if (ss[0] == "D" || ss[0] == "d")
                    {
                        int action = Int32.Parse(ss[1]);
                        if (action <= (int)DownloadCommands.Get_head_joint_information_of_skeleton_array_from_fused_kinect_data)
                            return execute_download_command(action);
                        else
                            return execute_download_command(action, Int32.Parse(ss[2]));
                    }
                    else
                        return execute_other_command(Int32.Parse(ss[0]));
                }
                catch
                {
                    //command format error!
                    return false;
                }
            }

            DenseMatrix getInverseTranfMatrix(DenseMatrix T) {


                    DenseMatrix R = new DenseMatrix(3, 3);
                    DenseMatrix t = new DenseMatrix(3, 1);
                    for (int row = 0; row < 3; ++row)
                    {
                        for (int col = 0; col < 3; ++col)
                        {
                            R[row,col] = T[row,col];
                        }
                        t[row,0] = T[row,3];
                    }

                    R = (DenseMatrix)R.Transpose();
                    t = -R * t;

                    DenseMatrix T2 = new DenseMatrix(4, 4);
                    for (int row = 0; row < 4; ++row)
                        for (int col = 0; col < 4; ++col)
                        {
                            if (col == 3 && row == 3)
                                T2[row, col] = 1;
                            else if (row == 3)
                                T2[row, col] = 0;
                            else if (col == 3)
                                T2[row, col] = t[row, 0];
                            else
                                T2[row, col] = R[row, col];
                        }
                    return T2;            
            }

            bool execute_download_command(int action, int kinect_index = -2)
            {
                //kinect_index == -1 means all kinects

                DownloadCommands c = (DownloadCommands)action;
               // server.updateServerMsg("Received data from " + getTypeString(clientType) + clientID + " : " + c.ToString());
                if (action > (int)DownloadCommands.Get_head_joint_information_of_skeleton_array_from_fused_kinect_data 
                    && kinect_index == -2) //kinect index not specified
                {
                    return false;
                } try {

                    switch (c)
                    {
                        case DownloadCommands.Get_total_number_of_alive_Kinect_Client:	//Get total number of alive Kinect Client
                            int count = 0;
                            foreach (myClient mc in server.clientList.Where(cc => cc.isWorking && cc.clientType == (int)clientTypes.KINECT))
                            {
                                count++;
                            }

                            sendData((int)DownloadCommands.Get_total_number_of_alive_Kinect_Client + "#" +count.ToString());
                            break;

                        case DownloadCommands.Get_kinect_matrices:
                            List<DenseMatrix> list = GUIComponents.fc.fp.getTranfMatrix();

                            string dataToSend = "";
                            for (int i = 0; i < list.Count; ++i ){
                                for (int row = 0; row < 4; ++row)
                                    for (int col = 0; col < 4; ++col)
                                    {
                                        dataToSend += list[i][row, col].ToString();// +" " + list[i][1, 3].ToString() + " " + list[i][2, 3].ToString() + "#";
                                        if (row == 3 && col == 3)
                                            dataToSend += "#";
                                        else
                                            dataToSend += " ";
                                    }  
                            }
                            dataToSend = list.Count.ToString() + "#" + dataToSend;
                            sendData((int)DownloadCommands.Get_kinect_matrices + "#" + dataToSend);
                            //sendData("ERROR");
                            break;

                        case DownloadCommands.Download_all_kinect_data_in_Base64_string_format://	Download all kinect data in Base64 string format
                            dataToSend = "";
                            int clientcount = 0;
                            foreach (myClient mc in server.clientList.Where(cc => cc.isWorking && cc.clientType == (int)clientTypes.KINECT))
                            {
                                dataToSend += mc.kinectParameter.GetAllParameterStringInBase64() + "#";
                                clientcount++;
                            }
                            dataToSend = clientcount.ToString() + "#" + dataToSend;
                            sendData((int)DownloadCommands.Download_all_kinect_data_in_Base64_string_format + "#" + dataToSend);

                            break;

                        case DownloadCommands.Download_fused_kinect_data_in_Base64_string_format://	Download fused kinect data in Base64 string format

                            if (clientType == (int)clientTypes.KINECT &&   MainWindow.getFusedDataToKinect())
                            {

                                KinectData k = server.fusedKinectParameter.Clone();
                                
                                list = GUIComponents.fc.fp.getTranfMatrix();
                                k.transformTo(getInverseTranfMatrix(list[clientID]));
                                sendData((int)DownloadCommands.Download_fused_kinect_data_in_Base64_string_format + "#" + k.GetAllParameterStringInBase64());
                            }
                            else if (clientType != (int)clientTypes.KINECT)
                                sendData((int)DownloadCommands.Download_fused_kinect_data_in_Base64_string_format + "#" + server.fusedKinectParameter.GetAllParameterStringInBase64());
                            break;

                        //case DownloadCommands.Get_total_number_of_faces_from_fused_kinect_data://   Get total number of faces from fused kinect data

                        //    if (server.fusedKinectParameter.faceArray != null)
                        //        sendData((int)DownloadCommands.Get_total_number_of_faces_from_fused_kinect_data + "#" + server.fusedKinectParameter.faceArray.Length.ToString());
                        //    else
                        //        sendData((int)DownloadCommands.Get_total_number_of_faces_from_fused_kinect_data + "#" + "0");
                        //    break;

                        case DownloadCommands.Get_total_number_of_skeletons_from_fused_kinect_data: // Get total number of skeletons from fused kinect data
                            if (server.fusedKinectParameter.skeletonArray != null)
                                sendData((int)DownloadCommands.Get_total_number_of_skeletons_from_fused_kinect_data + "#" + server.fusedKinectParameter.skeletonArray.Length.ToString());
                            else
                                sendData((int)DownloadCommands.Get_total_number_of_skeletons_from_fused_kinect_data + "#" + "0");
                            break;

                        //case DownloadCommands.Get_full_face_parameters_from_fused_kinect_data_orientation_point://	Get full face parameters from fused kinect data
                        //    sendData((int)DownloadCommands.Get_full_face_parameters_from_fused_kinect_data_orientation_point + "#" + server.fusedKinectParameter.getFaceArrayString(KinectData.FaceArrayType.FULL_ARRAY_POINT));
                        //    break;

                        //case DownloadCommands.Get_full_face_parameters_from_fused_kinect_data_radian://	Get full face parameters from fused kinect data 
                        //    sendData((int)DownloadCommands.Get_full_face_parameters_from_fused_kinect_data_radian + "#" + server.fusedKinectParameter.getFaceArrayString(KinectData.FaceArrayType.FULL_ARRAY_RADIAN));
                        //    break;

                        //case DownloadCommands.Get_face_positions_from_fused_kinect_data://	Get face positions from fused kinect data
                        //    sendData((int)DownloadCommands.Get_face_positions_from_fused_kinect_data + "#" + server.fusedKinectParameter.getFaceArrayString(KinectData.FaceArrayType.POSITION_ONLY));
                        //    break;

                        //case DownloadCommands.Get_face_orientation_point_from_fused_kinect_data://	Get face orientation point from fused kinect data
                        //    sendData((int)DownloadCommands.Get_face_orientation_point_from_fused_kinect_data + "#" + server.fusedKinectParameter.getFaceArrayString(KinectData.FaceArrayType.POINT_ONLY));
                        //    break;

                        //case DownloadCommands.Get_face_orientation_angle_from_fused_kinect_data: //	Get face orientation angle from fused kinect data
                        //    sendData((int)DownloadCommands.Get_face_orientation_angle_from_fused_kinect_data + "#" + server.fusedKinectParameter.getFaceArrayString(KinectData.FaceArrayType.RADIAN_ONLY));
                        //    break;

                        case DownloadCommands.Get_skeleton_position_array_from_fused_kinect_data://	Get skeleton position array from fused kinect data
                            sendData((int)DownloadCommands.Get_skeleton_position_array_from_fused_kinect_data + "#" + server.fusedKinectParameter.GetSkeletonArrayString(KinectData.SkeletonArrayType.POSITION_ONLY));
                            break;

                        case DownloadCommands.Get_full_skeleton_array_from_fused_kinect_data://	Get full skeleton array from fused kinect data
                            sendData((int)DownloadCommands.Get_full_skeleton_array_from_fused_kinect_data + "#" + server.fusedKinectParameter.GetSkeletonArrayString(KinectData.SkeletonArrayType.FULL_ARRAY));
                            break;

                        case DownloadCommands.Get_upper_body_joint_information_of_skeleton_array_from_fused_kinect_data://	Get upper body joint information of skeleton array from fused kinect data
                            sendData((int)DownloadCommands.Get_upper_body_joint_information_of_skeleton_array_from_fused_kinect_data + "#" + server.fusedKinectParameter.GetSkeletonArrayString(KinectData.SkeletonArrayType.UPPER_BODY));
                            break;

                        case DownloadCommands.Get_head_joint_information_of_skeleton_array_from_fused_kinect_data://	Get head joint information of skeleton array from fused kinect data
                            sendData((int)DownloadCommands.Get_head_joint_information_of_skeleton_array_from_fused_kinect_data + "#" + server.fusedKinectParameter.GetSkeletonArrayString(KinectData.SkeletonArrayType.HEAD_ONLY));
                            break;

                        //case DownloadCommands.Get_sound_parameters_from_a_specified_kinect://	Get sound parameters from a specified kinect
                        //    dataToSend = "";
                        //    clientcount = 0;

                        //    foreach (myClient mc in server.clientList.Where(cc => cc.isWorking && cc.clientType == (int)clientTypes.KINECT))
                        //    {
                        //        if (kinect_index == -1 || kinect_index == mc.clientID)
                        //        {
                        //            dataToSend += mc.kinectParameter.sp.getParameterString() + "#";
                        //            clientcount++;
                        //        }
                        //    }
                        //    dataToSend = clientcount.ToString() + "#" + dataToSend;
                        //    sendData((int)DownloadCommands.Get_sound_parameters_from_a_specified_kinect + "#" + dataToSend);
                        //    break;

                        //case DownloadCommands.Get_full_face_parameters_from_a_specified_kinect_rotation_in_orientation_point://	Get full face parameters from a specified kinect (rotation in orientation point)
                        //    sendData((int)DownloadCommands.Get_full_face_parameters_from_a_specified_kinect_rotation_in_orientation_point + "#" +sendFaceString(kinect_index, KinectData.FaceArrayType.FULL_ARRAY_POINT));
                        //    break;

                        //case DownloadCommands.Get_full_face_parameters_from_a_specified_kinect_rotation_in_radian://	Get full face parameters from a specified kinect (rotation in radian)
                        //    sendData((int)DownloadCommands.Get_full_face_parameters_from_a_specified_kinect_rotation_in_radian + "#" +sendFaceString(kinect_index, KinectData.FaceArrayType.FULL_ARRAY_RADIAN));
                        //    break;

                        //case DownloadCommands.Get_face_positions_from_a_specified_kinect://	Get face positions from a specified kinect
                        //    sendData((int)DownloadCommands.Get_face_positions_from_a_specified_kinect + "#" +sendFaceString(kinect_index, KinectData.FaceArrayType.POSITION_ONLY));
                        //    break;

                        //case DownloadCommands.Get_face_orientation_point_from_a_specified_kinect://	Get face orientation point from a specified kinect
                        //    sendData((int)DownloadCommands.Get_face_orientation_point_from_a_specified_kinect + "#" +sendFaceString(kinect_index, KinectData.FaceArrayType.POINT_ONLY));
                        //    break;

                        //case DownloadCommands.Get_face_orientation_angle_from_a_specified_kinect://	Get face orientation angle from a specified kinect
                        //    sendData((int)DownloadCommands.Get_face_orientation_angle_from_a_specified_kinect + "#" +sendFaceString(kinect_index, KinectData.FaceArrayType.RADIAN_ONLY));
                        //    break;

                        case DownloadCommands.Get_skeleton_position_array_from_a_specified_kinect://	Get skeleton position array from a specified kinect
                            sendData((int)DownloadCommands.Get_skeleton_position_array_from_a_specified_kinect + "#" + sendSkeletonString(kinect_index, KinectData.SkeletonArrayType.POSITION_ONLY));
                            break;

                        case DownloadCommands.Get_full_skeleton_array_from_a_specified_kinect://	Get full skeleton array from a specified kinect
                            sendData((int)DownloadCommands.Get_full_skeleton_array_from_a_specified_kinect + "#" + sendSkeletonString(kinect_index, KinectData.SkeletonArrayType.FULL_ARRAY));
                            break;

                        case DownloadCommands.Get_upper_body_joint_information_of_skeleton_array_from_a_specified_kinect://	Get upper body joint information of skeleton array from a specified kinect
                            sendData((int)DownloadCommands.Get_upper_body_joint_information_of_skeleton_array_from_a_specified_kinect + "#" + sendSkeletonString(kinect_index, KinectData.SkeletonArrayType.UPPER_BODY));
                            break;

                        case DownloadCommands.Get_head_joint_information_of_skeleton_array_from_a_specified_kinect://	Get head joint information of skeleton array from a specified kinect
                            sendData((int)DownloadCommands.Get_head_joint_information_of_skeleton_array_from_a_specified_kinect + "#" +sendSkeletonString(kinect_index, KinectData.SkeletonArrayType.HEAD_ONLY));
                            break;

                        case DownloadCommands.Get_object_frame:
                            //dataToSend = GUIComponents.synchyList.Count.ToString()+ "*";
                            dataToSend = "";
                            int synchyCount = 0;
                            for(int i=0; i<GUIComponents.synchyList.Count;++i) 
                            {
                                if (GUIComponents.synchyList[i].Center.Z != 0 && GUIComponents.synchyList[i].xUnit.Z != Double.NaN)
                                {
                                    ++synchyCount;
                                    dataToSend += GUIComponents.synchyList[i].getSynchyData() + "#";
                                }
                            }
                            dataToSend = synchyCount.ToString() +"#"+ dataToSend;
                            sendData((int)DownloadCommands.Get_object_frame + "#" + dataToSend);
                            break;

                        default: //unknow command
                            return false;
                    }
                    return true;
                }
                catch
                {
                    //command excution error
                    return false;
                }
            }

            string sendSkeletonString(int kinect_index, KinectData.SkeletonArrayType f) {
                string dataToSend = "";
                int clientcount = 0;

                foreach (myClient mc in server.clientList.Where(cc => cc.isWorking && cc.clientType == (int)clientTypes.KINECT))
                {
                    if (kinect_index == -1 || kinect_index == mc.clientID)
                    {
                        dataToSend += mc.kinectParameter.GetSkeletonArrayString(f) + "#";
                        clientcount++;
                    }
                }
                dataToSend = clientcount.ToString() + "#" + dataToSend;
                //sendData(dataToSend);
                return dataToSend;
            }

            bool execute_upload_command(int action, string data) {
                UploadCommands c = (UploadCommands)action;
               // server.updateServerMsg("Received data from " + getTypeString(clientType) + clientID + " : " + c.ToString());
                try
                {
                    switch (c)
                    {
                        case UploadCommands.Update_knect_data_in_Base64_format://	Update knect data in Base64 String format	Used by Kinect Client
                            if (clientType == (int)clientTypes.KINECT)
                            {

                                KinectData k = new KinectData();
                                k.AssignByAllParameterStringInBase64(data);
                                k.transformTo(GUIComponents.fc.fp.getTranfMatrix()[clientID]);
                                kinectParameter = k;
                                updateKinectInfo(kinectParameter.printKinectParameters());
                                return true;
                            }
                            else
                                return false;

                        case UploadCommands.Update_fused_knect_data_in_Base64_format://	Update fused knect data in Base64 String format	Used by Data Processor Client
                            if (clientType == (int)clientTypes.DATA_PROCESSOR)
                            {
                              //  server.fusedKinectParametersString = data;

                                server.fusedKinectParameter.AssignByAllParameterStringInBase64(data);

                                return true;
                            }
                            else
                                return false;

                        case UploadCommands.Update_raw_data:
                            if (clientType == (int)clientTypes.KINECT)
                            {
                                parentGUI.kinectCompList[clientID].updateKinectImages(data);//colorImageWritableBitmap,depthImageWritableBitmap);

                                return true;
                            }
                            return false;

                        default:
                            return false;

                    }
                }
                catch
                {
                    return false;
                }
            }

            bool execute_other_command(int action)
            {
                OtherCommands c = (OtherCommands)action;
               // server.updateServerMsg("Received data from " + getTypeString(clientType) + clientID + " : " + c.ToString());
                try
                {
                    switch (c)
                    {
                        case OtherCommands.Disconnect_from_server_server_close_connection://	Disconnect from server	server close connection
                            foreach (myClient cc in server.clientList)
                            {
                                if (cc.clientID == clientID && cc.clientType == clientType)
                                {
                                    try
                                    {
                                        isWorking = false;

                                        sendData(getTypeString(clientType) + clientID + " is Disconnected");
                                        cc.sendClientSocket.Close();
                                        cc.readClientSocket.Close();
                                        server.clientList.Remove(cc);
                                        server.updateServerMsg(getTypeString(clientType) + clientID + " is Disconnected");
                                        updateKinectInfo("Disconnected");
                                    }
                                    catch (Exception ex)
                                    {
                                        MessageBox.Show(ex.ToString());
                                        server.clientList.Remove(cc);
                                        server.updateServerMsg(getTypeString(clientType) + clientID + "is Disconnected!");
                                    }
                                    return true;
                                }
                            }
                            return false;

                        case OtherCommands.Disconnect_from_server_server_hangs_and_wait_for_this_client_to_close_connection://	Disconnect from server	server hangs and wait for this client to close connection
                            foreach (myClient cc in server.clientList)
                            {
                                if (cc.clientID == clientID && cc.clientType == clientType)
                                {
                                    try
                                    {
                                        while (true)
                                        {
                                            sendData(getTypeString(clientType) + clientID + " is closing...");

                                            Thread.Sleep(200);
                                            server.updateServerMsg(getTypeString(clientType) + clientID + " is closing...");
                                        }
                                    }
                                    catch {
                                        isWorking = false;

                                        server.clientList.Remove(cc);
                                        server.updateServerMsg(getTypeString(clientType) + clientID + " is Disconnected!");
                                        return true;
                                    }

                                }
                            }
                            return false;

                        default:
                            return false;
                    }

                }
                catch
                {
                    return false;
                }
            }

            delegate void updateKinectInfouCallback(string text);
            public void updateKinectInfo(string text) {
                if (!MainWindow.getUpdateGUI())
                    return;

                if (clientType != (int)clientTypes.KINECT)
                    return;

                if (this.parentGUI.kinectCompList[clientID].g.Dispatcher.Thread != Thread.CurrentThread)
                    this.parentGUI.kinectCompList[clientID].g.Dispatcher.Invoke(System.Windows.Threading.DispatcherPriority.Normal, new updateKinectInfouCallback(this.updateKinectInfo), text);
                else {
                    /* print kinect parameters on GUI */
                    this.parentGUI.kinectCompList[clientID].infoTextBlock.Text = "[Kinect " + clientID + "]" + Environment.NewLine
                        + kinectParameter.printKinectParameters() + Environment.NewLine;
                }
            }

            public void sendData(string dataToSend)
            {
               // try
                //{
                    //   dataToSend = dataToSend;
                    Byte[] sendBytes = Encoding.ASCII.GetBytes(dataToSend);
                    int dataLength = sendBytes.Length;
                    Byte[] dataLengthBytes = System.BitConverter.GetBytes(dataLength);

                    sendNetworkStream.Write(dataLengthBytes, 0, dataLengthBytes.Length);
                    sendNetworkStream.Flush();

                    sendNetworkStream.Write(sendBytes, 0, sendBytes.Length);
                    sendNetworkStream.Flush();
                //}
              /*  catch (Exception e)
                {

                    updateKinectInfo("Send Data ERROR!!" + Environment.NewLine + e.ToString());

                }*/
            }

            public void sendImageCommand()//object sender, RoutedEventArgs e)
            {
                int action = (int)DownloadCommands.Get_all_kinect_images;
                string datatosend = "d " + action.ToString() + " ";
                sendData(datatosend);
            }

            #endregion // Methods

        }
    }
}
