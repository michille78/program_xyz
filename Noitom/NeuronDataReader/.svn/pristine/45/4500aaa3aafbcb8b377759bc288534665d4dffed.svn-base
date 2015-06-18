/* Copyright: Copyright 2014 Beijing Noitom Technology Ltd. All Rights reserved.
* Pending Patents: PCT/CN2014/085659 PCT/CN2014/071006
* 
* Licensed under the Perception Neuron SDK License Beta Version (the “License");
* You may only use the Perception Neuron SDK when in compliance with the License,
* which is provided at the time of installation or download, or which
* otherwise accompanies this software in the form of either an electronic or a hard copy.
* 
* Unless required by applicable law or agreed to in writing, the Perception Neuron SDK
* distributed under the License is provided on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing conditions and
* limitations under the License.
*/


using System;
using System.Linq;
using System.Text;
using System.Runtime.InteropServices;  // For DllImport()


namespace PN_BVHDataReaderWraper
{
    #region Data types
    // Socket connection status
    public enum SocketConnectionStatus
    {
        CS_Connected,
        CS_Connecting,
        CS_Disconnected,
    };

    // Data version
    public struct DataVersion
    {
        public byte BuildNumb;         // Build number
        public byte Revision;          // Revision number
        public byte Minor;             // Subversion number
        public byte Major;             // Major version number
    };

    // Header format of BVH data
    [StructLayout(LayoutKind.Sequential, Pack=1)]
    public struct BvhDataHeader
    {
        public ushort HeaderToken1;    // Package start token: 0xDDFF
        public DataVersion DataVersion;// Version of community data format. e.g.: 1.0.0.2
        public UInt32 DataCount;       // Values count, 180 for without disp data
        public UInt32 bWithDisp;       // With/out dispement
        public UInt32 bWithReference;  // With/out reference bone data at first
        public UInt32 AvatarIndex;     // Avatar index
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 32)]
        public string AvatarName;      // Avatar name
        public UInt32 Reserved1;       // Reserved, only enable this package has 64bytes length
        public UInt32 Reserved2;       // Reserved, only enable this package has 64bytes length
        public ushort HeaderToken2;    // Package end token: 0xEEFF
    };

    [UnmanagedFunctionPointer(CallingConvention.StdCall)]
    public delegate void FrameDataReceived(IntPtr customObject, IntPtr sockRef, IntPtr BvhDataHeader, IntPtr data);

    [UnmanagedFunctionPointer(CallingConvention.StdCall)]
    public delegate void SocketStatusChanged(IntPtr customObject, IntPtr sockRef, SocketConnectionStatus status, [MarshalAs(UnmanagedType.LPStr)]string msg);
    #endregion

    // API exportor
    public class PN_DataReader
    {
        #region Importor definition
#if UNITY_IPHONE && !UNITY_EDITOR
		private const string PN_DataReaderImportor = "__Internal";
#elif _WINDOWS
		private const string PN_DataReaderImportor = "BVHDataReader.dll";
#else
        private const string PN_DataReaderImportor = "BVHDataReader";
#endif
        #endregion

        #region Initialize
        // Register receiving and parsed frame data callback
        [DllImport(PN_DataReaderImportor, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void BRRegisterFrameDataCallback(IntPtr customedObj, FrameDataReceived handle);

        // If any error, you can call 'strBRGetLastErrorMessage' to get error information
        [DllImport(PN_DataReaderImportor, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        //[return: MarshalAs(UnmanagedType.LPStr)]
		private static extern IntPtr BRGetLastErrorMessage();
		// If any error, you can call 'strBRGetLastErrorMessage' to get error information
        public static string strBRGetLastErrorMessage()
        {
            // Get message pointer
            IntPtr ptr = BRGetLastErrorMessage();
            // Construct a string from the pointer.
			return Marshal.PtrToStringAnsi(ptr);
        }
        #endregion

        #region TCP Service
        // Register TCP socket status callback
        [DllImport(PN_DataReaderImportor, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void BRRegisterConnectionStatusCallback(IntPtr customedObj, SocketStatusChanged handle);

        // Connect to server by TCP/IP
        [DllImport(PN_DataReaderImportor, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern  IntPtr BRConnectTo(string serverIP, int nPort);
        
        // Check TCP connect status
        [DllImport(PN_DataReaderImportor, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern SocketConnectionStatus BRGetConnectionStatus(IntPtr sockRef);

        // Disconnect TCP socket from server
        [DllImport(PN_DataReaderImportor, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void BRDisconnect(IntPtr sockRef);
        #endregion

        #region UPD Service
        // Start a UDP service to receive data at 'nPort'
        [DllImport(PN_DataReaderImportor, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern IntPtr BRStartUDPServiceAt(int nPort);

        // Check UDP sercie if is running
        [DllImport(PN_DataReaderImportor, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern bool BRIsUDPServiceRunning(IntPtr sockRef);

        // Stop UDP service
        [DllImport(PN_DataReaderImportor, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void BRStopUDPService(IntPtr sockRef);
        #endregion
    }
}
