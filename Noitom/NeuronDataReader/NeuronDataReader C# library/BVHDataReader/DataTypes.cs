using System;
using System.Collections.Generic;
using System.Text;
using System.Runtime.InteropServices;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Formatters.Binary;

namespace PN_BVHDataReader
{
    public enum ConnectionState
    {
        Disconnected,
        Connected,
    };

    [StructLayout(LayoutKind.Explicit, Size = 4)]
    public struct DataVersion
    {
        [FieldOffset(3)]
        public byte Major;
        [FieldOffset(2)]
        public byte Minor;
        [FieldOffset(1)]
        public byte Revision;
        [FieldOffset(0)]
        public byte BuildNumb;
    };

    //[StructLayoutAttribute(LayoutKind.Sequential, CharSet = CharSet.Ansi, Pack = 1)]
    //public struct BvhDataHeader
    //{
    //    public UInt16 BvhHeaderToken1;		// 0xDDFF
    //    public DataVersion DataVersion;		// Currently it is 1.0.0.3
    //    public UInt32 DataCount;			// 180 for without disp data
    //    public bool WithDisp;             // Bvh data with dispement
    //    public bool WithReference;        // Bvh data with reference bone data at first
    //    public UInt32 AvatarIndex;			// Avatar index
    //    [MarshalAs(UnmanagedType.AnsiBStr, SizeConst = 32)]
    //    public byte[] AvatarName;			// Avatar name
    //    public UInt32 Reserved1;			// Reserve, only enable the header has 64bytes length
    //    public UInt32 Reserved2;			// Reserve, only enable the header has 64bytes length
    //    public UInt16 BvhHeaderToken2;		// 0xEEFF
    //};
    
    /// <summary>
    /// Header format of BVH data
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
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

    /// <summary>
    /// 
    /// </summary>
    /// <param name="roleID"></param>
    /// <param name="frameData"></param>
    public delegate void FrameDataReceivedEvent(BvhDataHeader infoHeader, float[] frameData);

    /// <summary>
    /// 
    /// </summary>
    /// <param name="data"></param>
    /// <param name="length"></param>
    public delegate void SocketReceivedDataEvent(byte[] data, int length);

    /// <summary>
    /// 
    /// </summary>
    /// <param name="status"></param>
    /// <param name="message"></param>
    public delegate void SocketStatusChangedEvent(ConnectionState status, string message);
}
