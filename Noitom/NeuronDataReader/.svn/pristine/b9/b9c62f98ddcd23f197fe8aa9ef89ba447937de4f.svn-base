using System.Collections;
using System;
using System.Text;
using System.Runtime.InteropServices;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Formatters.Binary;

namespace PN_BVHDataReader
{
    public class ParsingBinaryData : IDataParseDelegate
    {
        private const int _headSize = 64;
        private UInt16 _headTag;

        // BVH header
        private BvhDataHeader _bvhHeader = new BvhDataHeader();
        // BVH data
        private float[] _floatValues = new float[180];
        // Frame data callback
        public override event FrameDataReceivedEvent FrameDataReceived;

        public ParsingBinaryData()
        {
            _bvhHeader.HeaderToken1 = 0xDDFF;
            _bvhHeader.HeaderToken2 = 0xEEFF;
        }

        /// <summary>
        /// Call this port to parse data
        /// </summary>
        /// <param name="data">Socket raw data.</param>
        /// <param name="len">Data length.</param>
        public override void Parse(byte[] data, int len)
        {
            // Too small, throw away
            if (len < _headSize) return;

            // Splite large pack
            int pos = 0;
            while (pos < len)
            {
                // check header
                // tag1
                _headTag = BitConverter.ToUInt16(data, pos);
                if (_headTag != 0xDDFF)
                {
                    // move index
                    pos++;
                    continue;
                }

                // tag2
                _headTag = BitConverter.ToUInt16(data, pos + _headSize - 2);
                if (_headTag != 0xEEFF)
                {
                    // move index
                    pos++;
                    continue;
                }

                // read whole header
                _bvhHeader = (BvhDataHeader)BytesToStruct(data, pos, typeof(BvhDataHeader));

                // check length
                if (_bvhHeader.DataCount <= 0)
                {
                    pos++;
                    continue;
                }

                // check memory length
                if (_bvhHeader.DataCount != _floatValues.Length)
                {
                    _floatValues = new float[_bvhHeader.DataCount];
                }

                // Bytes of data array
                int dataLen = (int)_bvhHeader.DataCount * sizeof(float);

                // byte[] to float[]
                Buffer.BlockCopy(data, pos + _headSize, _floatValues, 0, dataLen);

                // update position
                pos += _headSize + dataLen;

                // output
                if (FrameDataReceived != null)
                {
                    FrameDataReceived(_bvhHeader, _floatValues);
                }
            }
        }

        // Convert struct to byte[] array
        private byte[] StructToBytes(object structObj)
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

        // Convert byte[] array to struct
        private object BytesToStruct(byte[] bytes, int startIndex, Type strcutType)
        {
            int size = Marshal.SizeOf(strcutType);
            IntPtr buffer = Marshal.AllocHGlobal(size);
            try
            {
                Marshal.Copy(bytes, startIndex, buffer, size);
                return Marshal.PtrToStructure(buffer, strcutType);
            }
            finally
            {
                Marshal.FreeHGlobal(buffer);
            }
        }
    }
}