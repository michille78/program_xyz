using System.Collections;

namespace PN_BVHDataReader
{
    public abstract class IDataParseDelegate
    {
        /// <summary>
        /// Parse the input data. callback and output frame data by event
        /// </summary>
        /// <param name="data">Data.</param>
        /// <param name="len">Length.</param>
        public abstract void Parse(byte[] data, int len);

        /// <summary>
        /// Occured when frame data parsed.
        /// </summary>
        public abstract event FrameDataReceivedEvent FrameDataReceived;
    }
}