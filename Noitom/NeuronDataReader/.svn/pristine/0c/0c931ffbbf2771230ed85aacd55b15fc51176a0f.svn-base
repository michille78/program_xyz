using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using PN_BVHDataReader;

namespace demo
{
    class Program
    {
        static void Main(string[] args)
        {
            int counter = 0;

            // Create parser
			ParsingBinaryData delegater = new ParsingBinaryData();
			delegater.FrameDataReceived += delegate(BvhDataHeader bvhHeader, float[] frameData)
            {
                counter++;
                Console.WriteLine("Frame " + counter + ": " + frameData.Length);
            };	

            // Create connector
            SocketClient _tcpClient = new SocketClient();
            _tcpClient.StatusChanged += delegate(ConnectionState status, string message)
            {
                Console.WriteLine(status.ToString() + "||" + message);
            };	

			_tcpClient.DataParser = delegater;

            // connect to server
            _tcpClient.Connect("127.0.0.1", 7001);

            // Wait return key
            Console.ReadLine();

            // disconnect
            _tcpClient.Disconnect();
        }
    }
}
