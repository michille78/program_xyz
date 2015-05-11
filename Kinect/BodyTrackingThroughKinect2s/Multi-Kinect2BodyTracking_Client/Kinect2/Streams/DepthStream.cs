using System;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;

//-- Using Kinect2 API --//
using Microsoft.Kinect;

namespace Kinect2.Streams
{
    class DepthStream : ImageStream
    {
        #region Members

        /// <summary>
        /// Map depth range to byte range
        /// </summary>
        private const int MapDepthToByte = 8000 / 256;

        /// <summary>
        /// Reader for depth frames
        /// </summary>
        private Microsoft.Kinect.DepthFrameReader depthFrameReader = null;

        /// <summary>
        /// Intermediate storage for frame data converted to color
        /// </summary>
        private byte[] depthPixels = null;

        #endregion

        #region Properties

        /// <summary>
        /// Identification for the stream
        /// </summary>
        public override string StreamID
        {
            get { return "DepthStream"; }
        }

        #endregion

        #region Methods

        /// <summary>
        /// Constructor of DepthStream
        /// </summary>
        /// <param name="kinectSensor"></param>
        public DepthStream(Microsoft.Kinect.KinectSensor kinectSensor)
            : base(kinectSensor)
        { }

        /// <summary>
        /// Start to retrieve the frame
        /// </summary>
        public override void Open()
        {
            // Open the reader for the depth frames
            this.depthFrameReader = this.sensor.DepthFrameSource.OpenReader();
            // Wire handler for frame arrival
            this.depthFrameReader.FrameArrived += this.Reader_FrameArrived;
            // Get FrameDescription from DepthFrameSource
            this.frameDescription = this.sensor.DepthFrameSource.FrameDescription;
            // Allocate space to put the pixels being received and converted
            this.depthPixels = new byte[this.frameDescription.Width * this.frameDescription.Height];
            // Create the bitmap to display
            this.imageBitmap = new WriteableBitmap(this.frameDescription.Width, this.frameDescription.Height, 96.0, 96.0, PixelFormats.Gray8, null);
        }

        /// <summary>
        /// Handles the depth frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, DepthFrameArrivedEventArgs e)
        {
            bool depthFrameProcessed = false;

            using (DepthFrame depthFrame = e.FrameReference.AcquireFrame())
            {
                if (depthFrame != null)
                {
                    // the fastest way to process the body index data is to directly access 
                    // the underlying buffer
                    using (Microsoft.Kinect.KinectBuffer depthBuffer = depthFrame.LockImageBuffer())
                    {
                        // verify data and write the color data to the display bitmap
                        if (((this.frameDescription.Width * this.frameDescription.Height) == (depthBuffer.Size / this.frameDescription.BytesPerPixel)) &&
                            (this.frameDescription.Width == this.imageBitmap.PixelWidth) && (this.frameDescription.Height == this.imageBitmap.PixelHeight))
                        {
                            // Note: In order to see the full range of depth (including the less reliable far field depth)
                            // we are setting maxDepth to the extreme potential depth threshold
                            ushort maxDepth = ushort.MaxValue;

                            // If you wish to filter by reliable depth distance, uncomment the following line:
                            //// maxDepth = depthFrame.DepthMaxReliableDistance

                            this.ProcessDepthFrameData(depthBuffer.UnderlyingBuffer, depthBuffer.Size, depthFrame.DepthMinReliableDistance, maxDepth);
                            depthFrameProcessed = true;
                        }
                    }
                }
            }

            if (depthFrameProcessed)
            {
                this.RenderDepthPixels();
            }
        }

        /// <summary>
        /// Directly accesses the underlying image buffer of the DepthFrame to 
        /// create a displayable bitmap.
        /// This function requires the /unsafe compiler option as we make use of direct
        /// access to the native memory pointed to by the depthFrameData pointer.
        /// </summary>
        /// <param name="depthFrameData">Pointer to the DepthFrame image data</param>
        /// <param name="depthFrameDataSize">Size of the DepthFrame image data</param>
        /// <param name="minDepth">The minimum reliable depth value for the frame</param>
        /// <param name="maxDepth">The maximum reliable depth value for the frame</param>
        private unsafe void ProcessDepthFrameData(IntPtr depthFrameData, uint depthFrameDataSize, ushort minDepth, ushort maxDepth)
        {
            // depth frame data is a 16 bit value
            ushort* frameData = (ushort*)depthFrameData;

            // convert depth to a visual representation
            for (int i = 0; i < (int)(depthFrameDataSize / this.frameDescription.BytesPerPixel); ++i)
            {
                // Get the depth for this pixel
                ushort depth = frameData[i];

                // To convert to a byte, we're mapping the depth value to the byte range.
                // Values outside the reliable depth range are mapped to 0 (black).
                this.depthPixels[i] = (byte)(depth >= minDepth && depth <= maxDepth ? (depth / MapDepthToByte) : 0);
            }
        }

        /// <summary>
        /// Renders color pixels into the writeableBitmap.
        /// </summary>
        private void RenderDepthPixels()
        {
            this.imageBitmap.WritePixels(
                new Int32Rect(0, 0, this.imageBitmap.PixelWidth, this.imageBitmap.PixelHeight),
                this.depthPixels,
                this.imageBitmap.PixelWidth,
                0);
        }

        /// <summary>
        /// Base dipose method for inheritance using
        /// </summary>
        protected override void CloseManagedResource()
        {
            if (this.depthFrameReader != null)
                this.depthFrameReader.Dispose();
        }

        #endregion
    }
}
