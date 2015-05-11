using System;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;

//-- Using Kinect2 API --//
using Microsoft.Kinect;

namespace Kinect2.Streams
{
    class InfraredStream : ImageStream
    {
        #region Members

        /// <summary>
        /// Maximum value (as a float) that can be returned by the InfraredFrame
        /// </summary>
        private const float InfraredSourceValueMaximum = (float)ushort.MaxValue;

        /// <summary>
        /// The value by which the infrared source data will be scaled
        /// </summary>
        private const float InfraredSourceScale = 0.75f;

        /// <summary>
        /// Smallest value to display when the infrared data is normalized
        /// </summary>
        private const float InfraredOutputValueMinimum = 0.01f;

        /// <summary>
        /// Largest value to display when the infrared data is normalized
        /// </summary>
        private const float InfraredOutputValueMaximum = 1.0f;

        /// <summary>
        /// Reader for infrared frames
        /// </summary>
        private InfraredFrameReader infraredFrameReader = null;

        #endregion

        #region Properties

        /// <summary>
        /// Identification for the stream
        /// </summary>
        public override string StreamID {
            get { return "InfraredStream"; }
        }

        #endregion

        #region Methods

        /// <summary>
        /// Constructor of InfraredStream
        /// </summary>
        /// <param name="kinectSensor"></param>
        public InfraredStream(Microsoft.Kinect.KinectSensor kinectSensor)
            : base(kinectSensor)
        {}

        /// <summary>
        /// Start to retrieve the frame
        /// </summary>
        public override void Open() {
                // Open the reader for the depth frames
            this.infraredFrameReader = this.sensor.InfraredFrameSource.OpenReader();
                // Wire handler for frame arrival
            this.infraredFrameReader.FrameArrived += this.Reader_InfraredFrameArrived;
                // Get FrameDescription from InfraredFrameSource
            this.frameDescription = this.sensor.InfraredFrameSource.FrameDescription;
                // Create the bitmap to display
            this.imageBitmap = new WriteableBitmap(this.frameDescription.Width, this.frameDescription.Height, 96.0, 96.0, PixelFormats.Gray32Float, null);

        }

        /// <summary>
        /// Handles the infrared frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_InfraredFrameArrived(object sender, InfraredFrameArrivedEventArgs e)
        {
            // InfraredFrame is IDisposable
            using (InfraredFrame infraredFrame = e.FrameReference.AcquireFrame())
            {
                if (infraredFrame != null)
                {
                    // the fastest way to process the infrared frame data is to directly access 
                    // the underlying buffer
                    using (Microsoft.Kinect.KinectBuffer infraredBuffer = infraredFrame.LockImageBuffer())
                    {
                        // verify data and write the new infrared frame data to the display bitmap
                        if (((this.frameDescription.Width * this.frameDescription.Height) == (infraredBuffer.Size / this.frameDescription.BytesPerPixel)) &&
                            (this.frameDescription.Width == this.imageBitmap.PixelWidth) && (this.frameDescription.Height == this.imageBitmap.PixelHeight))
                        {
                            this.ProcessInfraredFrameData(infraredBuffer.UnderlyingBuffer, infraredBuffer.Size);
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Directly accesses the underlying image buffer of the InfraredFrame to 
        /// create a displayable bitmap.
        /// This function requires the /unsafe compiler option as we make use of direct
        /// access to the native memory pointed to by the infraredFrameData pointer.
        /// </summary>
        /// <param name="infraredFrameData">Pointer to the InfraredFrame image data</param>
        /// <param name="infraredFrameDataSize">Size of the InfraredFrame image data</param>
        private unsafe void ProcessInfraredFrameData(IntPtr infraredFrameData, uint infraredFrameDataSize)
        {
            // infrared frame data is a 16 bit value
            ushort* frameData = (ushort*)infraredFrameData;

            // lock the target bitmap
            this.imageBitmap.Lock();

            // get the pointer to the bitmap's back buffer
            float* backBuffer = (float*)this.imageBitmap.BackBuffer;

            // process the infrared data
            for (int i = 0; i < (int)(infraredFrameDataSize / this.frameDescription.BytesPerPixel); ++i)
            {
                // since we are displaying the image as a normalized grey scale image, we need to convert from
                // the ushort data (as provided by the InfraredFrame) to a value from [InfraredOutputValueMinimum, InfraredOutputValueMaximum]
                backBuffer[i] = Math.Min(InfraredOutputValueMaximum, (((float)frameData[i] / InfraredSourceValueMaximum * InfraredSourceScale) * (1.0f - InfraredOutputValueMinimum)) + InfraredOutputValueMinimum);
            }

            // mark the entire bitmap as needing to be drawn
            this.imageBitmap.AddDirtyRect(new Int32Rect(0, 0, this.imageBitmap.PixelWidth, this.imageBitmap.PixelHeight));

            // unlock the bitmap
            this.imageBitmap.Unlock();
        }

        /// <summary>
        /// Base dipose method for inheritance using
        /// </summary>
        protected override void CloseManagedResource() {
            if (this.infraredFrameReader != null)
                this.infraredFrameReader.Dispose();
        }

        #endregion
    }
}
