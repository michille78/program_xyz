using System;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;

//-- Using Kinect2 API --//
using Microsoft.Kinect;

namespace Kinect2.Streams
{
    class ColorStream : ImageStream
    {
        #region Members

        /// <summary>
        /// Reader for color frames
        /// </summary>
        private Microsoft.Kinect.ColorFrameReader colorFrameReader = null;

        #endregion

        #region Properties

        /// <summary>
        /// Identification for the stream
        /// </summary>
        public override string StreamID {
            get { return "ColorStream"; }
        }

        /// <summary>
        /// Hide original ImageSource with new, we used imageBitmap in SourceStream
        /// </summary>
        public override ImageSource ImageSource
        {
            get { return this.imageBitmap; }
        }

        #endregion

        #region Methods

        /// <summary>
        /// Constructor of ColorStream
        /// </summary>
        /// <param name="kinectSensor"></param>
        public ColorStream(Microsoft.Kinect.KinectSensor kinectSensor)
            : base(kinectSensor)
        {}

        /// <summary>
        /// Start to retrieve the frame
        /// </summary>
        public override void Open() {
                // Open the reader for the color frames
            this.colorFrameReader = this.sensor.ColorFrameSource.OpenReader();
                // Wire handler for frame arrival
            this.colorFrameReader.FrameArrived += this.Reader_ColorFrameArrived;
                // Create the colorFrameDescription from the ColorFrameSource using Bgra format
            this.frameDescription = this.sensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);
                // Create the bitmap to display
            this.imageBitmap = new WriteableBitmap(frameDescription.Width, frameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);
        }

        /// <summary>
        /// Handles the color frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private unsafe void Reader_ColorFrameArrived(object sender, ColorFrameArrivedEventArgs e)
        {
            /* ColorFrame is IDisposable */
            using (ColorFrame colorFrame = e.FrameReference.AcquireFrame()) {
                if (colorFrame != null) {
                    FrameDescription colorFrameDescription = colorFrame.FrameDescription;

                    using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer()) {
                        this.imageBitmap.Lock();

                        // Verify data and write the new color frame data to the display bitmap
                        if ((colorFrameDescription.Width == this.imageBitmap.PixelWidth) && (colorFrameDescription.Height == this.imageBitmap.PixelHeight)) {
                            colorFrame.CopyConvertedFrameDataToIntPtr(
                                this.imageBitmap.BackBuffer,
                                (uint)(colorFrameDescription.Width * colorFrameDescription.Height * 4),
                                ColorImageFormat.Bgra);

                            this.imageBitmap.AddDirtyRect(new Int32Rect(0, 0, this.imageBitmap.PixelWidth, this.imageBitmap.PixelHeight));
                        }

                        this.imageBitmap.Unlock();
                    }
                }
            }
        }

        /// <summary>
        /// Base dipose method for inheritance using
        /// </summary>
        protected override void CloseManagedResource() {
            if (this.colorFrameReader != null)
                this.colorFrameReader.Dispose();
        }

        #endregion
    }
}
