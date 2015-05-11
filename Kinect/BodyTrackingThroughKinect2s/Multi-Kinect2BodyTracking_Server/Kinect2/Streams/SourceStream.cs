using System;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;

//-- Using Kinect2 API --//
using Microsoft.Kinect;

namespace Kinect2.Streams
{
    /// <summary>
    /// Abstract class of source stream
    /// </summary>
    abstract class SourceStream : IDisposable
    {
        #region Members

        /// <summary>
        /// The core of KinectSensor API
        /// </summary>
        protected Microsoft.Kinect.KinectSensor sensor = null;

        #endregion

        #region Properties

        /// <summary>
        /// Identification for the stream
        /// </summary>
        public abstract string StreamID{
            get;
        }

        #endregion

        #region Methods

        /// <summary>
        /// Constructor of SourceStream, passing the KinectSensor core to this object
        /// </summary>
        /// <param name="kinectSensor"></param>
        public SourceStream(Microsoft.Kinect.KinectSensor kinectSensor) {
            this.sensor = kinectSensor;
        }

        /// <summary>
        /// Start to retrieve the frame
        /// </summary>
        abstract public void Open();

        //==============================================//
        //              Disposing Methods               //
        //==============================================//
        /// <summary>
        /// Implement the IDisposable interface
        /// </summary>
        public void Dispose() {
            this.CloseManagedResource();
        }

        /// <summary>
        /// Base dipose method for inheritance using
        /// </summary>
        protected abstract void CloseManagedResource();

        /// <summary>
        /// Alias for the dispose method
        /// </summary>
        public void Close() {
            this.Dispose();
        }

        #endregion
    }

    /// <summary>
    /// Class for manipulating image stream
    /// </summary>
    abstract class ImageStream : SourceStream
    {
        #region Members

        /// <summary>
        /// Bitmap to display
        /// </summary>
        protected WriteableBitmap imageBitmap = null;

        /// <summary>
        /// Description of the data contained in the source frame
        /// </summary>
        protected FrameDescription frameDescription = null;

        #endregion

        #region Properties

        /// <summary>
        /// Gets the bitmap of the source to display
        /// </summary>
        public virtual ImageSource ImageSource {
            get { return this.imageBitmap; }
        }

        #endregion

        #region Methods

        /// <summary>
        /// Constructor of ImageStream, passing the KinectSensor core to this object
        /// </summary>
        /// <param name="kinectSensor"></param>
        public ImageStream(Microsoft.Kinect.KinectSensor kinectSensor)
            : base(kinectSensor)
        {}

        /// <summary>
        /// Resize the WriteableBitmap
        /// </summary>
        /// <param name="srcBitmap"></param>
        /// <param name="width"></param>
        /// <param name="height"></param>
        /// <returns></returns>
        public void ResizeWriteableBitmap(ref WriteableBitmap srcBitmap, ref WriteableBitmap destBitmap, int width, int height) {
            var resizedBitmap = WriteableBitmapExtensions.Resize(srcBitmap, width, height, WriteableBitmapExtensions.Interpolation.Bilinear);

            Rect rec = new Rect(0, 0, width, height);
            using (resizedBitmap.GetBitmapContext()) {
                using (resizedBitmap.GetBitmapContext()) {
                    destBitmap.Blit(rec, resizedBitmap, rec, WriteableBitmapExtensions.BlendMode.None);
                }
            }
        }

        #endregion
    }

    /// <summary>
    /// Interface for accessing the 1-D dimension data
    /// </summary>
    /// <typeparam name="T"></typeparam>
    public interface DataProduction<T>
    {
        T DataSource {
            get;
        }
    } 
}
