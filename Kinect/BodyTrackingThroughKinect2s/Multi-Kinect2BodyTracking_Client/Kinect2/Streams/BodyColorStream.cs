using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;

//-- Using Kinect2 API --//
using Microsoft.Kinect;
using Kinect2.Parameters.Body;

namespace Kinect2.Streams
{
    class BodyColorStream : BodyStream
    {
        #region Members

        /// <summary>
        /// Reader for color frames
        /// </summary>
        private Microsoft.Kinect.ColorFrameReader colorFrameReader = null;

        /// <summary>
        /// Intermediate storage for color and body stream
        /// </summary>
        private WriteableBitmap bodyColorBitmap = null;
        private WriteableBitmap bodyBitmap = null;

        /// <summary>
        /// Use to render the bitmap for body stream
        /// </summary>
        private RenderTargetBitmap bitmapRender = null;

        /// <summary>
        /// Size of the RGB pixel in the bitmap
        /// </summary>
        private readonly int bytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;

        /// <summary>
        /// Intermediate storage for converting the DrawingImage to WriteableBitmap
        /// </summary>
        private byte[] bodyBytespixels = null;
        Image bodyImage = null;
        Grid rootGrid = new Grid();

        #endregion

        #region Properties

        /// <summary>
        /// Identification for the stream
        /// </summary>
        public override string StreamID {
            get { return "BodyColorStream"; }
        }

        /// <summary>
        /// Hide original ImageSource with new, we used imageBitmap in SourceStream
        /// </summary>
        public override ImageSource ImageSource {
            get { return this.bodyColorBitmap; }
        }

        #endregion

        #region Methods

        /// <summary>
        /// Constructor of BodyStream
        /// </summary>
        /// <param name="kinectSensor"></param>
        public BodyColorStream(Microsoft.Kinect.KinectSensor kinectSensor)
            : base(kinectSensor)
        { }

        /// <summary>
        /// Start to retrieve the frame
        /// </summary>
        public override void Open()
        {
            // Get the coordinate mapper
            this.coordinateMapper = this.sensor.CoordinateMapper;

            /* Preparing the bodycolor image to display bodies */
                // Get the color (display) extents
            FrameDescription frameDescription = this.sensor.ColorFrameSource.FrameDescription;
                // Get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

                // Initialize all bitmaps
            this.bodyColorBitmap = new WriteableBitmap(frameDescription.Width, frameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);
            this.bodyBitmap = new WriteableBitmap(frameDescription.Width, frameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);
            //this.bodyColorBitmapResized = new WriteableBitmap(frameDescription.Width / 4, frameDescription.Height / 4, 96.0, 96.0, PixelFormats.Bgr32, null);
            //this.bodyBitmapResized = new WriteableBitmap(frameDescription.Width / 4, frameDescription.Height / 4, 96.0, 96.0, PixelFormats.Bgr32, null);

                // Initialize body image
                // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();
                // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            bitmapRender = new RenderTargetBitmap(displayWidth, displayHeight, 96.0, 96.0, PixelFormats.Pbgra32);

                // Allocate space to put the pixels being received
            this.bodyBytespixels = new byte[frameDescription.Width * frameDescription.Height * this.bytesPerPixel];

                // Access the parameters of body structure
            bodyStructure = BodyStructure.Instance;

                // Open the reader for the body frames
            this.bodyFrameReader = this.sensor.BodyFrameSource.OpenReader();
            this.bodyFrameReader.FrameArrived += this.Reader_BodyFrameArrived;
                // Open the reader for the color frames
            this.colorFrameReader = this.sensor.ColorFrameSource.OpenReader();
            this.colorFrameReader.FrameArrived += this.Reader_ColorFrameArrived;

        }

        /// <summary>
        /// Base dipose method for inheritance using
        /// </summary>
        protected override void CloseManagedResource()
        {
            if (this.colorFrameReader != null)
                this.colorFrameReader.Dispose();

            if (this.bodyFrameReader != null)
                this.bodyFrameReader.Dispose();
        }

        #endregion

        #region EventHandlers

        /// <summary>
        /// Handles the color frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private unsafe void Reader_ColorFrameArrived(object sender, ColorFrameArrivedEventArgs e)
        {
            /* ColorFrame is IDisposable */
            using (ColorFrame colorFrame = e.FrameReference.AcquireFrame())
            {
                if (colorFrame != null)
                {
                    FrameDescription colorFrameDescription = colorFrame.FrameDescription;

                    using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer())
                    {
                        this.bodyColorBitmap.Lock();
                        // Verify data and write the new color frame data to the display bitmap
                        if ((colorFrameDescription.Width == this.bodyColorBitmap.PixelWidth) && (colorFrameDescription.Height == this.bodyColorBitmap.PixelHeight))
                        {
                            colorFrame.CopyConvertedFrameDataToIntPtr(
                                this.bodyColorBitmap.BackBuffer,
                                (uint)(colorFrameDescription.Width * colorFrameDescription.Height * 4),
                                ColorImageFormat.Bgra);

                            this.bodyColorBitmap.AddDirtyRect(new Int32Rect(0, 0, this.bodyColorBitmap.PixelWidth, this.bodyColorBitmap.PixelHeight));
                        }

                        this.bodyColorBitmap.Unlock();

                        /* Resize both two bitmaps */
                        //this.ResizeWriteableBitmap(ref bodyColorBitmap, ref bodyColorBitmapResized, bodyColorBitmap.PixelWidth / 4, bodyColorBitmap.PixelHeight / 4);
                        //this.ResizeWriteableBitmap(ref bodyBitmap, ref bodyBitmapResized, bodyBitmap.PixelWidth / 4, bodyBitmap.PixelHeight / 4);
                        //bodyColorBitmapResized = bodyColorBitmap.Resize(bodyColorBitmap.PixelWidth / 10, bodyColorBitmap.PixelHeight / 10, WriteableBitmapExtensions.Interpolation.Bilinear).Clone();
                        //bodyBitmapResized = bodyBitmap.Resize(bodyBitmap.PixelWidth / 10, bodyBitmap.PixelHeight / 10, WriteableBitmapExtensions.Interpolation.Bilinear).Clone();

                        /* Merge the color and body bitmaps */
                        using (bodyColorBitmap.GetBitmapContext())
                        {
                            using (bodyBitmap.GetBitmapContext(ReadWriteMode.ReadOnly))
                            {
                                /* Do merge */
                                Byte* bodyColorPtr = (Byte*)bodyColorBitmap.BackBuffer;
                                Byte* bodyPtr = (Byte*)bodyBitmap.BackBuffer;
                                int bufferLength = bodyColorBitmap.BackBufferStride * bodyColorBitmap.PixelHeight;

                                for (int i = 0; i < bufferLength; i = i + 4)
                                {
                                    if (bodyPtr[i] == 255 && bodyPtr[i + 1] == 255 && bodyPtr[i + 2] == 255)
                                        continue;

                                    bodyColorPtr[i + 0] = bodyPtr[i + 0];
                                    bodyColorPtr[i + 1] = bodyPtr[i + 1];
                                    bodyColorPtr[i + 2] = bodyPtr[i + 2];
                                }
                            }
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_BodyFrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                        this.bodies = new Body[bodyFrame.BodyCount];

                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }

            if (dataReceived)
            {
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.White, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    int penIndex = 0;
                    foreach (Body body in this.bodies)
                    {
                        Pen drawPen = this.bodyStructure.bodyColors[penIndex++];

                        if (body.IsTracked == true)
                        {
                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            /* Convert the joint points to depth (display) space */
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                    position.Z = this.bodyStructure.inferredZPositionClamp;

                                ColorSpacePoint colorSpacePoint = this.coordinateMapper.MapCameraPointToColorSpace(joints[jointType].Position);
                                jointPoints[jointType] = new Point(colorSpacePoint.X, colorSpacePoint.Y);
                            }

                            this.DrawBody(joints, jointPoints, dc, drawPen);

                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                        }
                    }
                        // Prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    /* Transforming DrawingImage to WriteableImage, are there any other better methods to do this? */
                    bodyImage = new Image { Source = imageSource, Width = this.displayWidth, Height = this.displayHeight };

                    rootGrid.Children.Clear();
                    rootGrid.Children.Add(bodyImage);
                    rootGrid.Measure(new Size(bodyImage.Width, bodyImage.Height));
                    rootGrid.Arrange(new Rect(0, 0, bodyImage.Width, bodyImage.Height));

                    bitmapRender.Clear();
                    bitmapRender.Render(rootGrid);
                    bitmapRender.CopyPixels(this.bodyBytespixels, displayWidth * this.bytesPerPixel, 0);
                    bodyBitmap.FromByteArray(this.bodyBytespixels);
                }
            }
        }

        #endregion
    }
}
