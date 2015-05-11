using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;

//-- Using Kinect2 API --//
using Microsoft.Kinect;
using Kinect2.Parameters.Body;

namespace Kinect2.Streams
{
    class BodyStream : ImageStream, DataProduction<Body[]>
    {
        #region Members

        /// <summary>
        /// Reader for Body frames
        /// </summary>
        protected Microsoft.Kinect.BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        protected CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Constant parameters for drawing bodies
        /// </summary>
        protected BodyStructure bodyStructure = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        protected Body[] bodies = null;

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        protected DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        protected DrawingImage imageSource;

        /// <summary>
        /// Width of display block
        /// </summary>
        protected int displayWidth = 400;

        /// <summary>
        /// Height of display block
        /// </summary>
        protected int displayHeight = 300;

        #endregion

        #region Properties

        /// <summary>
        /// Identification for the stream
        /// </summary>
        public override string StreamID {
            get { return "BodyStream"; }
        }

        /// <summary>
        /// Hide original ImageSource with new, we used DrawingImage in BodyStream
        /// </summary>
        public override ImageSource ImageSource {
            get { return this.imageSource; }
        }

        /// <summary>
        /// Implement the inferface to output 1-Dimension data
        /// </summary>
        public Body[] DataSource {
            get { return bodies; }
        }

        #endregion

        #region Methods

        /// <summary>
        /// Constructor of BodyStream
        /// </summary>
        /// <param name="kinectSensor"></param>
        public BodyStream(Microsoft.Kinect.KinectSensor kinectSensor)
            : base(kinectSensor)
        { }

        /// <summary>
        /// Start to retrieve the frame
        /// </summary>
        public override void Open()
        {
                // Get the coordinate mapper
            this.coordinateMapper = this.sensor.CoordinateMapper;

            /* Preparing the empty depth image to display bodies */
                // Get the depth (display) extents
            FrameDescription frameDescription = this.sensor.DepthFrameSource.FrameDescription;
                // Get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;
                // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();
                // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

                // Access the parameters of body structure
            bodyStructure = BodyStructure.Instance;

                // Open the reader for the body frames
            this.bodyFrameReader = this.sensor.BodyFrameSource.OpenReader();
            this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
        }

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e) {
            bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame()) {
                if (bodyFrame != null) {
                    if (this.bodies == null)
                        this.bodies = new Body[bodyFrame.BodyCount];

                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }

            if (dataReceived) {
                using (DrawingContext dc = this.drawingGroup.Open()) {
                        // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    int penIndex = 0;
                    foreach (Body body in this.bodies) {
                        Pen drawPen = this.bodyStructure.bodyColors[penIndex++];

                        if (body.IsTracked == true) {
                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            /* Convert the joint points to depth (display) space */
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys) {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                    position.Z = this.bodyStructure.inferredZPositionClamp;

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }

                            this.DrawBody(joints, jointPoints, dc, drawPen);

                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                        }
                    }
                        // Prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }
            }
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        protected void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bodyStructure.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.bodyStructure.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.bodyStructure.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], this.bodyStructure.jointThickness, this.bodyStructure.jointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        protected void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.bodyStructure.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        protected void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.bodyStructure.handClosedBrush, null, handPosition, this.bodyStructure.handSize, this.bodyStructure.handSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.bodyStructure.handOpenBrush, null, handPosition, this.bodyStructure.handSize, this.bodyStructure.handSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.bodyStructure.handLassoBrush, null, handPosition, this.bodyStructure.handSize, this.bodyStructure.handSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        protected void DrawClippedEdges(Body body, DrawingContext drawingContext) {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom)) {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - this.bodyStructure.clipBoundsThickness, this.displayWidth, this.bodyStructure.clipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top)) {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, this.bodyStructure.clipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left)) {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.bodyStructure.clipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right)) {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - this.bodyStructure.clipBoundsThickness, 0, this.bodyStructure.clipBoundsThickness, this.displayHeight));
            }
        }

        /// <summary>
        /// Base dipose method for inheritance using
        /// </summary>
        protected override void CloseManagedResource() {
            if (this.bodyFrameReader != null)
                this.bodyFrameReader.Dispose();
        }

        #endregion
    }
}
