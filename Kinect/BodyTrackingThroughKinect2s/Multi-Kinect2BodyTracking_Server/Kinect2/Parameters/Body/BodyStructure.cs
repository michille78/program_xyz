using System;
using System.Collections.Generic;
using System.Windows.Media;

//-- Using Kinect2 API --//
using Microsoft.Kinect;

//-- Design Patterns --//
using Patterns.Singleton;

namespace Kinect2.Parameters.Body
{
    class BodyStructure : SingletonBase<BodyStructure>
    {
        #region Members

        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        public readonly double handSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        public readonly double jointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        public readonly double clipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        public readonly float inferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        public readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        public readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        public readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        public readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        public readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        public readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Definition of bones
        /// </summary>
        public List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        public List<Pen> bodyColors;

        #endregion


        #region Properties

        //public List<Pen> BodyColors {
        //    get { return bodyColors; }
        //}

        //public List<Tuple<JointType, JointType>> Bones {
        //    get { return bones; }
        //}

        //public float InferredZPositionClamp {
        //    get { return inferredZPositionClamp; }
        //}

        //public double ClipBoundsThickness {
        //    get { return clipBoundsThickness; }
        //}

        //public Brush HandClosedBrush {
        //    get { return handClosedBrush; }
        //}

        //public Brush HandOpenBrush {
        //    get { return handOpenBrush; }
        //}

        //public Brush HandLassoBrush {
        //    get { return handLassoBrush; }
        //}

        //public Brush TrackedJointBrush {
        //    get { return trackedJointBrush; }
        //}

        //public Brush InferredJointBrush {
        //    get { return inferredJointBrush; }
        //}

        #endregion


        #region Methods

        private BodyStructure() {
            /* Initialzing body structure */
                // A bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();
                // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));
                // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));
                // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));
                // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));
                // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            /* Populate body colors, one for each BodyIndex */
            this.bodyColors = new List<Pen>();
            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));
        }

        #endregion
    }
}
