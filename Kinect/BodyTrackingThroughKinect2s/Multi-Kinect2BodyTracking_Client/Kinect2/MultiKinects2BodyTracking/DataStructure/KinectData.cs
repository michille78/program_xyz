using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using MathNet.Numerics.LinearAlgebra.Double;
using Microsoft.Kinect;
using System.Windows.Media.Media3D;
using System.IO;
using System.Runtime.Serialization.Formatters.Binary;
using System.Runtime.Serialization;


namespace Kinect2.MultiKinects2BodyTracking.DataStructure {
    /// <summary>
    /// The class of a 3D point
    /// </summary>
    public class Point3 {

        #region Members

        DenseMatrix point3;

        #endregion // Members

        #region Properties

        public double X
        {
            set { point3[0, 0] = value; }
            get { return point3[0, 0]; }
        }

        public double Y
        {
            set { point3[1, 0] = value; }
            get { return point3[1, 0]; }
        }

        public double Z
        {
            set { point3[2, 0] = value; }
            get { return point3[2, 0]; }
        }

        #endregion // Properties

        #region Methods

        public Point3()
        {
            point3 = new DenseMatrix(4, 1);
            point3[0, 0] = 0;
            point3[1, 0] = 0;
            point3[2, 0] = 0;
            point3[3, 0] = 1;
        }

        public Point3(double x, double y, double z)
        {
            point3 = new DenseMatrix(4, 1);
            point3[0, 0] = x;
            point3[1, 0] = y;
            point3[2, 0] = z;
            point3[3, 0] = 1;
        }

        public Point3(Point3 p) {
            point3 = new DenseMatrix(4, 1);
            this.SetValues(p);
            point3[3, 0] = 1;
        }

        public Point3(string s)
        {
            try
            {
                point3 = new DenseMatrix(4, 1);
                string[] ss = s.Split(' ');
                point3[0, 0] = double.Parse(ss[0]);
                point3[1, 0] = double.Parse(ss[1]);
                point3[2, 0] = double.Parse(ss[2]);
                point3[3, 0] = 1;
            }
            catch
            {
                    // Format error
                MessageBox.Show("[WARNING] 3D point format errored in string " + s);
            }
        }

        public Point3(DenseMatrix m)
        {
            point3 = new DenseMatrix(4, 1);
            point3[0, 0] = m[0, 0];
            point3[1, 0] = m[1, 0];
            point3[2, 0] = m[2, 0];
            point3[3, 0] = 1;
        }

        public void SetValues(double x, double y, double z)
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
        }

        public void SetValues(Point3 p)
        {
            this.SetValues(p.X, p.Y, p.Z);
        }

        public DenseMatrix GetPointInMatrix()
        {
            return point3;
        }

        public string GetParameterString()
        {
            return point3[0, 0].ToString("F2") + " " + point3[1, 0].ToString("F2") + " " + point3[2, 0].ToString("F2");// +" ";
        }

        public void TransformInPlace(DenseMatrix T)  //T:4x4 matrix
        {
            point3 = T * point3;
        }

        public Point3 TransformTo(DenseMatrix T) //T:4x4 matrix
        {
            Point3 p = new Point3(T * point3);
            return p;
        }

        #endregion // Methods
    }

    [Serializable]
    public class BodyData {

        #region Members

        protected Dictionary<JointType, Joint> joints = new Dictionary<JointType, Joint>();

        protected CameraSpacePoint centerPoint = new CameraSpacePoint();

        protected ulong trackingId = 0;

        protected TrackingState trackingState = TrackingState.NotTracked;

        #endregion // Members

        #region Properties

            // Gets or sets the skeleton's joints.
        public IDictionary<JointType, Joint> Joints { 
            get { return joints;}
            protected set {
                foreach (JointType jointType in Enum.GetValues(typeof(JointType)))
                    joints[jointType] = value[jointType];
            }
        }

            // Gets or sets the skeleton's position.
        public CameraSpacePoint Position {
            get { return centerPoint; }
            set { centerPoint = value; }
        }

            // Gets or sets the skeleton's tracking ID.
        public ulong TrackingId {
            get { return trackingId; }
            set { trackingId = value; }
        }

            // Gets or sets the skeleton's current tracking state.
        public TrackingState TrackingState {
            get { return trackingState; }
            set { trackingState = value; }
        }

        #endregion Properties

        #region Methods

            // Initializes a new instance of the BodyData class.
        public BodyData() {
            Joint empty;
            empty.Position = new CameraSpacePoint();
            empty.TrackingState = TrackingState.NotTracked;

            foreach (JointType jointType in Enum.GetValues(typeof(JointType))) {
                empty.JointType = jointType;
                joints.Add(jointType, empty);
            }

        }

        #endregion Methods

    };

    /// <summary>
    /// The class used to store the tracking results of the Kinect
    /// </summary>
    public class KinectData
    {

        #region Members

        StringCompressor compressor = new StringCompressor();
        public int kinectID = -1;
        public const int bodyCount = 6;

        public BodyData[] skeletonArray = new BodyData[bodyCount]
            {
                new BodyData(),
                new BodyData(),
                new BodyData(),
                new BodyData(),
                new BodyData(),
                new BodyData()
            };

        #endregion // Members

        /// <summary>
        /// Clone a KinectData object
        /// </summary>
        /// <returns></returns>
        public KinectData Clone()
        {
            KinectData newObject = new KinectData();
            newObject.kinectID = kinectID;
            
            try {
                if (skeletonArray != null) {
                    newObject.skeletonArray = new BodyData[skeletonArray.Length];
                    for (int i = 0; i < skeletonArray.Length; ++i) {
                        if (skeletonArray[i] != null)
                            newObject.skeletonArray[i] = skeletonArray[i].CloneSkeleton();
                    }
                }
            } catch {
                    // Format error
                MessageBox.Show("[WARNING] KinectData format errored!");
            }

            return newObject;
        }

        /// <summary>
        /// Print all data in string and compress it
        /// </summary>
        /// <returns></returns>
        public string GetAllParameterStringInBase64() {
                // If no data were stored
            if (kinectID == -1)
                return "";

            string stringizeData = "";
            try {
                stringizeData += kinectID.ToString() + "*";

                if (skeletonArray != null)
                    stringizeData += GetSkeletonArrayString(SkeletonArrayType.FULL_ARRAY);
                else
                    stringizeData += "0*";
            } catch {
                MessageBox.Show("[WARNING] Something was wrong in getAllParameterStringInBase64 function!");
            }

            return compressor.Compress(stringizeData);
        }

        /// <summary>
        /// Decode Base 64 string and save the data to correspondence objects
        /// </summary>
        /// <param name="s"></param>
        public void AssignByAllParameterStringInBase64(string s)
        {
            s = compressor.Decompress(s);

            string[] kinectData = s.Split('*');
            int index = 0;

            try {
                    // KinectID
                kinectID = Int32.Parse(kinectData[index++]);
                    // Body Counts
                int skeletonCount = Int32.Parse(kinectData[index++]);
                if (skeletonCount == 0)
                    skeletonArray = null;
                else { // Body Data
                    skeletonArray = new BodyData[skeletonCount];

                    for (int c = 0; c < skeletonCount; ++c) {
                            // BodyData.TrackingId
                        skeletonArray[c].TrackingId = UInt64.Parse(kinectData[index++]);
                            // BodyData.TrackingState
                        skeletonArray[c].TrackingState = getSkeletonTrackingStateFromString(kinectData[index++]);
                            // BodyData.Position
                        skeletonArray[c].Position = getSkeletonPointFromString(kinectData[index++]);

                        foreach (JointType jointType in Enum.GetValues(typeof(JointType)))
                            skeletonArray[c].Joints[jointType] = setJointPositionFromString(kinectData[index++], skeletonArray[c].Joints[jointType]);
                    }
                }
            } catch (Exception e) {
                string ss = e.ToString();
            }
        }

            // Define difference skeleton types for different applications
        public enum SkeletonArrayType { FULL_ARRAY, UPPER_BODY, POSITION_ONLY, HEAD_ONLY };
        public string GetSkeletonArrayString(SkeletonArrayType type) {
            if (skeletonArray == null)
                return "0*";
            else {
                string s = "";

                List<int> skeletonCount = new List<int>();
                
                try {
                    /* Determine the number of valid bodies in skeletonArray */
                    for (int i = 0; i < skeletonArray.Length; ++i) {
                        if (skeletonArray[i] == null)
                            continue;
                        if (skeletonArray[i].TrackingState != TrackingState.NotTracked)
                            skeletonCount.Add(i);
                    }
                    s += skeletonCount.Count + "*";

                    foreach (BodyData sk in skeletonArray) {
                            // 22*4 +4+1= 88+5=93 doubles <- WT...
                        if (sk == null)
                            continue;
                        if (sk.TrackingState == TrackingState.NotTracked)
                            continue;

                        s += sk.TrackingId + "*";

                        s += sk.TrackingState + "*";

                        if (type != SkeletonArrayType.HEAD_ONLY) {
                            s += sk.Joints[JointType.SpineMid].Position.X.ToString("F2") + " ";
                            s += sk.Joints[JointType.SpineMid].Position.Y.ToString("F2") + " ";
                            s += sk.Joints[JointType.SpineMid].Position.Z.ToString("F2") + "*";
                        }
                        if (type == SkeletonArrayType.POSITION_ONLY)
                            continue;

                        s += AddJointToString(sk.Joints[JointType.Head]);

                        if (type == SkeletonArrayType.HEAD_ONLY)
                            continue;

                        /* For upper body */
                        s += AddJointToString(sk.Joints[JointType.Neck]);
                        s += AddJointToString(sk.Joints[JointType.SpineShoulder]);
                        s += AddJointToString(sk.Joints[JointType.SpineBase]);

                        s += AddJointToString(sk.Joints[JointType.ShoulderLeft]);
                        s += AddJointToString(sk.Joints[JointType.ElbowLeft]);
                        s += AddJointToString(sk.Joints[JointType.WristLeft]);
                        s += AddJointToString(sk.Joints[JointType.HandLeft]);
                        s += AddJointToString(sk.Joints[JointType.HandTipLeft]);
                        s += AddJointToString(sk.Joints[JointType.ThumbLeft]);

                        s += AddJointToString(sk.Joints[JointType.ShoulderRight]);
                        s += AddJointToString(sk.Joints[JointType.ElbowRight]);
                        s += AddJointToString(sk.Joints[JointType.WristRight]);
                        s += AddJointToString(sk.Joints[JointType.HandRight]);
                        s += AddJointToString(sk.Joints[JointType.HandTipRight]);
                        s += AddJointToString(sk.Joints[JointType.ThumbRight]);

                        if (type == SkeletonArrayType.UPPER_BODY) //12 joints only
                            continue;

                        /* For lower body */
                        s += AddJointToString(sk.Joints[JointType.HipLeft]);
                        s += AddJointToString(sk.Joints[JointType.KneeLeft]);
                        s += AddJointToString(sk.Joints[JointType.AnkleLeft]);
                        s += AddJointToString(sk.Joints[JointType.FootLeft]);

                        s += AddJointToString(sk.Joints[JointType.HipRight]);
                        s += AddJointToString(sk.Joints[JointType.KneeRight]);
                        s += AddJointToString(sk.Joints[JointType.AnkleRight]);
                        s += AddJointToString(sk.Joints[JointType.FootRight]);
                    }
                } catch (Exception ex) {
                    string sss = ex.ToString();
                    MessageBox.Show("[WARNING] Something was wrong in GetSkeletonArrayString");
                }

                return s;
            }
        }

        //pring kinect data as readable string
        public string printKinectParameters()
        {
            string t = "";
            try
            {
                //t += "---------Print Kinect parameters---------" + Environment.NewLine;

                t += "<skeleton tracking>" + Environment.NewLine + Environment.NewLine;
                if (skeletonArray != null) {
                    for (int i = 0; i < skeletonArray.Length; ++i) {
                        if (skeletonArray[i] == null)
                            continue;
                        if (skeletonArray[i].TrackingState != TrackingState.NotTracked) {
                            t += "ID: " + skeletonArray[i].TrackingId +
                                 Environment.NewLine + "Status: " + skeletonArray[i].TrackingState;

                            t += Environment.NewLine + "Position: " + skeletonArray[i].Position.X.ToString("F2") + " " + skeletonArray[i].Position.Y.ToString("F2") + " " + skeletonArray[i].Position.Z.ToString("F2") + Environment.NewLine + Environment.NewLine;
                        }
                    }
                }
            } catch (Exception e) {
                t += Environment.NewLine + "fail to print! :" + e.ToString();
            }
            return t;
        }

        /// <summary>
        /// Print a joint data to string
        /// </summary>
        /// <param name="j"></param>
        /// <returns></returns>
        public string AddJointToString(Joint j) {
            string s = "";

            s += j.TrackingState.ToString();
            if (j.TrackingState != TrackingState.NotTracked) {
                s += " " + j.Position.X.ToString("F2") + 
                     " " + j.Position.Y.ToString("F2") +
                     " " + j.Position.Z.ToString("F2") + "*";
            }
            else {
                s += "*";
            }

            return s;
        }

        /// <summary>
        /// Set data to a joint from string
        /// </summary>
        /// <param name="s"></param>
        /// <param name="j"></param>
        /// <returns></returns>
        Joint setJointPositionFromString(string s, Joint j) {
            string[] data = s.Split(' ');

            j.TrackingState = getJointTrackingStateFromString(data[0]);
            if (j.TrackingState != TrackingState.NotTracked) {
                CameraSpacePoint ss = j.Position;
                ss.X = float.Parse(data[1]);
                ss.Y = float.Parse(data[2]);
                ss.Z = float.Parse(data[3]);
                j.Position = ss;
            }
            return j;
        }

        Joint setJointFromValue(CameraSpacePoint sp, TrackingState sts, Joint j) {
            j.Position = sp;
            j.TrackingState = sts;

            return j;
        }

        CameraSpacePoint getSkeletonPointFromString(string s) {
            string[] ss = s.Split(' ');
            CameraSpacePoint p = new CameraSpacePoint();

            p.X = float.Parse(ss[0]);
            p.Y = float.Parse(ss[1]);
            p.Z = float.Parse(ss[2]);

            return p;
        }


        TrackingState getSkeletonTrackingStateFromString(string s) {
            if (s == "NotTracked")
                return TrackingState.NotTracked;
            else if (s == "PositionOnly")
                return TrackingState.Inferred;
            else
                return TrackingState.Tracked;
        }

        TrackingState getJointTrackingStateFromString(string s) {
            if (s == "Tracked")
                return TrackingState.Tracked;
            else if (s == "Inferred")
                return TrackingState.Inferred;
            else
                return TrackingState.NotTracked;
        }

        /// <summary>
        /// Transform Kinect data according to T
        /// </summary>
        /// <param name="T"></param>
        public void transformTo(DenseMatrix T) {
                // Transform skeleton data
            if (skeletonArray != null) {
                foreach (BodyData s in skeletonArray) {
                    if (s == null)
                        return;

                    if (s.TrackingState == TrackingState.NotTracked)
                        continue;

                    s.Position = transformSkeletonPointto(s.Position, T);
                    foreach (JointType j in Enum.GetValues(typeof(JointType)))
                    {
                        CameraSpacePoint spp = transformSkeletonPointto(s.Joints[j].Position, T);
                        Joint jj = s.Joints[j];
                        jj.Position = spp;
                        s.Joints[j] = jj;
                    }
                }
            }
        }

        /// <summary>
        /// Transform Point3D using T
        /// </summary>
        /// <param name="p"></param>
        /// <param name="T"></param>
        /// <returns></returns>
        public Point3D transformPointto(Point3D p, DenseMatrix T) {
            DenseVector p1 = new DenseVector(4);
            p1[0] = p.X;
            p1[1] = p.Y;
            p1[2] = p.Z;
            p1[3] = 1;

            p1 = T * p1;

            Point3D p2 = new Point3D(p1[0], p1[1], p1[2]);
            return p2;
        }

        /// <summary>
        /// Transform CameraPoint using T
        /// </summary>
        /// <param name="p"></param>
        /// <param name="T"></param>
        /// <returns></returns>
        CameraSpacePoint transformSkeletonPointto(CameraSpacePoint p, DenseMatrix T) {
            DenseVector p1 = new DenseVector(4);
            p1[0] = p.X;
            p1[1] = p.Y;
            p1[2] = p.Z;
            p1[3] = 1;

            p1 = T * p1;

            CameraSpacePoint p2 = new CameraSpacePoint();
            p2.X = (float) p1[0];
            p2.Y = (float) p1[1];
            p2.Z = (float) p1[2];

            return p2;
        }
    }
}
