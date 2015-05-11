//This file handles fuse data mechanism

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MathNet.Numerics.LinearAlgebra.Double;
using System.Windows.Media.Media3D;
using System.Windows.Media;
using System.IO;
using System.Windows.Controls;
using System.Windows;
using System.Threading;
using System.Windows.Shapes;

using Microsoft.Kinect;
using _3DTools;
using Perspective.Wpf3D.Shapes;

using Kinect2.MultiKinects2BodyTracking.DataStructure;
using Kinect2.MultiKinects2BodyTracking.TCPConnection;

namespace Kinect2.MultiKinects2BodyTracking.Server {
    public class FusedDataProcessor {

        #region Members

        int kinectNum = 0;
        myServer server;
        
        KinectData fusedKinectParameters = new KinectData();
        List<KinectData> kinectList = new List<KinectData>();

        private List<DenseMatrix> transfMatrix = new List<DenseMatrix>();
        Object transfMxLock = new Object();
        
        TextBlock infoTxtBlk;
        Viewport3D show_viewport3D;

        GUIComponents.FusedComp parentGUI;

        private UpdateDataThread updateDataThread;
        public Thread updateDataThreadInMain;

        #endregion // Members

        #region Methods

        public List<DenseMatrix> getTranfMatrix() {
            List<DenseMatrix> list = new List<DenseMatrix>();
            lock (transfMatrix) {
                list = transfMatrix;
            }

            return list;
        }

        public static bool isWorking = true;//false;
        public FusedDataProcessor(int in_kinect_num, myServer in_server, GUIComponents.FusedComp in_guiGrid)
        {
            parentGUI = in_guiGrid;
            kinectNum = in_kinect_num;
            server = in_server;
            infoTxtBlk = in_guiGrid.infoTxtBlk;
            show_viewport3D = in_guiGrid.viewPort3D;
            getTransformationMatrix();
            updateDataThread = new UpdateDataThread(this);
            
            updateDataThreadInMain = new Thread(this.updateDataThread.ThreadProc);
            updateDataThreadInMain.IsBackground = true;
            updateDataThreadInMain.Start();
        }

        //int[] skeletonIndexes;

        /// <summary>
        /// Read calibration data from txt log file
        /// </summary>
        public void getTransformationMatrix() {
            transfMatrix.Clear();
            string text = System.IO.File.ReadAllText("matrices.txt");
            //Get transformation matrix from matrices.txt
            string[] matricesData = text.Split(' ');
            for (int i = 0; i < matricesData.Length; i++)
            {
                matricesData[i].TrimEnd('\n');
            }

            int step = 16; // a matrix has 16 numbers

            if (matricesData.Length < step * kinectNum)
            {
                MessageBox.Show("Please calibration first");
                for (int i = 0; i < kinectNum; ++i)
                {
                    transfMatrix.Add(DenseMatrix.CreateIdentity(4));
                }
            }
            else
            {
                for (int i = 0; i < kinectNum; ++i)
                {
                    DenseMatrix m = new DenseMatrix(4);
                    for (int row = 0; row < 4; ++row)
                        for (int col = 0; col < 4; ++col)
                        {
                            m[row, col] = Convert.ToDouble(matricesData[step * i + row * 4 + col]);
                        }
                    transfMatrix.Add(m);
                }
            }
        }

        /// <summary>
        /// Update fused values using kinect datas
        /// </summary>
        public void updateKinectList() {
            kinectList.Clear();
            for (int i = 0; i < server.clientList.Count; ++i)
                if (server.clientList[i].clientType == (int)clientTypes.KINECT && server.clientList[i].isWorking)
                {
                    kinectList.Add(server.clientList[i].kinectParameter);

                }

            if (kinectList.Count == 0) return;

            fusedKinectParameters = kinectList[0].Clone();
            for (int i = 1; i < kinectList.Count; ++i)
            {
                //fusedKinectParameters = getAverageResult(1 / ((double)i + 1), i);
                fusedKinectParameters.skeletonArray = computeFusedSkeleton();
                //fusedKinectParameters.faceArray = computeFusedFace();
            }
        }

        double getDist(Point3D p1, Point3D p2) {
            return Math.Sqrt((p1.X - p2.X) * (p1.X - p2.X) + (p1.Y - p2.Y) * (p1.Y - p2.Y) + (p1.Z - p2.Z) * (p1.Z - p2.Z));
        }

        double getDist(CameraSpacePoint p1, CameraSpacePoint p2) {
            return Math.Sqrt((p1.X - p2.X) * (p1.X - p2.X) + (p1.Y - p2.Y) * (p1.Y - p2.Y) + (p1.Z - p2.Z) * (p1.Z - p2.Z));
        }

        double getDist(Point3 p1, Point3 p2) {
            return Math.Sqrt((p1.X - p2.X) * (p1.X - p2.X) + (p1.Y - p2.Y) * (p1.Y - p2.Y) + (p1.Z - p2.Z) * (p1.Z - p2.Z));
        }

        /// <summary>
        /// Record the ID of tracking kinect with skeleton
        /// </summary>
        class skeletonWithSource {
            public Body skeleton;
            public int sourceIndex;

            public skeletonWithSource(Body s, int i) {
                skeleton = s;
                sourceIndex = i;
            }
        }

        double skeletonMatchingTH = 0.55;
        private double distObjectiveFunction(double dist) {
            //TODO: parameter tunning
            double a = 1, b = 1;
            return a*Math.Exp(-b*dist);
        }

        /// <summary>
        /// Get result using majority decision mechanism
        /// </summary>
        /// <param name="skeleList"></param>
        /// <param name="ID"></param>
        /// <param name="kinectID"></param>
        /// <returns></returns>
        private BodyData getSkeletonAvg(List<BodyData> skeleList, int ID, List<int> kinectID) {
            BodyData result = new BodyData();
            result.TrackingState = TrackingState.Tracked;
            result.TrackingId = (ulong) ID;
            int skeleNum = skeleList.Count;
            float[] weight = new float[skeleNum];

            //2. compute weightings 
            for (int thisKinect = 0; thisKinect < skeleNum; ++thisKinect) {
                if(skeleList[thisKinect].TrackingState != TrackingState.Tracked) {
                    weight[thisKinect] = 0;
                    continue;
                }

                /* compute N : number of joints computed by this skeleton */
                    // Difference measurement between this skeleton and other skeletons
                float D = 0;
                foreach (JointType jointIndex in Enum.GetValues(typeof(JointType))) {
                    int M = 0;
                    float distSum = 0;
                    if (skeleList[thisKinect].Joints[jointIndex].TrackingState == TrackingState.Tracked ) {  
                        /* compute M : number of skeletons tracking the same joint */
                        for (int i = 0; i < skeleNum; ++i) {
                            if (i == thisKinect || skeleList[i].TrackingState == TrackingState.Tracked) 
                                continue;

                            if(skeleList[i].Joints[jointIndex].TrackingState == TrackingState.Tracked) {
                                ++M;
                                distSum += (float) getDist(skeleList[thisKinect].Joints[jointIndex].Position,
                                                    skeleList[i].Joints[jointIndex].Position);
                            }
                        }
                        if (M != 0)
                            D += distSum / M;
                    }
                }
                weight[thisKinect] = (float)distObjectiveFunction(D);
            }


            //-------------------------------log data------------------------------------------
            
            //TODO: modify the log format
            string data = "";
            DateTime now = DateTime.Now;

            //Log data format: [time] [kinect ID] [position x y z] [joint 0 x y z] [joint 1 x y z] 
            //[joint 2 x y z][joint 3 x y z][joint 4 x y z][joint 5 x y z][joint 6 x y z][joint 7 x y z]
            //[joint 8 x y z][joint 9 x y z][joint 10 x y z][joint 11 x y z][joint 12 x y z][joint 13 x y z][joint 14 x y z]
            //[joint 15 x y z][joint 16 x y z][joint 17 x y z][joint 18 x y z][joint 19 x y z][weighting] 
            for (int i = 0; i < skeleNum; ++i)
            {
                data += now.Millisecond + " " + kinectID[i] + " ";
                data += skeleList[i].Position.X + " "
                      + skeleList[i].Position.Y + " "
                      + skeleList[i].Position.Z + " ";

                foreach (JointType jointIndex in Enum.GetValues(typeof(JointType))) {
                    data += skeleList[i].Joints[jointIndex].Position.X + " "
                          + skeleList[i].Joints[jointIndex].Position.Y + " "
                          + skeleList[i].Joints[jointIndex].Position.Z + " ";           
                }
                data += weight[i] + Environment.NewLine;
            }

            System.IO.File.AppendAllText("skeletonData.txt", data);
            
            //-------------------------------log data end---------------------------------------

            //3. compute the weighted average of each joint
            foreach (JointType jointIndex in Enum.GetValues(typeof(JointType))) {
                Joint joint = result.Joints[jointIndex];//new Joint();
                joint.TrackingState = TrackingState.NotTracked;
               
                float x_sum =0, y_sum =0, z_sum =0;
                float denominator = 0;

                for(int i=0; i< skeleNum; ++i) {
                    if(skeleList[i].Joints[jointIndex].TrackingState == TrackingState.Tracked || 
                       skeleList[i].Joints[jointIndex].TrackingState == TrackingState.Inferred) {  
                        x_sum += weight[i]*skeleList[i].Joints[jointIndex].Position.X;
                        y_sum += weight[i]*skeleList[i].Joints[jointIndex].Position.Y;
                        z_sum += weight[i]*skeleList[i].Joints[jointIndex].Position.Z;
                        denominator += weight[i];

                            // Treat both "NotTracked" and "Tracked" as "Tracked"
                        if(joint.TrackingState != TrackingState.Tracked)
                            joint.TrackingState = TrackingState.Tracked;
                    }                        
                }

                if(joint.TrackingState == TrackingState.Tracked ) {
                    CameraSpacePoint sp = new CameraSpacePoint();
                    sp.X = x_sum/denominator;
                    sp.Y = y_sum/denominator;
                    sp.Z = z_sum/denominator;
                    joint.Position = sp;
                } else{
                    CameraSpacePoint sp = new CameraSpacePoint();
                    sp.X = sp.Y = sp.Z =0;
                    joint.Position = sp;
                }
                result.Joints[jointIndex] = joint;
            }

            float xsum = 0, ysum = 0, zsum = 0;
            float denominator2 = 0;
             
            for(int i = 0; i < skeleNum; ++i) 
            {
                xsum += weight[i] * skeleList[i].Position.X;
                ysum += weight[i] * skeleList[i].Position.Y;
                zsum += weight[i] * skeleList[i].Position.Z;
                denominator2 += weight[i];
            }

            CameraSpacePoint skelp = new CameraSpacePoint();
            skelp.X = xsum / denominator2;
            skelp.Y = ysum / denominator2;
            skelp.Z = zsum / denominator2;
            result.Position = skelp;

            return result;
        }

        public BodyData[] computeFusedSkeleton() {
            List<List<int>> kinectIDList = new List<List<int>>();
            List<List<BodyData>> skeletonList = new List<List<BodyData>>(); // skeletonList[type][index in the type]
            
            //1. put similar skeletons( near head position ) into the same list
            for (int kinectIndex = 0; kinectIndex < kinectList.Count; ++kinectIndex) {
                if (kinectList[kinectIndex].skeletonArray == null)
                    continue;  

                for (int skeletonIndex = 0; skeletonIndex < kinectList[kinectIndex].skeletonArray.Length; ++skeletonIndex) {
                        //ignore NotTracked and PositionOnly skeletons!
                    if (kinectList[kinectIndex].skeletonArray[skeletonIndex].TrackingState != TrackingState.Tracked)
                        continue;
                        //ignore skeletons with head not tracked!!
                    if (kinectList[kinectIndex].skeletonArray[skeletonIndex].Joints[JointType.Head].TrackingState == TrackingState.NotTracked)
                        continue;

                    bool isExisting = false;
                    for (int i = 0; i < skeletonList.Count; ++i) {
                        double d = getDist(kinectList[kinectIndex].skeletonArray[skeletonIndex].Joints[JointType.Head].Position,
                                                                                 skeletonList[i][0].Joints[JointType.Head].Position);
                            if (d < skeletonMatchingTH)
                            {
                                isExisting = true;
                                skeletonList[i].Add(kinectList[kinectIndex].skeletonArray[skeletonIndex]);
                                kinectIDList[i].Add(kinectIndex);
                                break;
                            }
                    }

                    if (!isExisting) {
                        List<BodyData> newType = new List<BodyData>();
                        newType.Add(kinectList[kinectIndex].skeletonArray[skeletonIndex]); 
                        skeletonList.Add(newType);

                        List<int> newID = new List<int>();
                        newID.Add(kinectIndex);
                        kinectIDList.Add(newID);
                    }
                }
            }

            int skeletonNum = skeletonList.Count;
            BodyData[] result = new BodyData[skeletonNum];

            for (int typeIndex = 0; typeIndex < skeletonNum; ++typeIndex) {
                result[typeIndex] = getSkeletonAvg(skeletonList[typeIndex], typeIndex, kinectIDList[typeIndex]);//step 2,3 here
            }
            return result;    
        }

        /// <summary>
        /// Old averaging function
        /// Only deal with the average of sound pose now
        /// </summary>
        /// <param name="ratio"></param>
        /// <param name="kinectIndex"></param>
        /// <returns></returns>
        //public KinectData getAverageResult(double ratio, int kinectIndex) //average* result = kp1*ratio + kp2*(1-ratio)
        //{
        //    KinectData result = new KinectData();
        //    result.kinectID = 0;// KinectClientList[0].kinectID;
           
        //    try
        //    {
        //        //result.sp.beamAngle = getAverage(kinectList[kinectIndex].sp.beamAngle, fusedKinectParameters.sp.beamAngle, ratio);
        //        //result.sp.sourceAngle = getAverage(kinectList[kinectIndex].sp.sourceAngle, fusedKinectParameters.sp.sourceAngle, ratio);
        //        //result.sp.sourceConfidence = getAverage(kinectList[kinectIndex].sp.sourceConfidence, fusedKinectParameters.sp.sourceConfidence, ratio);
        //    }
        //    catch { }

        //    return result;
        //}

        double getAverage(double d1, double d2, double ratio) {
            if (d1 == 0)
                return d2;
            if (d2 == 0)
                return d1;
            return (d1 * ratio + d2 * (1 - ratio));
        }

        /// <summary>
        /// 3D drawing functions for showing skeleton, synchy, and kinect on GUI (3DTools library is used)
        /// </summary>
        /// <param name="p0"></param>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        /// <returns></returns>
        private Model3DGroup CreateTriangleModel(Point3D p0, Point3D p1, Point3D p2) {
            MeshGeometry3D mesh = new MeshGeometry3D();
            mesh.Positions.Add(p0);
            mesh.Positions.Add(p1);
            mesh.Positions.Add(p2);
            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(1);
            mesh.TriangleIndices.Add(2);
            Vector3D normal = CalculateNormal(p0, p1, p2);
            mesh.Normals.Add(normal);
            mesh.Normals.Add(normal);
            mesh.Normals.Add(normal);
            Material material = new DiffuseMaterial(
                new SolidColorBrush(Colors.DarkKhaki));
            GeometryModel3D model = new GeometryModel3D(
                mesh, material);
            Model3DGroup group = new Model3DGroup();
            group.Children.Add(model);

            return group;
        }

        private Vector3D CalculateNormal(Point3D p0, Point3D p1, Point3D p2) {
            Vector3D v0 = new Vector3D(
                p1.X - p0.X, p1.Y - p0.Y, p1.Z - p0.Z);
            Vector3D v1 = new Vector3D(
                p2.X - p1.X, p2.Y - p1.Y, p2.Z - p1.Z);
            return Vector3D.CrossProduct(v0, v1);
        }

        private void ClearViewport()
        {
            ModelVisual3D m;
            for (int i = show_viewport3D.Children.Count - 1; i >= 0; i--)
            {
                m = (ModelVisual3D)show_viewport3D.Children[i];
                if (m != null)
                    if (m.Content is DirectionalLight == false)
                        show_viewport3D.Children.Remove(m);
            }
        }

        private void drawCube()
        {
            Model3DGroup cube = new Model3DGroup();
            Point3D p0 = new Point3D(0, 0, 0);
            Point3D p1 = new Point3D(1, 0, 0);
            Point3D p2 = new Point3D(1, 0, 1);
            Point3D p3 = new Point3D(0, 0, 1);
            Point3D p4 = new Point3D(0, 1, 0);
            Point3D p5 = new Point3D(1, 1, 0);
            Point3D p6 = new Point3D(1, 1, 1);
            Point3D p7 = new Point3D(0, 1, 1);
            //front side triangles
            cube.Children.Add(CreateTriangleModel(p3, p2, p6));
            cube.Children.Add(CreateTriangleModel(p3, p6, p7));
            //right side triangles
            cube.Children.Add(CreateTriangleModel(p2, p1, p5));
            cube.Children.Add(CreateTriangleModel(p2, p5, p6));
            //back side triangles
            cube.Children.Add(CreateTriangleModel(p1, p0, p4));
            cube.Children.Add(CreateTriangleModel(p1, p4, p5));
            //left side triangles
            cube.Children.Add(CreateTriangleModel(p0, p3, p7));
            cube.Children.Add(CreateTriangleModel(p0, p7, p4));
            //top side triangles
            cube.Children.Add(CreateTriangleModel(p7, p6, p5));
            cube.Children.Add(CreateTriangleModel(p7, p5, p4));
            //bottom side triangles
            cube.Children.Add(CreateTriangleModel(p2, p3, p0));
            cube.Children.Add(CreateTriangleModel(p2, p0, p1));

            ModelVisual3D model = new ModelVisual3D();
            model.Content = cube;
            this.show_viewport3D.Children.Add(model);

        }

        private Model3DGroup BuildNormals(Point3D p0, Point3D p1, Point3D p2, Vector3D normal)
        {
            Model3DGroup normalGroup = new Model3DGroup();
            Point3D p;
            ScreenSpaceLines3D normal0Wire = new ScreenSpaceLines3D();
            ScreenSpaceLines3D normal1Wire = new ScreenSpaceLines3D();
            ScreenSpaceLines3D normal2Wire = new ScreenSpaceLines3D();
            Color c = Colors.Blue;
            int width = 3;
            normal0Wire.Thickness = width;
            normal0Wire.Color = c;
            normal1Wire.Thickness = width;
            normal1Wire.Color = c;
            normal2Wire.Thickness = width;
            normal2Wire.Color = c;
            double num = 1;
            double mult = .01;
            double denom = mult * 3.0;// Convert.ToDouble(normalSizeTextBox.Text);
            double factor = num / denom;
            p = Vector3D.Add(Vector3D.Divide(normal, factor), p0);
            normal0Wire.Points.Add(p0);
            normal0Wire.Points.Add(p);
            p = Vector3D.Add(Vector3D.Divide(normal, factor), p1);
            normal1Wire.Points.Add(p1);
            normal1Wire.Points.Add(p);
            p = Vector3D.Add(Vector3D.Divide(normal, factor), p2);
            normal2Wire.Points.Add(p2);
            normal2Wire.Points.Add(p);

            //Normal wires are not models, so we can't
            //add them to the normal group.  Just add them
            //to the viewport for now...
            this.show_viewport3D.Children.Add(normal0Wire);
            this.show_viewport3D.Children.Add(normal1Wire);
            this.show_viewport3D.Children.Add(normal2Wire);
            return normalGroup;
        }

        private void drawSynchy()
        {
            DiffuseMaterial material = new DiffuseMaterial(Brushes.Wheat);
            int lineWidth = 3;
            for (int i = 0; i < GUIComponents.synchyList.Count; ++i)
            {
                if (GUIComponents.synchyList[i].Center.Z == 0) continue;

                if ( (bool)parentGUI.show3DCheck.IsChecked)
                {
                    Bar3D baseBar = new Bar3D();
                    baseBar.SideCount = 100;

                    Transform3DGroup transGroupBase = new Transform3DGroup();
                    //   transGroupBase.Children.Add(new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0, 0), -90)));
                    transGroupBase.Children.Add(new ScaleTransform3D(0.1, 0.1, 0.05));
                    transGroupBase = getSynchyTransGroup(transGroupBase, i);

                    double heightFactor = 1.7;
                    transGroupBase.Children.Add(new TranslateTransform3D(-1 * Vector3D.Divide(GUIComponents.synchyList[i].zPar, heightFactor))); //new Vector3D(height*GUIComponents.synchyList[i].zPar.X, GUIComponents.synchyList[i].zPar.Y, GUIComponents.synchyList[i].zPar.Z)));
                    transGroupBase.Children.Add(new TranslateTransform3D(new Vector3D(GUIComponents.synchyList[i].Center.X, GUIComponents.synchyList[i].Center.Y, GUIComponents.synchyList[i].Center.Z)));
                    baseBar.Transform = transGroupBase;//.Identity;
                    baseBar.Material = material;
                    show_viewport3D.Children.Add(baseBar);

                    Box3D bodyBox = new Box3D();
                    double l = 0.08, w = 0.08, h = 0.12;
                    Transform3DGroup transGroupBody = new Transform3DGroup();
                    transGroupBody.Children.Add(new ScaleTransform3D(l, w, h));
                    transGroupBody.Children.Add(new TranslateTransform3D(new Vector3D(-l / 2, -w / 2, -h / 2))); //new Vector3D(height*GUIComponents.synchyList[i].zPar.X, GUIComponents.synchyList[i].zPar.Y, GUIComponents.synchyList[i].zPar.Z)));

                    transGroupBody = getSynchyTransGroup(transGroupBody, i);
                    transGroupBody.Children.Add(new TranslateTransform3D(new Vector3D(GUIComponents.synchyList[i].Center.X, GUIComponents.synchyList[i].Center.Y, GUIComponents.synchyList[i].Center.Z)));
                    bodyBox.Transform = transGroupBody;
                    bodyBox.Material = material;
                    show_viewport3D.Children.Add(bodyBox);

                    Spherical3D headSphere = new Spherical3D();
                    Transform3DGroup transGroupHead = new Transform3DGroup();
                    transGroupHead.Children.Add(new ScaleTransform3D(0.1, 0.1, 0.1));
                    transGroupHead = getSynchyTransGroup(transGroupHead, i);
                    transGroupHead.Children.Add(new TranslateTransform3D(Vector3D.Divide(GUIComponents.synchyList[i].zPar, heightFactor)));
                    transGroupHead.Children.Add(new TranslateTransform3D(new Vector3D(GUIComponents.synchyList[i].Center.X, GUIComponents.synchyList[i].Center.Y, GUIComponents.synchyList[i].Center.Z)));
                    headSphere.Transform = transGroupHead;
                    headSphere.Material = material;
                    show_viewport3D.Children.Add(headSphere);

                    ScreenSpaceLines3D x = new ScreenSpaceLines3D();
                    x.Thickness = lineWidth;
                    x.Color = Colors.Red;
                    Point3D origin = new Point3D(GUIComponents.synchyList[i].Center.X, GUIComponents.synchyList[i].Center.Y, GUIComponents.synchyList[i].Center.Z);
                    Point3D x_unit = new Point3D(GUIComponents.synchyList[i].xUnit.X, GUIComponents.synchyList[i].xUnit.Y, GUIComponents.synchyList[i].xUnit.Z);

                    x.Points.Add(origin);
                    x.Points.Add(x_unit);
                    this.show_viewport3D.Children.Add(x);
                }
                else
                {

                    ScreenSpaceLines3D x = new ScreenSpaceLines3D();
                    ScreenSpaceLines3D y = new ScreenSpaceLines3D();
                    ScreenSpaceLines3D z = new ScreenSpaceLines3D();

                    x.Thickness = lineWidth;
                    x.Color = Colors.Gray;
                    y.Thickness = lineWidth;
                    y.Color = Colors.BlueViolet;
                    z.Thickness = lineWidth;
                    z.Color = Colors.Pink;

                    Point3D origin = new Point3D(GUIComponents.synchyList[i].Center.X, GUIComponents.synchyList[i].Center.Y, GUIComponents.synchyList[i].Center.Z);
                    Point3D x_unit = new Point3D(GUIComponents.synchyList[i].xUnit.X, GUIComponents.synchyList[i].xUnit.Y, GUIComponents.synchyList[i].xUnit.Z);
                    Point3D y_unit = new Point3D(GUIComponents.synchyList[i].yUnit.X, GUIComponents.synchyList[i].yUnit.Y, GUIComponents.synchyList[i].yUnit.Z);
                    Point3D z_unit = new Point3D(GUIComponents.synchyList[i].zUnit.X, GUIComponents.synchyList[i].zUnit.Y, GUIComponents.synchyList[i].zUnit.Z);
                    x.Points.Add(origin);
                    x.Points.Add(x_unit);

                    y.Points.Add(origin);
                    y.Points.Add(y_unit);

                    z.Points.Add(origin);
                    z.Points.Add(z_unit);

                    this.show_viewport3D.Children.Add(x);
                    this.show_viewport3D.Children.Add(y);
                    this.show_viewport3D.Children.Add(z);
                }
            }


        }

        Transform3DGroup getSynchyTransGroup(Transform3DGroup t, int index)
        {
            double rad2Deg = 180 / Math.PI;
            //use z-N-Z Euler Angle rotation
            Vector3D N = Vector3D.CrossProduct(new Vector3D(0, 0, 1),GUIComponents.synchyList[index].zPar);
            N.Normalize();


            t.Children.Add(new RotateTransform3D(
                new AxisAngleRotation3D(
                    new Vector3D(0, 0, 1),
                    rad2Deg * Math.Acos(-GUIComponents.synchyList[index].zPar.Y / Math.Sqrt(1 - Math.Pow(GUIComponents.synchyList[index].zPar.Z, 2))))));
            //rotate around y
            t.Children.Add(new RotateTransform3D(
                new AxisAngleRotation3D(N,
                    rad2Deg * Math.Acos(GUIComponents.synchyList[index].zPar.Z))));
            //rotate around z
            t.Children.Add(new RotateTransform3D(
                new AxisAngleRotation3D(GUIComponents.synchyList[index].zPar, 
                    rad2Deg * Math.Acos(GUIComponents.synchyList[index].yPar.Z / Math.Sqrt(1 - Math.Pow(GUIComponents.synchyList[index].zPar.Z, 2))))));
            return t;
        }

        private void drawFrameAxis(int kinectIndex)
        {
            if ((bool)parentGUI.show3DCheck.IsChecked)
            {
                Box3D kinectBox = new Box3D();
                DiffuseMaterial material = new DiffuseMaterial(Brushes.Black);
                kinectBox.Material = material;

                Transform3DGroup t = new Transform3DGroup();
                double l = 0.2, w = 0.03, h = 0.03;
                t.Children.Add(new ScaleTransform3D(l, w, h));
                t.Children.Add(new TranslateTransform3D(new Vector3D(-l / 2, -w / 2, -h / 2))); //new Vector3D(height*GUIComponents.synchyList[i].zPar.X, GUIComponents.synchyList[i].zPar.Y, GUIComponents.synchyList[i].zPar.Z)));


                Point3D origin = new Point3D(0, 0, 0);
                Point3D x_unit = new Point3D(1, 0, 0);
                Point3D y_unit = new Point3D(0, 1, 0);
                Point3D z_unit = new Point3D(0, 0, 1);


                KinectData kp = new KinectData();

                origin = kp.transformPointto(origin, transfMatrix[kinectIndex]);
                x_unit = kp.transformPointto(x_unit, transfMatrix[kinectIndex]);
                y_unit = kp.transformPointto(y_unit, transfMatrix[kinectIndex]);
                z_unit = kp.transformPointto(z_unit, transfMatrix[kinectIndex]);

                Vector3D x = new Vector3D(x_unit.X - origin.X, x_unit.Y - origin.Y, x_unit.Z - origin.Z);
                Vector3D y = new Vector3D(y_unit.X - origin.X, y_unit.Y - origin.Y, y_unit.Z - origin.Z);
                Vector3D z = new Vector3D(z_unit.X - origin.X, z_unit.Y - origin.Y, z_unit.Z - origin.Z);

                x.Normalize();
                y.Normalize();
                z.Normalize();
                Vector3D X = new Vector3D(1, 0, 0);
                Vector3D Y = new Vector3D(0, 1, 0);
                Vector3D Z = new Vector3D(0, 0, 1);

                if (kinectIndex != 0)
                {
                    double rad2Deg = 180 / Math.PI;
                    //use z-N-Z Euler Angle rotation
                    Vector3D N = Vector3D.CrossProduct(new Vector3D(0, 0, 1),z);
                    N.Normalize();
                    double Z1 = z.X;// Vector3D.DotProduct(Z, x);
                    double Z2 = z.Y;// Vector3D.DotProduct(Z, y);
                    double Z3 = z.Z;// Vector3D.DotProduct(Z, z);
                    double X3 = x.Z;// Vector3D.DotProduct(X, z);
                    double Y3 = y.Z;// Vector3D.DotProduct(Y, z);
                    
                    t.Children.Add(new RotateTransform3D(
                        new AxisAngleRotation3D(
                            Z,
                            rad2Deg * Math.Atan2(Z1,-Z2)//Math.Acos(-z.Y / Math.Sqrt(1 - Math.Pow(z.Z, 2)))
                            )));
                    //rotate around y
                    t.Children.Add(new RotateTransform3D(
                        new AxisAngleRotation3D(N,
                            rad2Deg * Math.Acos(Z3))));
                    //rotate around z
                    t.Children.Add(new RotateTransform3D(
                        new AxisAngleRotation3D(z,
                            rad2Deg * Math.Atan2(X3,Y3)//Math.Acos(y.Z / Math.Sqrt(1 - Math.Pow(z.Z, 2))))));
                            )));
                  
                    t.Children.Add(new TranslateTransform3D(new Vector3D(origin.X, origin.Y, origin.Z)));
                }
                kinectBox.Transform = t;
                show_viewport3D.Children.Add(kinectBox);


                ScreenSpaceLines3D Vz = new ScreenSpaceLines3D();
                int width = 2;
                double factor = 0.15;
                Vz.Points.Add(origin);
                Vz.Points.Add(new Point3D(z.X * factor + origin.X, z.Y * factor + origin.Y, z.Z * factor + origin.Z));
                Vz.Thickness = width;
                Vz.Color = Colors.Red;
                show_viewport3D.Children.Add(Vz);
            }
            
            else
            {
                ScreenSpaceLines3D x = new ScreenSpaceLines3D();
                ScreenSpaceLines3D y = new ScreenSpaceLines3D();
                ScreenSpaceLines3D z = new ScreenSpaceLines3D();
                int width = 3;
                double factor = 0.2;
                //    Color c = Colors.Violet;
                x.Thickness = width;
                x.Color = Colors.Black;
                y.Thickness = width;
                y.Color = Colors.Blue;
                z.Thickness = width;
                z.Color = Colors.Red;

                Point3D origin = new Point3D(0, 0, 0);
                Point3D x_unit = new Point3D(factor, 0, 0);
                Point3D y_unit = new Point3D(0, factor, 0);
                Point3D z_unit = new Point3D(0, 0, factor);


                KinectData kp = new KinectData();

                origin = kp.transformPointto(origin, transfMatrix[kinectIndex]);
                x_unit = kp.transformPointto(x_unit, transfMatrix[kinectIndex]);
                y_unit = kp.transformPointto(y_unit, transfMatrix[kinectIndex]);
                z_unit = kp.transformPointto(z_unit, transfMatrix[kinectIndex]);

                
               /*
                x_unit = new Point3D(x_unit.X - origin.X, x_unit.Y - origin.Y, x_unit.Z - origin.Z);
                y_unit = new Point3D(y_unit.X - origin.X, y_unit.Y - origin.Y, y_unit.Z - origin.Z);
                z_unit = new Point3D(z_unit.X - origin.X, z_unit.Y - origin.Y, z_unit.Z - origin.Z);
                origin = new Point3D(0, 0, 0); 
                */
                x.Points.Add(origin);
                x.Points.Add(x_unit);

                y.Points.Add(origin);
                y.Points.Add(y_unit);

                z.Points.Add(origin);
                z.Points.Add(z_unit);

                this.show_viewport3D.Children.Add(x);
                this.show_viewport3D.Children.Add(y);
                this.show_viewport3D.Children.Add(z);
            }
        }

        private void drawSkeleton(BodyData s) {
            if (s.TrackingState != TrackingState.Tracked)
                return;

            try {
                addSkeletonLine(s.Joints[JointType.SpineShoulder], s.Joints[JointType.Head]);
                addSkeletonLine(s.Joints[JointType.SpineShoulder], s.Joints[JointType.ShoulderLeft]);
                addSkeletonLine(s.Joints[JointType.SpineShoulder], s.Joints[JointType.ShoulderRight]);
                addSkeletonLine(s.Joints[JointType.SpineShoulder], s.Joints[JointType.SpineMid]);

                addSkeletonLine(s.Joints[JointType.ElbowLeft], s.Joints[JointType.ShoulderLeft]);
                addSkeletonLine(s.Joints[JointType.ElbowLeft], s.Joints[JointType.WristLeft]);
                addSkeletonLine(s.Joints[JointType.ElbowRight], s.Joints[JointType.ShoulderRight]);
                addSkeletonLine(s.Joints[JointType.ElbowRight], s.Joints[JointType.WristRight]);
                addSkeletonLine(s.Joints[JointType.WristLeft], s.Joints[JointType.HandLeft]);
                addSkeletonLine(s.Joints[JointType.WristRight], s.Joints[JointType.HandRight]);
            }
            catch {}
        }

        void addSkeletonLine(Joint j1, Joint j2)
        {
            if (j1.TrackingState == TrackingState.NotTracked || j2.TrackingState == TrackingState.NotTracked)
                return;

            for (int i = 0; i < kinectList.Count; ++i)
            {
                CameraSpacePoint p = new CameraSpacePoint();
                p.X = (float) transfMatrix[i][0, 3];
                p.Y = (float) transfMatrix[i][1, 3];
                p.Z = (float) transfMatrix[i][2, 3];

                    //Too close to kinect origin
                if (Math.Sqrt((j1.Position.Z - p.Z) * (j1.Position.Z - p.Z) + (j1.Position.X - p.X) * (j1.Position.X - p.X) + (j1.Position.Y - p.Y) * (j1.Position.Y - p.Y)) < 0.2 ||
                    Math.Sqrt((j2.Position.Z - p.Z) * (j2.Position.Z - p.Z) + (j2.Position.X - p.X) * (j2.Position.X - p.X) + (j2.Position.Y - p.Y) * (j2.Position.Y - p.Y)) < 0.2)
                    return;
            } 

            int width = 5;
            Color trackedColor = Colors.Violet;
            Color inferredColor = Colors.Gray;
            Color c;

            if (j1.TrackingState == TrackingState.Tracked &&
                j2.TrackingState == TrackingState.Tracked)
                c = trackedColor;
            else
                c = inferredColor;

            ScreenSpaceLines3D l = new ScreenSpaceLines3D();
            l.Thickness = width;
            l.Color = c;

            l.Points.Add(getPoint3DfromJoint(j1));
            l.Points.Add(getPoint3DfromJoint(j2));

            this.show_viewport3D.Children.Add(l);
        }

        private Point3D getPoint3DfromJoint(Joint j) {
            Point3D p = new Point3D();
            p.X = j.Position.X;
            p.Y = j.Position.Y;
            p.Z = j.Position.Z;

            return p;
        }

        //update GUI to show data
        delegate void UpdateResultCallback(string text, bool draw);
        public void updateResult(string text, bool draw)
        {
            if (this.infoTxtBlk.Dispatcher.Thread != Thread.CurrentThread) {
                this.infoTxtBlk.Dispatcher.Invoke(System.Windows.Threading.DispatcherPriority.Normal, new UpdateResultCallback(this.updateResult), text, draw);
            }
            else {
                try {
                    if (kinectList == null)
                        return;

                    this.infoTxtBlk.Text = "[Fused Kinect Result]" + Environment.NewLine + fusedKinectParameters.printKinectParameters();
                        //show synchy info
                    this.infoTxtBlk.Text = this.infoTxtBlk.Text + "[Synchy Info] num = " + GUIComponents.synchyList.Count + Environment.NewLine;
                    for (int i = 0; i < GUIComponents.synchyList.Count; ++i) {
                        this.infoTxtBlk.Text = this.infoTxtBlk.Text + i + " : " + GUIComponents.synchyList[i].printSynchy() + Environment.NewLine;
                    }

                    MatrixProcessor mp = new MatrixProcessor();
                    server.fusedKinectParameter = fusedKinectParameters;

                    if (!draw)
                        return;

                        //3D plot functions
                    show_viewport3D.Children.Clear();
                    ClearViewport();
                    
                    PerspectiveCamera cam = (PerspectiveCamera)show_viewport3D.Camera;
                    show_viewport3D.Children.Add(new ModelVisual3D() { Content = new DirectionalLight(Colors.White, new Vector3D(cam.LookDirection.X, cam.LookDirection.Y, cam.LookDirection.Z)) });
                    show_viewport3D.Children.Add(new ModelVisual3D() { Content = new DirectionalLight(Colors.White, new Vector3D(-cam.LookDirection.X, -cam.LookDirection.Y, -cam.LookDirection.Z)) });
                   
                    drawSynchy();

                    PerspectiveCamera c = (PerspectiveCamera)show_viewport3D.Camera.GetCurrentValueAsFrozen();
                    for (int i = 0; i < transfMatrix.Count; ++i)
                        drawFrameAxis(i);

                    if (fusedKinectParameters != null) {
                        if (fusedKinectParameters.skeletonArray != null)
                            for (int j = 0; j < fusedKinectParameters.skeletonArray.Length; ++j)
                                if (fusedKinectParameters.skeletonArray[j] != null)
                                    drawSkeleton(fusedKinectParameters.skeletonArray[j]);
                    }

                    show_viewport3D.UpdateLayout();
                } catch (Exception ex) {
                    string ss = ex.ToString();
                }
            }
        }

        #endregion Methods
    }

    public class UpdateDataThread {
        FusedDataProcessor fp;

        public UpdateDataThread(FusedDataProcessor infp)
        {
            fp = infp;
        }
        //delegate void printValueCallback(TextBlock t);

        public void ThreadProc()
        {
            while (FusedDataProcessor.isWorking)
            {
                fp.updateKinectList();

                fp.updateResult("", true);

                Thread.Sleep(100);
            }
        }
    }
}
