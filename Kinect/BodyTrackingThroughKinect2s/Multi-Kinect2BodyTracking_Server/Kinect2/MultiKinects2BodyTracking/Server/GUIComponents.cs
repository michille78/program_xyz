//This file handles main GUI components of server program

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Controls;
using System.Windows;
using System.Windows.Media.Media3D;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Threading;

using Kinect2.MultiKinects2BodyTracking.DataStructure;
using Kinect2.MultiKinects2BodyTracking.TCPConnection;

using _3DTools;
using MathNet.Numerics.LinearAlgebra.Double;

namespace Kinect2.MultiKinects2BodyTracking.Server
{
    public class GUIComponents {
        #region Members

        /* Data members */
        static public List<Synchy> synchyList = new List<Synchy>();
        static public int synchyTimes = 0;
        static public List<Point3> LList = new List<Point3>();
        static public List<Point3> RList = new List<Point3>();
        static public List<Point3> CList = new List<Point3>();

        /* GUI parameters */
        private const int kinectH = 800;
        private const int singleKinectW = 320;
        private int serverH = 200;
        private int fusedDataW = 400;
        int kinectNUM = 0;

        /* Processing Components */
        public static FusedComp fc;
        private myServer server;
        public Calibration cali = null;
        public List<KinectComp> kinectCompList = new List<KinectComp>();

        /* GUI Components */
        public TextBlock serverMsgTxtBlock = new TextBlock();
        
        public Button calibrationBtn = new Button();
        public Button addSynchyBtn = new Button();
        public Button clearSynchyBtn = new Button();
        public Button updateAllImageBtn = new Button();

        public Grid kinectGrid = new Grid();
        public Grid fusedGrid = new Grid();
        public Grid rootGrid;// = new Grid();

        #endregion // Members

        #region Methods

        /// <summary>
        /// Handle the GUI layout
        /// </summary>
        /// <param name="kinect_num"></param>
        /// <param name="rootWindow"></param>
        public GUIComponents(int kinect_num, MainWindow rootWindow) {
            kinectNUM = kinect_num;

            /* Set the size of the window */
            rootGrid = rootWindow.rootGrid;
            rootGrid.Width = singleKinectW * kinectNUM + fusedDataW;
            rootGrid.Height = kinectH + serverH;

                // Setup the layout of the GUI
            buildRootGrid(rootGrid);

            /* Message Text Block */
            serverMsgTxtBlock.Text = "No server message";
            Grid.SetColumn(serverMsgTxtBlock, 0);
            Grid.SetRow(serverMsgTxtBlock, 1);
            rootGrid.Children.Add(serverMsgTxtBlock);

            int currentHeight = 0;
            int btnH = 40;

            /* Calibration Button */
            calibrationBtn.VerticalAlignment = VerticalAlignment.Top;
            calibrationBtn.Height = btnH;
            calibrationBtn.Margin = new Thickness(0, currentHeight, 0, 0);
            currentHeight += btnH;
            calibrationBtn.Content = "Do Calibration";
            Grid.SetColumn(calibrationBtn, 1);
            Grid.SetRow(calibrationBtn, 1);
            rootGrid.Children.Add(calibrationBtn);
            calibrationBtn.Click += new RoutedEventHandler(runCalibration);

            /* updateAllImage Button */
            updateAllImageBtn.VerticalAlignment = VerticalAlignment.Top;
            updateAllImageBtn.Height = btnH;
            updateAllImageBtn.Margin = new Thickness(0, currentHeight, 0, 0);
            currentHeight += btnH;
            updateAllImageBtn.Content = "Update All Images";
            Grid.SetColumn(updateAllImageBtn, 1);
            Grid.SetRow(updateAllImageBtn, 1);
            rootGrid.Children.Add(updateAllImageBtn);
            updateAllImageBtn.Click += new RoutedEventHandler(updateAllImagesEvent);

            /* addSynchy Button */
            addSynchyBtn.VerticalAlignment = VerticalAlignment.Top;
            addSynchyBtn.Content = "Add Synchy";
            addSynchyBtn.Height = btnH;
            addSynchyBtn.Margin = new Thickness(0, currentHeight, 0, 0);
            currentHeight += btnH;
            Grid.SetColumn(addSynchyBtn, 1);
            Grid.SetRow(addSynchyBtn, 1);
            rootGrid.Children.Add(addSynchyBtn);
            addSynchyBtn.Click += new RoutedEventHandler(startAddSynchy);

            /* clearSynchy Button */
            clearSynchyBtn.VerticalAlignment = VerticalAlignment.Top;
            clearSynchyBtn.Content = "Delete a Synchy";
            clearSynchyBtn.Height = btnH;
            clearSynchyBtn.Margin = new Thickness(0, currentHeight, 0, 0);
            currentHeight += btnH;
            Grid.SetColumn(clearSynchyBtn, 1);
            Grid.SetRow(clearSynchyBtn, 1);
            rootGrid.Children.Add(clearSynchyBtn);
            clearSynchyBtn.Click += new RoutedEventHandler(deleteASynchy);

            /* Build the grid to contain multi kinectComps */
            kinectGrid.ShowGridLines = true;
            RowDefinition r0 = new RowDefinition();
            r0.Height = new GridLength(kinectH);
            kinectGrid.RowDefinitions.Add(r0);
            Grid.SetColumn(kinectGrid, 0);
            rootGrid.Children.Add(kinectGrid);

            for (int i = 0; i < kinect_num; ++i) {
                ColumnDefinition c = new ColumnDefinition();
                c.Width = new GridLength(singleKinectW);
                kinectGrid.ColumnDefinitions.Add(c);
                kinectCompList.Add(new KinectComp(kinectH, singleKinectW, this, i));
            }

            fusedGrid.Height = kinectH;
            Grid.SetColumn(fusedGrid, 1);
            Grid.SetRow(fusedGrid, 0);
            rootGrid.Children.Add(fusedGrid);

            /* Read synchy from data */
            string synchyData = System.IO.File.ReadAllText("synchyLog.txt");
            string[] synchyDataList = synchyData.Split('*');
            if (synchyDataList.Length > 1) {
                int num = Convert.ToInt32(synchyDataList[0]);
                for (int i = 1; i <= num; ++i) {
                    string[] cors = synchyDataList[i].Split(' ');
                    int index = 0;
                    Synchy s = new Synchy();
                    s.Center.X = Convert.ToDouble(cors[index++]);
                    s.Center.Y = Convert.ToDouble(cors[index++]);
                    s.Center.Z = Convert.ToDouble(cors[index++]);
                    s.xUnit.X = Convert.ToDouble(cors[index++]);
                    s.xUnit.Y = Convert.ToDouble(cors[index++]);
                    s.xUnit.Z = Convert.ToDouble(cors[index++]);

                    s.yUnit.X = Convert.ToDouble(cors[index++]);
                    s.yUnit.Y = Convert.ToDouble(cors[index++]);
                    s.yUnit.Z = Convert.ToDouble(cors[index++]);

                    s.zUnit.X = Convert.ToDouble(cors[index++]);
                    s.zUnit.Y = Convert.ToDouble(cors[index++]);
                    s.zUnit.Z = Convert.ToDouble(cors[index++]);

                    s.xPar.X = Convert.ToDouble(cors[index++]);
                    s.xPar.Y = Convert.ToDouble(cors[index++]);
                    s.xPar.Z = Convert.ToDouble(cors[index++]);

                    s.yPar.X = Convert.ToDouble(cors[index++]);
                    s.yPar.Y = Convert.ToDouble(cors[index++]);
                    s.yPar.Z = Convert.ToDouble(cors[index++]);

                    s.zPar.X = Convert.ToDouble(cors[index++]);
                    s.zPar.Y = Convert.ToDouble(cors[index++]);
                    s.zPar.Z = Convert.ToDouble(cors[index++]);

                    synchyList.Add(s);
                }
            }
        }

        /// <summary>
        /// Set the server to connect to
        /// </summary>
        /// <param name="in_server"></param>
        /// <param name="mw"></param>
        public void setServer(myServer in_server, MainWindow mw) {
            server = in_server;
            fc = new FusedComp(fusedGrid, mw, kinectNUM, server);
        }

        /// <summary>
        /// Run calibration window
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void runCalibration(object sender, RoutedEventArgs e) {
            if (cali == null)
                cali = new Calibration(kinectNUM, this);

            updateAllImages();
            cali.updateImages();
            cali.show();
        }

        /// <summary>
        /// Update all images when button is clicked
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        void updateAllImagesEvent(object sender, RoutedEventArgs e) {
            updateAllImages();
        }

        /// <summary>
        /// Start to add synchy
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        void startAddSynchy(object sender, RoutedEventArgs e) {
            for (int i = 0; i < kinectCompList.Count; ++i) {
                kinectCompList[i].setClickEventHandlers();
            }
            MessageBox.Show("Click Synchy in RGB image in the order of \"right-hand, left-hand, center\" in the image");
        }

        public void removeMouseHandlers() {
            for (int i = 0; i < kinectCompList.Count; ++i)
                kinectCompList[i].removeClickEventHandlers();
        }

        /// <summary>
        /// Delete the last Synchy
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        void deleteASynchy(object sender, RoutedEventArgs e) {
            removeMouseHandlers();
            synchyTimes = 0;

            if (synchyList.Count > 0)
                synchyList.RemoveAt(synchyList.Count - 1);

            LList.Clear();
            RList.Clear();
            CList.Clear();
            System.IO.File.WriteAllText("synchyLog.txt", "");
        }

        /// <summary>
        /// Send update image command to kinect clients to get image data
        /// </summary>
        public void updateAllImages() {
            if (!MainWindow.getUpdateGUI())
                return;

            for (int i = 0; i < server.clientList.Count; ++i) {
                if (server.clientList[i].clientType == (int)clientTypes.KINECT && server.clientList[i].isWorking) {
                    server.clientList[i].sendImageCommand();
                    Thread.Sleep(20);
                    server.clientList[i].sendImageCommand();
                    Thread.Sleep(20);
                    server.clientList[i].sendImageCommand();
                    Thread.Sleep(20);
                    server.clientList[i].sendImageCommand();
                    Thread.Sleep(20);
                    server.clientList[i].sendImageCommand();
                    Thread.Sleep(20);
                    server.clientList[i].sendImageCommand();
                }
            }
        }

        /// <summary>
        /// Build root layout
        /// </summary>
        /// <param name="g"></param>
        private void buildRootGrid(Grid g) {
            g.ShowGridLines = true;

            RowDefinition r0 = new RowDefinition();
            r0.Height = new GridLength(kinectH);
            ColumnDefinition c0 = new ColumnDefinition();
            c0.Width = new GridLength(singleKinectW * kinectNUM);

            RowDefinition r1 = new RowDefinition();
            r1.Height = new GridLength(serverH);
            ColumnDefinition c1 = new ColumnDefinition();
            c1.Width = new GridLength(fusedDataW);

            g.RowDefinitions.Add(r0);
            g.ColumnDefinitions.Add(c0);
            g.RowDefinitions.Add(r1);
            g.ColumnDefinitions.Add(c1);
        }

        #endregion //Methods

        /// <summary>
        /// Define the values needed by a Synchy
        /// </summary>
        public class Synchy {

            #region Members

            public Vector3D LHand;
            public Vector3D RHand;
            public Vector3D Center;

            /* X,Y,Z axis points with unit length */
            public Vector3D xUnit;
            public Vector3D yUnit;
            public Vector3D zUnit;

            /* Vectors parallel to axies */
            public Vector3D xPar;
            public Vector3D yPar;
            public Vector3D zPar;

            #endregion // Members

            #region Methods

            public void reset() {
                LHand = new Vector3D(0,0,0);
                RHand = new Vector3D(0, 0, 0);
                Center = new Vector3D(0, 0, 0);
                xUnit = new Vector3D(0, 0, 0);
                yUnit = new Vector3D(0, 0, 0);
                zUnit = new Vector3D(0, 0, 0);
            }

            public Vector3D transformVector3D(Vector3D v, DenseMatrix T){
                DenseMatrix m = new DenseMatrix(4,1);
                m[0,0] = v.X;
                m[1,0] = v.Y;
                m[2,0] = v.Z;
                m[3,0] = 1;
                m = T * m;
                return new Vector3D(m[0,0],m[1,0],m[2,0]);
            }

            /// <summary>
            /// z: up, y:left, x:front
            /// Compute synchy frame using left hand, right hand, and center points.
            /// </summary>
            /// <param name="kinectIndex"></param>
            public void computeCoordinate(int kinectIndex) {
                List<DenseMatrix> list = GUIComponents.fc.fp.getTranfMatrix();
                if (list.Count < kinectIndex + 1) {
                    MessageBox.Show("Calibration matrix error! Please calibrate first!");
                    return;
                }

                LHand = transformVector3D(LHand, GUIComponents.fc.fp.getTranfMatrix()[kinectIndex]);
                RHand = transformVector3D(RHand, GUIComponents.fc.fp.getTranfMatrix()[kinectIndex]);
                Center = transformVector3D(Center, GUIComponents.fc.fp.getTranfMatrix()[kinectIndex]);

                yPar = LHand - RHand;
                Vector3D v1 = RHand - Center;
                Vector3D v2 = LHand - Center;
                zPar = Vector3D.CrossProduct(v1,v2);
                xPar = Vector3D.CrossProduct(yPar,zPar);
                
                xPar.Normalize();
                yPar.Normalize();
                zPar.Normalize();

                double factor = 5;
                xPar = Vector3D.Divide(xPar, factor);
                yPar = Vector3D.Divide(yPar, factor);
                zPar = Vector3D.Divide(zPar, factor);

                xUnit = Center + xPar;
                yUnit = Center + yPar;
                zUnit = Center + zPar;

                /* Log data to synchyLog.txt */
                string data = synchyList.Count.ToString()+"*";
                for(int i=0; i < synchyList.Count; ++i) {
                    data += 
                     synchyList[i].Center.X + " " + synchyList[i].Center.Y + " " + synchyList[i].Center.Z + " "
                    + synchyList[i].xUnit.X + " " + synchyList[i].xUnit.Y + " " + synchyList[i].xUnit.Z + " "
                    + synchyList[i].yUnit.X + " " + synchyList[i].yUnit.Y + " " + synchyList[i].yUnit.Z + " "
                    + synchyList[i].zUnit.X + " " + synchyList[i].zUnit.Y + " " + synchyList[i].zUnit.Z + " "
                    + synchyList[i].xPar.X + " " + synchyList[i].xPar.Y + " " + synchyList[i].xPar.Z + " "
                    + synchyList[i].yPar.X + " " + synchyList[i].yPar.Y + " " + synchyList[i].yPar.Z + " "
                    + synchyList[i].zPar.X + " " + synchyList[i].zPar.Y + " " + synchyList[i].zPar.Z+"*";
                }
                System.IO.File.WriteAllText("synchyLog.txt",data);
            }

            /// <summary>
            /// Pring Synchy value on GUI
            /// </summary>
            /// <returns></returns>
            public string printSynchy() {
                if (zUnit.Z == 0)
                    return "Setting...";
                else
                    return Center.X.ToString("F2") + " " + Center.Y.ToString("F2") + " " + Center.Z.ToString("F2");
            }

            /// <summary>
            /// Get synchy data in string format
            /// </summary>
            /// <returns></returns>
            public string getSynchyData() {
                if (zUnit.Z == 0)
                    return "";
                else
                    return Center.X.ToString("F2") + " " + Center.Y.ToString("F2") + " " + Center.Z.ToString("F2")+ " "
                         + xUnit.X.ToString("F2") + " " + xUnit.Y.ToString("F2") + " " + xUnit.Z.ToString("F2") + " "
                         + yUnit.X.ToString("F2") + " " + yUnit.Y.ToString("F2") + " " + yUnit.Z.ToString("F2") + " "
                         + zUnit.X.ToString("F2") + " " + zUnit.Y.ToString("F2") + " " + zUnit.Z.ToString("F2");
            }

            #endregion // Methods
        }

        /// <summary>
        /// This is the GUI component to show fused result
        /// </summary>
        public class FusedComp {

            #region Members

            /* GUI Components */
            public TextBlock infoTxtBlk = new TextBlock();
            public Viewport3D viewPort3D;
            public CheckBox show3DCheck = new CheckBox();
            Label show3DLabel = new Label();

            static public CheckBox sendFusedDataToKinectCheck = new CheckBox();
            public Label sendFusedDataToKinectLabel = new Label();

            static public CheckBox updateGUICheck = new CheckBox();
            public Label updateGUILabel = new Label();

                // Square
            int viewPortH = 400;
            TrackballDecorator trackBallMouse;
            int itemHeight = 25;

            public FusedDataProcessor fp;

            #endregion // Members

            #region Methods

            /// <summary>
            /// Handle GUI components and layout
            /// </summary>
            /// <param name="parentGrid"></param>
            /// <param name="mw"></param>
            /// <param name="kinectNUM"></param>
            /// <param name="server"></param>
            public FusedComp(Grid parentGrid, MainWindow mw, int kinectNUM, myServer server) {
                show3DCheck.IsChecked = true;
                show3DCheck.Height = itemHeight;
                show3DLabel.Content = "Show 3D model";
                show3DLabel.Height = itemHeight;

                sendFusedDataToKinectCheck.Height = itemHeight;
                sendFusedDataToKinectCheck.IsChecked = false;
                sendFusedDataToKinectCheck.VerticalAlignment = VerticalAlignment.Top;
                sendFusedDataToKinectCheck.Checked += new RoutedEventHandler(sendFusedDataToKinectCheck_Checked);
                sendFusedDataToKinectCheck.Unchecked += new RoutedEventHandler(sendFusedDataToKinectCheck_Unchecked);

                sendFusedDataToKinectLabel.Height = itemHeight;
                sendFusedDataToKinectLabel.VerticalAlignment = VerticalAlignment.Top;
                sendFusedDataToKinectLabel.Content = "Show fused data on client";

                updateGUICheck.Checked += new RoutedEventHandler(updateGUICheck_Checked);
                updateGUICheck.Unchecked += new RoutedEventHandler(updateGUICheck_Unchecked);
                updateGUICheck.IsChecked = true;
                updateGUICheck.Height = itemHeight;
                updateGUICheck.VerticalAlignment = VerticalAlignment.Top;

                updateGUILabel.Height = itemHeight;
                updateGUILabel.VerticalAlignment = VerticalAlignment.Top;
                updateGUILabel.Content = "Update GUI";
                
                trackBallMouse = mw.trackball;
                viewPort3D = mw.show_viewport3D;
                parentGrid.ShowGridLines = true;
                parentGrid.Width = viewPortH;
                RowDefinition r0 = new RowDefinition();
                r0.Height = new GridLength(parentGrid.Height - viewPortH - itemHeight*3);

                RowDefinition r1 = new RowDefinition();
                r1.Height = new GridLength(viewPortH + itemHeight*3);

                ColumnDefinition c0 = new ColumnDefinition();
                c0.Width = new GridLength(viewPortH);

                parentGrid.RowDefinitions.Add(r1);
                parentGrid.ColumnDefinitions.Add(c0);
                parentGrid.RowDefinitions.Add(r0);

                trackBallMouse.Height = viewPortH;
                trackBallMouse.Width = viewPortH;
                viewPort3D.Height = viewPortH;
                viewPort3D.Width = viewPortH;

                Grid.SetColumn(show3DLabel, 0);
                Grid.SetRow(show3DLabel, 0);
                show3DLabel.VerticalAlignment = VerticalAlignment.Top;
                show3DLabel.Margin = new Thickness(20, 0, 0, 0);
                parentGrid.Children.Add(show3DLabel);

                show3DCheck.Margin = new Thickness(5, 7, 0, 0);
                Grid.SetColumn(show3DCheck, 0);
                Grid.SetRow(show3DCheck, 0);
                show3DCheck.VerticalAlignment = VerticalAlignment.Top;
                parentGrid.Children.Add(show3DCheck);


                sendFusedDataToKinectCheck.Margin = new Thickness(5, 7 + itemHeight, 0, 0);
                sendFusedDataToKinectLabel.Margin = new Thickness(20, itemHeight, 0, 0);
                Grid.SetColumn(sendFusedDataToKinectCheck, 0);
                Grid.SetRow(sendFusedDataToKinectCheck, 0);
                parentGrid.Children.Add(sendFusedDataToKinectCheck);
                parentGrid.Children.Add(sendFusedDataToKinectLabel);

                updateGUICheck.Margin = new Thickness(5, 7 + itemHeight * 2, 0, 0);
                updateGUILabel.Margin = new Thickness(20, itemHeight * 2, 0, 0);
                Grid.SetColumn(updateGUICheck, 0);
                Grid.SetRow(updateGUILabel, 0);
                parentGrid.Children.Add(updateGUILabel);
                parentGrid.Children.Add(updateGUICheck);

                mw.rootGrid.Children.Remove(trackBallMouse);
                trackBallMouse.VerticalAlignment = VerticalAlignment.Top;
                trackBallMouse.Margin = new Thickness(0, itemHeight*3, 0, 0);
                Grid.SetColumn(trackBallMouse, 0);
                Grid.SetRow(trackBallMouse, 0);
                parentGrid.Children.Add(trackBallMouse);
    
                infoTxtBlk.Text = "no data";
                Grid.SetColumn(infoTxtBlk, 0);
                Grid.SetRow(infoTxtBlk, 1);
                parentGrid.Children.Add(infoTxtBlk);


                PerspectiveCamera cam = new PerspectiveCamera();
                cam.FarPlaneDistance = 100;
                cam.Position = new Point3D(3, 3, 3);
                cam.FarPlaneDistance = 100;
                cam.NearPlaneDistance = 0.1;
                cam.FieldOfView = 70;
                cam.LookDirection = new Vector3D(-1, -1, -1);
                cam.UpDirection = new Vector3D(0, 1, 0);

                viewPort3D.Camera = cam;  
                fp = new FusedDataProcessor(kinectNUM, server, this);
            }

            /// <summary>
            /// Handle check boxes 
            /// </summary>
            /// <param name="sender"></param>
            /// <param name="e"></param>
            void updateGUICheck_Unchecked(object sender, RoutedEventArgs e)
            {
                MainWindow.setUpdateGUI(false);
            }

            void updateGUICheck_Checked(object sender, RoutedEventArgs e)
            {
                MainWindow.setUpdateGUI(true);
            }

            void sendFusedDataToKinectCheck_Unchecked(object sender, RoutedEventArgs e)
            {
                MainWindow.setFusedDataToKinect(false) ;
            }

            void sendFusedDataToKinectCheck_Checked(object sender, RoutedEventArgs e)
            {
                MainWindow.setFusedDataToKinect(true);
                //throw new NotImplementedException();
            }

            #endregion // Methods
        }
        
        /// <summary>
        /// This is the GUI component to show individual kinect data
        /// </summary>
        public class KinectComp {

            #region Members

            /* GUI Components */
            private const float imageH = 240;
            private const float imageW = 320;
            private const int gridRowNum = 5;
            private const int gridColNum = 1;
            private int rowHeight = 25;
            Label titleLabel = new Label();
            public System.Windows.Controls.Image rgbImg = new System.Windows.Controls.Image();
            public System.Windows.Controls.Image depthImg = new System.Windows.Controls.Image();
            public TextBlock infoTextBlock = new TextBlock();
            private const int WIDTH = 640;
            private const int HEIGHT = 480;
            public Grid g = new Grid(); // this grid

            /* Data members */
            public WriteableBitmap colorImageWritableBitmap;
            public WriteableBitmap depthImageWritableBitmap;
            public float[] point3DArray;
            int kinectIndex = -1;
            
            private static readonly int Bgr32BytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;

            GUIComponents parentGUI;

            #endregion // Members

            #region Methods

            /// <summary>
            /// Initialize components and layout
            /// </summary>
            /// <param name="h"></param>
            /// <param name="w"></param>
            /// <param name="in_parentGUI"></param>
            /// <param name="in_kinectIndex"></param>
            public KinectComp(int h, float w, GUIComponents in_parentGUI, int in_kinectIndex) {
                parentGUI = in_parentGUI;
                kinectIndex = in_kinectIndex;

                // Add this grid as the child of kinect grid (contains multiple kinectComps)
                g.Height = h;
                g.Width = w;
                parentGUI.kinectGrid.Children.Add(g);
                Grid.SetRow(g, 0);
                Grid.SetColumn(g, kinectIndex);
                titleLabel.Content = "Kinect " + kinectIndex.ToString();
                infoTextBlock.Text = "no data input";
                colorImageWritableBitmap = new WriteableBitmap(1920, 1080, 96, 96, PixelFormats.Bgr32, null);
                depthImageWritableBitmap = new WriteableBitmap(512, 424, 96, 96, PixelFormats.Gray8, null); //depth data is 320*240
                arrangeComponents();
            }

            /// <summary>
            /// Click this button and follow the instruction to add new synchy to list
            /// repeat 3 times to get better result
            /// </summary>
            /// <param name="obj"></param>
            /// <param name="e"></param>
            public void addSynchy(object obj, System.Windows.Input.MouseButtonEventArgs e)
            {
                System.Windows.Point p = e.GetPosition(rgbImg);

                int index = (int)(p.X * (1920 / rgbImg.Width) + p.Y * (1920 / rgbImg.Width) * 1920) * 3;

                if (point3DArray[index + 2] == 0)
                    return;

                if (GUIComponents.synchyList.Count == 0 )
                    GUIComponents.synchyList.Add(new Synchy()); 

                    //check if the last item is finished
                else if(GUIComponents.synchyList[GUIComponents.synchyList.Count-1].Center.X != 0) {
                    GUIComponents.synchyList.Add(new Synchy());           
                }

                int sIndex = GUIComponents.synchyList.Count - 1;
                if (GUIComponents.LList.Count < GUIComponents.synchyTimes + 1)
                {
                    GUIComponents.LList.Add(new Point3(point3DArray[index], point3DArray[index + 1], point3DArray[index + 2]));
                }
                else if (GUIComponents.RList.Count < GUIComponents.synchyTimes + 1)
                {
                    GUIComponents.RList.Add(new Point3(point3DArray[index], point3DArray[index + 1], point3DArray[index + 2]));
                }
                else
                {
                    GUIComponents.CList.Add(new Point3(point3DArray[index], point3DArray[index + 1], point3DArray[index + 2]));
                    GUIComponents.synchyTimes++;

                    
                    if (GUIComponents.synchyTimes >= 3)
                    {

                        for (int i = 0; i < GUIComponents.synchyTimes; ++i)
                        {
                            GUIComponents.synchyList[sIndex].LHand.X += GUIComponents.LList[i].X;
                            GUIComponents.synchyList[sIndex].LHand.Y += GUIComponents.LList[i].Y;
                            GUIComponents.synchyList[sIndex].LHand.Z += GUIComponents.LList[i].Z;

                            GUIComponents.synchyList[sIndex].RHand.X += GUIComponents.RList[i].X;
                            GUIComponents.synchyList[sIndex].RHand.Y += GUIComponents.RList[i].Y;
                            GUIComponents.synchyList[sIndex].RHand.Z += GUIComponents.RList[i].Z;

                            GUIComponents.synchyList[sIndex].Center.X += GUIComponents.CList[i].X;
                            GUIComponents.synchyList[sIndex].Center.Y += GUIComponents.CList[i].Y;
                            GUIComponents.synchyList[sIndex].Center.Z += GUIComponents.CList[i].Z;
                        }

                        //compute the average of 3 times calibration
                        GUIComponents.synchyList[sIndex].LHand.X /= GUIComponents.synchyTimes;
                        GUIComponents.synchyList[sIndex].LHand.Y /= GUIComponents.synchyTimes;
                        GUIComponents.synchyList[sIndex].LHand.Z /= GUIComponents.synchyTimes;

                        GUIComponents.synchyList[sIndex].RHand.X /= GUIComponents.synchyTimes;
                        GUIComponents.synchyList[sIndex].RHand.Y /= GUIComponents.synchyTimes;
                        GUIComponents.synchyList[sIndex].RHand.Z /= GUIComponents.synchyTimes;

                        GUIComponents.synchyList[sIndex].Center.X /= GUIComponents.synchyTimes;
                        GUIComponents.synchyList[sIndex].Center.Y /= GUIComponents.synchyTimes;
                        GUIComponents.synchyList[sIndex].Center.Z /= GUIComponents.synchyTimes;

                        GUIComponents.LList.Clear();
                        GUIComponents.RList.Clear();
                        GUIComponents.CList.Clear();

                        GUIComponents.synchyTimes = 0;

                    }
                    else {
                        int leftTimes = 3 - GUIComponents.synchyTimes;
                        MessageBox.Show("Update image and repeat " + leftTimes.ToString()+" more times");
                        return;
                    }

                    GUIComponents.synchyList[sIndex].computeCoordinate(kinectIndex);
                    for (int i = 0; i < GUIComponents.synchyList.Count; ++i)
                    {
                        if (GUIComponents.synchyList[i].Center.Z == 0 || Double.IsNaN(GUIComponents.synchyList[i].xUnit.Z))
                        {
                            synchyList.RemoveAt(i);
                            --i;   
                        }
                    }
                    parentGUI.removeMouseHandlers();
                }
            }

            /// <summary>
            /// Handle the layout
            /// </summary>
            private void arrangeComponents() {
                rgbImg.Width = g.Width;
                rgbImg.Height = g.Width * (imageH / imageW);
                rgbImg.IsEnabled = true;

                depthImg.Width = g.Width;
                depthImg.Height = g.Width * (imageH / imageW);

                    // Must build grid before adding children
                buildGrid(g);
                int rowNum = 0;
                addAsChild(titleLabel, g, rowNum++, 0);
                addAsChild(rgbImg, g, rowNum++, 0);
                addAsChild(depthImg, g, rowNum++, 0);

                infoTextBlock.Height = (kinectH - rgbImg.Height - depthImg.Height) - rowHeight * 2;
                addAsChild(infoTextBlock, g, rowNum++, 0);
            }

            /// <summary>
            /// Handle the layout
            /// </summary>
            /// <param name="g"></param>
            private void buildGrid(Grid g) {
                g.ShowGridLines = true;
                for (int i = 0; i < 1; ++i) {
                    GridLength hh = new GridLength(rowHeight);
                    RowDefinition r = new RowDefinition();
                    r.Height = hh;
                    g.RowDefinitions.Add(r);
                }

                RowDefinition r0 = new RowDefinition();
                r0.Height = new GridLength(rgbImg.Height);
                g.RowDefinitions.Add(r0);

                ColumnDefinition c0 = new ColumnDefinition();
                c0.Width = new GridLength(rgbImg.Width);
                g.ColumnDefinitions.Add(c0);

                RowDefinition r1 = new RowDefinition();
                r1.Height = new GridLength(depthImg.Height);
                g.RowDefinitions.Add(r1);

                RowDefinition r2 = new RowDefinition();
                r2.Height = new GridLength((kinectH - rgbImg.Height - depthImg.Height));
                g.RowDefinitions.Add(r2);
            }

            /// <summary>
            /// Handle the relationship between GUI components
            /// </summary>
            /// <param name="elem"></param>
            /// <param name="g"></param>
            /// <param name="rowNum"></param>
            /// <param name="colNum"></param>
            private void addAsChild(UIElement elem, Grid g, int rowNum, int colNum) {
                if (rowNum > gridRowNum || colNum > gridColNum) {
                    MessageBox.Show("Row or Colum number ERROR!");
                    return;
                }
                Grid.SetColumn(elem, colNum);
                Grid.SetRow(elem, rowNum);
                g.Children.Add(elem);
            }

            /// <summary>
            /// Update kinect images on GUI
            /// Use delegate because image data and GUI are in different threads
            /// </summary>
            /// <param name="imageData"></param>
            delegate void updateKinectImagesCallback(string imageData);
            public void updateKinectImages(string imageData)
            {
                if (!MainWindow.getUpdateGUI())
                    return;

                if (this.rgbImg.Dispatcher.Thread != Thread.CurrentThread)
                    this.rgbImg.Dispatcher.Invoke(System.Windows.Threading.DispatcherPriority.Normal, new updateKinectImagesCallback(this.updateKinectImages), imageData);
                else {
                    try {
                        string[] imageDatas = imageData.Split('*');
                        StringCompressor s = new StringCompressor();
                        byte[] rgbData = s.DecompressByteArray(imageDatas[0]);
                        byte[] depthDataByte = s.DecompressByteArray(imageDatas[1]);

                        if (rgbData == null || depthDataByte == null) {
                            this.infoTextBlock.Text = "Image reading ERROR!!" + Environment.NewLine + this.infoTextBlock.Text;
                            return;
                        }

                        /* Transfer Byte Array Data back to WriteableBitmap*/
                        this.colorImageWritableBitmap.WritePixels(
                            new Int32Rect(0, 0, 1920, 1080), rgbData, 7680, 0);

                        this.depthImageWritableBitmap.WritePixels(
                            new Int32Rect(0, 0, 512, 424), depthDataByte, 512, 0);

                        this.rgbImg.Source = this.colorImageWritableBitmap;
                        this.depthImg.Source = this.depthImageWritableBitmap;

                        byte[] point3DDataByte = s.DecompressByteArray(imageDatas[2]);
                        point3DArray = new float[point3DDataByte.Length * 3];
                        Buffer.BlockCopy(point3DDataByte, 0, point3DArray, 0, point3DDataByte.Length);

                        if (parentGUI.cali != null)
                            parentGUI.cali.updateImages();
                    }
                    catch {
                        this.infoTextBlock.Text = "Image reading ERROR!!" + Environment.NewLine + this.infoTextBlock.Text;
                    }
                }
            }

            #endregion // Methods

            #region EventHanders

            /// <summary>
            /// The event handler for mouse clicking on RGB image
            /// </summary>
            public void setClickEventHandlers()
            { 
                rgbImg.MouseDown += new System.Windows.Input.MouseButtonEventHandler(addSynchy);
            }

            public void removeClickEventHandlers()
            {
                rgbImg.MouseDown -= new System.Windows.Input.MouseButtonEventHandler(addSynchy);
            }

            #endregion // EventHanders
        }

    }
}
