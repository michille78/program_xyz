//This file is used to handle calibration functions

using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media.Imaging;
using System.Windows.Media;
using System.Threading;
using System.Threading.Tasks;

using Microsoft.Kinect;
using MathNet.Numerics.LinearAlgebra.Double;

using ConnectedComponents;
using Kinect2.MultiKinects2BodyTracking.DataStructure;

namespace Kinect2.MultiKinects2BodyTracking.Server
{
    public class Calibration
    {
        public Window calibrationWindow = new Window();
        
        //define the window size and image size
        const double minWindowH = 700;
        const double kinectH = 200;
        const double imageW = 640;
        const double imageH = 480;

        //define the radius of calibration red ball
        const double BALL_RADIUS = 0.04; // radius of red ball = 4cm

        Grid rootGrid = new Grid();
        Grid g = new Grid();

        Image[] rgbImageArray, depthImageArray, filteredImageArray, resultImageArray;
        int kinectNUM = 0;
        WriteableBitmap[] rgbArray, depthArray, filteredArray, resultArray;
        List<float[]> point3DArray;
        int gridColumnNUM = 4;//number of needed columns to contain components
        int buttonsW = 300;

        //define parameters for image processing 
        List<DenseMatrix>[] pointsToCalibrate;
        public double RGRatio = 1.5; // R/G threshold 
        public double RBRatio = 1.5; // R/B threshold
        public int erodeValue = 5;   //erode radius to eliminate fragments in the image
        public int dilateValue = 10; //dilate radius to fill the holes in the image

        //define GUI components
        Slider erodeSlider = new Slider();
        Slider dilateSlider = new Slider();
        Slider RGRatioSlider = new Slider();
        Slider RBRatioSlider = new Slider();

        Label erodeLabel = new Label();
        Label dilateLabel = new Label();
        Label RGRatioLabel = new Label();
        Label RBRatioLabel = new Label();

        Button getNewPointBtn = new Button();
        Button updateImageBtn = new Button();
        
        Button resetBtn = new Button();
        Button returnBtn = new Button();
        

        Button getMatrixFromAllBtn = new Button();
        Button addNewMatrixBtn = new Button();
        TextBlock infoTxtBlk = new TextBlock();

        GUIComponents parentGUI;


        ScrollViewer viewer = new ScrollViewer();
        string prevMatrices = "";

        //initialize the GUI components of calibration window
        public Calibration(int kinectNum, GUIComponents in_parentGUI)
        {
            parentGUI = in_parentGUI;
            kinectNUM = kinectNum;

            //build images grid
            rgbImageArray = new Image[kinectNUM];
            depthImageArray = new Image[kinectNUM];
            filteredImageArray = new Image[kinectNUM];
            resultImageArray = new Image[kinectNUM];

            rgbArray = new WriteableBitmap[kinectNUM];
            depthArray = new WriteableBitmap[kinectNUM];
            filteredArray = new WriteableBitmap[kinectNUM];
            resultArray = new WriteableBitmap[kinectNUM];

            for (int i = 0; i < kinectNUM; ++i)
            {
                rgbImageArray[i] = new Image();
                depthImageArray[i] = new Image();
                filteredImageArray[i] = new Image();
                resultImageArray[i] = new Image();
            }

            buildRootGrid();
            buildImagesGrid();
          
        }

        void initialize() {
            if (!calibrationWindow.IsActive)
            {
                calibrationWindow.Content = null;   
                calibrationWindow = new Window();
                viewer.VerticalScrollBarVisibility = ScrollBarVisibility.Auto;
                viewer.HorizontalScrollBarVisibility = ScrollBarVisibility.Auto;
                calibrationWindow.Content = viewer;
                viewer.Content = rootGrid;   
            }
            calibrationWindow.Title = "Calibration";
            pointsToCalibrate = new List<DenseMatrix>[kinectNUM];
        }

        //initialize and show the calibration window
        public void show()
        {
            initialize();
            calibrationWindow.Show();
        }

        //build GUI layout
        void buildRootGrid()
        {
            double currentHeight = 0;
            double itemMargin = 10;

            getNewPointBtn.VerticalAlignment = VerticalAlignment.Top;
            getNewPointBtn.Height = 30;
            getNewPointBtn.Margin = new Thickness(0, currentHeight, 0, 0);
            currentHeight += getNewPointBtn.Height + itemMargin;
            getNewPointBtn.Content = "Get New Point";
            getNewPointBtn.Click += new RoutedEventHandler(getSamplePoints);

            updateImageBtn.VerticalAlignment = VerticalAlignment.Top;
            updateImageBtn.Height = 30;
            updateImageBtn.Margin = new Thickness(0, currentHeight, 0, 0);
            currentHeight += updateImageBtn.Height + itemMargin;
            updateImageBtn.Content = "Update Image";
            updateImageBtn.Click += new RoutedEventHandler(updateAllImages);

            getMatrixFromAllBtn.VerticalAlignment = VerticalAlignment.Top;
            getMatrixFromAllBtn.Height = 30;
            getMatrixFromAllBtn.Margin = new Thickness(0, currentHeight, 0, 0);
            currentHeight += getMatrixFromAllBtn.Height + itemMargin;
            getMatrixFromAllBtn.Content = "Get Matrix From All Kinects";
            getMatrixFromAllBtn.Click += new RoutedEventHandler(getMatrixFromAll);


            addNewMatrixBtn.VerticalAlignment = VerticalAlignment.Top;
            addNewMatrixBtn.Height = 30;
            addNewMatrixBtn.Margin = new Thickness(0, currentHeight, 0, 0);
            currentHeight += addNewMatrixBtn.Height + itemMargin;
            addNewMatrixBtn.Content = "Add New Matrices from New Kinects";
            addNewMatrixBtn.Click += new RoutedEventHandler(addNewMatrix);

            resetBtn.VerticalAlignment = VerticalAlignment.Top;
            resetBtn.Height = 30;
            resetBtn.Margin = new Thickness(0, currentHeight, 0, 0);
            currentHeight += resetBtn.Height + itemMargin;
            resetBtn.Content = "Reset";
            resetBtn.Click += new RoutedEventHandler(resetPoints);
            
            returnBtn.VerticalAlignment = VerticalAlignment.Top;
            returnBtn.Height = 30;
            returnBtn.Margin = new Thickness(0, currentHeight, 0, 0);
            currentHeight += returnBtn.Height + itemMargin;
            returnBtn.Content = "Return to Previous Matrices";
            returnBtn.Click += new RoutedEventHandler(returnToPreviousMatrices);
            
            dilateLabel.VerticalAlignment = VerticalAlignment.Top;
            dilateLabel.Height = 30;
            dilateLabel.Margin = new Thickness(0, currentHeight, 0, 0);
            currentHeight += dilateLabel.Height + itemMargin;

            dilateSlider.VerticalAlignment = VerticalAlignment.Top;
            dilateSlider.Height = 30;
            dilateSlider.Margin = new Thickness(0, currentHeight, 0, 0);//.Top = 150;
            currentHeight += dilateSlider.Height + itemMargin;
            dilateSlider.Value = dilateValue;
            dilateSlider.ValueChanged += new RoutedPropertyChangedEventHandler<double>(updateValuesFromSlider);
            
            erodeLabel.VerticalAlignment = VerticalAlignment.Top;
            erodeLabel.Height = 30;
            erodeLabel.Margin = new Thickness(0, currentHeight, 0, 0);
            currentHeight += erodeLabel.Height + itemMargin;

            erodeSlider.VerticalAlignment = VerticalAlignment.Top;
            erodeSlider.Height = 30;
            erodeSlider.Margin = new Thickness(0, currentHeight, 0, 0);
            currentHeight += erodeSlider.Height + itemMargin;
            erodeSlider.Value = erodeValue;
            erodeSlider.ValueChanged += new RoutedPropertyChangedEventHandler<double>(updateValuesFromSlider);

            RGRatioLabel.VerticalAlignment = VerticalAlignment.Top;
            RGRatioLabel.Height = 30;
            RGRatioLabel.Margin = new Thickness(0, currentHeight, 0, 0);
            currentHeight += RGRatioLabel.Height + itemMargin;

            RGRatioSlider.VerticalAlignment = VerticalAlignment.Top;
            RGRatioSlider.Height = 30;
            RGRatioSlider.Margin = new Thickness(0, currentHeight, 0, 0);
            currentHeight += RGRatioSlider.Height + itemMargin;
            RGRatioSlider.Value = RGRatio;
            RGRatioSlider.ValueChanged += new RoutedPropertyChangedEventHandler<double>(updateValuesFromSlider);

            RBRatioLabel.VerticalAlignment = VerticalAlignment.Top;
            RBRatioLabel.Height = 30;
            RBRatioLabel.Margin = new Thickness(0, currentHeight, 0, 0);
            currentHeight += RBRatioLabel.Height + itemMargin;

            RBRatioSlider.VerticalAlignment = VerticalAlignment.Top;
            RBRatioSlider.Height = 30;
            RBRatioSlider.Margin = new Thickness(0, currentHeight, 0, 0);
            currentHeight += RBRatioSlider.Height + itemMargin;
            RBRatioSlider.Value = RBRatio;
            RBRatioSlider.ValueChanged += new RoutedPropertyChangedEventHandler<double>(updateValuesFromSlider);

            updatelabels();

            infoTxtBlk.VerticalAlignment = VerticalAlignment.Top;
            infoTxtBlk.Height = (kinectNUM * imageH - currentHeight > 500) ? kinectNUM * kinectH - currentHeight : 800;
            infoTxtBlk.Width = buttonsW;
            infoTxtBlk.Margin = new Thickness(0, currentHeight, 0, 0);
            currentHeight += infoTxtBlk.Height + itemMargin;
            infoTxtBlk.Text = "No message";
            
            rootGrid.ShowGridLines = true;
            //build root grid
            RowDefinition r0 = new RowDefinition();
            r0.Height = new GridLength(currentHeight);
            rootGrid.RowDefinitions.Add(r0);

            ColumnDefinition c0 = new ColumnDefinition();
            c0.Width = new GridLength(kinectH * (imageW / imageH) * gridColumnNUM + 25);//25 = index label width
            rootGrid.ColumnDefinitions.Add(c0);

            ColumnDefinition c1 = new ColumnDefinition();
            c1.Width = new GridLength(buttonsW);
            rootGrid.ColumnDefinitions.Add(c1);

            addToRootGrid(g, 0, 0);
            
            addToRootGrid(getNewPointBtn, 0, 1);
            addToRootGrid(updateImageBtn, 0, 1);
            addToRootGrid(resetBtn, 0, 1);
            addToRootGrid(returnBtn, 0, 1);
            addToRootGrid(getMatrixFromAllBtn, 0, 1);
            addToRootGrid(addNewMatrixBtn, 0, 1);
            
            addToRootGrid(dilateLabel, 0, 1);
            addToRootGrid(dilateSlider, 0, 1);

            addToRootGrid(erodeSlider, 0, 1);
            addToRootGrid(erodeLabel, 0, 1);

            addToRootGrid(RGRatioLabel, 0, 1);
            addToRootGrid(RGRatioSlider, 0, 1);

            addToRootGrid(RBRatioLabel, 0, 1);
            addToRootGrid(RBRatioSlider, 0, 1);
            addToRootGrid(infoTxtBlk, 0, 1);
        }

        //update labels to show current values
        void updatelabels()
        {
            dilateLabel.Content = "Dilate R :" + dilateValue.ToString();
            erodeLabel.Content = "Erode R :" + erodeValue.ToString();
            RBRatioLabel.Content = "RB ratio :" + RBRatio.ToString();
            RGRatioLabel.Content = "RG ratio :" + RGRatio.ToString();
        }

        //result calibration points
        void resetPoints(object sender, RoutedEventArgs e)
        {

            if (pointsToCalibrate != null)
            {
                for (int i = 0; i < pointsToCalibrate.Length; ++i)
                {
                    if (pointsToCalibrate[i] != null)
                    {
                        pointsToCalibrate[i].Clear();

                    }
                }
            }

            updateText("Points Reseted!!");
        }

        //update parameter values when slider changes
        void updateValuesFromSlider(object sender, RoutedEventArgs e)
        {   
            RGRatio = RGRatioSlider.Value;
            RBRatio = RBRatioSlider.Value;
            erodeValue = (int)erodeSlider.Value;
            dilateValue = (int)dilateSlider.Value;
            updatelabels();
        }

        //return to previous matrices, used when calibration fails
        void returnToPreviousMatrices(object sender, RoutedEventArgs e) {
            if (prevMatrices == "") 
                return;
            else {
                TextWriter tw1 = new StreamWriter("matrices.txt");
                tw1.Write(prevMatrices);
                tw1.Close();    
                GUIComponents.fc.fp.getTransformationMatrix();
            }        
        }

        //get transformation matrix from kinect 0
        void getMatrixFromAll(object sender, RoutedEventArgs e) {
            //check for points avaliable
            if (pointsToCalibrate == null)
            {
                MessageBox.Show("Avaliable points not enough. Please Get more points!!");             
                return;
            }
            if (pointsToCalibrate[0] == null)
            {
                MessageBox.Show("Avaliable points not enough. Please Get more points!!");
                return;
            }

            //remove failed points
            for (int i = 0; i < pointsToCalibrate[0].Count; ++i)
            {
                bool removeThisPoint = false;
                for (int j = 0; j < pointsToCalibrate.Length; ++j)
                    if (pointsToCalibrate[j][i][2, 0] < 0.5 || pointsToCalibrate[j][i][2, 0] > 2.5)
                    {
                        removeThisPoint = true;
                    }
                if (removeThisPoint)
                {
                    for (int j = 0; j < pointsToCalibrate.Length; ++j)
                        pointsToCalibrate[j].RemoveAt(i);
                    i--;
                }
            }

            //return if there are not enough avaliable points
            if (pointsToCalibrate[0].Count < 4) {
                int n = 4-pointsToCalibrate[0].Count;
                MessageBox.Show("Avaliable points not enough. Please Get "+ n.ToString() +  " more points!!");
                return;
            }

            List<DenseMatrix> transfMatrixList = new List<DenseMatrix>();
            MatrixProcessor mp = new MatrixProcessor();

            transfMatrixList.Add(DenseMatrix.CreateIdentity(4)); //transformation matrix for kinect 0
            for (int i = 1; i < pointsToCalibrate.Length;++i ) 
                transfMatrixList.Add(mp.getTransformationMatrixFromPoint(pointsToCalibrate[0], pointsToCalibrate[i]));

            //Compute error sum, when is the sum of errors between the corresponding points
            List<double> errorList = new List<double>();
            for (int j = 0; j < pointsToCalibrate.Length; ++j)
            {
                double errorSum = 0;
                for (int i = 0; i < pointsToCalibrate[0].Count; ++i)
                {
                    DenseMatrix dm0 = (DenseMatrix)pointsToCalibrate[0][i].Clone();
                    DenseMatrix dm1 = (DenseMatrix)pointsToCalibrate[j][i].Clone();
                    dm1 = transfMatrixList[j] * dm1;
                    errorSum += Math.Sqrt((dm0[0, 0] - dm1[0, 0]) * (dm0[0, 0] - dm1[0, 0]) + (dm0[1, 0] - dm1[1, 0]) * (dm0[1, 0] - dm1[1, 0])
                        + (dm0[2, 0] - dm1[2, 0]) * (dm0[2, 0] - dm1[2, 0]));
                }
                errorList.Add(errorSum);
            }


            //print result
            string textToPrint = "";
            for (int j = 0; j < pointsToCalibrate.Length; ++j)
            {
                textToPrint += "[KINECT " + j.ToString() + "] error sum = " + errorList[j] + Environment.NewLine +
                mp.printMatrix(transfMatrixList[j]);   
            }
            MessageBox.Show("calibration result = " + Environment.NewLine + textToPrint);

            prevMatrices = System.IO.File.ReadAllText("matrices.txt");
            TextWriter tw1 = new StreamWriter("matrices.txt");
            for (int j = 0; j < pointsToCalibrate.Length; ++j)
            {
                for (int row = 0; row < 4; ++row)
                    for (int col = 0; col < 4;++col )
                        tw1.Write(transfMatrixList[j][row ,col].ToString()+" ");
            }
            tw1.Close();
            GUIComponents.fc.fp.getTransformationMatrix();
        }

        //get transformation matrix from vector more then current matrice
        void addNewMatrix(object sender, RoutedEventArgs e) {
            if (pointsToCalibrate == null)
            {
                MessageBox.Show("Avaliable points not enough. Please Get more points!!");             
                return;
            }
            if (pointsToCalibrate[0] == null)
            {
                MessageBox.Show("Avaliable points not enough. Please Get more points!!");
                return;
            }

            //get current matrices
            //Get transformation matrix from matrices.txt
            string text = System.IO.File.ReadAllText("matrices.txt");
            string[] matricesData = text.Split(' ');
            for (int i = 0; i < matricesData.Length; i++)
            {
                matricesData[i].TrimEnd('\n');
            }

            int step = 16; // a matrix
            int oldKinectNum = matricesData.Length / step;
            List<DenseMatrix> transfMatrix = new List<DenseMatrix>();
            MatrixProcessor mp = new MatrixProcessor();
            for (int i = 0; i < oldKinectNum; ++i)
            {
                DenseMatrix m = new DenseMatrix(4);
                for (int row = 0; row < 4; ++row)
                    for (int col = 0; col < 4; ++col)
                    {
                        m[row, col] = Convert.ToDouble(matricesData[step * i + row * 4 + col]);
                    }
                transfMatrix.Add(m);
            }
            MessageBox.Show("Existing matrix num = "+  transfMatrix.Count);
        
            //check whether the point is enough for each kinect without matrix
            for(int kinectIndex = oldKinectNum; kinectIndex<pointsToCalibrate.Length;++kinectIndex ) {
                int pointNum = 0;
                for(int pointIndex =0; pointIndex < pointsToCalibrate[kinectIndex].Count; ++pointIndex) {
                    if( pointsToCalibrate[kinectIndex][pointIndex][2,0] > 0.5) 
                    {
                        bool hasKnownPoint = false;
                        //one of the previous kinect has value
                        for(int i =0;i<oldKinectNum;++i) {
                            if(pointsToCalibrate[i][pointIndex][2,0] > 0.5) {
                                hasKnownPoint = true;
                                break;
                            }
                        }
                        if(hasKnownPoint) 
                            pointNum++;
                    }
                }
                
                if(pointNum < 4) {
                    MessageBox.Show("Get "+(4-pointNum)+" more points for kinect "+kinectIndex);
                    return;
                }
            }

            //using existing information to calibrate kinects without matrix yet
            string textToPrint = "";
            for (int kinectIndex = oldKinectNum; kinectIndex < pointsToCalibrate.Length; ++kinectIndex)
            {
                //int pointNum = 0;
                List<DenseMatrix> pointsToCalibrate0 = new List<DenseMatrix>();
                List<DenseMatrix> pointsToCalibrate1 = new List<DenseMatrix>();
                for (int pointIndex = 0; pointIndex < pointsToCalibrate[kinectIndex].Count; ++pointIndex)
                {
                    if (pointsToCalibrate[kinectIndex][pointIndex][2, 0] > 0.5)
                    {
                       // bool hasKnownPoint = false;
                        //one of the existing kinects has value
                        for (int i = 0; i < oldKinectNum; ++i)
                        {
                            if (pointsToCalibrate[i][pointIndex][2, 0] > 0.5)
                            {
                                pointsToCalibrate0.Add(transfMatrix[i] * pointsToCalibrate[i][pointIndex]);
                                pointsToCalibrate1.Add(pointsToCalibrate[kinectIndex][pointIndex]);
                                break;
                            }
                        }
                    }
                }

                transfMatrix.Add(mp.getTransformationMatrixFromPoint(pointsToCalibrate0,pointsToCalibrate1));
                double errorSum = 0;

                for (int i = 0; i < pointsToCalibrate0.Count; ++i)
                {
                    DenseMatrix dm0 = (DenseMatrix)pointsToCalibrate0[i].Clone();
                    DenseMatrix dm1 = (DenseMatrix)pointsToCalibrate1[i].Clone();
                    dm1 = transfMatrix[kinectIndex] * dm1;
                    errorSum += Math.Sqrt((dm0[0, 0] - dm1[0, 0]) * (dm0[0, 0] - dm1[0, 0]) + (dm0[1, 0] - dm1[1, 0]) * (dm0[1, 0] - dm1[1, 0])
                        + (dm0[2, 0] - dm1[2, 0]) * (dm0[2, 0] - dm1[2, 0]));
                }
                textToPrint += "[KINECT " + kinectIndex.ToString() + "] error sum = " + errorSum + 
                    Environment.NewLine +mp.printMatrix(transfMatrix[kinectIndex]);
            }
            MessageBox.Show("calibration result = " + Environment.NewLine + textToPrint);

            //store previous result
            prevMatrices = text;
            TextWriter tw1 = new StreamWriter("matrices.txt");
            for (int j = 0; j < pointsToCalibrate.Length; ++j)
            {
                for (int row = 0; row < 4; ++row)
                    for (int col = 0; col < 4;++col )
                        tw1.Write(transfMatrix[j][row ,col].ToString()+" ");
            }
            tw1.Close();
            GUIComponents.fc.fp.getTransformationMatrix();
        }
        
        //handle GUI layout
        void buildImagesGrid()
        {
            g.ShowGridLines = true;

            //to show kinect label
            ColumnDefinition c = new ColumnDefinition();
            c.Width = new GridLength(25);
            g.ColumnDefinitions.Add(c);

            for (int i = 0; i < gridColumnNUM; ++i)
            {
                ColumnDefinition c0 = new ColumnDefinition();
                c0.Width = new GridLength(kinectH * imageW / imageH);
                g.ColumnDefinitions.Add(c0);
            }

            for (int i = 0; i < kinectNUM; ++i)
            {
                RowDefinition r0 = new RowDefinition();
                r0.Height = new GridLength(kinectH);
                g.RowDefinitions.Add(r0);
            }

            for (int i = 0; i < kinectNUM; ++i)
            {
                Label l = new Label();
                l.Content = i.ToString();
                addToKinectGrid(l, i, 0);
                addToKinectGrid(rgbImageArray[i], i, 1);
                addToKinectGrid(depthImageArray[i], i, 2);
                addToKinectGrid(filteredImageArray[i], i, 3);
                addToKinectGrid(resultImageArray[i], i, 4);
            }
        }
        void addToRootGrid(UIElement elem, int row, int col)
        {
            Grid.SetColumn(elem, col);
            Grid.SetRow(elem, row);
            rootGrid.Children.Add(elem);
        }
        void addToKinectGrid(UIElement elem, int row, int col)
        {
            if (row > kinectNUM || col > gridColumnNUM)
            {
                MessageBox.Show("Row or Colum number ERROR!");
                return;
            }
            Grid.SetColumn(elem, col);
            Grid.SetRow(elem, row);
            g.Children.Add(elem);
        }

        //update image data to GUI. Use delegate because the data and GUI are in different threads
        delegate bool updateImagesCallback();
        public bool updateImages() 
        {
            if (this.g.Dispatcher.Thread != Thread.CurrentThread)
            {
                this.g.Dispatcher.Invoke(System.Windows.Threading.DispatcherPriority.Normal, new updateImagesCallback(this.updateImages));//, in_rgbBitmaps, in_depthBitmaps, in_3DPointArray);
            }
            else
            {
                for (int i = 0; i < kinectNUM; ++i)
                {
                    rgbArray[i] = parentGUI.kinectCompList[i].colorImageWritableBitmap.Clone();
                    depthArray[i] = parentGUI.kinectCompList[i].depthImageWritableBitmap.Clone();
                    rgbImageArray[i].Source = rgbArray[i];
                    depthImageArray[i].Source = depthArray[i];
                }


                if (filteredArray[0] == null)
                {
                    for (int i = 0; i < kinectNUM; ++i)
                    {
                        filteredArray[i] = new WriteableBitmap(rgbArray[0]);
                        resultArray[i] = new WriteableBitmap(rgbArray[0]);
                    }
                }

                //update 3D point info
                point3DArray = new List<float[]>();

                for (int i = 0; i < kinectNUM; ++i)
                {
                    point3DArray.Add(parentGUI.kinectCompList[i].point3DArray);
                }
            }
            return true;
        }

        public int getPixelDataIndex(int col, int row, int step, int stride)
        {
            return (col * step + row * stride);
        }

        //update the images from all kinects
        void updateAllImages(object sender, RoutedEventArgs e)
        {
            parentGUI.updateAllImages();

            int i = 0;
            while (i < 1000000)
            {
                ++i;
            }
            updateImages();
        }

        //get the position of red ball from RGB image and the corresponding 3D coordinate
        void getSamplePoints(object sender, RoutedEventArgs e)
        {
            if (pointsToCalibrate[0] == null)
            {
                for (int i = 0; i < kinectNUM; ++i)
                    pointsToCalibrate[i] = new List<DenseMatrix>();
            }

            int stride = (int)imageW * ((rgbArray[0].Format.BitsPerPixel + 7) / 8);
            int step = ((rgbArray[0].Format.BitsPerPixel + 7) / 8);
            

            for (int kinectIndex = 0; kinectIndex < kinectNUM; ++kinectIndex)
            {
                byte[] pixel = new byte[(int)imageH * stride];
                byte[] rgbPixelData = new byte[(int)imageH * stride];
                rgbArray[kinectIndex].CopyPixels(rgbPixelData, stride, 0);
                Parallel.For(0, (int)imageH, y =>
                {
                    int x;
                    for (x = 0; x < imageW; x++)
                    {
                        int index = getPixelDataIndex(x, y, step, stride);
                        byte R_value = rgbPixelData[index + 2];
                        byte B_value = rgbPixelData[index];
                        byte G_value = rgbPixelData[index + 1];

                        if (R_value / (G_value + 1) > RGRatio && R_value / (B_value + 1) > RBRatio)
                        {
                            pixel[index] = 255;
                            pixel[index + 1] = 255;
                            pixel[index + 2] = 255;
                            continue;
                        }
                        pixel[index] = 0;
                        pixel[index + 1] = 0;
                        pixel[index + 2] = 0;
                    }
                });

                filteredArray[kinectIndex].WritePixels(new Int32Rect(0, 0, (int)imageW, (int)imageH), pixel, stride, 0);
                filteredImageArray[kinectIndex].Source = filteredArray[kinectIndex];
                //**************compute eroded bitmap*********************
                int pixel_length = (int)imageH * stride;
                byte[] eroded_pixel = new byte[pixel_length];
                //set all pixels to black
                for (int col = 0; col < imageW; ++col)
                {
                    for (int row = 0; row < imageH; ++row)
                    {
                        int index = getPixelDataIndex(col, row, step, stride);
                        eroded_pixel[index] = 0;
                        eroded_pixel[index + 1] = 0;
                        eroded_pixel[index + 2] = 255;
                    }
                }

                int erodeR = (int)erodeValue;// Convert.ToInt32(erodeSlider.Value);
                int dilateR = (int)dilateValue;// Convert.ToInt32(dilateSlider.Value);
                for (int row = erodeR; row < imageH - erodeR; ++row)
                {
                    for (int col = erodeR; col < imageW - erodeR; ++col)
                    {
                        int row_local = row - erodeR;
                        int col_local = col - erodeR;
                        if (pixel[getPixelDataIndex(col, row, step, stride)] == 0)
                        {
                            for (int row_r = 0; row_r < erodeR * 2; ++row_r)
                                for (int col_r = 0; col_r < erodeR * 2; ++col_r)
                                {
                                    eroded_pixel[getPixelDataIndex(col_local + col_r, row_local + row_r, step, stride) + 2] = 0;
                                }
                        }
                    }
                }

                //**************compute dilated bitmap*********************
                byte[] dilated_pixel = new byte[pixel_length];
                Parallel.For(0, pixel_length, i =>
                {
                    dilated_pixel[i] = 0;
                });

                Parallel.For(dilateR, (int)imageH - dilateR, row =>
                {
                    Parallel.For(dilateR, (int)imageW - dilateR, col =>
                    {
                        int row_local = row - dilateR;
                        int col_local = col - dilateR;
                        if (eroded_pixel[getPixelDataIndex(col, row, step, stride) + 2] == 255)
                        {
                            for (int row_r = 0; row_r < dilateR * 2; ++row_r)
                                for (int col_r = 0; col_r < dilateR * 2; ++col_r)
                                {
                                    dilated_pixel[getPixelDataIndex(col_local + col_r, row_local + row_r, step, stride) + 2] = 255;
                                }
                        }
                    });
                });

                //**************compute connected componenets on bitmap*********************
                Field field = new Field((int)imageW, (int)imageH);
                for (int col = 0; col < (int)imageW; col++)
                    for (int row = 0; row < (int)imageH; row++)
                    {
                        Color c = Color.FromArgb(0, dilated_pixel[getPixelDataIndex(col, row, step, stride) + 2],
                                                dilated_pixel[getPixelDataIndex(col, row, step, stride) + 2],
                                                dilated_pixel[getPixelDataIndex(col, row, step, stride) + 2]);
                        field[col, row] = c;
                    }
                var components = Algorithm.FindConnectedComponents(field);

                if (components.Count == 0)
                {
                    MessageBox.Show("No component found");
                }

                Random random = new Random();
                foreach (var component in components)
                {
                    Color color = Color.FromRgb(Convert.ToByte(random.Next(0, 255)), Convert.ToByte(random.Next(0, 255)), Convert.ToByte(random.Next(0, 255)));//field.GetRandomColor();
                    foreach (var span in component.Spans)
                    {
                        for (int x = span.StartX; x <= span.EndX; x++)
                        {
                            if (dilated_pixel[getPixelDataIndex(x, span.Y, step, stride) + 2] == 0) continue;
                            dilated_pixel[getPixelDataIndex(x, span.Y, step, stride)] = color.B;
                            dilated_pixel[getPixelDataIndex(x, span.Y, step, stride) + 1] = color.G;
                            dilated_pixel[getPixelDataIndex(x, span.Y, step, stride) + 2] = color.R;
                        }
                    }
                }

                //**************find the larget component************** 
                int largestIndex = 0;
                int largestSize = components[0].getTotalSize();
                for (int i = 1; i < components.Count; ++i)
                {
                    int size = components[i].getTotalSize();
                    if (size > largestSize && size < 100000)
                    {
                        largestIndex = i;
                        largestSize = size;
                    }
                }
                //**************compute center pixel**************
                int centerX = -1;
                int centerY = -1;
                foreach (var span in components[largestIndex].Spans)
                {
                    for (int x = span.StartX; x <= span.EndX; x++)
                    {
                        centerX += x;
                        centerY += span.Y;
                    }
                }
                centerX /= largestSize;
                centerY /= largestSize;

                //**************label center**************

                for (int x = centerX - 1; x <= centerX + 1; ++x)
                    for (int y = centerY - 1; y <= centerY + 1; ++y)
                    {
                        dilated_pixel[getPixelDataIndex(x, y, step, stride)] = 255;
                        dilated_pixel[getPixelDataIndex(x, y, step, stride) + 1] = 255;
                        dilated_pixel[getPixelDataIndex(x, y, step, stride) + 2] = 255;
                    }

                resultArray[kinectIndex].WritePixels(new Int32Rect(0, 0, (int)imageW, (int)imageH), dilated_pixel, stride, 0);
                resultImageArray[kinectIndex].Source = resultArray[kinectIndex];
                
                DenseMatrix m = new DenseMatrix(4, 1);
                int pointIndex = (centerX + centerY * (int)imageW) * 3;
                if (centerX != 319 && point3DArray[kinectIndex][pointIndex + 2] > 0.3)//z value > 0.3 
                {
                    m[0, 0] = point3DArray[kinectIndex][pointIndex];//sp.X;
                    m[1, 0] = point3DArray[kinectIndex][pointIndex + 1];//sp.Y;
                    //add ball radius
                    m[2, 0] = point3DArray[kinectIndex][pointIndex + 2]+BALL_RADIUS;// sp.Z;
                    m[3, 0] = 1;
                }
                else
                {
                    m[0, 0] = 0;
                    m[1, 0] = 0;
                    m[2, 0] = 0;
                    m[3, 0] = 1;
              //      isAvaliable = false;
                }

                pointsToCalibrate[kinectIndex].Add(m);
                updateText("[KINECT " + kinectIndex.ToString() + "] add newpoint (" + m[0, 0].ToString("0.###") + "," + m[1, 0].ToString("0.###") + "," + m[2, 0].ToString("0.###") + ")");
            }
                   }

        //print information to GUI
        int lineNUM = 15;
        Queue<string> textToPrint = new Queue<string>();
        void updateText(string text)
        {
            if (textToPrint.Count < lineNUM)
            {
                textToPrint.Enqueue(text);
            }
            else
            {
                textToPrint.Enqueue(text);
                textToPrint.Dequeue();
            }

            infoTxtBlk.Text = "";
            foreach (string s in textToPrint)
            {
                infoTxtBlk.Text = s + Environment.NewLine + infoTxtBlk.Text;    
            }
        }
    }


}
