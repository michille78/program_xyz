using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.IO.Compression;
using System.Windows;
using MathNet.Numerics.LinearAlgebra.Double;
using System.Runtime.Serialization.Formatters.Binary;
using System.Runtime.Serialization;
using Microsoft.Kinect;

namespace Kinect2.MultiKinects2BodyTracking.DataStructure
{
    static class ExtensionHelpers {
        /// <summary>
        /// Clone skeleton class
        /// </summary>
        /// <param name="skOrigin"></param>
        /// <returns></returns>
        public static BodyData CloneSkeleton(this BodyData skOrigin) {
            try {
                MemoryStream ms = new MemoryStream();
                BinaryFormatter bf = new BinaryFormatter();

                bf.Serialize(ms, skOrigin);
                ms.Position = 0;
                object obj = bf.Deserialize(ms);
                ms.Close();
                return obj as BodyData;
            } catch (Exception ex) {
                string ss = ex.ToString();
                MessageBox.Show(ss);

                object obj =  new object();
                return obj as BodyData;
            }
        }
    }

    //This class compress and decompress string to Base64 format
    [Serializable]
    public class StringCompressor
    {
        //compress byte array to Base 64 string
        public string CompressByteArray(byte[] byteArray)
        {
            try
            {
                MemoryStream ms = new MemoryStream();
                using (GZipStream zip = new GZipStream(ms, CompressionMode.Compress, true))
                {
                    zip.Write(byteArray, 0, byteArray.Length);
                }
                ms.Position = 0;
                byte[] compressed = new byte[ms.Length];
                ms.Read(compressed, 0, compressed.Length);
                byte[] gzBuffer = new byte[compressed.Length + 4];
                System.Buffer.BlockCopy(compressed, 0, gzBuffer, 4, compressed.Length);
                System.Buffer.BlockCopy(BitConverter.GetBytes(byteArray.Length), 0, gzBuffer, 0, 4);
                return Convert.ToBase64String(gzBuffer);
            }
            catch
            {
                return "";
            }
        }
        //decompress from Base 64 String format to Byte array
        public byte[] DecompressByteArray(string compressedText)
        {
            try
            {
                byte[] gzBuffer = Convert.FromBase64String(compressedText);
                using (MemoryStream ms = new MemoryStream())
                {
                    int msgLength = BitConverter.ToInt32(gzBuffer, 0);
                    ms.Write(gzBuffer, 4, gzBuffer.Length - 4);
                    byte[] buffer = new byte[msgLength];

                    ms.Position = 0;
                    using (GZipStream zip = new GZipStream(ms, CompressionMode.Decompress))
                    {
                        zip.Read(buffer, 0, buffer.Length);
                    }
                    return buffer;
                }
            }
            catch
            {
                return null;
            }
        }

        //compress string to Base64 String 
        public string Compress(string text)
        {
            try
            {
                byte[] buffer = Encoding.UTF8.GetBytes(text);
                MemoryStream ms = new MemoryStream();
                using (GZipStream zip = new GZipStream(ms, CompressionMode.Compress, true))
                {
                    zip.Write(buffer, 0, buffer.Length);
                }
                ms.Position = 0;
                byte[] compressed = new byte[ms.Length];
                ms.Read(compressed, 0, compressed.Length);
                byte[] gzBuffer = new byte[compressed.Length + 4];
                System.Buffer.BlockCopy(compressed, 0, gzBuffer, 4, compressed.Length);
                System.Buffer.BlockCopy(BitConverter.GetBytes(buffer.Length), 0, gzBuffer, 0, 4);


                string result = Convert.ToBase64String(gzBuffer);

                //result = result.TrimEnd('=');
                return result;

            }
            catch
            {
                return "";
            }

        }
        //decompress string from Base64 String
        public string Decompress(string compressedText)
        {
            try
            {
                byte[] gzBuffer = Convert.FromBase64String(compressedText);
                using (MemoryStream ms = new MemoryStream())
                {
                    int msgLength = BitConverter.ToInt32(gzBuffer, 0);
                    ms.Write(gzBuffer, 4, gzBuffer.Length - 4);
                    byte[] buffer = new byte[msgLength];

                    ms.Position = 0;
                    using (GZipStream zip = new GZipStream(ms, CompressionMode.Decompress))
                    {
                        zip.Read(buffer, 0, buffer.Length);
                    }
                    return Encoding.UTF8.GetString(buffer);
                }
            }
            catch
            {
                return "";
            }
        }
    }


    //This class defines tools needed for matrix computation
    public class MatrixProcessor
    {
        //Get matrix whose rows are the input points 
        private DenseMatrix getMatrixFromPoints(List<DenseVector> v)
        {
            int s = v.Count();
            DenseMatrix V = new DenseMatrix(4, s);
            for (int i = 0; i < s; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    V[j, i] = v[i][j];
                }
                V[3, i] = 1;
            }
            return V;
        } 

        //compute the transformation matrix between 2 point sets
        //Reference : http://nghiaho.com/?page_id=671
        public DenseMatrix getTransformationMatrixFromPoint(List<DenseMatrix> v0, List<DenseMatrix> v)
        {
            //make an identity matrix
            DenseMatrix T = new DenseMatrix(4, 4);
            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 4; ++j)
                    if (i == j)
                        T[i, j] = 1;
                    else
                        T[i, j] = 0;
            if (v0.Count == 0 || v.Count == 0 || (v0.Count != v.Count)) return T;

            //compute the sum of point set 0
            DenseMatrix avg0 = (DenseMatrix)v0[0].Clone();
            for (int i = 1; i < v0.Count(); ++i)
            {
                for (int j = 0; j < 3; ++j)
                    avg0[j, 0] += v0[i][j, 0];
            }

            //compute the sum of point set 1
            DenseMatrix avg = (DenseMatrix)v[0].Clone();
            for (int i = 1; i < v.Count(); ++i)
            {
                for (int j = 0; j < 3; ++j)
                    avg[j, 0] += v[i][j, 0];
            }

            //compute the average center of point set 0 and point set 1
            for (int j = 0; j < 3; ++j)
            {
                avg0[j, 0] = avg0[j, 0] / v.Count;
                avg[j, 0] = avg[j, 0] / v.Count;
            }

            //compute rotation matrix and translation vector
            var v0_bar = (DenseMatrix)(v0[0] - avg0).Transpose();
            DenseMatrix H = (v[0] - avg) * v0_bar;
            for (int i = 1; i < v.Count(); ++i)
            {
                var v_bar = (DenseMatrix)(v0[i] - avg0).Transpose();
                H += (v[i] - avg) * v_bar; 
            }

            var svd = H.Svd(true);  
            svd.Solve(H);
            var U = (DenseMatrix)svd.U;
            var VT = (DenseMatrix)svd.VT;
            var W = (DenseMatrix)svd.W;
            VT = (DenseMatrix)VT.Transpose();
            U = (DenseMatrix)U.Transpose();
            
            //rotation matrix
            var R = (DenseMatrix)VT * U;

            //translation vector
            var t = (DenseMatrix)avg0 - R * avg;

            for (int i = 0; i < 4; ++i)
            {
                for (int j = 0; j < 4; ++j)
                {
                    if (i == 3 && j == 3)
                    {
                        T[i, j] = 1;
                    }
                    else if (i == 3)
                    {
                        T[i, j] = 0;
                    }
                    else if (j == 3)
                    {
                        T[i, j] = t[i, 0];
                    }
                    else
                    {
                        T[i, j] = R[i, j];
                    }
                }
            }
            return T;
        }
             
        //print the content of a matrix to string
        public string printMatrix(DenseMatrix m)
        {
            string s = "";
            for (int row = 0; row < m.RowCount; row++)
            {
                for (int col = 0; col < m.ColumnCount; ++col)
                    s += m[row, col].ToString("F2") + " ";
                s += Environment.NewLine;

            }
            s += Environment.NewLine;
            return s;
        }

    }
}
