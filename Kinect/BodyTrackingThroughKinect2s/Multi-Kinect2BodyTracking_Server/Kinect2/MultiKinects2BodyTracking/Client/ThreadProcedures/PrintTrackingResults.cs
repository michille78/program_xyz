using System;
using System.IO;
using System.Windows.Controls;
    // Using multi-threads
using System.Threading;

namespace Kinect2.MultiKinects2BodyTracking.Client.ThreadProcedures {   
    /// <summary>
    /// The thread procedure used to print kinect data on GUI
    /// </summary>
    public class PrintTrackingResults {

        #region Members

        /// <summary>
        /// MainWindow of the current application
        /// </summary>
        private MainWindow mw;

        #endregion // Members

        #region Methods

        /// <summary>
        /// Constructors, used to pass the current MainWindow of the application
        /// </summary>
        /// <param name="m"></param>
        public PrintTrackingResults(MainWindow _m) {
            mw = _m;
        }

        delegate void printValueCallback(TextBlock t);
        public void printValue(TextBlock t) {
            if (mw.result_TextBlock.Dispatcher.Thread != Thread.CurrentThread)
                mw.result_TextBlock.Dispatcher.Invoke(System.Windows.Threading.DispatcherPriority.Normal, new printValueCallback(this.printValue), mw.result_TextBlock);
            else
                mw.result_TextBlock.Text = mw.kinectparameters_local.printKinectParameters();
        }

        public void ThreadProc() {
            while (mw.printResultThreadAlive == true) {
                printValue(mw.result_TextBlock);
                    // Slow down the GUI updating rate to reduce cost
                Thread.Sleep(750);
            }
        }

        #endregion // Methods
    }
}
