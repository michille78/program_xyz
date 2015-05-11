using System;

namespace Kinect2.Exceptions
{
    class StreamHasNotBeenOpened : Exception
    {
        private string stringName = null;

        public override string Message {
            get {
                return "The stream " + stringName + "has not been opened!";
            }
        }

        public StreamHasNotBeenOpened(string name) {
            stringName = name;
        }
    }
}
