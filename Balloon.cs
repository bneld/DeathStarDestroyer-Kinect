using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Drawing;
using System.Windows;
using System.Windows.Media;
namespace Microsoft.Samples.Kinect.BodyBasics
{
    class Balloon
    {
        private Point location;
        private double diameter;
        private Boolean exploded;

        public Balloon(Point xlocation, double diameter, Boolean exploded){
            this.location = xlocation;
            this.diameter = diameter;
            
            this.exploded = exploded;
        }

        public double getXLocation()
        {
            return location.X;
        }

        public double getYLocation()
        {
            return location.Y;
        }

        public double getDiameter()
        {
            return diameter;
        }

        public Boolean getExploded()
        {
            return exploded;
        }

        public void setPoint(Point p)
        {
            location = p;
        }
        
        public void setDiameter(double d)
        {
            diameter = d;
        }

        public void setExploded(Boolean e)
        {
            exploded = e;
        }
    }
}
