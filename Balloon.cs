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
        private Color color;
        private Boolean exploded;

        public Balloon(Point location, double diameter, Color color){
            this.location = location;
            this.diameter = diameter;
            this.color = color;
            this.exploded = true;
        }

        public Point getPoint()
        {
            return location;
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

        public void setColor(Color c)
        {
            color = c;
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
