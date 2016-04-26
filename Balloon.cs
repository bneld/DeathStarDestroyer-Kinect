using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Drawing;
using System.Windows;
using System.Windows.Media;
using System.Diagnostics;
namespace Microsoft.Samples.Kinect.BodyBasics
{
    class Balloon
    {
        private Point location;
        private double diameter;
        private Boolean exploded;
        private Boolean visible;
        private int explosionRadius;
        private int ticks;

        public Balloon(Point xlocation, double diameter, Boolean exploded, Boolean visible, int ticks){
            this.location = xlocation;
            this.diameter = diameter;
            this.exploded = exploded;
            this.visible = visible;
            explosionRadius = 5;
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

        public int getTicks()
        {
            return ticks;
        }

        public Boolean getVisible()
        {
            return visible;
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
            if (e)
            {
                this.setVisible(false);
            }
        }

        public void setVisible(Boolean v)
        {
            visible = v;
        }

        public void setTicks(int ticks)
        {
            this.ticks = ticks;
        }

        public void decrementTicks()
        {
            this.ticks--;
            if (ticks < 0)
            {
                setVisible(false);
            }
        }

        public int getExplosionRadius()
        {
            return explosionRadius;
        }
        public void increaseExplosionRadius()
        {
            explosionRadius += 5;
        }
    }
}
