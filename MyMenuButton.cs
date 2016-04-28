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
    public class MyMenuButton
    {
        private String text;
        private SolidColorBrush color;
        private SolidColorBrush fontColor;
        private System.Windows.Media.Pen borderColor;
        private int x;
        private int y;
        private int height;
        private int width;
        private Rect rectangle;

        public MyMenuButton(string t, SolidColorBrush fC, SolidColorBrush c, System.Windows.Media.Pen bC, int x, int y, int w, int h)
        {
            text = t;
            fontColor = fC;
            color = c;
            borderColor = bC;
            this.x = x;
            this.y = y;
            height = h;
            width = w;
            this.rectangle = new Rect(x, y, w, h);
        }

        public String getText()
        {
            return text;
        }

        public int getX()
        {
            return x;
        }

        public int getY()
        {
            return y;
        }

        public int getHeight()
        {
            return height;
        }

        public int getWidth()
        {
            return width;
        }

        public SolidColorBrush getColor()
        {
            return color;
        }

        public System.Windows.Media.Pen getBorderColor()
        {
            return borderColor;
        }

        public SolidColorBrush getFontColor()
        {
            return fontColor;
        }

        public Rect getRect()
        {
            return rectangle;
        }
    }
}