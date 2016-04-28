//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserv
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.BodyBasics
{


    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using System.Runtime.InteropServices;
    using System.Linq;
    using System.Windows.Controls;

    //using System.Drawing;
    using System.Drawing.Drawing2D;
    using System.Threading.Tasks;
    using System.Reflection;
    using System.Drawing.Imaging;
    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        
        private const int numberOfCirclesAcross = 10;
        private const int numberOfCirclesDown = 10;
        //private const int numberOfSidesOnExplosion = 8;
        private const int numberOfSidesOnExplosion = 24;
        //precompute angles for explosion
        //private double[] explodeXAngles = {1, 0.7071, 0, -0.7071, -1, -0.7071, 0, 0.7071};
        //private double[] explodeYAngles = {0, 0.7071, 1, 0.7071, 0, -0.7071, -1, -0.7071};
        private double[] explodeXAngles = { 1, 0.9659, 0.8660, 0.7071, 0.5, 0.2588, 0, -0.2588, -0.5, - 0.7071, -0.8660, -0.9659, -1, -0.9659, -0.8660, -0.7071, -0.5, -0.2588, 0, 0.2588, 0.5, 0.7071, 0.8660, 0.9659};
        private double[] explodeYAngles = { 0, 0.2588, 0.5, 0.7071, 0.8660, 0.9659, 1, 0.9659, 0.8660, 0.7071, 0.5, 0.2588, 0, -0.2588, -0.5, -0.7071, -0.8660, -0.9659, -1, -0.9659, -0.8660, -0.7071, -0.5, -0.2588 };
        static List<Balloon> backgroundBalloons;
        private int mode = 0; // 0 for start menu , 1 for main game, 2 for khaled's mode, 3 for brian's  

        private double circleDiameter;
        private const double HandSize = 30;
        private const double JointThickness = 3;
        private const double ClipBoundsThickness = 10;
        private const float InferredZPositionClamp = 0.1f;
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));
        private readonly Brush customBrush = new SolidColorBrush(Color.FromArgb(255, 255, 0, 0));
        private readonly Brush inferredJointBrush = Brushes.Yellow;
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);
        private DrawingGroup drawingGroup;
        private DrawingImage imageSource;
        public KinectSensor kinectSensor = null;
        private CoordinateMapper coordinateMapper = null;
        private BodyFrameReader bodyFrameReader = null;
        private Body[] bodies = null;
        private List<Point> circles;
        private List<Tuple<JointType, JointType>> bones;
        private int displayWidth;
        private int displayHeight;
        private List<Pen> bodyColors;
        private string statusText = null;
        
        private List<Balloon> balloons;
        private bool bothHandsClosed = false;
        private bool rightHandClosed = false;
        private bool leftHandClosed = false;
        private bool rightHandLasso = false;
        private bool leftHandLasso = false;



        //Khaled Mode Variables
        private int k_width = 0;
        private int k_height = 0;
        private byte[] k_pixels = null;
        private WriteableBitmap k_bitmap = null;
        private ColorFrameReader k_colorReader = null;
        private BodyFrameReader k_bodyReader = null;
        private IList<Body> k_bodies = null;
        private int kMode = 0;
        private bool photoTaken = false;
        private String userPhotoPath; 



        private int timeCounter = 0;
        private int maxBalloonsVisible;
        private int currentBalloonsVisible;
        private Random rnd = new Random();
        private int userScore = 0;

        private Boolean countdown = true;
        private Boolean startTimer = true;
        private Point prevXWingPoint;
        private BitmapImage userPhotoBitmap;

        //Transformation related 
        private TransformGroup rtrGrp;
        private SkewTransform rtrSkw;
        private RotateTransform rtrRot;
        private TranslateTransform rtrTns;
        private ScaleTransform rtrScl;


        private TransformGroup ltrGrp;
        private SkewTransform ltrSkw;
        private RotateTransform ltrRot;
        private TranslateTransform ltrTns;
        private ScaleTransform ltrScl;


        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
        
            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();
            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();
            this.circles = new List<Point>();
            
            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();
            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();

            //Create the grid of Balloons
            createCircleGrid();


            //Transformations related 
            //Right wing 
            rtrSkw = new SkewTransform(0, 0);
            rtrRot = new RotateTransform(0);
            rtrTns = new TranslateTransform(0, 0);
            rtrScl = new ScaleTransform(1, 1);

            rtrGrp = new TransformGroup();
            rtrGrp.Children.Add(rtrSkw);
            rtrGrp.Children.Add(rtrRot);
            rtrGrp.Children.Add(rtrTns);
            rtrGrp.Children.Add(rtrScl);
            //left wing
            ltrSkw = new SkewTransform(0, 0);
            ltrRot = new RotateTransform(0);
            ltrTns = new TranslateTransform(0, 0);
            ltrScl = new ScaleTransform(1, 1);

            ltrGrp = new TransformGroup();
            ltrGrp.Children.Add(ltrSkw);
            ltrGrp.Children.Add(ltrRot);
            ltrGrp.Children.Add(ltrTns);
            ltrGrp.Children.Add(ltrScl);


            //wing.Visibility = Visibility.Visible;



            //Khaled's Initializations. 
            this.k_width = this.kinectSensor.ColorFrameSource.FrameDescription.Width;
            this.k_height = this.kinectSensor.ColorFrameSource.FrameDescription.Height;

            this.k_colorReader = this.kinectSensor.ColorFrameSource.OpenReader();
            this.k_colorReader.FrameArrived += ColorReader_FrameArrived;

            this.k_bodyReader = this.kinectSensor.BodyFrameSource.OpenReader();
            this.k_bodyReader.FrameArrived += BodyReader_FrameArrived;

            this.k_pixels = new byte[this.k_width * this.k_height * 4];
            this.k_bitmap = new WriteableBitmap(this.k_width, this.k_height, 96.0, 96.0, PixelFormats.Bgra32, null);

            this.k_bodies = new Body[this.kinectSensor.BodyFrameSource.BodyCount];

            khaledMode.Source = this.k_bitmap;
            khaledMode.Visibility = Visibility.Hidden;

            //BackgroundPic.IsEnabled = false;
            //BackgroundPic.Visibility = Visibility.Hidden;
           
        }

        
        // INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        
        public event PropertyChangedEventHandler PropertyChanged;

        
        // Gets the bitmap to display
        
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            
            leftWing.RenderTransform = ltrGrp;
            RightWing.RenderTransform = rtrGrp;
           
            //lSclX.Value = slSclY.Value = 1;
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            }
        }

        
        /// Execute shutdown tasks
       
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }

            if (this.k_colorReader != null)
            {
                this.k_colorReader.Dispose();
            }

            if (this.k_bodyReader != null)
            {
                this.k_bodyReader.Dispose();
            }

        }

        
        // Handles the body frame data arriving from the sensor
    
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    if (this.circles == null)
                    {
                        //this.circles = new 
                    }
                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }

            if (dataReceived)
            {
              using (DrawingContext dc = this.drawingGroup.Open())
                {
                    
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));


                    //Draw 
                    if (mode == 0) drawStartMenu(dc);
                    else if (mode == 1) drawCircleGrid(dc);

                    int penIndex = 0;
                    foreach (Body body in this.bodies)
                    {
                        Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {
                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);

                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }
                            
                            
                        
                            this.DrawBody(joints, jointPoints, dc, drawPen);
                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            //ltrTns.X = jointPoints[JointType.HandLeft].X;
                            //ltrTns.Y = jointPoints[JointType.HandLeft].Y;
                            // Draw the xWing along the hand 
                            //leftWing.Margin = new Thickness(this.Width - jointPoints[JointType.HandLeft].X, jointPoints[JointType.HandLeft].Y,
                            //             jointPoints[JointType.HandLeft].X, this.Height - jointPoints[JointType.HandLeft].Y);
                            //RightWing.Margin = new Thickness(this.Width - jointPoints[JointType.HandRight].X, jointPoints[JointType.HandRight].Y,
                            //       jointPoints[JointType.HandRight].X, this.Height - jointPoints[JointType.HandRight].Y);
                            //ltrTns.X = jointPoints[JointType.HandLeft].X;
                            //ltrTns.Y = jointPoints[JointType.HandLeft].Y;
                                                       



                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                            this.startGame(dc, jointPoints[JointType.HandLeft], jointPoints[JointType.HandRight]);
                            //this.drawCircles(body.HandRightState, jointPoints[JointType.HandRight], dc);

                            if (body.HandLeftState == HandState.Closed && body.HandRightState == HandState.Closed) {
                                this.bothHandsClosed = true;
                                Console.WriteLine("Both Hands are  closed");   
                            }
                            else if ( body.HandLeftState == HandState.Closed)
                             {
                                leftHandClosed = true;
                                Console.WriteLine("Left Hand Closed");
                             }
                           else   if (body.HandRightState == HandState.Closed)
                            {
                                rightHandClosed = true;
                                Console.WriteLine("Rigt Hand CLosed ");
                            }
                          else    if(body.HandLeftState == HandState.Lasso )
                            {
                                this.leftHandLasso = true;
                            }
                          else   if(body.HandRightState == HandState.Lasso)
                            {
                                this.rightHandLasso = true; 
                            }
                            else {
                                this.bothHandsClosed = false;
                                this.leftHandClosed = false;
                                this.rightHandClosed = false;
                                this.rightHandLasso = false;
                                this.leftHandLasso = false;
                               
                            }
                            
                            //Console.WriteLine("Left " + body.Joints[JointType.HandLeft].Position);

                        }
                    }
                    
                    // prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }
            }
        }

     
        /// Draws a body
       /// <param name="joints">joints to draw</param>
       
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
             
                if (!jointType.Equals(JointType.KneeLeft) && !jointType.Equals(JointType.KneeRight) && !jointType.Equals(JointType.AnkleLeft) && !jointType.Equals(JointType.AnkleRight) && !jointType.Equals(JointType.FootRight) && !jointType.Equals(JointType.FootLeft) && !jointType.Equals(JointType.Head) && !jointType.Equals(JointType.Neck) && !jointType.Equals(JointType.SpineShoulder) && !jointType.Equals(JointType.SpineMid) && !jointType.Equals(JointType.SpineBase) && !jointType.Equals(JointType.HipRight) && !jointType.Equals(JointType.HipLeft))
                {
                    Brush drawBrush = null;

                    TrackingState trackingState = joints[jointType].TrackingState;

                    if (trackingState == TrackingState.Tracked)
                    {
                        drawBrush = this.trackedJointBrush;
                    }
                    else if (trackingState == TrackingState.Inferred)
                    {
                        drawBrush = this.inferredJointBrush;
                    }

                    if (drawBrush != null)
                    {
                        drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                    }
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
       
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }


        //Draw Circle 
        private void drawCircle( Brush b,  DrawingContext drawingContext, double x , double y , double diameter, int mod)
        {
            if (mod == 0) // Normal Circle
            {

                drawingContext.DrawEllipse(b, null, new Point(x, y), 20, 20);
                drawingContext.DrawImage(new BitmapImage(new Uri(@"Images/spaceship.png", UriKind.RelativeOrAbsolute)), new Rect(x - 28, y - 28, 60, 60));
            }
          

        }

        private void drawXWing(DrawingContext dc , double x , double y, double angle)
        {
            
            ltrRot.Angle = angle;
           Canvas.SetLeft(leftWing, x - 195 );
           Canvas.SetTop(leftWing, y - 145 );
            Console.WriteLine("X: " + x + " Y: " + y);
           



        }

        public static BitmapImage ToBitmapImage(System.Drawing.Bitmap bitmap)
        {
            using (var memory = new MemoryStream())
            {
                bitmap.Save(memory, ImageFormat.Png);
                memory.Position = 0;

                var bitmapImage = new BitmapImage();
                bitmapImage.BeginInit();
                bitmapImage.StreamSource = memory;
                bitmapImage.CacheOption = BitmapCacheOption.OnLoad;
                bitmapImage.EndInit();

                return bitmapImage;
            }
        }



        //public static System.Drawing.Bitmap RotateImage(System.Drawing.Bitmap image, float angle)
        //{
        //    System.Drawing.Bitmap rotatedImage = new System.Drawing.Bitmap(image.Width, image.Height);
        //    using (System.Drawing.Graphics g = System.Drawing.Graphics.FromImage(rotatedImage))
        //    {
        //        g.TranslateTransform(image.Width / 2, image.Height / 2);
        //        g.RotateTransform(angle);
        //        g.TranslateTransform(-image.Width / 2, -image.Height / 2);
        //        g.DrawImage(image, new System.Drawing.Point(0, 0));

        //    }
        //    return rotatedImage;
        //}


        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>

        /// 
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
           switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    
                     if(this.mode == 1)  drawXWing(drawingContext, handPosition.X, handPosition.Y, 44.0);


                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
            prevXWingPoint = handPosition;
        }

        public double calclXWingAngle(System.Windows.Point p)
        {
            return 0;
        }
        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
       
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
      
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }

        private void readBitmap(String filePath)
        {


        }


        




        //////////////////////GAME METHODS



        private Balloon detectHit(Point LeftHandPositon, Point rightHandPosition )
        {
            for(int i = 0; i < backgroundBalloons.Count; i++)
            {
                double pX = backgroundBalloons[i].getXLocation();
                double pY = backgroundBalloons[i].getYLocation();
                if ((distance(LeftHandPositon.X, LeftHandPositon.Y, pX, pY) <= circleDiameter / 2) && backgroundBalloons[i].getVisible())
                {
                    backgroundBalloons[i].setExploded(true);
                    userScore++;
                }
                if ((distance(rightHandPosition.X, rightHandPosition.Y, pX, pY) <= circleDiameter / 2) && backgroundBalloons[i].getVisible())
                {
                    backgroundBalloons[i].setExploded(true);
                    userScore++;
                }
                    


            }
            return null;
        }
        private double distance(double x1, double y1, double x2, double y2)
        {
            return Math.Sqrt(Math.Pow(x2 - x1, 2) + Math.Pow(y2 - y1, 2));
        }


        private int checkUserSelection(double x , double y )
        {
            if (this.bothHandsClosed == true)
            {

                if (x > 50 && x < 450 && y > 10 && y < 110) return 1; // Game Option 
                if (x > 50 && x < 450 && y > 120 && y < 220)
                {
                    MainMode.Visibility = Visibility.Hidden;
                    MainMode.IsEnabled = false;
                    khaledLineImage.Points.Clear();
                    
                    khaledMode.Visibility = Visibility.Visible;
                    khaledMode.IsEnabled = true;
                    Console.WriteLine("KHALED MODE");
                    return 2;
                }

                if (x > 50 && x < 450 && y > 230 && y < 330)
                {
                    Console.WriteLine("BRIAN's MODE");
                    return 3;
                }

            }

            return 0; 
           
        }

        public void drawStartMenu(DrawingContext dc)
        {
            dc.DrawRectangle(Brushes.Yellow, new Pen(Brushes.Red, 6), new Rect(50, 10, 400, 100));
            dc.DrawRectangle(Brushes.Blue, new Pen(Brushes.Red, 6), new Rect(50, 120, 400, 100));
            dc.DrawRectangle(Brushes.Green, new Pen(Brushes.Red, 6), new Rect(50, 230, 400, 100));

        }



        //Create Balloon objects the backGround circles 
        public void createCircleGrid()
        {
            backgroundBalloons = new List<Balloon>();

            //DrawCricles Across 
            circleDiameter = this.displayWidth / numberOfCirclesAcross;

            //Draw Circles down 
            double y = 3 * circleDiameter / 2;
            while (y < (this.displayHeight - circleDiameter))
            {
                double x = 3 * circleDiameter / 2;
                while (x < (this.displayWidth - circleDiameter))
                {
                    backgroundBalloons.Add(new Balloon(new System.Windows.Point(x, y), circleDiameter, false, false, 300));

                    x += circleDiameter;
                }

                y += circleDiameter;

            }
        }

        //Draw Ballons in balloon list 
        public void drawCircleGrid(DrawingContext dr)
        {
            //Console.Clear();

            maxBalloonsVisible = 5;
            currentBalloonsVisible = 0;

            int randomBalloonLocation = 0;

            //count current balloons visible
            for (int i = 0; i < backgroundBalloons.Count; i++)
            {
                if (backgroundBalloons[i].getVisible())
                    currentBalloonsVisible++;
            }
            
            while (currentBalloonsVisible < maxBalloonsVisible)
            {
                randomBalloonLocation = rnd.Next(0, backgroundBalloons.Count);

                backgroundBalloons[randomBalloonLocation].setVisible(true);

                currentBalloonsVisible++;
            }

                for (int i = 0; i < backgroundBalloons.Count; i++)
                {
                    if (backgroundBalloons[i].getExploded() == false && backgroundBalloons[i].getVisible() == true)
                    {
                        drawCircle(Brushes.Yellow, dr, backgroundBalloons[i].getXLocation(), backgroundBalloons[i].getYLocation(), backgroundBalloons[i].getDiameter(), 0);
                    }
                    else if (backgroundBalloons[i].getExploded() == true)
                    {
                        if (backgroundBalloons[i].getExplosionRadius() < 200)
                        {
                            drawExplosion(dr, backgroundBalloons[i]);
                            backgroundBalloons[i].increaseExplosionRadius();
                            backgroundBalloons[i].decreaseExplosionOpacity();
                        }
                    }

                }
        }
        public void drawExplosion(DrawingContext dr, Balloon balloon)
        {
            double x = balloon.getXLocation();
            double y = balloon.getYLocation();
            int size = balloon.getExplosionRadius();

            Random rand = new Random();
            //RadialGradientBrush gb = new RadialGradientBrush(Colors.Red, Colors.White);
            Color c1 = Color.FromRgb((byte)rand.Next(1, 255), (byte)rand.Next(1, 255), (byte)rand.Next(1, 255));
            Color c2 = Color.FromRgb((byte)rand.Next(1, 255), (byte)rand.Next(1, 255), (byte)rand.Next(1, 255));
            RadialGradientBrush gb = new RadialGradientBrush(c1, c2);
            gb.GradientOrigin = new Point(x, y);
            gb.RadiusX = 30;
            gb.RadiusY = 30;
            gb.Opacity = balloon.getExplosionOpacity();
            gb.Center = new Point(x, y);
            dr.DrawEllipse(gb, null, new Point(x, y), 30, 30);
            for (int j = 0; j < numberOfSidesOnExplosion; j++)
            {
                dr.DrawEllipse(getRandomColorBrush(), null, new Point(x + size*explodeXAngles[j], y + size*explodeYAngles[j]), 3, 3);
            }
        }
        public SolidColorBrush getRandomColorBrush()
        {
            Random r = new Random();
            r.Next(1, 255);
            return new SolidColorBrush(Color.FromArgb((byte)245, (byte)r.Next(1, 255), (byte)r.Next(1, 255), (byte)r.Next(1, 255)));
        }
        private System.Drawing.Bitmap RotateImage(System.Drawing.Bitmap bmp, float angle)
        {
            System.Drawing.Bitmap rotatedImage = new System.Drawing.Bitmap(bmp.Width, bmp.Height);
            using (System.Drawing.Graphics g = System.Drawing.Graphics.FromImage(rotatedImage))
            {
                g.TranslateTransform(bmp.Width / 2, bmp.Height / 2); //set the rotation point as the center into the matrix
                g.RotateTransform(angle); //rotate
                g.TranslateTransform(-bmp.Width / 2, -bmp.Height / 2); //restore rotation point into the matrix
                g.DrawImage(bmp, new System.Drawing.Point(0, 0)); //draw the image on the new bitmap
            }

            return rotatedImage;
        }


        
        public void startGame(DrawingContext dc , Point leftHandPosition , Point rightHandPosition  )
        {

            if (this.rightHandLasso == true && this.leftHandLasso == true)

            {
                this.mode = 0;
                this.khaledMode.Visibility = Visibility.Hidden;
                this.khaledModeImage.Visibility = Visibility.Hidden;
                this.khaledLineImage.Points.Clear();   
                this.MainMode.Visibility = Visibility.Visible;
                
            }
            else if (this.mode == 0)//Main Menu Selection
            {
                this.mode = checkUserSelection(rightHandPosition.X, rightHandPosition.Y);
                timeCounter = 0;
                userScore = 0;
            }
            else if (this.mode == 1) // Game Mode 
            {
                detectHit(leftHandPosition, rightHandPosition);
                drawCircleGrid(dc);
                timeCounter++;
                Console.WriteLine(timeCounter / 30);
                timeKeeper(dc);
            }
            else if (this.mode == 2)// Khaled Mode 
            {
                if(this.kMode == 0)
                {
                    if (this.leftHandClosed = true && this.rightHandLasso == true )
                    {
                        takeScreenshot(dc );
                       
                        
                    }
                }
                else if (this.kMode == 1)
                {

                    //  dc.DrawImage(this.userPhotoBitmap, new Rect(0, 0 , this.Width,this.Height));

                    Uri imageUri = new Uri(userPhotoPath, UriKind.Relative);
                    BitmapImage source  = new BitmapImage(new Uri(userPhotoPath, UriKind.RelativeOrAbsolute));
                 
                    khaledMode.Visibility = Visibility.Hidden;
                    canvasUserImage.Source = source;
                    khaledModeImage.Visibility = Visibility.Visible;

                    //Drawing Menus 
                    
                    dc.DrawRectangle(Brushes.Blue, new Pen(), new Rect(0, 0, 100, 200));
                }
            }

            else if (this.mode == 3)//Brian's Mode 
            {

            }
        }

        ///Patrick's Method

        ///
        private void timeKeeper(DrawingContext drawingContext){
            String time = Convert.ToString(timeCounter / 30);
            FormattedText timeText = new FormattedText(
                    time,
                    CultureInfo.GetCultureInfo("en-us"),
                    FlowDirection.LeftToRight,
                    new Typeface("Verdana"),
                    32,
                    Brushes.White);

            drawingContext.DrawText(timeText, new Point((displayWidth / 2) - (timeText.WidthIncludingTrailingWhitespace / 2), timeText.Height));

            String score = Convert.ToString(userScore);

            FormattedText scoreText = new FormattedText(
                    score,
                    CultureInfo.GetCultureInfo("en-us"),
                    FlowDirection.LeftToRight,
                    new Typeface("Helvetica"),
                    40,
                    Brushes.Green);

            drawingContext.DrawText(scoreText, new Point((displayWidth / 2) - (scoreText.WidthIncludingTrailingWhitespace / 2), displayHeight - scoreText.Height));
        }



       ///Khaled's Methods 
        private void ColorReader_FrameArrived(object sender, ColorFrameArrivedEventArgs e)
        {
            using (var frame = e.FrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    frame.CopyConvertedFrameDataToArray(this.k_pixels, ColorImageFormat.Bgra);

                    this.k_bitmap.Lock();
                    Marshal.Copy(this.k_pixels, 0, this.k_bitmap.BackBuffer, this.k_pixels.Length);
                    this.k_bitmap.AddDirtyRect(new Int32Rect(0, 0, this.k_width, this.k_height));
                    this.k_bitmap.Unlock();
                }
            }
        }

        private void BodyReader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            using (var frame = e.FrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    frame.GetAndRefreshBodyData(this.k_bodies);

                    Body body = this.k_bodies.Where(b => b.IsTracked).FirstOrDefault();

                    if (body != null)
                    {
                        Joint handRight = body.Joints[JointType.HandRight];

                        if (handRight.TrackingState != TrackingState.NotTracked)
                        {
                            CameraSpacePoint handRightPosition = handRight.Position;
                            ColorSpacePoint handRightPoint = this.kinectSensor.CoordinateMapper.MapCameraPointToColorSpace(handRightPosition);

                            float x = handRightPoint.X;
                            float y = handRightPoint.Y;

                            if (!float.IsInfinity(x) && !float.IsInfinity(y))
                            {
                                if (this.mode ==2 && this.kMode == 1 )
                                {
                                  //  canvas.Visibility = Visibility.Visible;
                                    khaledLineImage.Points.Add(new System.Windows.Point { X = x, Y = y });
                                  
                                }
                                
                            }
                        }
                    }
                }
            }
        }


        private void takeScreenshot(DrawingContext dc )
        {
            if (this.k_bitmap != null)
            {
                // create a png bitmap encoder which knows how to save a .png file
                BitmapEncoder encoder = new PngBitmapEncoder();

                // create frame from the writable bitmap and add to encoder
                encoder.Frames.Add(BitmapFrame.Create(this.k_bitmap));
                string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);
                string path = Path.Combine(myPhotos, "userPhoto.png");

                // write the new file to disk
                try
                {
                    // FileStream is IDisposable
                    using (FileStream fs = new FileStream(path, FileMode.Create))
                    {
                        encoder.Save(fs);
                        //Set the background photo to be the photo that was just saved 


                        //Image finalImage = new Image();
                        //  finalImage.Width = this.Width; 
                        //BitmapImage logo = new BitmapImage();
                        //logo.BeginInit();
                        //logo.UriSource = new Uri(path , UriKind.RelativeOrAbsolute);
                        //logo.EndInit();
                        //this.kMode = 1; 
                        //this.khaledMode.Source = null;
                        userPhotoPath = path;
                        this.kMode = 1;
                        
                        

                     //   BitmapImage imageBitmap = new BitmapImage(imageUri);
                       




                    }

                }
                catch (IOException)
                {
                  //  this.StatusText = string.Format(CultureInfo.CurrentCulture, Properties.Resources.FailedScreenshotStatusTextFormat, path);
                }
            }
        }




    }
}
