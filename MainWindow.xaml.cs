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
        private Point prevXWingPointLeft;
        private Point prevXWingPointRight;
        private double xWingAngleLeft;
        private double xWingAngleRight;

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
        
     





        // Other 
        private int timeCounter = 0;
        private int timeCounterInSeconds = 0;
        private int maxBalloonsVisible;
        private int currentBalloonsVisible;
        private Random rnd = new Random();
        private int userScore = 0;
        private int userLives = 5;

        private Boolean countdown = true;
        private Boolean startTimer = true;


        private MyMenuButton playButton;
        private MyMenuButton khaledButton;
        private MyMenuButton brianButton;

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
        private Image pointer;
        private BitmapImage pointerSource; 


        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {

            Console.WriteLine(Directory.GetCurrentDirectory());

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
            //cameraCounterTextBlock = new TextBlock();
            //cameracanvas.Children.Add(cameraCounterTextBlock);

            khaledMode.Source = this.k_bitmap;
            khaledMode.Visibility = Visibility.Hidden;
            //Set all Editing menu option to invisible 
            EraserButton.Visibility = Visibility.Hidden;
            SaveButton.Visibility = Visibility.Hidden;
            ColorButton.Visibility = Visibility.Hidden;
            StrokeButton.Visibility = Visibility.Hidden;
            backMenu.Visibility = Visibility.Hidden;
            RestartButton.Visibility = Visibility.Hidden;
            LeftHandPointer.Visibility = Visibility.Hidden;
            ExitButton.Visibility = Visibility.Hidden;
            Stroke1.Visibility = Visibility.Hidden;
            Stroke2.Visibility = Visibility.Hidden;
            Stroke3.Visibility = Visibility.Hidden;
            Stroke4.Visibility = Visibility.Hidden;
            GreenColor.Visibility = Visibility.Hidden;
            RedColor.Visibility = Visibility.Hidden;
            BlackColor.Visibility = Visibility.Hidden;
            BlueColor.Visibility = Visibility.Hidden;


            //BackgroundPic.IsEnabled = false;
            //BackgroundPic.Visibility = Visibility.Hidden;


            playButton = new MyMenuButton("Survival", Brushes.Yellow, Brushes.Gray, new Pen(Brushes.DarkGray, 5), (int) (this.displayWidth * 0.2), (int) (this.displayHeight * .2), (int) (this.displayWidth * .6), (int) (this.displayHeight * .2));
            khaledButton = new MyMenuButton("Khaled's Mode", Brushes.Red, Brushes.White, new Pen(Brushes.Blue, 5), (int) (this.displayWidth * 0.2), (int) (this.displayHeight * .4 + 10), (int) (this.displayWidth * .6), (int) (this.displayHeight * .2));
            brianButton = new MyMenuButton("Brian's Mode", Brushes.Black, Brushes.DarkGray, new Pen(Brushes.Red, 5), (int) (this.displayWidth * 0.2), (int) (this.displayHeight * .6 + 20), (int) (this.displayWidth * .6), (int) (this.displayHeight * .2));
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
            ltrRot.CenterX = 25;
            ltrRot.CenterY = 25;
            rtrRot.CenterX = 25;
            rtrRot.CenterY = 25;
     
            leftWing.RenderTransform = ltrGrp;
            RightWing.RenderTransform = rtrGrp;

            //Khaled Mode 
            pointerHand.Visibility = Visibility.Hidden; // The Brush Pointer 

            



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
                            
                            if(this.mode != 1) this.DrawBody(joints, jointPoints, dc, drawPen);
                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc, true);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc, false);

                            this.startGame(dc, jointPoints[JointType.HandLeft], jointPoints[JointType.HandRight]);
                            //this.drawCircles(body.HandRightState, jointPoints[JointType.HandRight], dc);

                            if (body.HandLeftState == HandState.Closed && body.HandRightState == HandState.Closed) {
                                this.bothHandsClosed = true;
                                this.leftHandClosed = true;
                                this.rightHandClosed = true;
                            }
                            else if ( body.HandLeftState == HandState.Closed)
                            {
                                leftHandClosed = true;
                            }
                            else if (body.HandRightState == HandState.Closed)
                            {
                                rightHandClosed = true;
                            }
                            else if(body.HandLeftState == HandState.Lasso )
                            {
                                this.leftHandLasso = true;
                            }
                            else if(body.HandRightState == HandState.Lasso)
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

        private void drawXWing(DrawingContext dc , double x , double y, double angle, bool isLeft)
        {
            Console.WriteLine("Angle: " + angle);
            if (isLeft)
            {
                leftWing.Visibility = Visibility.Visible;
                ltrRot.Angle = angle;
                Canvas.SetLeft(leftWing, x - 225);
                Canvas.SetTop(leftWing, y - 100);
            }
            else
            {
                RightWing.Visibility = Visibility.Visible;
                rtrRot.Angle = angle;
                Canvas.SetLeft(RightWing, x - 225);
                Canvas.SetTop(RightWing, y - 100);
            }
   
        }

        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext, Boolean isLeft)
        {
            switch (handState)
            {
                case HandState.Closed:
                    //drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);

                    if (this.mode == 0) //start menu
                    {
                        drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                        drawCircle(Brushes.Blue, drawingContext, handPosition.X, handPosition.Y, 25, 1);
                    }
                    else if (this.mode == 1) //primary game mode
                    {
                        if (isLeft)
                        {
                            xWingAngleLeft = calcXWingAngle(handPosition, prevXWingPointLeft);
                            drawXWing(drawingContext, handPosition.X, handPosition.Y, xWingAngleLeft, true);
                        }
                        else
                        {
                            xWingAngleRight = calcXWingAngle(handPosition, prevXWingPointRight);
                            drawXWing(drawingContext, handPosition.X, handPosition.Y, xWingAngleRight, false);
                        }
                        leftWing.Visibility = Visibility.Visible;
                        RightWing.Visibility = Visibility.Visible;
                    }
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;
                    
                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }

            if (isLeft) prevXWingPointLeft = handPosition;
            else prevXWingPointRight = handPosition;
        }
        public double calcSlope(Point p1, Point p2)
        {
            if (p1 == null || p2 == null) return 0;
            if (p1.X - p2.X == 0) return 100;
            double slope = (p1.Y - p2.Y) / (p1.X - p2.X);
            return slope;
        }
    
        public double calcXWingAngle(Point current, Point prev)
        {
            if (current == null || prev == null) return 0;

            double slope = (current.Y - prev.Y) / (current.X - prev.X);
            double x1 = prev.X;
            double y1 = prev.Y;
            double x2 = current.X;
            double y2 = current.Y;

            if (x2 < x1 && y2 < y1) //angles 180-270
            {
                if (slope <= 0.5774) return 180;
                else if (slope <= 1.732) return 210;
                else return 240;
            }
            else if (x2 < x1 && y2 > y1) //angles 90-180
            {
                if (slope >= -0.5774) return 150;
                else if (slope >= -1.732) return 120;
                else return 90;
            }
            else if (x2 > x1 && y2 > y1) //angles 0-90
            {
                if (slope <= 0.5774) return 0;
                else if (slope <= 1.732) return 30;
                else return 60;
            }
            else if (x2 > x1 && y2 < y1) //angles 270-360
            {
                if (slope >= -0.5774) return 330;
                else if (slope >= -1.732) return 300;
                else return 270;
            }
            else if (y1 == y2)
            {
                if (x1 < x2) return 0;
                else return 180;
            }
            else if (x1 == x2)
            {
                if (y1 < y2) return 90;
                else return 270;
            }
            else return 0;
        }

        private double RadianToDegree(double angle)
        {
            return angle * (180.0 / Math.PI);
        }
        private double degreeToRadian(double angle)
        {
            return angle * (Math.PI / 180.0);
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

        
        // Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
       
      
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


        private void detectHit(Point handPosition, double angle)
        {
            for(int i = 0; i < backgroundBalloons.Count; i++)
            {
                double pX = backgroundBalloons[i].getXLocation();
                double pY = backgroundBalloons[i].getYLocation();

                if ((distance(handPosition.X, handPosition.Y, pX, pY) <= circleDiameter / 2) && backgroundBalloons[i].getVisible())
                {
                    double size = backgroundBalloons[i].getExplosionRadius();
                    backgroundBalloons[i].setExploded(true);
                    double adjustAngle = 180 + angle;
                    Console.WriteLine("aj "  + adjustAngle);
                    //Debugger.Break();
                    backgroundBalloons[i].setExplosionAngle(adjustAngle);
                    backgroundBalloons[i].setDeformLocation(new Point(pX + size*Math.Cos(degreeToRadian(adjustAngle)), pY + size*Math.Cos(degreeToRadian(adjustAngle))));
                    userScore++;
                }

            }
        }

        private double distance(double x1, double y1, double x2, double y2)
        {
            return Math.Sqrt(Math.Pow(x2 - x1, 2) + Math.Pow(y2 - y1, 2));
        }


        private int checkUserSelection(double x , double y )
        {
            if (this.bothHandsClosed == true)
            {

                if (x > playButton.getX() && x < playButton.getX() + playButton.getWidth() && y < playButton.getY() + playButton.getHeight() && y > playButton.getY())
                {
                    //Console.WriteLine("Button (X): " + playButton.getX() + " |(Y): " + playButton.getY() + " |(X+W): " + (playButton.getY() + playButton.getWidth()) + " |(Y-H): " + (playButton.getY() - playButton.getHeight()));
                    //Console.WriteLine("Hand (X): " + x + " | (Y): " + y);
                    return 1; // Game Option 
                }
                else if (x > khaledButton.getX() && x < khaledButton.getX() + khaledButton.getWidth() && y < khaledButton.getY() + khaledButton.getHeight() && y > playButton.getY())
                {
                   return 2; // Khaled Mode 
                }
                else if (x > brianButton.getX() && x < brianButton.getX() + brianButton.getWidth() && y < brianButton.getY() + brianButton.getHeight() && y > brianButton.getY())
                {
                    //Console.WriteLine("BRIAN's MODE");
                    return 3;
                }
            }
            return 0; 
        }

        public void drawStartMenu(DrawingContext dc)
        {
            dc.DrawRectangle(playButton.getColor(), playButton.getBorderColor(), playButton.getRect());
            dc.DrawRectangle(khaledButton.getColor(), khaledButton.getBorderColor(), khaledButton.getRect());
            dc.DrawRectangle(brianButton.getColor(), brianButton.getBorderColor(), brianButton.getRect());

            var starJedi = new Typeface(new FontFamily(new Uri("pack://application:,,,/"), "/Resources/#Starjedi"), FontStyles.Normal, FontWeights.Regular, FontStretches.Normal);

            FormattedText titleText = new FormattedText(
                "Deathstar Destroyer",
                CultureInfo.GetCultureInfo("en-us"),
                FlowDirection.LeftToRight,
                new Typeface("Star Jedi Regular"),
                60,
                Brushes.Yellow);

            FormattedText playText = new FormattedText(
                    playButton.getText(),
                    CultureInfo.GetCultureInfo("en-us"),
                    FlowDirection.LeftToRight,
                    new Typeface("Star Jedi Regular"),
                    32,
                    playButton.getFontColor());

            FormattedText khaledText = new FormattedText(
                    khaledButton.getText(),
                    CultureInfo.GetCultureInfo("en-us"),
                    FlowDirection.LeftToRight,
                    new Typeface("Star Jedi Regular"),
                    32,
                    khaledButton.getFontColor());

            FormattedText brianText = new FormattedText(
                    brianButton.getText(),
                    CultureInfo.GetCultureInfo("en-us"),
                    FlowDirection.LeftToRight,
                    new Typeface("Star Jedi Regular"),
                    32,
                    brianButton.getFontColor());

            dc.DrawText(titleText, new Point((displayWidth - titleText.WidthIncludingTrailingWhitespace) / 2, 0));

            dc.DrawText(playText, new Point(playButton.getX() + (playButton.getWidth() - playText.WidthIncludingTrailingWhitespace) / 2, playButton.getY() + (playButton.getHeight() - playText.Height) / 2));
            dc.DrawText(khaledText, new Point(khaledButton.getX() + (khaledButton.getWidth() - khaledText.WidthIncludingTrailingWhitespace) / 2, khaledButton.getY() + (khaledButton.getHeight() - khaledText.Height) / 2));
            dc.DrawText(brianText, new Point(brianButton.getX() + (brianButton.getWidth() - brianText.WidthIncludingTrailingWhitespace) / 2, brianButton.getY() + (brianButton.getHeight() - brianText.Height) / 2));
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

            maxBalloonsVisible = 1 + timeCounterInSeconds/15;
            currentBalloonsVisible = 0;

            int randomBalloonLocation = 0;

            //count current balloons visible
            for (int i = 0; i < backgroundBalloons.Count; i++)
            {
                if (backgroundBalloons[i].getVisible())
                {
                    if (backgroundBalloons[i].getTicks() <= 0)
                    {
                        backgroundBalloons[i].setVisible(false);
                        userLives--;
                    }
                    else
                    {
                        backgroundBalloons[i].decrementTicks();
                        currentBalloonsVisible++;
                    }
               }
            }

            while (currentBalloonsVisible < maxBalloonsVisible)
            {
                randomBalloonLocation = rnd.Next(0, backgroundBalloons.Count);

                backgroundBalloons[randomBalloonLocation].setVisible(true);
                backgroundBalloons[randomBalloonLocation].setTicks(300 / (1 + timeCounterInSeconds/ 30));
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
            double angle = balloon.getExplosionAngle(); //in degrees
            Console.WriteLine(angle);
            //Debugger.Break();

            //Point deformPoint = balloon.getDeformLocation();
            Point deformPoint = new Point(x + size * Math.Cos(degreeToRadian(angle)), y + size * Math.Cos(degreeToRadian(angle)));

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

            Point p1 = new Point(x + size * Math.Cos(degreeToRadian( angle - 30)), y + size * Math.Sin(degreeToRadian(angle - 30)));
            Point p2 = new Point(x + size * Math.Cos(degreeToRadian(angle + 30)), y + size * Math.Sin(degreeToRadian(angle + 30)));
            double sl = calcSlope(p1, p2);
            double intercept = p1.Y - sl * p1.X;

          
            for (int angleCount = 0; angleCount < 360; angleCount += 15)
            {
                if (angle % 15 != 0) Debugger.Break();

                double localSize = size;
                Point currPoint = currPoint = new Point(x + size * explodeXAngles[angleCount / 15]
                        , y + size * explodeYAngles[angleCount / 15]);
                if (angleCount == angle)
                {
                    //currPoint = new Point(deformPoint.X + localSize * Math.Cos(degreeToRadian(180 + angle))
                    //, deformPoint.Y + localSize * Math.Sin(degreeToRadian(180 + angle)));
                    //currPoint = new Point(x, y);
                    Point reflPoint = invertPoint(sl, intercept, currPoint);

                    dr.DrawEllipse(getRandomColorBrush(), null, reflPoint, 3, 3);
                }
                else if(angleCount == angle - 15)
                {
                    //currPoint = new Point(deformPoint.X + localSize * Math.Cos(degreeToRadian(180 - 15 + angle))
                    //, deformPoint.Y + localSize * Math.Sin(degreeToRadian(180 - 15 + angle)));
                    //currPoint = new Point(x + localSize/2 * Math.Cos(degreeToRadian(angleCount))
                    //, y + localSize/2 * Math.Sin(degreeToRadian(angleCount)));
                    Point reflPoint = invertPoint(sl, intercept, currPoint);
                    dr.DrawEllipse(getRandomColorBrush(), null, reflPoint, 3, 3);
                }
                else if(angleCount == angle - 30)
                {
                    //currPoint = new Point(deformPoint.X + localSize * Math.Cos(degreeToRadian(180 - 30 + angle))
                    //    , deformPoint.Y + localSize * Math.Sin(degreeToRadian(180 - 30 + angle)));
                    Point reflPoint = invertPoint(sl, intercept, currPoint);
                    dr.DrawEllipse(getRandomColorBrush(), null, reflPoint, 3, 3);
                }
                else if (angleCount == angle + 15)
                {
                    //currPoint = new Point(deformPoint.X + localSize * Math.Cos(degreeToRadian(180 + angle))
                    //  , deformPoint.Y + localSize * Math.Sin(degreeToRadian(180 + angle)));
                    Point reflPoint = invertPoint(sl, intercept, currPoint);

                    dr.DrawEllipse(getRandomColorBrush(), null, reflPoint, 3, 3);
                }
                else if(angleCount == angle + 30)
                {
                    //currPoint = new Point(deformPoint.X + localSize * Math.Cos(degreeToRadian(180 + 30 + angle))
                    //    , deformPoint.Y + localSize * Math.Sin(degreeToRadian(180 + 30 + angle)));
                    Point reflPoint = invertPoint(sl, intercept, currPoint);
                    dr.DrawEllipse(getRandomColorBrush(), null, reflPoint, 3, 3);
                }
                else
                {
                    //currPoint = new Point(x + size * explodeXAngles[angleCount / 15]
                    //    , y + size * explodeYAngles[angleCount/15]);
                    dr.DrawEllipse(getRandomColorBrush(), null, currPoint, 3, 3);
                }
                
                //dr.DrawEllipse(getRandomColorBrush(), null, currPoint, 3, 3);
            }
        }
        public Point invertPoint(double lineSlope, double lineInter, Point p)
        {
            double x = p.X;
            double y = p.Y;
            double c = lineInter;
            double a = lineSlope;
            double d = (x + (y - c) * a) / (1 + Math.Pow(a, 2));
            double x2 = 2*d - x;
            double y2 = 2*d*a - y + 2*c;
            return new Point(x2, y2);
        }
        public Point rotatePoint(double cx, double cy, double angle, Point p) //takes radians
        {
            double s = Math.Sin(angle);
            double c = Math.Cos(angle);

            // translate point back to origin:
            p.X -= cx;
            p.Y -= cy;

            // rotate point
            double xnew = p.X * c - p.Y * s;
            double ynew = p.X * s + p.Y * c;

            // translate point back:
            p.X = xnew + cx;
            p.Y = ynew + cy;
            return p;
        }
        public Point findTranslation(Point prev, Point next)
        {
            return new Point(prev.X - next.X, prev.Y - next.Y);
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
            if (this.mode != 1)
            {
                leftWing.Visibility = Visibility.Hidden;
                RightWing.Visibility = Visibility.Hidden;
            }
            if (this.rightHandLasso == true && this.leftHandLasso == true  && this.mode != 2 )

            {
                this.mode = 0;
                this.khaledMode.Visibility = Visibility.Hidden;
                this.khaledModeImage.Visibility = Visibility.Hidden;
                this.canvasImage.Children.Clear();
                this.khaledLineImage.Points.Clear();   
                this.MainMode.Visibility = Visibility.Visible;

                this.userLives = 5;
               
            }
            else if (this.mode == 0)//Main Menu Selection
            {
                this.kMode = 0;
                this.mode = checkUserSelection(rightHandPosition.X, rightHandPosition.Y);
                timeCounter = 0;
                timeCounterInSeconds = 0;
                userScore = 0;
                userLives = 5;

            }
            else if (this.mode == 1) // Game Mode 
            {
                if(this.leftHandClosed)
                    detectHit(leftHandPosition, xWingAngleLeft);
                if (this.rightHandClosed)
                    detectHit(rightHandPosition, xWingAngleRight);
                drawCircleGrid(dc);
                timeKeeper(dc);
                drawLives(dc);

                if (userLives <= 0)
                    this.mode = 0;
                
            }
            else if (this.mode == 2)// Khaled Mode 
            {

                //General Stuff that must be done 
                    //Kmode 0 : Take a Picture mode 
                    //Kmode 1 : Edit Picture Mode 
              
                
                if (this.kMode == 0)
                {
                  
                    this.khaledMode.Visibility = Visibility.Visible;
                    this.khaledModeImage.Visibility = Visibility.Hidden;
                    setMenuOptionsVisible(false);
                    // canvasImage.Children.Clear();
                    canvasUserImage.Source = null;
                    khaledMode.Visibility = Visibility.Visible;

                    if (this.leftHandClosed = true && this.rightHandLasso == true )
                    {
                     
                            takeScreenshot();
                           // this.kMode = 1; // Go to edit picture mode 
                        
                    }
                }
                if (this.kMode == 1)
                {
                    
                    Uri imageUri = new Uri(userPhotoPath, UriKind.RelativeOrAbsolute);
                    BitmapImage source  = new BitmapImage(new Uri(userPhotoPath, UriKind.RelativeOrAbsolute));
                    khaledMode.Visibility = Visibility.Hidden;
                    canvasUserImage.Source = source;
                    khaledModeImage.Visibility = Visibility.Visible;
                    
                }
            }




            else if (this.mode == 3)//Brian's Mode 
            {

            }
        }

        ///Patrick's Method

        ///
        private void timeKeeper(DrawingContext drawingContext){
            String time = Convert.ToString(timeCounterInSeconds);
            FormattedText timeText = new FormattedText(
                    time,
                    CultureInfo.GetCultureInfo("en-us"),
                    FlowDirection.LeftToRight,
                    new Typeface("Verdana"),
                    32,
                    Brushes.White);

            drawingContext.DrawText(timeText, new Point((displayWidth / 2) - (timeText.WidthIncludingTrailingWhitespace / 2), 10));

            String score = Convert.ToString(userScore);

            FormattedText scoreText = new FormattedText(
                    score,
                    CultureInfo.GetCultureInfo("en-us"),
                    FlowDirection.LeftToRight,
                    new Typeface("Helvetica"),
                    40,
                    Brushes.Green);

            drawingContext.DrawText(scoreText, new Point((displayWidth) - (scoreText.WidthIncludingTrailingWhitespace), displayHeight - scoreText.Height));

            timeCounter++;
            timeCounterInSeconds = timeCounter / 30;
        }


        private void drawSkywalker(DrawingContext drawingContext, double x, double y)
        {
            
            var outPutDirectory = Path.GetDirectoryName(Assembly.GetExecutingAssembly().CodeBase);
            var rebelPath = Path.Combine(outPutDirectory, "Images\\rebelLogo.png");
            BitmapImage rebelLogo = new BitmapImage(new Uri(rebelPath, UriKind.RelativeOrAbsolute));
            var skywalkerPath = Path.Combine(outPutDirectory, "Images\\skywalker.png");
            BitmapImage skywalker = new BitmapImage(new Uri(rebelPath, UriKind.RelativeOrAbsolute));

            drawingContext.DrawImage(rebelLogo, new Rect(x, y, 55, 55));
            drawingContext.DrawImage(skywalker, new Rect(x+5, y+10, 45, 45));

        }

        private void drawLives(DrawingContext dc)
        {
            FormattedText livesText = new FormattedText(
                    "LIVES:",
                    CultureInfo.GetCultureInfo("en-us"),
                    FlowDirection.LeftToRight,
                    new Typeface("Helvetica"),
                    20,
                    Brushes.Yellow);

            dc.DrawText(livesText, new Point(0, displayHeight - 90));
            for (int i = 0; i < userLives; i++)
            {
                drawSkywalker(dc, 0 + i*60, displayHeight - 60);
            }
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
                        Joint handLeft = body.Joints[JointType.HandLeft];

                        if (handRight.TrackingState != TrackingState.NotTracked)
                        {
                            CameraSpacePoint handRightPosition = handRight.Position;
                            ColorSpacePoint handRightPoint = this.kinectSensor.CoordinateMapper.MapCameraPointToColorSpace(handRightPosition);

                            CameraSpacePoint handLeftPosition = handLeft.Position;
                            ColorSpacePoint handLeftPoint = this.kinectSensor.CoordinateMapper.MapCameraPointToColorSpace(handLeftPosition);

                            float x = handRightPoint.X;
                            float y = handRightPoint.Y;

                            float lx = handLeftPoint.X;
                            float ly = handLeftPoint.Y;

                            if (!float.IsInfinity(x) && !float.IsInfinity(y))
                            {
                                if ( this.kMode == 1  )
                                {
                                    //Draw !!!
                                   
                                    pointerHand.Visibility = Visibility.Visible;
                                    if (rightHandClosed)
                                    {
                                        khaledLineImage.Points.Add(new System.Windows.Point { X = x, Y = y });
                                    }
                                    Canvas.SetLeft(pointerHand, x );
                                    Canvas.SetTop(pointerHand, y );

                                    if (leftHandClosed)
                                    {
                                        LeftHandPointer.Visibility = Visibility.Visible;
                                        setMenuOptionsVisible(true);
                                        Canvas.SetLeft(LeftHandPointer, lx);
                                        Canvas.SetTop(LeftHandPointer, ly);
                                        checkForPaintModeSelection(lx, ly);

                                        if (Stroke1.Visibility == Visibility.Visible)
                                        {
                                            checkForStrokeOptions(lx, ly);
                                        }
                                        if(GreenColor.Visibility == Visibility.Visible)
                                        {
                                            checkForColorOptions(lx, ly);
                                        }
                                    }
                                    if (leftHandClosed == false)
                                    {
                                        setMenuOptionsVisible(false);
                                        LeftHandPointer.Visibility = Visibility.Hidden;

                                    }

                                }
                                

                            }
                        }
                    }
                }
            }
        }

        private void setMenuOptionsVisible(bool Show)
        {
            if (Show)
            {
                EraserButton.Visibility = Visibility.Visible;
                SaveButton.Visibility = Visibility.Visible;
                ColorButton.Visibility = Visibility.Visible;
                StrokeButton.Visibility = Visibility.Visible;
                backMenu.Visibility = Visibility.Visible;
                RestartButton.Visibility = Visibility.Visible;
                ExitButton.Visibility = Visibility.Visible;
                GreenColor.Visibility = Visibility.Visible;
                RedColor.Visibility = Visibility.Visible;
                BlackColor.Visibility = Visibility.Visible;
                BlueColor.Visibility = Visibility.Visible;
                Stroke1.Visibility = Visibility.Visible;
                Stroke2.Visibility = Visibility.Visible;
                Stroke3.Visibility = Visibility.Visible;
                Stroke4.Visibility = Visibility.Visible;

            }
            else
            {
                EraserButton.Visibility = Visibility.Hidden;
                SaveButton.Visibility = Visibility.Hidden;
                ColorButton.Visibility = Visibility.Hidden;
                StrokeButton.Visibility = Visibility.Hidden;
                backMenu.Visibility = Visibility.Hidden;
                RestartButton.Visibility = Visibility.Hidden;
                ExitButton.Visibility = Visibility.Hidden;
                GreenColor.Visibility = Visibility.Hidden;
                RedColor.Visibility = Visibility.Hidden;
                BlackColor.Visibility = Visibility.Hidden;
                BlueColor.Visibility = Visibility.Hidden;
                Stroke1.Visibility = Visibility.Hidden;
                Stroke2.Visibility = Visibility.Hidden;
                Stroke3.Visibility = Visibility.Hidden;
                Stroke4.Visibility = Visibility.Hidden;
            }
        }


        private void takeScreenshot()
        {
            if (this.k_bitmap != null)
            {
                // create a png bitmap encoder which knows how to save a .png file
                BitmapEncoder encoder = new PngBitmapEncoder();

                // create frame from the writable bitmap and add to encoder
                encoder.Frames.Add(BitmapFrame.Create(this.k_bitmap));
                DateTime dateTime = DateTime.UtcNow.Date;
                string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);
                string path = Path.Combine(myPhotos, "userPhoto.png"+dateTime.ToString());
               

                myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);
                 path = Path.Combine(myPhotos, "userPhoto.png");
                // write the new file to disk
                try
                    {

                    //Overwriting by deleting an existant older version  
                   
                    // FileStream is IDisposable
                    using (FileStream fs = new FileStream(path, FileMode.Create))
                        {

                            encoder.Save(fs);
                            userPhotoPath = path;
                            
                        kMode = 1;
                        }

                    }
                    catch (IOException)
                    {
                        //  this.StatusText = string.Format(CultureInfo.CurrentCulture, Properties.Resources.FailedScreenshotStatusTextFormat, path);
                    }
            
            }
        }

        private void checkForPaintModeSelection(float x, float y) // Based of Left Hand position 
        {
            Console.WriteLine("X :" +x + "Y :" + x);
            Console.WriteLine("Left: " + Canvas.GetLeft(ColorButton) + "|| TOP: " + Canvas.GetTop(ColorButton));
            if ( x > Canvas.GetLeft(ColorButton) &&  x < Canvas.GetLeft(ColorButton) + 96 && y > Canvas.GetTop(ColorButton) && y < Canvas.GetTop(ColorButton) + 96)
            {
                //Color Button 
                GreenColor.Visibility = Visibility.Visible;
                RedColor.Visibility = Visibility.Visible;
                BlackColor.Visibility = Visibility.Visible;
                RedColor.Visibility = Visibility.Visible;
            }

            if (x > Canvas.GetLeft(StrokeButton) && x < Canvas.GetLeft(StrokeButton) + 96 && y > Canvas.GetTop(StrokeButton) && y < Canvas.GetTop(StrokeButton) + 96)
            {

                Console.WriteLine("Stroke OPTION");
                Stroke1.Visibility = Visibility.Visible;
                Stroke2.Visibility = Visibility.Visible;
                Stroke3.Visibility = Visibility.Visible;
                Stroke4.Visibility = Visibility.Visible;
            }

            if (x > Canvas.GetLeft(SaveButton) && x < Canvas.GetLeft(SaveButton) + 96 && y > Canvas.GetTop(SaveButton) && y < Canvas.GetTop(SaveButton) + 96)
            {
                Console.WriteLine("Save OPTION");
                saveImage();
                takeScreenshot();
            }

            if (x > Canvas.GetLeft(EraserButton) && x < Canvas.GetLeft(EraserButton) + 96 && y > Canvas.GetTop(EraserButton) && y < Canvas.GetTop(EraserButton) + 96)
            {

                Console.WriteLine("Eraser  OPTION");
                khaledLineImage.Points.Clear();
            }


            if (x > Canvas.GetLeft(RestartButton) && x < Canvas.GetLeft(RestartButton) + 96 && y > Canvas.GetTop(RestartButton) && y < Canvas.GetTop(RestartButton) + 96)
            {
                khaledLineImage.Points.Clear();
                //userPhotoPath = "";
                this.kMode = 0;
                Console.WriteLine("Restart  OPTION");
            }



            if (x > Canvas.GetLeft(ExitButton) && x < Canvas.GetLeft(ExitButton) + 96 && y > Canvas.GetTop(ExitButton) && y < Canvas.GetTop(ExitButton) + 96)
            {
                Environment.Exit(0);
                //  Console.WriteLine("Exit   OPTION");
            }

        }

        private void checkForStrokeOptions(float x , float y )
        {
           if(x > Canvas.GetLeft(Stroke1) && x < Canvas.GetLeft(Stroke1) + 100 && y > Canvas.GetTop(Stroke1) && y < Canvas.GetTop(Stroke1) + 100)
            {
                khaledLineImage.StrokeThickness = 7;
                //Stroke1.Visibility = Visibility.Hidden;
                //Stroke2.Visibility = Visibility.Hidden;
                //Stroke3.Visibility = Visibility.Hidden;
                //Stroke4.Visibility = Visibility.Hidden;

            }

            if (x > Canvas.GetLeft(Stroke2) && x < Canvas.GetLeft(Stroke2) + 100 && y > Canvas.GetTop(Stroke2) && y < Canvas.GetTop(Stroke2) + 100)
            {
                khaledLineImage.StrokeThickness = 20;
                //Stroke1.Visibility = Visibility.Hidden;
                //Stroke2.Visibility = Visibility.Hidden;
                //Stroke3.Visibility = Visibility.Hidden;
                //Stroke4.Visibility = Visibility.Hidden;

            }
            if (x > Canvas.GetLeft(Stroke3) && x < Canvas.GetLeft(Stroke3) + 100 && y > Canvas.GetTop(Stroke3) && y < Canvas.GetTop(Stroke3) + 100)
            {
                khaledLineImage.StrokeThickness = 30;
                //Stroke1.Visibility = Visibility.Hidden;
                //Stroke2.Visibility = Visibility.Hidden;
                //Stroke3.Visibility = Visibility.Hidden;
                //Stroke4.Visibility = Visibility.Hidden;

            }

            if (x > Canvas.GetLeft(Stroke4) && x < Canvas.GetLeft(Stroke4) + 100 && y > Canvas.GetTop(Stroke4) && y < Canvas.GetTop(Stroke4) + 100)
            {
                khaledLineImage.StrokeThickness = 35;
                //Stroke1.Visibility = Visibility.Hidden;
                //Stroke2.Visibility = Visibility.Hidden;
                //Stroke3.Visibility = Visibility.Hidden;
                //Stroke4.Visibility = Visibility.Hidden;

            }
        }

        private void checkForColorOptions(float x, float y)
        {
            if (x > Canvas.GetLeft(RedColor) && x < Canvas.GetLeft(RedColor) + 100 && y > Canvas.GetTop(RedColor) && y < Canvas.GetTop(RedColor) + 100)
            {
                khaledLineImage.Stroke = Brushes.Red;
                //GreenColor.Visibility = Visibility.Hidden;
                //RedColor.Visibility = Visibility.Hidden;
                //BlackColor.Visibility = Visibility.Hidden;
                //BlueColor.Visibility = Visibility.Hidden;



            }

            if (x > Canvas.GetLeft(BlackColor) && x < Canvas.GetLeft(BlackColor) + 100 && y > Canvas.GetTop(BlackColor) && y < Canvas.GetTop(BlackColor) + 100)
            {
                khaledLineImage.Stroke = Brushes.Black;
                //GreenColor.Visibility = Visibility.Hidden;
                //RedColor.Visibility = Visibility.Hidden;
                //BlackColor.Visibility = Visibility.Hidden;
                //RedColor.Visibility = Visibility.Hidden;
                //BlueColor.Visibility = Visibility.Hidden;


            }
            if (x > Canvas.GetLeft(GreenColor) && x < Canvas.GetLeft(GreenColor) + 100 && y > Canvas.GetTop(GreenColor) && y < Canvas.GetTop(GreenColor) + 100)
            {
                khaledLineImage.Stroke = Brushes.Green;
                //GreenColor.Visibility = Visibility.Hidden;
                //RedColor.Visibility = Visibility.Hidden;
                //BlackColor.Visibility = Visibility.Hidden;
                //BlueColor.Visibility = Visibility.Hidden;


            }

            if (x > Canvas.GetLeft(BlueColor) && x < Canvas.GetLeft(BlueColor) + 100 && y > Canvas.GetTop(BlueColor) && y < Canvas.GetTop(BlueColor) + 100)
            {
                khaledLineImage.Stroke = Brushes.Blue;
                //GreenColor.Visibility = Visibility.Hidden;
                //RedColor.Visibility = Visibility.Hidden;
                //BlackColor.Visibility = Visibility.Hidden;
                //BlueColor.Visibility = Visibility.Hidden;


            }
        }

        private void saveImage()
        {
            var bmpScreenshot = new System.Drawing.Bitmap(System.Windows.Forms.Screen.PrimaryScreen.Bounds.Width,
                               System.Windows.Forms.Screen.PrimaryScreen.Bounds.Height,
                               System.Drawing.Imaging.PixelFormat.Format32bppArgb);

            // Create a graphics object from the bitmap.
            var gfxScreenshot = System.Drawing.Graphics.FromImage(bmpScreenshot);

            // Take the screenshot from the upper left corner to the right bottom corner.
            gfxScreenshot.CopyFromScreen(System.Windows.Forms.SystemInformation.VirtualScreen.X,
                          System.Windows.Forms.SystemInformation.VirtualScreen.Y,
                           0,
                           0,
                           System.Windows.Forms.SystemInformation.VirtualScreen.Size,
                           System.Drawing.CopyPixelOperation.SourceCopy);

            DateTime dateTime = DateTime.UtcNow.Date;
            string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);
            string path = Path.Combine(myPhotos, "mySavedImage.png"+dateTime.ToString());
            bmpScreenshot.Save("D:\\Screenshot.png", ImageFormat.Png);
        }

    }
}
