package team2809.smartdashboard.extensions;

// FRC
import edu.wpi.first.smartdashboard.properties.BooleanProperty;
import edu.wpi.first.smartdashboard.properties.IntegerProperty;
import edu.wpi.first.smartdashboard.robot.Robot;
import edu.wpi.first.wpijavacv.*;

// OpenCV
import static com.googlecode.javacv.cpp.opencv_imgproc.*;
import static com.googlecode.javacv.cpp.opencv_core.*;
import static com.googlecode.javacv.cpp.opencv_highgui.*;

// Java
import java.awt.Graphics;


public class VisionTargeting extends WPICamera_lessBandwidth_extension {
    public static final String NAME = "Vision Targeting";

    // params
    public final IntegerProperty threshold = new IntegerProperty(this, "threshold",100);
    public final IntegerProperty dilate = new IntegerProperty(this, "dilate",1);
    public final IntegerProperty erode = new IntegerProperty(this, "erode",1);
    public final BooleanProperty saveImages = new BooleanProperty(this,"saveImages", false);

    // target info
    public int x_target;
    public int y_target;

    // points for the overlay line
    public static CvPoint a = new CvPoint( 160, 0 );
    public static CvPoint b = new CvPoint( 160, 240 );
    public static WPIPoint A = new WPIPoint( 160, 0 );
    public static WPIPoint B = new WPIPoint( 160, 240 );

    private int frame_count;
    private int img_num;


    ///////////////////////////////////////////////////////////////////////////
    @Override
    public void init() {
        super.init();
        frame_count = 0;
        img_num = 0;
    }


    ///////////////////////////////////////////////////////////////////////////
    @Override
    public WPIImage processImage(WPIColorImage rawImage) {

        IplImage image = IplImage.createFrom(rawImage.getBufferedImage());

        Polygon Target = findTarget( image );

        // image processed now continue if we have a valid target
        if(Target != null) {

            // draw target;
            rawImage.drawRect(Target.getX(), Target.getY(), Target.getWidth(), Target.getHeight(), WPIColor.RED, 3);

            x_target = (Target.getX() + (Target.getWidth()/2));
            y_target = (Target.getY() + (Target.getHeight()/2));
            Robot.getTable().putInt("x_target", x_target);
            Robot.getTable().putInt("y_target", y_target);

        } else {
            x_target = 0;
            y_target = 0;
            Robot.getTable().putInt("x_target", x_target);
            Robot.getTable().putInt("y_target", y_target);
        }// end if

        // also draw the center line
        cvLine(image, a, b, CvScalar.RED, 3, 8, 0 );

        frame_count = frame_count + 1;

        rawImage.drawLine(A, B, WPIColor.RED, 3);
        
        return rawImage;
    }// end processImage


    ///////////////////////////////////////////////////////////////////////////
    private Polygon findTarget(IplImage image) {

        /////////////////
        // convert to hsv color space
        IplImage YCrCb_image = IplImage.create(image.cvSize(), 8, 3);
        cvCvtColor(image,YCrCb_image,CV_BGR2YCrCb); // axis cam is BGR


        /////////////////
        // separate the channels
        IplImage Y = IplImage.create(image.cvSize(), 8, 1);
        IplImage Cr = IplImage.create(image.cvSize(), 8, 1);
        IplImage Cb = IplImage.create(image.cvSize(), 8, 1);
        cvSplit(YCrCb_image,Y,Cr,Cb,null);

        /////////////////
        // image subtraction
        IplImage sub = IplImage.create(image.cvSize(), 8, 1);
        cvSub(Y,Cr,sub,null);

        /////////////////
        // thresholding
        IplImage thresh = IplImage.create(image.cvSize(), 8, 1);
        cvThreshold(sub, thresh, threshold.getValue().intValue(), 255, CV_THRESH_BINARY);

        /////////////////
        // morphlogical operations
        IplImage binary = IplImage.create(image.cvSize(), 8, 1);
        cvDilate(thresh, binary, null, dilate.getValue().intValue());
        cvErode(binary, binary, null, erode.getValue().intValue());

        /////////////////
        // find contours
        IplImage tempBinary = IplImage.create(image.cvSize(), image.depth(), 1);
        cvCopy(binary, tempBinary);
        CvMemStorage storage = CvMemStorage.create();
        CvSeq contours = new CvSeq();
        cvFindContours(tempBinary, storage, contours, 256, CV_RETR_LIST, CV_CHAIN_APPROX_TC89_KCOS);

        /////////////////
        // polygon approx
        Polygon bestTarget = null;

        // iterate through contours
        while(contours != null) {

            CvSeq c = cvCloneSeq(contours, storage);
            c = cvApproxPoly( c, c.header_size(), storage, CV_POLY_APPROX_DP, 3, 0 );

            Polygon p = new Polygon(c);

            // is convex quadrilateral
            if(p.isConvex() && p.getNumVertices() == 4) {

                CvPoint[] points = p.getPoints();

                double side1 = getLength(points[0], points[1]);
                double side2 = getLength(points[1], points[2]);
                double side3 = getLength(points[2], points[3]);
                double side4 = getLength(points[3], points[0]);

                double ratio1 = Math.abs(Math.log(side1 / side3));
                double ratio2 = Math.abs(Math.log(side2 / side4));

                // If the lengths of the top to bottom and left to right sides are very close, the shape is a rectangle
                if(ratio1 < 0.15 && ratio2 < 0.15) {
                    if(bestTarget == null || p.getY() < bestTarget.getY()) {
                        bestTarget = p;
                    }// end if
                }// end if

            }// end if

            // iterate to next contour
            contours = contours.h_next();
        }// end while
        
        // save images if
        int shooting = Robot.getTable().getInt("shooting");
        if( (shooting == 1) && (saveImages.getValue().booleanValue()) ) {
            if(frame_count % 15 == 0) {
                cvSaveImage( "images/"+img_num+"img"+".png", image );
                cvSaveImage( "images/"+img_num+"YCrCb"+".png", YCrCb_image );
                cvSaveImage( "images/"+img_num+"Y"+".png", Y );
                cvSaveImage( "images/"+img_num+"Cr"+".png", Cr );
                cvSaveImage( "images/"+img_num+"Cb"+".png", Cb );
                cvSaveImage( "images/"+img_num+"sub"+".png", sub );
                cvSaveImage( "images/"+img_num+"thresh"+".png", thresh );
                cvSaveImage( "images/"+img_num+"binary"+".png", binary );
                img_num = img_num + 1;
            }// end if
        }// end if

        // memory managment
        YCrCb_image.release();
        Y.release();
        Cr.release();
        Cb.release();
        sub.release();
        thresh.release();
        binary.release();
        tempBinary.release();

       return bestTarget;
    }


    ///////////////////////////////////////////////////////////////////////////
    // euclidean distance between 2 points
    private static double getLength(CvPoint a, CvPoint b) {
        int deltax = a.x() - b.x();
        int deltay = a.y() - b.y();
        return Math.sqrt((deltax * deltax) + (deltay * deltay));
    }// end getLength


    ///////////////////////////////////////////////////////////////////////////
    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        // write target values
        g.drawString("x_target: " + Integer.toString(x_target), 105, 315);
        g.drawString("y_target: " + Integer.toString(y_target), 205, 315);
    }

}// end VisionTargeting

