package frc_cv_test;

import java.io.*;

import com.googlecode.javacv.CanvasFrame;
import static com.googlecode.javacv.cpp.opencv_imgproc.*;
import static com.googlecode.javacv.cpp.opencv_core.*;
import static com.googlecode.javacv.cpp.opencv_highgui.*;

public class Main {

    public static void main(String[] args) {

        // create image windows
        final CanvasFrame canvas = new CanvasFrame("Image");
        int wait = 1;
        float x_error = 0;
        float y_error = 0;
        float w_error = 0;
        float h_error = 0;
        int img_count = 0;

        try {
            //BufferedReader reader = new BufferedReader(new FileReader("SimpleTest.csv"));
            BufferedReader reader = new BufferedReader(new FileReader("GroundTruth.csv"));
            BufferedWriter writer = new BufferedWriter(new FileWriter("TestResults.csv"));
            String text = null;

            // repeat until all lines is read
            while ((text = reader.readLine()) != null) {

                ///////////////////////////////////////////////////////////////
                ///////////////////////////////////////////////////////////////
                // read in data

                // process input string
                String[] split;
                String delim = ",";
                split = text.split(delim);
                int img_num = new Integer(split[0]);
                float x_truth = new Float(split[1]);
                float y_truth = new Float(split[2]);
                float w_truth = new Float(split[3]);
                float h_truth = new Float(split[4]);

                // read image
                String filename = "images/" + "img" + img_num + ".png";
                final IplImage image = cvLoadImage(filename,1);
                System.out.println(img_num);
                img_count = img_count + 1;

                // display image
                canvas.showImage(image);
                canvas.waitKey(wait);

                
                ///////////////////////////////////////////////////////////////
                ///////////////////////////////////////////////////////////////
                // process the image
                
                // init result vars
                float x_target = 0;
                float y_target = 0;
                float w_target = 0;
                float h_target = 0;


                /////////////////
                // convert to hsv color space
                IplImage YCrCb_image = IplImage.create(image.cvSize(), 8, 3);
                cvCvtColor(image,YCrCb_image,CV_RGB2YCrCb);

                // display image
                canvas.showImage(YCrCb_image);
                canvas.waitKey(wait);

                /////////////////
                // separate the channels
                IplImage Y = IplImage.create(image.cvSize(), 8, 1);
                IplImage Cr = IplImage.create(image.cvSize(), 8, 1);
                IplImage Cb = IplImage.create(image.cvSize(), 8, 1);
                cvSplit(YCrCb_image,Y,Cr,Cb,null);

                /////////////////
                // image subtraction
                IplImage sub = IplImage.create(image.cvSize(), 8, 1);
                cvSub(Y,Cb,sub,null);

                // display image
                canvas.showImage(sub);
                canvas.waitKey(wait);

                /////////////////
                // thresholding
                IplImage binary = IplImage.create(image.cvSize(), 8, 1);
                cvThreshold(sub, binary, 50, 255, CV_THRESH_BINARY);

                // display image
                canvas.showImage(binary);
                canvas.waitKey(wait);

                /////////////////
                // morphlogical operations
                cvDilate(binary, binary, null, 1);
                cvErode(binary, binary, null, 1);

                // display image
                canvas.showImage(binary);
                canvas.waitKey(wait);
                cvSaveImage( "processed_images/"+img_num+"thresh"+".png", binary );


                ///////////////////////////////////////////////////////////////
                ///////////////////////////////////////////////////////////////
                // Find the Targets

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
                IplImage target_image = IplImage.create(image.cvSize(), 8, 3);
                cvCopy(image, target_image);

                while(contours != null) {

                    CvSeq c = cvCloneSeq(contours, storage);
                    c = cvApproxPoly( c, c.header_size(), storage, CV_POLY_APPROX_DP, 3, 0 );

                    Polygon p = new Polygon(c);
                    cvDrawContours(target_image, p.getCVSeq(), cvScalar(0.,0.,255.,0.), cvScalar(0.,0.,0.,0.), 0, 3, 8);

                    // is convex quadrilateral
                    if(p.isConvex() && p.getNumVertices() == 4) {

                        cvDrawContours(target_image, p.getCVSeq(), cvScalar(255.,0.,0.,0.), cvScalar(0.,0.,0.,0.), 0, 3, 8);

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

                /////////////////
                // image processed now continue if we have a valid target
                if(bestTarget != null) {

                    cvDrawContours(target_image, bestTarget.getCVSeq(), cvScalar(0.,255.,0.,0.), cvScalar(0.,0.,0.,0.), 0, 3, 8);

                    x_target = (bestTarget.getX() + (bestTarget.getWidth()/2));
                    y_target = (bestTarget.getY() + (bestTarget.getHeight()/2));
                    w_target = bestTarget.getWidth();
                    h_target = bestTarget.getHeight();

                    //Robot.getTable().putInt("x_target", x_target);
                    //Robot.getTable().putInt("y_target", y_target);
                    //Robot.getTable().putInt("w_target", w_target);

                }
                else {
                    x_target = 0;
                    y_target = 0;
                    w_target = 0;
                    h_target = 0;
                    //Robot.getTable().putInt("x_target", x_target);
                    //Robot.getTable().putInt("y_target", y_target);
                    //Robot.getTable().putInt("w_target", w_target);
                }// end if

                
                // display image
                canvas.showImage(target_image);
                canvas.waitKey(wait);
                cvSaveImage( "processed_images/"+img_num+"targets"+".png", target_image );

                
                ///////////////////////////////////////////////////////////////
                ///////////////////////////////////////////////////////////////
                // save results
                String out = img_num + "," + x_target + "," + y_target + "," + w_target + "," + h_target + "\n";
                writer.write(out, 0, out.length());

                x_error = x_error + (float)Math.sqrt((x_target - x_truth)*(x_target - x_truth));
                y_error = y_error + (float)Math.sqrt((y_target - y_truth)*(y_target - y_truth));
                w_error = w_error + (float)Math.sqrt((w_target - w_truth)*(w_target - w_truth));
                h_error = h_error + (float)Math.sqrt((h_target - h_truth)*(h_target - h_truth));

            }// end while

            // close the file
            reader.close();
            writer.close();
            System.out.println("Testing Done. Results: ");
            x_error = x_error / img_count;
            y_error = y_error / img_count;
            w_error = w_error / img_count;
            h_error = h_error / img_count;
            System.out.println("x_error: " + x_error + " " + "y_error: " + y_error + " " + "w_error: " + w_error + " " + "h_error: " + h_error);

        } catch(Exception e) {
            e.printStackTrace();
        }// end catch

    }// end main

    // euclidean distance between 2 points
    private static double getLength(CvPoint a, CvPoint b) {
        int deltax = a.x() - b.x();
        int deltay = a.y() - b.y();
        return Math.sqrt((deltax * deltax) + (deltay * deltay));
    }// end getLength

}// end class
