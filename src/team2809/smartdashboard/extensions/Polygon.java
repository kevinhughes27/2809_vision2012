package team2809.smartdashboard.extensions;

import static com.googlecode.javacv.cpp.opencv_imgproc.*;
import static com.googlecode.javacv.cpp.opencv_core.*;

public class Polygon {

    CvSeq polygon;
    CvRect boundingRect;

    Polygon(CvSeq data) {
        polygon = data;
    }

    CvSeq getCVSeq() {
        return polygon;
    }

    /**
     *
     * @return an array of CvPoints of the vertices of the polygon
     */
    public CvPoint[] getPoints() {
        CvPoint point = new CvPoint(getNumVertices());
        CvPoint[] points= new CvPoint[getNumVertices()];
        cvCvtSeqToArray(polygon, point.position(0), CV_WHOLE_SEQ);
        for (int j = 0; j < getNumVertices(); j++) {
            points[j] = new CvPoint(point.position(j).x(), point.position(j).y());
        }
        return points;
    }

    /**
     *
     * @return the width of the bounding rectangle of the polygon
     */
    public int getWidth() {
        if (boundingRect == null) {
            boundingRect = cvBoundingRect(polygon, 0);
        }
        return boundingRect.width();
    }

    /**
     *
     * @return the height of the bounding rectangle of the polygon
     */
    public int getHeight() {
        if (boundingRect == null) {
            boundingRect = cvBoundingRect(polygon, 0);
        }
        return boundingRect.height();
    }

    /**
     *
     * @return the x coord of the top left corner of the bounding
     * rectangle of the polygon
     */
    public int getX() {
        if (boundingRect == null) {
            boundingRect = cvBoundingRect(polygon, 0);
        }
        return boundingRect.x();
    }

    /**
     *
     * @return the y coord of the top left corner of the bounding
     * rectangle of the polygon
     */
    public int getY() {
        if (boundingRect == null) {
            boundingRect = cvBoundingRect(polygon, 0);
        }
        return boundingRect.y();
    }

    /**
     *
     * @return the number of vertices in the polygon
     */
    public int getNumVertices() {
        return polygon.total();

    }

    /**
     *
     * @return whether or not the polygon is convex
     */
    public boolean isConvex() {
        return cvCheckContourConvexity(polygon) == 0 ? false : true;
    }

    /**
     *
     * @return the area in pixels of the polygon
     */
    public int getArea() {
        return Math.abs((int) cvContourArea(polygon, CV_WHOLE_SEQ, -1));
    }

    /**
     *
     * @return the perimeter in pixels of the polygon
     */
    public int getPerimeter() {
        return (int) cvArcLength(polygon, CV_WHOLE_SEQ, -1);
    }

    public void disposed() {
        polygon.deallocate();
    }
}