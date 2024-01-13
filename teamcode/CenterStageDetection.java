package org.firstinspires.ftc.teamcode;

//THIS ONE WORKS 



import static org.firstinspires.ftc.teamcode.CenterStageDetection.Location.CENTER;
import static org.firstinspires.ftc.teamcode.CenterStageDetection.Location.LEFT;
import static org.firstinspires.ftc.teamcode.CenterStageDetection.Location.RIGHT;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CenterStageDetection extends OpenCvPipeline {
/*
YELLOW  = Parking Left
CYAN    = Parking Middle
MAGENTA = Parking Right
 */

    public enum ColorDetected {
        GRAY,
        MAGENTA,
        CYAN
    }

    public enum Location {
        LEFT,
        CENTER,
        RIGHT;
    }

    // Lower and upper boundaries for colors
    private static final Scalar
            lower_gray_bounds  = new Scalar(140, 140, 140, 255),
            upper_gray_bounds  = new Scalar(170, 170, 170, 255),
            lower_cyan_bounds    = new Scalar(0, 45, 120, 255),
            upper_cyan_bounds    = new Scalar(50, 255, 255, 255),
            lower_magenta_bounds = new Scalar(155, 20, 60, 255),
            upper_magenta_bounds = new Scalar(255, 60, 100, 255);

    // Color definitions
    private final Scalar
            GRAY = new Scalar(103, 103, 103),
            CYAN    = new Scalar(0, 255, 255),
            MAGENTA = new Scalar(255, 0, 255);

    // Percent and mat definitions
    public double grayPercentLeft, grayPercentCenter, cyanPercentLeft, cyanPercentCenter, magentaPercentLeft, magentaPercentCenter;
    private Mat grayMat = new Mat(), cyanMat = new Mat(), magentaMat = new Mat(), blurredMatLeft = new Mat(), blurredMatCenter = new Mat();

//    // Anchor point definitions
//    Point sleeve_pointA = new Point(
//            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
//            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
//    Point sleeve_pointB = new Point(
//            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // Running variable storing the parking position
    private volatile ColorDetected colorLeft;
    private volatile ColorDetected colorMiddle;

    private volatile Location location = CENTER;
    private double maxPercentLeft;
    private double maxPercentCenter;

    @Override
    public Mat processFrame(Mat input) {
        // actual code.
        //Rect leftArea = new Rect(new Point(10,110), new Point(70,150));
        //Rect middleArea = new Rect(new Point(165,130), new Point(215,150));

        // 320 x 240

        //Rect leftArea = new Rect(10,110, 70,150);
        //Rect centerArea = new Rect(165,130, 215,150);

        ///Rect leftArea = new Rect(1,10,90,120);
        ///Rect centerArea = new Rect(155,20,80,80);

        Rect leftArea = new Rect(85,70,25,25);
        Rect centerArea = new Rect(235,80,20,20);

        //Rect leftRect = new Rect(1,1,219,959);
        //Rect centerRect = new Rect(221,1,839,959);
        //Rect rightRect = new Rect(1061,1,219,959);

        //Rect leftArea = new Rect(2,40, 80,200);
        //Rect middleArea = new Rect(100,200, 200,80);

        // Noise reduction
        Imgproc.blur(input, blurredMatLeft, new Size(5, 5));
        blurredMatLeft = blurredMatLeft.submat(leftArea);

        // Apply Morphology
        Mat kernelLeft = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMatLeft, blurredMatLeft, Imgproc.MORPH_CLOSE, kernelLeft);

        // Gets channels from given source mat
        Core.inRange(blurredMatLeft, lower_gray_bounds, upper_gray_bounds, grayMat);
        Core.inRange(blurredMatLeft, lower_cyan_bounds, upper_cyan_bounds, cyanMat);
        Core.inRange(blurredMatLeft, lower_magenta_bounds, upper_magenta_bounds, magentaMat);

        grayPercentLeft = Core.countNonZero(grayMat);
        cyanPercentLeft = Core.countNonZero(cyanMat);
        magentaPercentLeft = Core.countNonZero(magentaMat);

        Imgproc.blur(input, blurredMatCenter, new Size(5, 5));
        blurredMatCenter = blurredMatCenter.submat(centerArea);

        // Apply Morphology
        Mat kernelMiddle = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMatCenter, blurredMatCenter, Imgproc.MORPH_CLOSE, kernelMiddle);

        // Gets channels from given source mat
        Core.inRange(blurredMatCenter, lower_gray_bounds, upper_gray_bounds, grayMat);
        Core.inRange(blurredMatCenter, lower_cyan_bounds, upper_cyan_bounds, cyanMat);
        Core.inRange(blurredMatCenter, lower_magenta_bounds, upper_magenta_bounds, magentaMat);

        // Gets color specific values
        grayPercentCenter = Core.countNonZero(grayMat);
        cyanPercentCenter = Core.countNonZero(cyanMat);
        magentaPercentCenter = Core.countNonZero(magentaMat);

        // Calculates the highest amount of pixels being covered on each side
        maxPercentLeft = Math.max((grayPercentLeft), (Math.max(cyanPercentLeft, magentaPercentLeft)));
        maxPercentCenter = Math.max((grayPercentCenter), (Math.max(cyanPercentCenter, magentaPercentCenter)));

        // Checks all percentages, will highlight bounding box in camera preview
        // based on what color is being detected

        if (maxPercentLeft == grayPercentLeft) {
            colorLeft = ColorDetected.GRAY;
            Imgproc.rectangle(
                    input,
                    leftArea,
                    GRAY,
                    2
            );
        } else if (maxPercentLeft == cyanPercentLeft) {
            colorLeft = ColorDetected.CYAN;
            Imgproc.rectangle(
                    input,
                    leftArea,
                    CYAN,
                    2
            );
        } else if (maxPercentLeft == magentaPercentLeft) {
            colorLeft = ColorDetected.MAGENTA;
            Imgproc.rectangle(
                    input,
                    leftArea,
                    MAGENTA,
                    2
            );
        }

        if (maxPercentCenter == grayPercentCenter) {
            colorMiddle = ColorDetected.GRAY;
            Imgproc.rectangle(
                    input,
                    centerArea,
                    GRAY,
                    2
            );
        } else if (maxPercentCenter == cyanPercentCenter) {
            colorMiddle = ColorDetected.CYAN;
            Imgproc.rectangle(
                    input,
                    centerArea,
                    CYAN,
                    2
            );
        } else if (maxPercentCenter == magentaPercentCenter) {
            colorMiddle = ColorDetected.MAGENTA;
            Imgproc.rectangle(
                    input,
                    centerArea,
                    MAGENTA,
                    2
            );
        }

        // Memory cleanup

        if ((colorLeft == CenterStageDetection.ColorDetected.CYAN) || (colorLeft == CenterStageDetection.ColorDetected.MAGENTA))
            location = LEFT;
        else if ((colorMiddle == CenterStageDetection.ColorDetected.CYAN) || (colorMiddle == CenterStageDetection.ColorDetected.MAGENTA))
            location = CENTER;
        else
            location = RIGHT;

        return input;
    }

    // Returns an enum being the current position where the robot will park
    public ColorDetected getColorLeft() {
        return colorLeft;
    }

    public double getmaxPercentLeft() {
        return maxPercentLeft;
    }

    public double getmaxPercentCenter() {
        return maxPercentCenter;
    }

    public ColorDetected getColorMiddle() {
        return colorMiddle;
    }

    public Location getLocation() {return location;}

    public void finalize() throws Throwable {
        close();
        super.finalize();
    }

    public void close() {
        blurredMatLeft.release();
        blurredMatCenter.release();
        grayMat.release();
        cyanMat.release();
        magentaMat.release();
    }
}





