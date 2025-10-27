package org.firstinspires.ftc.teamcode.mechanics;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class Vision extends OpenCvPipeline {


    // Telemetry telemetry;
/*
    public Vision(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

 */


    private double bluePercentMid, redPercentMid, redPercentRight, bluePercentRight, redPercentLeft, bluePercentLeft;

    public static int REGION_WIDTH = 70;
    public static int REGION_HEIGHT = 25;

    private final Scalar
            blue = new Scalar(0, 255, 255),
            yellow = new Scalar(255, 255, 0);

    private static final Scalar
            //bounds are in RGB
            lower_blue_bounds = new Scalar(2, 45, 60, 255),
            upper_blue_bounds = new Scalar(130, 230, 230, 255),
            lower_red_bounds = new Scalar(90, 0, 40, 255),
            upper_red_bounds = new Scalar(255, 120, 255, 255);

    private static Point MidBoxPoint = new Point(320 / 2 - REGION_WIDTH * 1.5, 240 / 2);
    private static Point MidBoxPoint2 = new Point(320 / 2 + REGION_WIDTH * 1.7, 240 / 2);
    private static Point leftBoxPoint2 = new Point(320 / 3 - REGION_WIDTH / 1.7,    240 / 2);
    private static Point RightBoxPoint = new Point(320 / 3 + REGION_WIDTH * 1.7, 240 / 2);

    Point sleeve_pointA = new Point(
            (MidBoxPoint.x - 30) + REGION_WIDTH,
            MidBoxPoint.y);


    Point sleeve_pointB = new Point(
            leftBoxPoint2.x - 50,
            leftBoxPoint2.y + REGION_HEIGHT * 1.9+ 30 );

    Point sleeve_pointC = new Point(
            (MidBoxPoint2.x+10) - REGION_WIDTH,
            (MidBoxPoint.y) + REGION_HEIGHT*1.2);

    //goes first
    Point sleeve_pointD = new Point(
            (MidBoxPoint.x-10) + REGION_WIDTH,
            (MidBoxPoint.y));

    Point sleeve_pointE = new Point(
            (MidBoxPoint2.x + 30) - REGION_WIDTH,
            MidBoxPoint2.y +30
    );
    Point sleeve_pointF = new Point(
            RightBoxPoint.x + 75,
            RightBoxPoint.y + REGION_HEIGHT * 1.9 + 50
    );


    public enum Position {
        LEFT,
        RIGHT,
        CENTER,
        NONE
    }
//monke

    private volatile Position Bposition = Position.NONE, Rposition = Position.NONE;

    private Mat redMat = new Mat(), blurredMat3 = new Mat(), redMat3 = new Mat(), blueMat3 = new Mat(), redMat2 = new Mat(), blueMat2 = new Mat(), blueMat = new Mat(), blurredMat = new Mat(), blurredMat2 = new Mat(), kernel = new Mat();

    @Override
    public Mat processFrame(Mat input) {

        //left
//        Imgproc.rectangle(
//                input,
//                sleeve_pointA,
//                sleeve_pointB,
//                yellow,
//                2
//        );
//        //right
//        Imgproc.rectangle(
//                input,
//                sleeve_pointE,
//                sleeve_pointF,
//                yellow,
//                2
//        );//mid
//        Imgproc.rectangle(
//                input,
//                sleeve_pointD,
//                sleeve_pointC,
//                yellow,
//                2
//        );


        Imgproc.blur(input, blurredMat, new Size(5, 5));
        Imgproc.blur(input, blurredMat2, new Size(5, 5));
        Imgproc.blur(input, blurredMat3, new Size(5,5));

        //mid
        blurredMat= blurredMat.submat(new Rect(sleeve_pointD, sleeve_pointC));
        //left
        blurredMat2 = blurredMat2.submat(new Rect(sleeve_pointA, sleeve_pointB));
        //right
        blurredMat3 = blurredMat3.submat(new Rect(sleeve_pointE, sleeve_pointF));

        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,new Size(3,3));

        Imgproc.morphologyEx(blurredMat, blurredMat, Imgproc.MORPH_CLOSE, kernel);
        Imgproc.morphologyEx(blurredMat2, blurredMat2, Imgproc.MORPH_CLOSE, kernel);
        Imgproc.morphologyEx(blurredMat3, blurredMat3, Imgproc.MORPH_CLOSE, kernel);

        //mid
        Core.inRange(blurredMat, lower_red_bounds, upper_red_bounds, redMat);
        Core.inRange(blurredMat, lower_blue_bounds, upper_blue_bounds, blueMat);
        //left
        Core.inRange(blurredMat2, lower_blue_bounds, upper_blue_bounds, redMat2);
        Core.inRange(blurredMat2, lower_red_bounds, upper_red_bounds, blueMat2);
        //right
        Core.inRange(blurredMat3, lower_blue_bounds, upper_blue_bounds, redMat3);
        Core.inRange(blurredMat3, lower_red_bounds, upper_red_bounds, blueMat3);

        redPercentMid = Core.countNonZero(redMat);
        redPercentLeft = Core.countNonZero(redMat2);
        redPercentRight = Core.countNonZero(redMat3);

        bluePercentMid = Core.countNonZero(blueMat);
        bluePercentLeft = Core.countNonZero(blueMat2);
        bluePercentRight = Core.countNonZero(blueMat3);


        double maxPercentBlue = Math.max(Math.max(bluePercentRight,bluePercentLeft),bluePercentMid);
        double maxPercentRed = Math.max(Math.max(redPercentRight,redPercentLeft),redPercentMid);
        double maxPercentTotal = Math.max(maxPercentBlue,maxPercentRed);



        if ((redPercentMid == maxPercentRed && redPercentMid >= maxPercentTotal)) {
            // Red or Blue detected in the left box

            Rposition = Position.CENTER;
            Imgproc.rectangle(
                    input,
                    sleeve_pointD,
                    sleeve_pointC,
                    yellow,
                    2
            );

        }
        else if ((redPercentLeft == maxPercentRed && redPercentLeft >= maxPercentTotal)) {

            // Red or Blue detected in the left box
            Rposition = Position.LEFT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    yellow,
                    2
            );
        }
        else if ((redPercentRight == maxPercentRed && redPercentRight >= maxPercentTotal) ){
            // Red or Blue detected in the right  box
            Rposition = Position.RIGHT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointE,
                    sleeve_pointF,
                    yellow,
                    2
            );
        }
        if (bluePercentMid == maxPercentBlue && bluePercentMid >=  maxPercentTotal)  {
            Bposition = Position.CENTER;
            Imgproc.rectangle(
                    input,
                    sleeve_pointD,
                    sleeve_pointC,
                    yellow,
                    2
            );
        }
        else if (bluePercentLeft == maxPercentBlue && bluePercentLeft >=  maxPercentTotal) {
            Bposition = Position.LEFT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    yellow,
                    2
            );
        }
        else if (bluePercentRight == maxPercentBlue && bluePercentRight >=  maxPercentTotal) {
            Bposition = Position.RIGHT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointE,
                    sleeve_pointF,
                    yellow,
                    2
            );
        }
        blurredMat.release();
        blurredMat2.release();
        blurredMat3.release();
        redMat.release();
        redMat2.release();
        redMat3.release();
        blueMat.release();
        blueMat2.release();
        blueMat2.release();
        kernel.release();



        return input;

    }
    public Position getBluePosition() {
        return Bposition;
    }

    public Position getRedPosition() {
        return Rposition;
    }

    public double getRedPercentMid(){
        return redPercentMid;
    }
    public double getRedPercentRight(){
        return redPercentRight;
    }
    public double getRedPercentLeft(){
        return redPercentLeft;
    }
    public double getBluePercentMid(){
        return bluePercentMid;
    }
    public double getBluePercentRight(){
        return bluePercentRight;
    }
    public double getBluePercentLeft(){
        return bluePercentLeft;
    }
}