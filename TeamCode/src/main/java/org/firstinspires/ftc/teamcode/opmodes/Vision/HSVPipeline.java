/**
 * Created by Ability Edge#18273
 * - Elior Yousefi, and Eitan Kravets
 */
package org.firstinspires.ftc.teamcode.opmodes.Vision;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class HSVPipeline extends OpenCvPipeline
{

    public static double lowValuesTSEH = 142;
    public static double highValuesTSEH = 176;
    public static double lowValuesTSES = 116;
    public static double highValuesTSES = 200;
    public static double lowValuesTSEV = 62;
    public static double highValuesTSEV = 135;
    // creating a mast with the same resolution of the webcam for the place to display the detected team shipping element
    Mat mask = new Mat(1280,720,0);//
    Mat inputHSV = new Mat(1280,720,0);

    // creating 3 rectangles(sections) for checking the colors inside them.
    final Rect LEFT_SEC = new Rect(
            new Point(500.666667,0),//mask.cols()/7, mask.rows()/5 * 2
            new Point(200,500));//mask.cols()/7 * 2, mask.rows()/5 * 4)


    final Rect CENTER_SEC = new Rect(
            new Point(803.333334,0),
            new Point(500.666667,500));

    final Rect RIGHT_SEC = new Rect(
            new Point(1080,0),
            new Point(803.333334,500));

    public static boolean DEBUG = true;

    public static double threshold = 1000;

    // the list of locations that can be
    public enum Location{
        Left,
        Center,
        Right,
        Not_Found
    }

    public Location location = Location.Not_Found;

    // color for the rectangles to show on the screen
    Scalar colorBarcodeRect = new Scalar(0, 255, 0);

    public Telemetry telemetry;

    boolean barcodeLeft, barcodeCenter, barcodeRight;

    @Override
    public Mat processFrame(Mat input) {
        // HSV low and high values for our team shipping element.
        Scalar lowValuesTSE = new Scalar(lowValuesTSEH, lowValuesTSES, lowValuesTSEV);
        Scalar highValuesTSE = new Scalar(highValuesTSEH, highValuesTSES, highValuesTSEV);

        Imgproc.cvtColor(input, inputHSV, Imgproc.COLOR_RGB2HSV);

        Core.inRange(inputHSV, lowValuesTSE, highValuesTSE, mask);

        // taking sections from the mask to another mat
        Mat left = mask.submat(LEFT_SEC);
        Mat center = mask.submat(CENTER_SEC);
        Mat right = mask.submat(RIGHT_SEC);

        barcodeLeft = Core.countNonZero(left) > threshold;
        barcodeCenter = Core.countNonZero(center) > threshold;
        barcodeRight = Core.countNonZero(right) > threshold;

        // we release the mats for use.
        left.release();
        center.release();
        right.release();

        //checking which barcode is found.
        if(barcodeLeft && barcodeCenter && barcodeRight){
            // NOT FOUND
            setLocation(Location.Not_Found);
        }else if(barcodeLeft){
            setLocation(Location.Left);
        }else if(barcodeCenter){
            setLocation(Location.Center);
        }else if(barcodeRight){
            setLocation(Location.Right);
        }else{
            // NOT FOUND
            setLocation(Location.Not_Found);
        }

        Imgproc.cvtColor(mask, mask, Imgproc.COLOR_GRAY2RGB);

        // creating 3 rectangles at the mask mat with the color @colorBarcodeRect@ and the rectangle points LEFT/CENTER/RIGHT.
        Imgproc.rectangle(mask, LEFT_SEC, colorBarcodeRect);
        Imgproc.rectangle(mask, CENTER_SEC, colorBarcodeRect);
        Imgproc.rectangle(mask, RIGHT_SEC, colorBarcodeRect);

        return mask;
    }

    // getting location of the team shipping element.
    public Location getLocation() {
        return location;
    }

    public void setLocation(Location location) {
        this.location = location;
    }

    public void setTelementry(Telemetry telemetry){ this.telemetry = telemetry; }
}