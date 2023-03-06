package org.firstinspires.ftc.teamcode.tata.OpenCVDetector;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class CapstoneDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        pos1,
        pos2,
        pos3
    }
    private Location location = Location.pos1;

    private Rect CAP_LEFT_ROI = new Rect(
            new Point(50, 25),
            new Point(150, 125));
    private Rect CAP_RIGHT_ROI = new Rect(
            new Point(170, 25),
            new Point(270, 125));
    static double PERCENT_COLOR_THRESHOLD = 0.1;
    private tataAutonomousBase.SideColor side;

    public CapstoneDetector(Telemetry t, tataAutonomousBase.SideColor sc) {

        telemetry = t;
        side = sc;

    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSV = new Scalar(15, 30, 50);
        Scalar highHSV = new Scalar(42, 255, 255);

        //Scalar lowHSV = new Scalar(200, 10, 20);
        //Scalar highHSV = new Scalar(255, 100, 100);
        //ducks above
        //CHANGE THIS ONCE U RUN CODE

        Core.inRange(mat, lowHSV, highHSV, mat);
        if (side == tataAutonomousBase.SideColor.Red) {
            CAP_LEFT_ROI = new Rect(
                    new Point(70, 25),
                    new Point(170, 125));

            CAP_RIGHT_ROI = new Rect(
                    new Point(200, 25),
                    new Point(300, 125));
        }

        Mat left = mat.submat(CAP_LEFT_ROI);
        Mat right = mat.submat(CAP_RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] / CAP_LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / CAP_RIGHT_ROI.area() / 255;

        left.release();
        right.release();
/*
        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");
*/
        boolean capLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean capRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (capLeft) {
            location = Location.pos1;
            //telemetry.addData("Capstone Location", "pos1");
        }
        else if (capRight) {
            location = Location.pos2;
            //telemetry.addData("Capstone Location", "pos2");
        }
        else {
            location = Location.pos3;
            //telemetry.addData("Capstone Location", "pos3");
        }
        //telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorNotCap = new Scalar(255, 0, 0);
        Scalar colorCap = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, CAP_LEFT_ROI, location == Location.pos1? colorCap:colorNotCap);
        Imgproc.rectangle(mat, CAP_RIGHT_ROI, location == Location.pos2? colorCap:colorNotCap);

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}