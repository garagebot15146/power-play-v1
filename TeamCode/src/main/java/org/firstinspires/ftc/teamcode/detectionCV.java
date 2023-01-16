//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//@TeleOp(name = "detectionCV", group = "Tests")
////@Disabled
//public class detectionCV extends LinearOpMode {
//
//    OpenCvCamera camera;
//
//    public String stack = "";
//
//
//    @Override
//    public void runOpMode() {
//        // Camera Init
//        int cameraMonitorViewId = this
//                .hardwareMap
//                .appContext
//                .getResources().getIdentifier(
//                        "cameraMonitorViewId",
//                        "id",
//                        hardwareMap.appContext.getPackageName()
//                );
//
//        camera = OpenCvCameraFactory
//                .getInstance()
//                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//        // Loading pipeline
//        RingPipeline visionPipeline = new RingPipeline();
//        camera.setPipeline(visionPipeline);
//
//        // Start Streaming
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//        });
//
//        // Stream Camera
//        FtcDashboard.getInstance().startCameraStream(camera, 30);
//
//
//        waitForStart();
//
//        while (opModeIsActive()) {
////            telemetry.addData("Stack:", stack);
//            telemetry.addData("Region:", visionPipeline.avgTop);
//            telemetry.update();
//        }
//
//        FtcDashboard.getInstance().stopCameraStream();
//
//    }
//
//    // Pipeline class
//    class RingPipeline extends OpenCvPipeline {
//
//        // Constants
//        double width = 100;
//        double height = 100;
//
//        final int CORNER_X = 830;
//        final int CORNER_Y = 550;
//
//        // Working Mat variables
//        Mat yCbCrChan2Mat = new Mat();
//        Mat region = new Mat();
//
//        // Drawing variables
//        Scalar GRAY = new Scalar(220, 220, 220); // RGB values for gray.
//        Scalar GREEN = new Scalar(0, 255, 0); // RGB values for green.
//        Scalar RED = new Scalar(255, 0, 0); // RGB values for red.
//
//        // Variables that will store the results of our pipeline
//        public int avgTop;
//        public int avgBottom;
//        public int threshold = 117;
//
//        // Space which we will annalise data
//        public Point Square1 = new Point(CORNER_X, CORNER_Y);
//        public Point Square2 = new Point(CORNER_X - width, CORNER_Y - height);
//
//        // Drawing Points
//        int SquareX = (int) ((Square1.x + Square2.x) / 2);
//        int SquareY = (int) ((Square1.y + Square2.y) / 2);
//
//
//        @Override
//        public Mat processFrame(Mat input) {
//
//            // Img processing
//            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
//            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores
//
//            region = yCbCrChan2Mat.submat(new Rect(Square1, Square2));
//            avgTop = (int) Core.mean(region).val[0];
//
////            if (avgTop < threshold) {
////                stack = "FOUR";
////            } else if (avgTop > threshold) {
////                stack = "ONE";
////            } else {
////                stack = "ZERO";
////            }
//
//            // Draw Rectangle
//            Imgproc.rectangle(
//                    input,
//                    Square1,
//                    Square2,
//                    GRAY,
//                    2
//            );
//
//            switch (stack) {
//                case "FOUR":
//                    // Top Region
//                    Imgproc.rectangle(
//                            input,
//                            Square1,
//                            Square2,
//                            GREEN,
//                            2
//                    );
//
//                    break;
//                case "ONE":
//                    // Top Region
//                    Imgproc.rectangle(
//                            input,
//                            Square1,
//                            Square2,
//                            RED,
//                            2
//                    );
//
//                    break;
//                case "ZERO":
//                    // Top Region
//                    Imgproc.rectangle(
//                            input,
//                            Square1,
//                            Square2,
//                            GRAY,
//                            2
//                    );
//                    break;
//            }
//            return input;
//        }
//    }
//
//}
//
//
