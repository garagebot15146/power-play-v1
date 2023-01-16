//package org.firstinspires.ftc.teamcode.Settings;
//
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//public class ConePipeline extends OpenCvPipeline {
//
//    public enum ConePosition {
//        LEFT,
//        CENTER,
//        RIGHT
//    }
//
//    // Constants
//    double width = 100;
//    double height = 115;
//
//    final int CORNER_X = 1275;
//    final int CORNER_Y = 300;
//
//    // Working Mat variables
//    Mat yCbCrChan2Mat = new Mat();
//    Mat region = new Mat();
//
//    // Drawing variables
//    Scalar GRAY = new Scalar(220, 220, 220); // RGB values for gray.
//    Scalar BLACK = new Scalar(0, 0, 0); // RGB values for gray.
//    Scalar YELLOW = new Scalar(255, 255, 0);
//    Scalar PINK = new Scalar(255, 20, 147); // RGB values for green.
//    Scalar BLUE = new Scalar(30, 144, 255); // RGB values for red.
//
//    // Variables that will store the results of our pipeline
//    public int avg;
//    public int upperThresh = 145;
//    public int lowerThresh = 115;
//
//    // Space which we will annalise data
//    public Point Square1 = new Point(CORNER_X, CORNER_Y);
//    public Point Square2 = new Point(CORNER_X - width, CORNER_Y - height);
//
//    // Drawing Points
//    int SquareX = (int) ((Square1.x + Square2.x) / 2);
//    int SquareY = (int) ((Square1.y + Square2.y) / 2);
//
//    /*
//     * Points which actually define the sample region rectangles, derived from above values
//     *
//     * Example of how points A and B work to define a rectangle
//     *
//     *   ------------------------------------
//     *   | (0,0) Point A                    |
//     *   |                                  |
//     *   |                                  |
//     *   |                                  |
//     *   |                                  |
//     *   |                                  |
//     *   |                                  |
//     *   |                  Point B (70,50) |
//     *   ------------------------------------
//     *
//     */
//
//    private volatile ConePosition position = ConePosition.LEFT;
//
//    @Override
//    public Mat processFrame(Mat input) {
//
//        // Img processing
//        Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
//        Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores
//
//        region = yCbCrChan2Mat.submat(new Rect(Square1, Square2));
//        avg = (int) Core.mean(region).val[0];
//
//        // Draw Rectangle
//        Imgproc.rectangle(
//                input,
//                Square1,
//                Square2,
//                BLACK,
//                5
//        );
//
//        if (avg > upperThresh) {
//            position = ConePosition.RIGHT;
////            Imgproc.rectangle(
////                    input,
////                    Square1,
////                    Square2,
////                    BLUE,
////                    2
////            );
//        } else if (avg <= upperThresh && avg >= lowerThresh) {
//            position = ConePosition.CENTER;
////            Imgproc.rectangle(
////                    input,
////                    Square1,
////                    Square2,
////                    PINK,
////                    2
////            );
//        } else {
//            position = ConePosition.LEFT;
////            Imgproc.rectangle(
////                    input,
////                    Square1,
////                    Square2,
////                    YELLOW,
////                    2
////            );
//        }
//
//        return input;
//    }
//
//    public ConePosition getPosition() {
//        return position;
//    }
//
//    public int getAnalysis() {
//        return avg;
//    }
//}