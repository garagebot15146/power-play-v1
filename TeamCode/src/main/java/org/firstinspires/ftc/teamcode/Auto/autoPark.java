//package org.firstinspires.ftc.teamcode.Auto;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.Settings.AprilTagPipeline;
//import org.firstinspires.ftc.teamcode.Settings.OpenCVPipeline;
//import org.firstinspires.ftc.teamcode.Settings.drive.HWMap;
//import org.firstinspires.ftc.teamcode.Settings.trajectorysequence.TrajectorySequence;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.apriltag.AprilTagDetection;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//import java.util.ArrayList;
//
//@Autonomous(name = "autoPark", group = "auto")
//@Disabled
//public class autoPark extends LinearOpMode {
//
//    // VISION
//    OpenCvCamera camera;
//    String CVconePos = "CENTER";
//    String ATconePos = "NOT_SET";
//    String conePos = "CENTER";
//    boolean ATLock = true;
//
//    @Override
//    public void runOpMode() {
//        HWMap drive = new HWMap(hardwareMap);
//
//        //trajectories
//        Pose2d startPos = new Pose2d(0, 0, Math.toRadians(90));
//
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
//        // Loading Pipeline
//        OpenCVPipeline visionPipeline = new OpenCVPipeline();
//        AprilTagPipeline aprilTagDetectionPipeline = new AprilTagPipeline();
//
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
//        // Stream Camera Onto Dash
//        FtcDashboard.getInstance().startCameraStream(camera, 30);
//
//        // Output Log
//        telemetry.addData("Status", "Pipeline Initializing");
//        telemetry.update();
//
//        // Start Auto
//        while (!isStarted() && !isStopRequested()) {
//            camera.setPipeline(visionPipeline);
//            telemetry.addData("CV Position", visionPipeline.getPosition());
//            telemetry.addData("CV Analysis", visionPipeline.getAnalysis());
//            sleep(300);
//
//            camera.setPipeline(aprilTagDetectionPipeline);
//            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
//            if(ATLock){
//                if (detections != null) {
//                    if (detections.size() != 0) {
//                        int aprilTagID = detections.get(0).id;
//                        switch (aprilTagID){
//                            case 1:
//                                ATconePos = "LEFT";
//                                break;
//                            case 2:
//                                ATconePos = "CENTER";
//                                break;
//                            default:
//                                ATconePos = "RIGHT";
//                                break;
//                        }
////                        ATLock = false;
//                    }
//                }
//            }
//            telemetry.addData("AT Position", ATconePos);
//            sleep(500);
//            telemetry.update();
//        }
//
//        CVconePos = visionPipeline.getPosition().name();
//
//        if(ATconePos == "NOT_SET"){
//            conePos = CVconePos;
//        } else {
//            conePos = ATconePos;
//        }
//
//        telemetry.addData("Cone Vision", conePos);
//        telemetry.addData("Analysis", visionPipeline.getAnalysis());
//        telemetry.update();
//        FtcDashboard.getInstance().stopCameraStream();
//
//        //LEFT
//        switch (conePos) {
//            case "LEFT":
//                TrajectorySequence driveZone1 = drive.trajectorySequenceBuilder(startPos)
//                        .lineToLinearHeading(new Pose2d(0, -2.4, Math.toRadians(90)))
//                        .turn(Math.toRadians(3.2))
//                        .forward(-1.9)
//                        .build();
//                drive.followTrajectorySequence(driveZone1);
//                telemetry.addData("Cone Position:", "LEFT");
//                break;
//            case "CENTER":
//                TrajectorySequence driveZone2 = drive.trajectorySequenceBuilder(startPos)
//                        .lineToLinearHeading(new Pose2d(0, -2.25, Math.toRadians(90)))
//                        .build();
//                drive.followTrajectorySequence(driveZone2);
//                telemetry.addData("Cone Position:", "CENTER");
//                break;
//            default:
//                TrajectorySequence driveZone3 = drive.trajectorySequenceBuilder(startPos)
//                        .lineToLinearHeading(new Pose2d(0, -2.4, Math.toRadians(90)))
//                        .turn(Math.toRadians(3.2))
//                        .forward(1.9)
//                        .build();
//                drive.followTrajectorySequence(driveZone3);
//                telemetry.addData("Cone Position:", "RIGHT");
//                break;
//        }
//
//    }
//
//}
//
//
