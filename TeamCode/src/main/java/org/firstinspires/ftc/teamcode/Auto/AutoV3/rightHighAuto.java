package org.firstinspires.ftc.teamcode.Auto.AutoV3;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Commands.commands.High.cycleHighCommand;
import org.firstinspires.ftc.teamcode.Commands.subsystem.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.Commands.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Commands.commands.liftCommand;
import org.firstinspires.ftc.teamcode.Commands.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Settings.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.Settings.OpenCVPipeline;
import org.firstinspires.ftc.teamcode.Settings.drive.HWMap;
import org.firstinspires.ftc.teamcode.Settings.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "rightHighAuto", group = "auto")
@Config
//@Disabled
public class rightHighAuto extends LinearOpMode {
    HWMap drive;
    public static double toPoleBack = 48;
    public static double toPoleLineX = 34;
    public static double toPoleLineY = -4;
    public static double toPoleLineH = -18;

    public static double parkCenterLineX = 33;
    public static double parkCenterLineY = -16.5;
    public static double parkCenterLineH = 0;

    public static double parkLeftMove = 23;
    public static double parkLeftTurn = 90;

    public static double parkRightMove = 23;
    public static double parkRightTurn = 180;

    public static double goPark = 25.5;

    private ElapsedTime timer = new ElapsedTime();;

    // VISION
    OpenCvCamera camera;
    String CVconePos = "CENTER";
    String ATconePos = "NOT_SET";
    String conePos = "CENTER";
    boolean ATLock = true;

    @Override
    public void runOpMode() {
        // Camera Init
        int cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );

        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Loading Pipeline
        OpenCVPipeline visionPipeline = new OpenCVPipeline();
        AprilTagPipeline aprilTagDetectionPipeline = new AprilTagPipeline();

        // Start Streaming
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {

            }
        });

        // Stream Camera Onto Dash
        FtcDashboard.getInstance().startCameraStream(camera, 30);

        // Output Log
        telemetry.addData("Status", "Pipeline Initializing");
        telemetry.update();

        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap);
        LiftSubsystem liftSubsystem = new LiftSubsystem(hardwareMap);
        ExtendSubsystem extendSubsystem = new ExtendSubsystem(hardwareMap);
        drive = new HWMap(hardwareMap);

        Pose2d startPose = new Pose2d(34, -72 + (15.5 / 2), Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        TrajectorySequence toPole = drive.trajectorySequenceBuilder(startPose)
                .back(toPoleBack)
                .lineToLinearHeading(new Pose2d(toPoleLineX, toPoleLineY, Math.toRadians(toPoleLineH)))
                .build();

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(toPole.end())
                .setAccelConstraint(drive.getAccelerationConstraint(50))
                .setVelConstraint(drive.getVelocityConstraint(50, 40, 13.6))
                .setTurnConstraint(40, 40)
                .lineToLinearHeading(new Pose2d(parkCenterLineX, parkCenterLineY, Math.toRadians(parkCenterLineH)))
                .lineToLinearHeading(new Pose2d(parkCenterLineX - parkLeftMove, parkCenterLineY, Math.toRadians(parkLeftTurn)))
                .build();

        TrajectorySequence parkCenter = drive.trajectorySequenceBuilder(toPole.end())
                .setAccelConstraint(drive.getAccelerationConstraint(50))
                .setVelConstraint(drive.getVelocityConstraint(50, 40, 13.6))
                .lineToLinearHeading(new Pose2d(32.5, -25, Math.toRadians(270)))
                .build();

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(toPole.end())
                .setAccelConstraint(drive.getAccelerationConstraint(50))
                .setVelConstraint(drive.getVelocityConstraint(50, 40, 13.6))
                .setTurnConstraint(40, 40)
                .lineToLinearHeading(new Pose2d(parkCenterLineX, parkCenterLineY + 4, Math.toRadians(parkCenterLineH)))
                .lineToLinearHeading(new Pose2d(parkCenterLineX + parkRightMove + 2, parkCenterLineY + 3, Math.toRadians(parkRightTurn)))
                .build();

        CommandScheduler.getInstance().reset();

        // Start Auto
        while (!isStarted() && !isStopRequested()) {
            camera.setPipeline(visionPipeline);
            telemetry.addData("CV Position", visionPipeline.getPosition());
            telemetry.addData("CV Analysis", visionPipeline.getAnalysis());
            sleep(300);

            camera.setPipeline(aprilTagDetectionPipeline);
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
            if(ATLock){
                if (detections != null) {
                    if (detections.size() != 0) {
                        int aprilTagID = detections.get(0).id;
                        switch (aprilTagID){
                            case 1:
                                ATconePos = "LEFT";
                                break;
                            case 2:
                                ATconePos = "CENTER";
                                break;
                            default:
                                ATconePos = "RIGHT";
                                break;
                        }
                        ATLock = false;
                    }
                }
            }
            telemetry.addData("AT Position", ATconePos);
            sleep(500);
            telemetry.update();
        }

        CVconePos = visionPipeline.getPosition().name();

        if(ATconePos == "NOT_SET"){
            conePos = CVconePos;
        } else {
            conePos = ATconePos;
        }

        telemetry.addData("Cone Vision", conePos);
        telemetry.addData("Analysis", visionPipeline.getAnalysis());
        telemetry.update();
        FtcDashboard.getInstance().stopCameraStream();

        timer.reset();
        drive.followTrajectorySequence(toPole);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new cycleHighCommand(intakeSubsystem, liftSubsystem, extendSubsystem, 5),
                        new cycleHighCommand(intakeSubsystem, liftSubsystem, extendSubsystem, 4),
                        new cycleHighCommand(intakeSubsystem, liftSubsystem, extendSubsystem, 3),
                        new cycleHighCommand(intakeSubsystem, liftSubsystem, extendSubsystem, 2),
                        new cycleHighCommand(intakeSubsystem, liftSubsystem, extendSubsystem, 1),
                        new InstantCommand(() -> intakeSubsystem.down(0)),
                        new liftCommand(liftSubsystem, "HIGH", 1200),
                        new WaitCommand(100),
                        new liftCommand(liftSubsystem, "BOTTOM", 1000)
                        )
        );

        while (opModeIsActive() && timer.seconds() < goPark) {
            CommandScheduler.getInstance().run();
            liftSubsystem.loop();
            extendSubsystem.loop();
            telemetry.addData("Lift", liftSubsystem.position());
            telemetry.addData("Extend", extendSubsystem.position());
            telemetry.addData("Distance", extendSubsystem.distance());
            telemetry.update();
        }
        while(opModeIsActive() && timer.seconds() < goPark + 0.5){
            liftSubsystem.pullDown();
            extendSubsystem.pullIn();
            liftSubsystem.loop();
            extendSubsystem.loop();
            intakeSubsystem.down(0);
        }

        switch (conePos) {
            case "LEFT":
                drive.followTrajectorySequence(parkLeft);
                telemetry.addData("Cone Position:", "LEFT");
                break;
            case "CENTER":
                drive.followTrajectorySequence(parkCenter);
                telemetry.addData("Cone Position:", "CENTER");
                break;
            case "RIGHT":
                drive.followTrajectorySequence(parkRight);
                telemetry.addData("Cone Position:", "CENTER");
                break;

            default:
                drive.followTrajectorySequence(parkCenter);
                telemetry.addData("Cone Position:", "DEFAULT");
                break;
        }
    }

}



