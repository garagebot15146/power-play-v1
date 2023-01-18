package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Settings.drive.HWMap;
import org.firstinspires.ftc.teamcode.Settings.trajectorysequence.TrajectorySequence;

@Autonomous(name = "leftBlue", group = "auto")
//@Disabled
public class leftBlue extends LinearOpMode {
    HWMap drive;

    // Runtime
    private ElapsedTime runtime = new ElapsedTime();

    // Cone Stack
    double[] intakeAngleList = {0.95, 0.87, 0.79, 0.71, 0.65};
    double[] clawAngleList = {0.95, 0.95, 0.95, 0.94, 0.94};
    int[] horizontalSlideList = {480, 480, 485, 490, 490};

    // Servo Positions
    double claw1 = 1;
    double claw2 = 0.7;

    double clawAngle1 = 0.95;
    double clawAngle2 = 0.42;
    double clawAngle3 = 0.7;

    double intakeAngle1 = 0.95;
    double intakeAngle2 = 0.11;
    double intakeAngle3 = 0.15;

    double clawRotate1 = 0;
    double clawRotate2 = 0.74;

    @Override
    public void runOpMode() {
        drive = new HWMap(hardwareMap);

        Pose2d startPose = new Pose2d(-34, -72 + (15.5 / 2), Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        String conePos = "LEFT";

        TrajectorySequence outtake0 = drive.trajectorySequenceBuilder(startPose)
                .back(50)
                .UNSTABLE_addTemporalMarkerOffset(0, () ->  liftAsync(1695, 1700))
                .splineToLinearHeading(new Pose2d(-31, -8, Math.toRadians(-140)), Math.toRadians(180))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () ->  liftAsync(0, 6300))
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () ->  clawReset(5))
                .build();

        TrajectorySequence intake5 = drive.trajectorySequenceBuilder(outtake0.end())
				.splineToLinearHeading(new Pose2d(-45, -11, Math.toRadians(180)), Math.toRadians(0))
                .build();

        TrajectorySequence outtake5 = drive.trajectorySequenceBuilder(intake5.end())
                .UNSTABLE_addTemporalMarkerOffset(0, () ->  liftAsync(1695, 1700))
                .splineToLinearHeading(new Pose2d(-31, -8, Math.toRadians(-140)), Math.toRadians(180))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () ->  liftAsync(0, 6300))
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () ->  clawReset(4))
                .build( );

        TrajectorySequence intake4 = drive.trajectorySequenceBuilder(outtake5.end())
                .splineToLinearHeading(new Pose2d(-45, -11, Math.toRadians(180)), Math.toRadians(0))
                .build();

        TrajectorySequence outtake4 = drive.trajectorySequenceBuilder(intake4.end())
                .UNSTABLE_addTemporalMarkerOffset(0, () ->  liftAsync(1695, 1700))
                .splineToLinearHeading(new Pose2d(-31, -8, Math.toRadians(-140)), Math.toRadians(180))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () ->  liftAsync(0, 6300))
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () ->  clawReset(3))
                .build( );

        TrajectorySequence intake3 = drive.trajectorySequenceBuilder(outtake4.end())
                .splineToLinearHeading(new Pose2d(-45, -11, Math.toRadians(180)), Math.toRadians(0))
                .build();

        TrajectorySequence outtake3 = drive.trajectorySequenceBuilder(intake3.end())
                .UNSTABLE_addTemporalMarkerOffset(0, () ->  liftAsync(1695, 1700))
                .splineToLinearHeading(new Pose2d(-31, -8, Math.toRadians(-140)), Math.toRadians(180))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () ->  liftAsync(0, 6300))
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () ->  clawReset(2))
                .build( );

        TrajectorySequence intake2 = drive.trajectorySequenceBuilder(outtake3.end())
                .splineToLinearHeading(new Pose2d(-45, -11, Math.toRadians(180)), Math.toRadians(0))
                .build();

        TrajectorySequence outtake2 = drive.trajectorySequenceBuilder(intake2.end())
                .UNSTABLE_addTemporalMarkerOffset(0, () ->  liftAsync(1695, 1700))
                .splineToLinearHeading(new Pose2d(-31, -8, Math.toRadians(-140)), Math.toRadians(180))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () ->  liftAsync(0, 6300))
                .waitSeconds(0.3)
                .build( );


        // Horizontal Slides
        drive.leftHorizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightHorizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.leftHorizontalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.rightHorizontalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Vertical Slides
        drive.leftVerticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightVerticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.leftVerticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.rightVerticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize Servos
        drive.intakeAngle.setPosition(intakeAngle3);
        drive.clawRotate.setPosition(clawRotate1);
        drive.clawAngle.setPosition(clawAngle3);
        clawOpen();

        waitForStart();
        if (isStopRequested()) return;
        telemetry.update();

        drive.followTrajectorySequence(outtake0);

        drive.followTrajectorySequence(intake5);
        sleep(450);
        intake(5);
        drive.followTrajectorySequence(outtake5);

        drive.followTrajectorySequence(intake4);
        sleep(450);
        intake(4);
        drive.followTrajectorySequence(outtake4);

        drive.followTrajectorySequence(intake3);
        sleep(450);
        intake(3);
        drive.followTrajectorySequence(outtake3);

        drive.followTrajectorySequence(intake2);
        sleep(450);
        intake(2);
        drive.followTrajectorySequence(outtake2);


//        switch (conePos) {
//            case "LEFT":
//                deposit();
//                intake(1);
//                sleep(3000);
//                sleep(2000);
////                drive.followTrajectorySequence(driveZone1);
//                telemetry.addData("Cone Position:", "LEFT");
//                break;
//            case "CENTER":
//                telemetry.addData("Cone Position:", "CENTER");
//                break;
//            default:
//                telemetry.addData("Cone Position:", "RIGHT");
//                break;
//        }

    }

    public void extend(int ticks) {
        drive.leftHorizontalSlide.setTargetPosition(ticks);
        drive.rightHorizontalSlide.setTargetPosition(ticks);
        runtime.reset();

        drive.leftHorizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightHorizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.leftHorizontalSlide.setVelocity(2000);
        drive.rightHorizontalSlide.setVelocity(2000);

        while(opModeIsActive() && Math.abs(drive.leftHorizontalSlide.getCurrentPosition() - ticks) > 40){

        }
    }

    public void lift(int ticks) {
        drive.leftVerticalSlide.setTargetPosition(ticks);
        drive.rightVerticalSlide.setTargetPosition(ticks);

        drive.leftVerticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightVerticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.leftVerticalSlide.setVelocity(6400);
        drive.rightVerticalSlide.setVelocity(6400);

        while(opModeIsActive() && Math.abs(drive.leftVerticalSlide.getCurrentPosition() - ticks) > 40){

        }
    }

    public void liftAsync(int ticks, int speed) {
        drive.leftVerticalSlide.setTargetPosition(ticks);
        drive.rightVerticalSlide.setTargetPosition(ticks);

        drive.leftVerticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightVerticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.leftVerticalSlide.setVelocity(speed);
        drive.rightVerticalSlide.setVelocity(speed);
    }

    public void clawOpen() {
        drive.claw.setPosition(claw1);
    }

    public void clawClose() {
        drive.claw.setPosition(claw2);
    }

    public void intake(int cones) {
        switch (cones) {
            case 1:
                extend(horizontalSlideList[0]);
                transfer();
                break;
            case 2:
                extend(horizontalSlideList[1]);
                transfer();
                break;
            case 3:
                extend(horizontalSlideList[2]);
                transfer();
                break;
            case 4:
                extend(horizontalSlideList[3]);
                transfer();
                break;
            case 5:
//                drive.intakeAngle.setPosition(intakeAngleList[4]);
//                drive.clawAngle.setPosition(clawAngleList[4]);
//                drive.clawRotate.setPosition(clawRotate1);
                extend(horizontalSlideList[4]);
                transfer();
                break;
            default:
                break;
        }
        telemetry.addData("Cones:", cones);
    }

    public void deposit(int cones) {
        lift(1680);
        sleep(450);
        lift(0);
        if(!(cones == 0 || cones == 1)){
            clawReset(cones - 1);
        }
    }

    public void transfer() {
        clawClose();
        sleep(500);
        drive.intakeAngle.setPosition(intakeAngle2);
        sleep(300);
        drive.clawRotate.setPosition(clawRotate2);
        drive.clawAngle.setPosition(clawAngle2);
        extend(0);
        sleep(300);
        clawOpen();
        sleep(500);
    }

    public void clawReset(int cones){
        switch (cones) {
            case 1:
                drive.intakeAngle.setPosition(intakeAngleList[0]);
                drive.clawAngle.setPosition(clawAngleList[0]);
                drive.clawRotate.setPosition(clawRotate1);
                break;
            case 2:
                drive.intakeAngle.setPosition(intakeAngleList[1]);
                drive.clawAngle.setPosition(clawAngleList[1]);
                drive.clawRotate.setPosition(clawRotate1);
                break;
            case 3:
                drive.intakeAngle.setPosition(intakeAngleList[2]);
                drive.clawAngle.setPosition(clawAngleList[2]);
                drive.clawRotate.setPosition(clawRotate1);
                break;
            case 4:
                drive.intakeAngle.setPosition(intakeAngleList[3]);
                drive.clawAngle.setPosition(clawAngleList[3]);
                drive.clawRotate.setPosition(clawRotate1);
                break;
            case 5:
                drive.intakeAngle.setPosition(intakeAngleList[4]);
                drive.clawAngle.setPosition(clawAngleList[4]);
                drive.clawRotate.setPosition(clawRotate1);
                break;
            default:
                break;
        }
    }

}


