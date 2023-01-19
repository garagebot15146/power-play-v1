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
    int[] horizontalSlideList = {470, 470, 480, 490, 490};

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
        int outtakeX = -35;
        int outtakeY = -3;
        double wait1 = 0.35;
        double wait2 = 0.3;

        TrajectorySequence outtake0 = drive.trajectorySequenceBuilder(startPose)
                .back(50)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> liftAsync(1950, 2000))
                .splineToLinearHeading(new Pose2d(-31, -8, Math.toRadians(-140)), Math.toRadians(180))
                .waitSeconds(wait1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> liftAsync(0, 2300))
                .waitSeconds(wait2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> clawReset(5))
                .build();

        TrajectorySequence intake5 = drive.trajectorySequenceBuilder(outtake0.end())
				.splineToLinearHeading(new Pose2d(-45, -11, Math.toRadians(180)), Math.toRadians(0))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake(5))
                .waitSeconds(0.1)
                .build();

        TrajectorySequence outtake5 = drive.trajectorySequenceBuilder(intake5.end())
                .UNSTABLE_addTemporalMarkerOffset(0.06, () -> liftAsync(1950, 2000))
                .splineToLinearHeading(new Pose2d(outtakeX, outtakeY, Math.toRadians(-140)), Math.toRadians(180))
                .waitSeconds(wait1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> liftAsync(0, 2300))
                .waitSeconds(wait2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> clawReset(4))
                .build();

        TrajectorySequence intake4 = drive.trajectorySequenceBuilder(outtake5.end())
                .splineToLinearHeading(new Pose2d(-45, -11, Math.toRadians(180)), Math.toRadians(0))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake(4))
                .waitSeconds(0.1)
                .build();

        TrajectorySequence outtake4 = drive.trajectorySequenceBuilder(intake4.end())
                .UNSTABLE_addTemporalMarkerOffset(0.06, () -> liftAsync(1950, 2000))
                .splineToLinearHeading(new Pose2d(outtakeX, outtakeY, Math.toRadians(-140)), Math.toRadians(180))
                .waitSeconds(wait1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> liftAsync(0, 2300))
                .waitSeconds(wait2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> clawReset(3))
                .build();

        TrajectorySequence intake3 = drive.trajectorySequenceBuilder(outtake4.end())
                .splineToLinearHeading(new Pose2d(-45, -11, Math.toRadians(180)), Math.toRadians(0))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake(3))
                .waitSeconds(0.1)
                .build();

        TrajectorySequence outtake3 = drive.trajectorySequenceBuilder(intake3.end())
                .UNSTABLE_addTemporalMarkerOffset(0.06, () -> liftAsync(1950, 2000))
                .splineToLinearHeading(new Pose2d(outtakeX, outtakeY, Math.toRadians(-140)), Math.toRadians(180))
                .waitSeconds(wait1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> liftAsync(0, 2300))
                .waitSeconds(wait2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> clawReset(2))
                .build();


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
        drive.followTrajectorySequence(outtake5);

        drive.followTrajectorySequence(intake4);
        drive.followTrajectorySequence(outtake4);

        drive.followTrajectorySequence(intake3);
        drive.followTrajectorySequence(outtake3);


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

    public void extend(int ticks, int speed) {
        drive.leftHorizontalSlide.setTargetPosition(ticks);
        drive.rightHorizontalSlide.setTargetPosition(ticks);
        runtime.reset();

        drive.leftHorizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightHorizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.leftHorizontalSlide.setVelocity(speed);
        drive.rightHorizontalSlide.setVelocity(speed);

//        while (opModeIsActive() && Math.abs(drive.leftHorizontalSlide.getCurrentPosition() - ticks) > 40) {
//
//        }
    }

    public void lift(int ticks) {
        drive.leftVerticalSlide.setTargetPosition(ticks);
        drive.rightVerticalSlide.setTargetPosition(ticks);

        drive.leftVerticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightVerticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.leftVerticalSlide.setVelocity(6400);
        drive.rightVerticalSlide.setVelocity(6400);

        while (opModeIsActive() && Math.abs(drive.leftVerticalSlide.getCurrentPosition() - ticks) > 5) {
            telemetry.addData("Encoder", drive.leftVerticalSlide.getCurrentPosition());
            telemetry.update();
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
                extend(horizontalSlideList[0], 3400);
                sleep(400);
                transfer();
                break;
            case 2:
                extend(horizontalSlideList[1], 3400);
                sleep(400);
                transfer();
                break;
            case 3:
                extend(horizontalSlideList[2], 3400);
                sleep(400);
                transfer();
                break;
            case 4:
                extend(horizontalSlideList[3], 3400);
                sleep(400);
                transfer();
                break;
            case 5:
                extend(horizontalSlideList[4], 3400);
                sleep(400);
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
        if (!(cones == 0 || cones == 1)) {
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
        extend(0, 2000);
        sleep(700);
        clawOpen();
        sleep(500);
    }

    public void clawReset(int cones) {
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


