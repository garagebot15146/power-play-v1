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
    double[] intakeAngleList = {0.95, 0.87, 0.79, 0.71, 0.63};
    double[] clawAngleList = {0.95, 0.95, 0.95, 0.94, 0.94};
    int[] horizontalSlideList = {600, 570, 570, 570, 570};

    // Servo Positions
    double claw1 = 1;
    double claw2 = 0.7;

    double clawAngle1 = 0.95;
    double clawAngle2 = 0.32;
    double clawAngle3 = 0.78;

    double intakeAngle1 = 0.95;
    double intakeAngle2 = 0.04;
    double intakeAngle3 = 0.17;

    double clawRotate1 = 0;
    double clawRotate2 = 0.74;

    @Override
    public void runOpMode() {
        drive = new HWMap(hardwareMap);

        Pose2d startPose = new Pose2d(-34, -72 + (15.5 / 2), Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        String conePos = "LEFT";

//        TrajectorySequence driveZone1 = drive.trajectorySequenceBuilder(startPose)
//                .back(54)
//                .turn(Math.toRadians(-45))
//                .waitSeconds(1)
//
//                //collect and drop cone 2
//                .splineToLinearHeading(new Pose2d(-42, -12, Math.toRadians(180)), Math.toRadians(0))
//                .waitSeconds(1)
//                .splineToLinearHeading(new Pose2d(-34, -8, Math.toRadians(-140)), Math.toRadians(180))
//                .waitSeconds(1)
//
//                //collect and drop cone 3
//                .splineToLinearHeading(new Pose2d(-42, -12, Math.toRadians(180)), Math.toRadians(0))
//                .waitSeconds(1)
//                .splineToLinearHeading(new Pose2d(-34, -8, Math.toRadians(-140)), Math.toRadians(180))
//                .waitSeconds(1)
//
//                //collect and drop cone 4
//                .splineToLinearHeading(new Pose2d(-42, -12, Math.toRadians(180)), Math.toRadians(0))
//                .waitSeconds(1)
//                .splineToLinearHeading(new Pose2d(-34, -8, Math.toRadians(-140)), Math.toRadians(180))
//                .waitSeconds(1)
//
//                //collect and drop cone 5
//                .splineToLinearHeading(new Pose2d(-42, -12, Math.toRadians(180)), Math.toRadians(0))
//                .waitSeconds(1)
//                .splineToLinearHeading(new Pose2d(-34, -8, Math.toRadians(-140)), Math.toRadians(180))
//                .waitSeconds(1)
//
//                //collect and drop cone 6
//                .splineToLinearHeading(new Pose2d(-42, -12, Math.toRadians(180)), Math.toRadians(0))
//                .waitSeconds(1)
//                .splineToLinearHeading(new Pose2d(-34, -8, Math.toRadians(-140)), Math.toRadians(180))
//                .waitSeconds(1)
//                .build( );

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

        waitForStart();
        if (isStopRequested()) return;
        telemetry.update();

        intake(1);
        deposit();
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

    public void extend(int ticks, double timeout) {
        drive.leftHorizontalSlide.setTargetPosition(ticks);
        drive.rightHorizontalSlide.setTargetPosition(ticks);
        runtime.reset();

        drive.leftHorizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightHorizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.leftHorizontalSlide.setVelocity(2000);
        drive.rightHorizontalSlide.setVelocity(2000);

        while (opModeIsActive() && (runtime.seconds() < timeout) && drive.leftHorizontalSlide.isBusy() && drive.rightHorizontalSlide.isBusy()) {
        }

        drive.leftHorizontalSlide.setVelocity(0);
        drive.rightHorizontalSlide.setVelocity(0);

    }

    public void lift(int ticks, double timeout) {
        drive.leftVerticalSlide.setTargetPosition(ticks);
        drive.rightVerticalSlide.setTargetPosition(ticks);
        runtime.reset();

        drive.leftVerticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightVerticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.leftVerticalSlide.setVelocity(3000);
        drive.rightVerticalSlide.setVelocity(3000);

        while (opModeIsActive() && (runtime.seconds() < timeout) && drive.leftVerticalSlide.isBusy() && drive.rightVerticalSlide.isBusy()) {
            telemetry.addData("Left Lift", drive.leftVerticalSlide.getCurrentPosition());
            telemetry.addData("Right Lift", drive.rightVerticalSlide.getCurrentPosition());
            telemetry.update();
        }

        drive.leftVerticalSlide.setVelocity(0);
        drive.rightVerticalSlide.setVelocity(0);
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
                drive.intakeAngle.setPosition(intakeAngleList[0]);
                drive.clawAngle.setPosition(clawAngleList[0]);
                clawOpen();
                extend(horizontalSlideList[0], 4);
                break;
            case 2:
                break;
            case 3:
                break;
            case 4:
                break;
            case 5:
                break;
            default:
                break;
        }
        transfer();
        telemetry.addData("Cones:", cones);
    }

    public void deposit() {
        lift(1590, 4);
        sleep(500);
        lift(0, 4);
    }

    public void transfer() {
        clawClose();
        sleep(400);
        extend(0, 4);
        drive.intakeAngle.setPosition(intakeAngle2);
        drive.clawRotate.setPosition(clawRotate2);
        drive.clawAngle.setPosition(clawAngle2);
        sleep(500);
        clawOpen();
        sleep(500);
        drive.intakeAngle.setPosition(intakeAngle3);
        drive.clawRotate.setPosition(clawRotate1);
        drive.clawAngle.setPosition(clawAngle3);
        sleep(500);
    }

}


