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
    int[] horizontalSlideList = {500, 500, 510, 510, 510};

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

        TrajectorySequence cone0 = drive.trajectorySequenceBuilder(startPose)
                .back(50)
//                .turn(Math.toRadians(-57))
//                .back(3)
                .splineToLinearHeading(new Pose2d(-31, -8, Math.toRadians(-140)), Math.toRadians(180))
                .build();

        TrajectorySequence intake0 = drive.trajectorySequenceBuilder(cone0.end())
				.splineToLinearHeading(new Pose2d(-45, -11, Math.toRadians(180)), Math.toRadians(0))
                .build();

        TrajectorySequence outtake = drive.trajectorySequenceBuilder(intake0.end())
				.splineToLinearHeading(new Pose2d(-34, -8, Math.toRadians(-140)), Math.toRadians(180))
                .build( );

        TrajectorySequence intake = drive.trajectorySequenceBuilder(outtake.end())
                .splineToLinearHeading(new Pose2d(-45, -11, Math.toRadians(180)), Math.toRadians(0))
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

        waitForStart();
        if (isStopRequested()) return;
        telemetry.update();

        drive.followTrajectorySequence(cone0);
        deposit(0);

        drive.followTrajectorySequence(intake0);
        sleep(450);
        intake(5);
        drive.followTrajectorySequence(outtake);
        deposit(5);

        drive.followTrajectorySequence(intake);
        sleep(450);
        intake(4);
        drive.followTrajectorySequence(outtake);
        deposit(4);

        drive.followTrajectorySequence(intake);
        sleep(450);
        intake(3);
        drive.followTrajectorySequence(outtake);
        deposit(3);



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
                drive.intakeAngle.setPosition(intakeAngleList[4]);
                drive.clawAngle.setPosition(clawAngleList[4]);
                drive.clawRotate.setPosition(clawRotate1);
                clawOpen();
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


