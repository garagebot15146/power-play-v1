//package org.firstinspires.ftc.teamcode.Auto;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Settings.drive.HWMap;
//import org.firstinspires.ftc.teamcode.Settings.trajectorysequence.TrajectorySequence;
//
//@Autonomous(name = "leftAuto", group = "auto")
////@Disabled
//public class leftAuto extends LinearOpMode {
//    HWMap drive;
//
//    // Runtime
//    private ElapsedTime runtime = new ElapsedTime();
//
//    // Cone Stack
//    double[] intakeAngleList = {0.64, 0.64, 0.55, 0.46, 0.4};
//    double[] clawAngleList = {0.92, 0.92, 0.91, 0.91, 0.91};
//    int[] horizontalSlideList = {1010, 970, 1010, 1065, 1120};
//
//    // Servo Positions
//    double claw1 = 0.7;
//    double claw2 = 1;
//
//    double clawAngle1 = 0.95;
//    double clawAngle2 = 0.37;
//    double clawAngle3 = 0.5;
//
//
//    double intakeAngle1 = 0.9;
//    double intakeAngle2 = 0.1;
//    double intakeAngle3 = 0.17;
//
//    double clawRotate1 = 0.74;
//    double clawRotate2 = 0;
//
//    @Override
//    public void runOpMode() {
//        drive = new HWMap(hardwareMap);
//
//        Pose2d startPose = new Pose2d(-34, -72 + (15.5 / 2), Math.toRadians(270));
//        drive.setPoseEstimate(startPose);
//
//        String conePos = "LEFT";
//
//        TrajectorySequence toPole = drive.trajectorySequenceBuilder(startPose)
//                .back(48)
//                .lineToLinearHeading(new Pose2d(-31.8, -4, Math.toRadians(188)))
//                .build();
//
//
//        // Horizontal Slides
//        drive.leftHorizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        drive.rightHorizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        drive.leftHorizontalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        drive.rightHorizontalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        // Vertical Slides
//        drive.leftVerticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        drive.rightVerticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        drive.leftVerticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        drive.rightVerticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        // Initialize Servos
//        drive.intakeAngle.setPosition(intakeAngle3);
//        drive.clawRotate.setPosition(clawRotate1);
//        drive.clawAngle.setPosition(clawAngle3);
//        clawOpen();
//
//        waitForStart();
//        if (isStopRequested()) return;
//        telemetry.update();
//
//        // AUTO
//        drive.followTrajectorySequence(toPole);
//        sleep(400);
//        cycle();
//    }
//
//    public void cycle(){
//        deposit(0);
//
//        intake(5);
//        deposit(5);
//
//        intake(4);
//        deposit3(4);
//
//        intake(3);
//        deposit3(3);
//
//        intake(2);
//        deposit3(2);
//
////        intake(1);
////        deposit(1);
//
//        sleep(2000);
//    }
//
//    public void extend(int ticks, double power) {
//        drive.leftHorizontalSlide.setTargetPosition(ticks);
//        drive.rightHorizontalSlide.setTargetPosition(ticks);
//        runtime.reset();
//
//        drive.leftHorizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        drive.rightHorizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        drive.leftHorizontalSlide.setPower(power);
//        drive.rightHorizontalSlide.setPower(power);
//
//        while (opModeIsActive() && Math.abs(drive.leftHorizontalSlide.getCurrentPosition() - ticks) > 10) {
//            telemetry.addData("Extend Encoder", drive.leftHorizontalSlide.getCurrentPosition());
//            telemetry.update();
//        }
//    }
//
//    public void liftUp() {
//        drive.stabilizer.setPosition(0);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.4)) {
//            drive.leftVerticalSlide.setPower(0.95);
//            drive.rightVerticalSlide.setPower(0.95);
//        }
//        drive.leftVerticalSlide.setPower(0);
//        drive.rightVerticalSlide.setPower(0);
//        drive.stabilizer.setPosition(0.2);
//    }
//
//    public void liftDown() {
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.5)) {
//            drive.leftVerticalSlide.setPower(-0.8);
//            drive.rightVerticalSlide.setPower(-0.8);
//        }
//        drive.leftVerticalSlide.setPower(0);
//        drive.rightVerticalSlide.setPower(0);
//    }
//
//    public void liftUp2() {
//        drive.stabilizer.setPosition(0);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.4)) {
//            drive.leftVerticalSlide.setPower(1);
//            drive.rightVerticalSlide.setPower(1);
//        }
//        drive.leftVerticalSlide.setPower(0);
//        drive.rightVerticalSlide.setPower(0);
//        drive.stabilizer.setPosition(0.2);
//    }
//
//    public void liftDown2() {
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.5)) {
//            drive.leftVerticalSlide.setPower(-1);
//            drive.rightVerticalSlide.setPower(-1);
//        }
//        drive.leftVerticalSlide.setPower(0);
//        drive.rightVerticalSlide.setPower(0);
//    }
//
//    public void liftUp3() {
//        drive.stabilizer.setPosition(0);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.2)) {
//            drive.leftVerticalSlide.setPower(1);
//            drive.rightVerticalSlide.setPower(1);
//        }
//        drive.leftVerticalSlide.setPower(0);
//        drive.rightVerticalSlide.setPower(0);
//        drive.stabilizer.setPosition(0.2);
//    }
//
//    public void liftDown3() {
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.5)) {
//            drive.leftVerticalSlide.setVelocity(-6000);
//            drive.rightVerticalSlide.setVelocity(-6000);
//        }
//        drive.leftVerticalSlide.setVelocity(0);
//        drive.rightVerticalSlide.setVelocity(0);
//    }
//
//    public void liftAsync(int ticks, double power) {
//        drive.leftVerticalSlide.setTargetPosition(ticks);
//        drive.rightVerticalSlide.setTargetPosition(ticks);
//
//        drive.leftVerticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        drive.rightVerticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        drive.leftVerticalSlide.setPower(power);
//        drive.rightVerticalSlide.setPower(power);
//    }
//
//    public void clawOpen() {
//        drive.claw.setPosition(claw1);
//    }
//
//    public void clawClose() {
//        drive.claw.setPosition(claw2);
//    }
//
//    public void intake(int cones) {
//        double power = 0.8;
//        switch (cones) {
//            case 1:
//                extend(horizontalSlideList[0], power);
//                sleep(130);
//                transfer();
//                break;
//            case 2:
//                extend(horizontalSlideList[1], 1);
//                sleep(130);
//                transfer();
//                break;
//            case 3:
//                extend(horizontalSlideList[2], power);
//                sleep(130);
//                transfer();
//                break;
//            case 4:
//                extend(horizontalSlideList[3], power);
//                sleep(130);
//                transfer();
//                break;
//            case 5:
//                clawReset(5);
//                extend(horizontalSlideList[4], power);
//                sleep(130);
//                transfer();
//                break;
//            default:
//                break;
//        }
//        telemetry.addData("Cones:", cones);
//    }
//
//    public void deposit(int cones) {
//        liftUp();
//        liftDown();
//        if (!(cones == 0 || cones == 1 || cones == 2)) {
//            clawReset(cones - 1);
//        }
//    }
//
//    public void deposit2(int cones) {
//        liftUp2();
//        liftDown2();
//        if (!(cones == 0 || cones == 1 || cones == 2)) {
//            clawReset(cones - 1);
//        }
//    }
//
//    public void deposit3(int cones) {
//        liftUp3();
//        liftDown3();
//        if (!(cones == 0 || cones == 1 || cones == 2)) {
//            clawReset(cones - 1);
//        }
//    }
//
//    public void transfer() {
//        clawClose();
//        sleep(300);
//        drive.clawAngle.setPosition(clawAngle3);
//        sleep(600);
//        drive.intakeAngle.setPosition(intakeAngle2);
//        drive.clawRotate.setPosition(clawRotate2);
//        extend(0, 1);
//        sleep(150);
//        drive.clawAngle.setPosition(clawAngle2);
//        sleep(200);
//        clawOpen();
//        sleep(150);
//    }
//
//    public void clawReset(int cones) {
//        switch (cones) {
//            case 1:
//                drive.intakeAngle.setPosition(intakeAngleList[0]);
//                drive.clawAngle.setPosition(clawAngleList[0]);
//                drive.clawRotate.setPosition(clawRotate1);
//                break;
//            case 2:
//                drive.intakeAngle.setPosition(intakeAngleList[1]);
//                drive.clawAngle.setPosition(clawAngleList[1]);
//                drive.clawRotate.setPosition(clawRotate1);
//                break;
//            case 3:
//                drive.intakeAngle.setPosition(intakeAngleList[2]);
//                drive.clawAngle.setPosition(clawAngleList[2]);
//                drive.clawRotate.setPosition(clawRotate1);
//                break;
//            case 4:
//                drive.intakeAngle.setPosition(intakeAngleList[3]);
//                drive.clawAngle.setPosition(clawAngleList[3]);
//                drive.clawRotate.setPosition(clawRotate1);
//                break;
//            case 5:
//                drive.intakeAngle.setPosition(intakeAngleList[4]);
//                drive.clawAngle.setPosition(clawAngleList[4]);
//                drive.clawRotate.setPosition(clawRotate1);
//                break;
//            default:
//                break;
//        }
//    }
//
//}