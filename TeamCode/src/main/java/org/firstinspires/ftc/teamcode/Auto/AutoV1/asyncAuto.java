//package org.firstinspires.ftc.teamcode.Auto;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Settings.drive.HWMap;
//import org.firstinspires.ftc.teamcode.Settings.trajectorysequence.TrajectorySequence;
//
//@Config
//@Autonomous(name = "asyncAuto", group = "auto")
////@Disabled
//public class asyncAuto extends LinearOpMode {
//    HWMap drive;
//
//    // Runtime
//    private ElapsedTime runtime = new ElapsedTime();
//    private ElapsedTime threshold = new ElapsedTime();
//
//
//    //PID
//    PIDController liftController;
//    public static double pL = 0.01, iL = 0, dL = 0;
//
//    PIDController extendController;
//    public static double pE = 0.01, iE = 0, dE = 0;
//
//    // Cone Stack
//    double[] intakeAngleList = {0.64, 0.64, 0.55, 0.46, 0.4};
//    double[] clawAngleList = {0.92, 0.92, 0.91, 0.91, 0.91};
//    int[] horizontalSlideList = {1010, 970, 1010, 1065, 1120};
//
//    // Positions
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
//        // SERVOS
//        drive.intakeAngle.setPosition(intakeAngle3);
//        drive.clawRotate.setPosition(clawRotate1);
//        drive.clawAngle.setPosition(clawAngle3);
//        drive.claw.setPosition(claw1);
//
//
//        liftController = new PIDController(pL, iL, dL);
//        extendController = new PIDController(pE, iE, dE);
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        waitForStart();
//        if (isStopRequested()) return;
//        telemetry.update();
//
//        slides(1000, 1190, 3.5);
//        sleep(500);
//        slides(0, 0, 3.5);
//    }
//
//    public void slides(int extendTarget, int liftTarget, double timeout) {
//        threshold.reset();
//
//        extendController.setPID(pE, iE, dE);
//        extendController.setSetPoint(extendTarget);
//
//        liftController.setPID(pL, iL, dL);
//        liftController.setSetPoint(liftTarget);
//
//        boolean lock = false;
//        while (!(extendController.atSetPoint() && liftController.atSetPoint()) && (threshold.seconds() < timeout)) {
//            int extendPos = drive.leftHorizontalSlide.getCurrentPosition();
//            int liftPos = drive.leftVerticalSlide.getCurrentPosition();
//
//            double pidExtend = extendController.calculate(extendPos, extendTarget);
//            double pidLift = liftController.calculate(liftPos, liftTarget);
//
//            drive.leftHorizontalSlide.setPower(pidExtend);
//            drive.rightHorizontalSlide.setPower(pidExtend);
//
//            drive.leftVerticalSlide.setPower(pidLift);
//            drive.rightVerticalSlide.setPower(pidLift);
//
//            telemetry.addData("Extend Pos", extendPos);
//            telemetry.addData("Extend Target", extendTarget);
//            telemetry.addData("Lift Pos", liftPos);
//            telemetry.addData("Lift Target", liftTarget);
//            telemetry.addData("Extend Set Point", extendController.atSetPoint());
//            telemetry.addData("Lift Set Point", liftController.atSetPoint());
//            telemetry.update();
//            if (Math.abs(liftTarget - liftPos) < 15) {
//                if (!lock) {
//                    runtime.reset();
//                    lock = true;
//                }
//                if (runtime.seconds() > 0.1) {
//                    break;
//                }
//            }
//        }
//        drive.leftHorizontalSlide.setPower(0);
//        drive.rightHorizontalSlide.setPower(0);
//        drive.leftVerticalSlide.setPower(0);
//        drive.rightVerticalSlide.setPower(0);
//    }
//
//
//    public void extend(int target) {
//        while (opModeIsActive()) {
//            extendController.setPID(pE, iE, dE);
//            int extendPos = drive.leftHorizontalSlide.getCurrentPosition();
//            int extendTarget = 1300;
//            double pidExtend = extendController.calculate(extendPos, extendTarget);
//
//            drive.leftHorizontalSlide.setPower(pidExtend);
//            drive.rightHorizontalSlide.setPower(pidExtend);
//
//            telemetry.addData("Extend Pos", extendPos);
//            telemetry.addData("Extend Target", extendTarget);
//            telemetry.update();
//        }
//    }
//
//    public void lift(int target) {
//        while (opModeIsActive()) {
//            liftController.setPID(pL, iL, dL);
//            int liftPos = drive.leftVerticalSlide.getCurrentPosition();
//            int liftTarget = 1000;
//            double pidLift = liftController.calculate(liftPos, liftTarget);
//
//            drive.leftVerticalSlide.setPower(pidLift);
//            drive.rightVerticalSlide.setPower(pidLift);
//
//            telemetry.addData("Lift Pos", liftPos);
//            telemetry.addData("Lift Target", liftTarget);
//            telemetry.update();
//        }
//    }
//
//}
//
//
