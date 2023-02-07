package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Settings.drive.HWMap;

@Config
@TeleOp(name = "teleOp", group = "Iterative Opmode")
//@Disabled
public class teleOp extends OpMode {

    static HWMap drive;

    // VOLTAGE
    VoltageSensor voltageSensor;
    double voltage;
    ElapsedTime voltageTimer;

    // GAMEPAD
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    // STABILIZER
    int stabilizerVertical = 300;

    // TOGGLE
    String toggleClaw = "init";
    String toggleClawAngle = "init";
    String toggleIntakeAngle = "init";
    String toggleClawRotate = "init";
    String toggleIntake = "init";
    String toggleFlipper = "init";
    String toggleStabilizer = "init";


    // SERVO POSITIONS
    double claw1 = 1;
    double claw2 = 0.7;

    double clawAngle1 = 0.02;
    double clawAngle2 = 0.71;
    double clawAngle3 = 0.27;

    double intakeAngle1 = 0.85;
    double intakeAngle2 = 0.13;
    double intakeAngle3 = 0.31;

    double clawRotate1 = 0.74;
    double clawRotate2 = 0;

    double leftFlipper1 = 1;
    double leftFlipper2 = 0.5;

    double rightFlipper1 = 0;
    double rightFlipper2 = 0.5;

    double stabilizer1 = 0;
    double stabilizer2 = 0.2;

    // STATE MACHINES
    public enum LiftState {
        LIFT_AUTO,
        LIFT_MANUAL
    }

    LiftState liftState = LiftState.LIFT_MANUAL;
    double pidLift = 0;

    // CLOCK
    private ElapsedTime runtime = new ElapsedTime();

    //PID
    PIDController liftController;
    public static int liftMax = 1300;
    public int liftTarget = 0;
    public static double pL = 0.0144, iL = 0.001, dL = 0.0001;

    PIDController extendController;
    public static int extendMax = 1000;
    public int extendTarget = 0;
    public static double pE = 0.005, iE = 0, dE = 0;

    @Override
    public void init() {
        drive = new HWMap(hardwareMap);

        //VOLTAGE
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltage = voltageSensor.getVoltage();
        voltageTimer = new ElapsedTime();
        voltageTimer.reset();

        //PID
        liftController = new PIDController(pL, iL, dL);
        extendController = new PIDController(pE, iE, dE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //HORIZONTAL SLIDES
        drive.leftHorizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightHorizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.leftHorizontalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.rightHorizontalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //VERTICAL SLIDES
        drive.leftVerticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightVerticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.leftVerticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.rightVerticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
        // GAMEPADS
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        //VOLTAGE
        if (voltageTimer.seconds() > 5) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }

        // GAMEPAD A

        // WHEELS
        double normal = 1;
        double slow = 0.6;
        double forward = -gamepad1.left_stick_y;
        double side = gamepad1.left_stick_x; //Positive means right
        double turn = gamepad1.right_stick_x; //Positive means turn right


        double leftFrontPower = forward + side + turn;
        double leftRearPower = forward - side + turn;
        double rightRearPower = forward + side - turn;
        double rightFrontPower = forward - side - turn;

        // Send power to wheel motors
        if (gamepad1.right_trigger > 0.2) {
            drive.leftFront.setPower(leftFrontPower * slow);
            drive.leftRear.setPower(leftRearPower * slow);
            drive.rightRear.setPower(rightRearPower * slow);
            drive.rightFront.setPower(rightFrontPower * slow);
        } else {
            drive.leftFront.setPower(leftFrontPower * normal);
            drive.leftRear.setPower(leftRearPower * normal);
            drive.rightRear.setPower(rightRearPower * normal);
            drive.rightFront.setPower(rightFrontPower * normal);
        }

        // GAMEPAD B

        //LIFT
        switch (liftState) {
            case LIFT_AUTO:
                liftController.setPID(pL, iL, dL);
                int liftPos = drive.leftVerticalSlide.getCurrentPosition();
                if (liftTarget > liftMax) {
                    liftTarget = liftMax;
                } else if (liftTarget < 0) {
                    liftTarget = 0;
                }
                pidLift = liftController.calculate(liftPos, liftTarget);
                drive.leftVerticalSlide.setPower(pidLift);
                drive.rightVerticalSlide.setPower(pidLift);
//                liftController.setPID(pL, iL, dL);
//                int liftPos = drive.leftVerticalSlide.getCurrentPosition();
//                if (liftTarget > liftMax) {
//                    liftTarget = liftMax;
//                } else if (liftTarget < 0) {
//                    liftTarget = 0;
//                }
//                pidLift = liftController.calculate(liftPos, liftTarget);
//                drive.leftVerticalSlide.setPower(pidLift);
//                drive.rightVerticalSlide.setPower(pidLift);
//
//                telemetry.addData("Voltage", voltage);
//                telemetry.addData("Lift Pos", liftPos);
//                telemetry.addData("Lift Target", liftTarget);
//
//                if (liftController.atSetPoint()) {
//                    liftState = LiftState.LIFT_MANUAL;
//                }

            case LIFT_MANUAL:
                double liftPower = -gamepad2.right_stick_y;
                drive.leftVerticalSlide.setPower(liftPower);
                drive.rightVerticalSlide.setPower(liftPower);
        }

//        liftController.setPID(pL, iL, dL);
//        int liftPos = drive.leftVerticalSlide.getCurrentPosition();
//        if (liftTarget > liftMax) {
//            liftTarget = liftMax;
//        } else if (liftTarget < 0) {
//            liftTarget = 0;
//        }
//        pidLift = liftController.calculate(liftPos, liftTarget);
//        drive.leftVerticalSlide.setPower(pidLift);
//        drive.rightVerticalSlide.setPower(pidLift);

        if (gamepad2.a) {
            liftState = LiftState.LIFT_AUTO;
            liftTarget = 10;
        } else if (gamepad2.b) {
            liftState = LiftState.LIFT_AUTO;
            liftTarget = 726;
        } else if (gamepad2.x) {
            liftState = LiftState.LIFT_AUTO;
            liftTarget = 1290;
        }
        telemetry.addData("Lift State", liftState);


        //EXTEND
        extendController.setPID(pE, iE, dE);
        int extendPos = drive.leftHorizontalSlide.getCurrentPosition();
        extendTarget -= (int) (gamepad2.left_stick_y * 10);
        if (extendTarget > extendMax) {
            extendTarget = extendMax;
        } else if (extendTarget < 0) {
            extendTarget = 0;
        }
        double pidExtend = extendController.calculate(extendPos, extendTarget);

        drive.leftHorizontalSlide.setPower(pidExtend);
        drive.rightHorizontalSlide.setPower(pidExtend);

        telemetry.addData("Extend Pos", extendPos);
        telemetry.addData("Extend Target", extendTarget);


        // CLAW
        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
            if (toggleClaw == "false" || toggleClaw == "init") {
                toggleClaw = "true";
            } else {
                toggleClaw = "false";
            }
        }
        if (toggleClaw == "true") {
            clawOpen();
            telemetry.addData("Claw", claw1);

        } else if (toggleClaw == "false") {
            clawClose();
            telemetry.addData("Claw", claw2);
        }

        // INTAKE
        if (gamepad2.dpad_up) {
            drive.intakeAngle.setPosition(intakeAngle2);
            drive.clawRotate.setPosition(clawRotate2);
            drive.clawAngle.setPosition(clawAngle2);
        } else if (gamepad2.dpad_down) {
            drive.clawAngle.setPosition(clawAngle1);
            drive.clawRotate.setPosition(clawRotate1);
            drive.intakeAngle.setPosition(intakeAngle1);
        }

        // FLIPPER
        if (currentGamepad2.y && !previousGamepad2.y) {
            if (toggleFlipper == "false" || toggleFlipper == "init") {
                toggleFlipper = "true";
            } else {
                toggleFlipper = "false";
            }
        }
        if (toggleFlipper == "true") {
            drive.leftFlipper.setPosition(leftFlipper2);
            drive.rightFlipper.setPosition(rightFlipper2);

            telemetry.addData("Left Flipper", leftFlipper2);
            telemetry.addData("Right Flipper", rightFlipper2);
        } else if (toggleFlipper == "false") {
            drive.leftFlipper.setPosition(leftFlipper1);
            drive.rightFlipper.setPosition(rightFlipper1);

            telemetry.addData("Left Flipper", leftFlipper1);
            telemetry.addData("Right Flipper", rightFlipper1);
        }

        // LOW POLE
        if (gamepad2.right_bumper) {
            lowPole();
        }

        // STABILIZER
        if (drive.leftVerticalSlide.getCurrentPosition() > stabilizerVertical) {
            drive.stabilizer.setPosition(stabilizer1);
        } else {
            drive.stabilizer.setPosition(stabilizer2);
        }

//        // CLAW ANGLE
//        if (currentGamepad2.a && !previousGamepad2.a) {
//            if (toggleClawAngle == "false" || toggleClawAngle == "init") {
//                toggleClawAngle = "true";
//            } else {
//                toggleClawAngle = "false";
//            }
//        }
//        if (toggleClawAngle == "true") {
//            drive.clawAngle.setPosition(clawAngle1);
//            telemetry.addData("Claw Angle", clawAngle1);
//
//        } else if (toggleClawAngle == "false") {
//            drive.clawAngle.setPosition(clawAngle2);
//            telemetry.addData("Claw Angle", clawAngle2);
//        }
//
////        // INTAKE ANGLE
//        if (currentGamepad2.b && !previousGamepad2.b) {
//            if (toggleIntakeAngle == "false" || toggleIntakeAngle == "init") {
//                toggleIntakeAngle = "true";
//            } else {
//                toggleIntakeAngle = "false";
//            }
//        }
//        if (toggleIntakeAngle == "true") {
//            drive.intakeAngle.setPosition(intakeAngle1);
//            telemetry.addData("Intake Angle", intakeAngle1);
//
//        } else if (toggleIntakeAngle == "false") {
//            drive.intakeAngle.setPosition(intakeAngle2);
//            telemetry.addData("Intake Angle", intakeAngle2);
//        }
//
//        // ROTATE ANGLE
//        if (currentGamepad2.x && !previousGamepad2.x) {
//            if (toggleClawRotate == "false" || toggleClawRotate == "init") {
//                toggleClawRotate = "true";
//            } else {
//                toggleClawRotate = "false";
//            }
//        }
//        if (toggleClawRotate == "true") {
//            drive.clawRotate.setPosition(clawRotate1);
//            telemetry.addData("Claw Rotate", clawRotate1);
//
//        } else if (toggleClawRotate == "false") {
//            drive.clawRotate.setPosition(clawRotate2);
//            telemetry.addData("Claw Rotate", clawRotate2);
//        }

        //STABILIZER
//        if (currentGamepad2.x && !previousGamepad2.x) {
//            if (toggleStabilizer == "false" || toggleStabilizer == "init") {
//                toggleStabilizer = "true";
//            } else {
//                toggleStabilizer = "false";
//            }
//        }
//        if (toggleStabilizer == "true") {
//            drive.stabilizer.setPosition(stabilizer1);
//            telemetry.addData("Stabilizer", stabilizer1);
//
//        } else if (toggleStabilizer == "false") {
//            drive.stabilizer.setPosition(stabilizer2);
//            telemetry.addData("Stabilizer", stabilizer2);
//        }

    }

    @Override
    public void stop() {
    }

    public void clawOpen() {
        drive.claw.setPosition(claw1);
    }

    public void clawClose() {
        drive.claw.setPosition(claw2);
    }

    public void lowPole() {
        drive.intakeAngle.setPosition(intakeAngle3);
        drive.clawRotate.setPosition(clawRotate1);
        drive.clawAngle.setPosition(clawAngle3);

    }

}
