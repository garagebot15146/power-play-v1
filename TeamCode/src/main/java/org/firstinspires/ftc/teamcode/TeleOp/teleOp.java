package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Settings.drive.HWMap;

@Config
@TeleOp(name = "teleOp", group = "Iterative Opmode")
//@Disabled
public class teleOp extends OpMode {

    static HWMap drive;

    // GAMEPAD
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    // TOGGLE
    String toggleClaw = "init";
    String toggleFlipper = "init";


    // THRESHOLDS
    public static int highPole = 655;
    public static int midPole = 365;
    public static int stabilizerVertical = 240;

    // SERVO POSITIONS
    public static double claw1 = 1;
    public static double claw2 = 0.52;

    public static double clawAngle1 = 0.07;
    public static double clawAngle2 = 0.64;
    public static double clawAngle3 = 0.23;

    public static double intakeAngle1 = 0.04;
    public static double intakeAngle2 = 0.66;
    public static double intakeAngle3 = 0.65;


    public static double clawRotate1 = 0;
    public static double clawRotate2 = 1;

    public static double leftFlipper1 = 0;
    public static double leftFlipper2 = 0.3;

    public static double rightFlipper1 = 1;
    public static double rightFlipper2 = 0.7;

    public static double stabilizer1 = 0;
    public static double stabilizer2 = 0.115;

    // STATE MACHINES
    public enum LiftState {
        LIFT_AUTO,
        LIFT_AUTO_SLOW,
        LIFT_MANUAL,
        REST
    }

    public enum ExtendState {
        EXTEND_MANUAL,
        TRANSFER,
        LOW_POLE
    }

    LiftState liftState = LiftState.LIFT_MANUAL;
    ExtendState extendState = ExtendState.EXTEND_MANUAL;

    boolean commandLock = false;
    boolean transferLock = false;
    double pidLift = 0;

    // CLOCK
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime transfertime = new ElapsedTime();
    private final ElapsedTime commandtime = new ElapsedTime();

    //PID
    PIDController liftController;
    public int liftTarget;
    public static double pL = 0.02, iL = 0.001, dL = 0.0004;

    @Override
    public void init() {
        drive = new HWMap(hardwareMap);

        //PID
        liftController = new PIDController(pL, iL, dL);
        liftController.setTolerance(20);

        //DASHBOARD
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //HORIZONTAL SLIDES
        drive.leftHorizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightHorizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.leftHorizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightHorizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //VERTICAL SLIDES
        drive.leftVerticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightVerticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.leftVerticalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightVerticalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.stabilizer.setPosition(stabilizer2);

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
        transfertime.reset();
        commandtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
        // ENCODERS
        int liftPos = drive.leftVerticalSlide.getCurrentPosition();
        int extensionPos = drive.leftHorizontalSlide.getCurrentPosition();

        // GAMEPADS
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        // GAMEPAD A

        // WHEELS
        double normal = 0.7;
        double slow = 0.35;
        double fast = 1;
        double forward = -gamepad1.left_stick_y;
        double side = gamepad1.left_stick_x; //Positive means right
        double turn = gamepad1.right_stick_x; //Positive means turn right


        double leftFrontPower = forward + side + turn;
        double leftRearPower = forward - side + turn;
        double rightRearPower = forward + side - turn;
        double rightFrontPower = forward - side - turn;

        // Send power to wheel motors
        if (gamepad1.right_trigger > 0.2) {
            drive.leftFront.setPower(leftFrontPower * fast);
            drive.leftRear.setPower(leftRearPower * fast);
            drive.rightRear.setPower(rightRearPower * fast);
            drive.rightFront.setPower(rightFrontPower * fast);
        } else {
            drive.leftFront.setPower(leftFrontPower * normal);
            drive.leftRear.setPower(leftRearPower * normal);
            drive.rightRear.setPower(rightRearPower * normal);
            drive.rightFront.setPower(rightFrontPower * normal);
        }

        // Fine Tune
        if (gamepad1.dpad_up) {
            drive.leftFront.setPower(slow);
            drive.leftRear.setPower(slow);
            drive.rightRear.setPower(slow);
            drive.rightFront.setPower(slow);
        } else if (gamepad1.dpad_down) {
            drive.leftFront.setPower(-slow);
            drive.leftRear.setPower(-slow);
            drive.rightRear.setPower(-slow);
            drive.rightFront.setPower(-slow);
        } else if (gamepad1.dpad_right) {
            drive.leftFront.setPower(slow);
            drive.leftRear.setPower(slow);
            drive.rightRear.setPower(-slow);
            drive.rightFront.setPower(-slow);
        } else if (gamepad1.dpad_left) {
            drive.leftFront.setPower(-slow);
            drive.leftRear.setPower(-slow);
            drive.rightRear.setPower(slow);
            drive.rightFront.setPower(slow);
        }

        // GAMEPAD B

        // STATE MACHINES
        switch (liftState) {
            case LIFT_AUTO:
                liftController.setPID(pL, iL, dL);
                pidLift = liftController.calculate(liftPos, liftTarget);
                lowPole();
                drive.leftVerticalSlide.setPower(pidLift);
                drive.rightVerticalSlide.setPower(pidLift);

                // Switch States
                if (liftController.atSetPoint()) {
                    liftState = LiftState.REST;
                }
                break;

            case LIFT_AUTO_SLOW:
                if (!commandLock) {
                    commandtime.reset();
                    commandLock = true;
                }

                if (commandtime.seconds() > 0.09) {
                    drive.stabilizer.setPosition(stabilizer2);
                }

                // Slows lift PID
                liftController.setPID(0.015, 0.0001, 0.0001);
                pidLift = liftController.calculate(liftPos, liftTarget);

                drive.leftVerticalSlide.setPower(Range.clip(pidLift, -1, 1) * 0.4);
                drive.rightVerticalSlide.setPower(Range.clip(pidLift, -1, 1) * 0.4);

                // Switch States
                if (liftController.atSetPoint()) {
                    liftState = LiftState.REST;
                    commandLock = false;
                }
                break;

            case LIFT_MANUAL:
                // Uses joystick no PID
                double liftPower = -gamepad2.right_stick_y;
                if (liftPower > 0) {
                    drive.leftVerticalSlide.setPower(liftPower * 1);
                    drive.rightVerticalSlide.setPower(liftPower * 1);
                } else {
                    drive.leftVerticalSlide.setPower(liftPower * 0.6);
                    drive.rightVerticalSlide.setPower(liftPower * 0.6);
                }

                // STABILIZER
                if (liftPos > stabilizerVertical) {
                    drive.stabilizer.setPosition(stabilizer1);
                } else {
                    drive.stabilizer.setPosition(stabilizer2);
                }
                break;

            case REST:
                drive.leftVerticalSlide.setPower(0);
                drive.rightVerticalSlide.setPower(0);
                break;
        }

        switch (extendState) {
            case EXTEND_MANUAL:
                drive.leftHorizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                drive.rightHorizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                if (!(gamepad1.left_trigger > 0.05 || gamepad1.right_trigger > 0.05)) {
                    double extendPower = -gamepad2.left_stick_y;
                    drive.leftHorizontalSlide.setPower(extendPower * 0.5);
                    drive.rightHorizontalSlide.setPower(extendPower * 0.5);
                } else {
                    if (gamepad1.left_trigger > 0.05) {
                        drive.leftHorizontalSlide.setPower(-gamepad1.left_trigger);
                        drive.rightHorizontalSlide.setPower(-gamepad1.left_trigger);
                    } else {
                        drive.leftHorizontalSlide.setPower(gamepad1.right_trigger);
                        drive.rightHorizontalSlide.setPower(gamepad1.right_trigger);
                    }
                }

                break;
            case TRANSFER:
                setExtension(0);
                if (extensionPos < 10) {
                    if (!transferLock) {
                        transfertime.reset();
                        transferLock = true;
                    }
                    intakeUp();
                    if (transfertime.seconds() > 0.95) {
                        toggleClaw = "true";
                        clawOpen();
                        if (transfertime.seconds() > 1.3) {
                            liftState = LiftState.REST;
                            drive.stabilizer.setPosition(stabilizer1);
                            if (transfertime.seconds() > 1.32) {
                                transferLock = false;
                                extendState = ExtendState.EXTEND_MANUAL;
                            }
                        }
                    }
                }
                break;
            case LOW_POLE:
                setExtension(0);
                if (extensionPos < 40) {
                    if (!transferLock) {
                        transfertime.reset();
                        transferLock = true;
                    }
                    lowPole();
                    if (transfertime.seconds() > 0.5) {
                        transferLock = false;
                        extendState = ExtendState.EXTEND_MANUAL;
                    }
                }
                break;
        }

        if (Math.abs(gamepad2.right_stick_y) > 0.01 && !(liftState == LiftState.LIFT_AUTO || liftState == LiftState.LIFT_AUTO_SLOW)) {
            liftState = LiftState.LIFT_MANUAL;
        }
        // Override the lift auto state
        if (gamepad2.right_trigger > 0.05) {
            liftState = LiftState.LIFT_MANUAL;
        }

        // INTAKE
        if (gamepad1.right_bumper) {
            extendState = ExtendState.TRANSFER;
        } else if (gamepad2.dpad_down || gamepad1.a) {
            intakeDown();
            drive.stabilizer.setPosition(stabilizer2);
            extendState = ExtendState.EXTEND_MANUAL;
        }

        // LOW POLE
        if (gamepad1.y) {
            extendState = ExtendState.LOW_POLE;
        }

        // Bottom Pole
        if (gamepad2.a) {
            drive.intakeAngle.setPosition(intakeAngle3);
            liftState = LiftState.LIFT_AUTO_SLOW;
            liftTarget = 0;
            // Mid Pole
        } else if (gamepad2.b) {
            drive.stabilizer.setPosition(stabilizer1);
            liftState = LiftState.LIFT_AUTO;
            liftTarget = midPole;
            // High Pole
        } else if (gamepad2.x) {
            drive.stabilizer.setPosition(stabilizer1);
            liftState = LiftState.LIFT_AUTO;
            liftTarget = highPole;
            // Reset Lift Zero Position
        } else if (gamepad2.left_trigger > 0.05) {
            //Reset Encoders
            drive.leftVerticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.rightVerticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            drive.leftVerticalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.rightVerticalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            drive.leftHorizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.rightHorizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            drive.leftHorizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.rightHorizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // CLAW
        if ((currentGamepad1.left_bumper && !previousGamepad1.left_bumper)) {
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

        // FLIPPER
        if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
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
        } else if (toggleFlipper == "false") {
            drive.leftFlipper.setPosition(leftFlipper1);
            drive.rightFlipper.setPosition(rightFlipper1);
            telemetry.addData("Left Flipper", leftFlipper1);
        }

        // TELEMETRY
        telemetry.addData("Lift State", liftState);
        telemetry.addData("Extend State", extendState);
        telemetry.addData("Lift Pos", liftPos);
        telemetry.addData("Extend Pos", extensionPos);
        telemetry.addData("Distance", drive.distanceSensor.getDistance(DistanceUnit.INCH));

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

    public void intakeDown() {
        drive.clawAngle.setPosition(clawAngle1);
        drive.clawRotate.setPosition(clawRotate1);
        drive.intakeAngle.setPosition(intakeAngle1);
    }

    public void intakeUp() {
        drive.intakeAngle.setPosition(intakeAngle2);
        drive.clawRotate.setPosition(clawRotate2);
        drive.clawAngle.setPosition(clawAngle2);
    }

    public void setExtension(int target) {
        drive.leftHorizontalSlide.setTargetPosition(target);
        drive.rightHorizontalSlide.setTargetPosition(target);

        drive.leftHorizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightHorizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.leftHorizontalSlide.setVelocity(7000);
        drive.rightHorizontalSlide.setVelocity(7000);
    }

}