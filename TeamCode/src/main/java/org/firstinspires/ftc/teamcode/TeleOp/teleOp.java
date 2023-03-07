package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
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

    // STABILIZER

    // TOGGLE
    String toggleClaw = "init";
    String toggleClawAngle = "init";
    String toggleIntakeAngle = "init";
    String toggleClawRotate = "init";
    String toggleIntake = "init";
    String toggleFlipper = "init";
    String toggleStabilizer = "init";


    // THRESHOLDS
    public static int highPole = 645;
    public static int midPole = 355;
    public static int stabilizerVertical = 250;

    // SERVO POSITIONS
    public static double claw1 = 1;
    public static double claw2 = 0.5;

    public static double clawAngle1 = 0.1;
    public static double clawAngle2 = 0.59;
    public static double clawAngle3 = 0.22;

    public static double intakeAngle1 = 1;
    public static double intakeAngle2 = 0.41;
    public static double intakeAngle3 = 0.5;


    public static double clawRotate1 = 0;
    public static double clawRotate2 = 1;

    public static double leftFlipper1 = 1;
    public static double leftFlipper2 = 0.5;

    public static double rightFlipper1 = 0;
    public static double rightFlipper2 = 0.5;

    public static double stabilizer1 = 0;
    public static double stabilizer2 = 0.1;

    // STATE MACHINES
    public enum LiftState {
        LIFT_AUTO,
        LIFT_AUTO_SLOW,
        LIFT_MANUAL,
        CYCLE
    }

    public enum ExtendState {
        EXTEND_MANUAL,
        TRANSFER,
        LOW_POLE
    }

    public enum CycleState {
        START,
        INTAKE_UP,
        DEPOSIT
    }

    LiftState liftState = LiftState.LIFT_MANUAL;
    ExtendState extendState = ExtendState.EXTEND_MANUAL;
    CycleState cycleState = CycleState.START;

    public static int cycleReset = 200;
    boolean clawLock = false;
    boolean intakeLock = false;
    boolean commandLock = false;
    double pidLift = 0;

    // CLOCK
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime cycletime = new ElapsedTime();
    private final ElapsedTime transfertime = new ElapsedTime();
    private final ElapsedTime commandtime = new ElapsedTime();

    //PID
    PIDController liftController;
    public int liftMax = highPole;
    public int liftTarget = 0;
    public static double pL = 0.03, iL = 0.001, dL = 0.0003;

    PIDController extendController;
    public static int extendMax = 1000;
    public int extendTarget = 0;
    public static double pE = 0.02, iE = 0.001, dE = 0.0001;

    @Override
    public void init() {
        drive = new HWMap(hardwareMap);

        //COLOR SENSOR
//        drive.colorSensor.enableLed(true);

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
        cycletime.reset();
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
                if (!commandLock) {
                    commandtime.reset();
                    commandLock = true;
                }
                // Uses lift PID
                liftController.setPID(pL, iL, dL);
                pidLift = liftController.calculate(liftPos, liftTarget);

                if (commandtime.seconds() > 0.25) {
                    lowPole();
                    drive.leftVerticalSlide.setPower(pidLift);
                    drive.rightVerticalSlide.setPower(pidLift);
                }

                // Switch States
                if (liftController.atSetPoint()) {
                    liftState = LiftState.LIFT_MANUAL;
                    commandLock = false;
                }
                break;

            case LIFT_AUTO_SLOW:
                if (!commandLock) {
                    commandtime.reset();
                    commandLock = true;
                }

                if (commandtime.seconds() > 0.05) {
                    drive.stabilizer.setPosition(stabilizer2);
                }

                // Slows lift PID
                liftController.setPID(0.015, 0.0001, 0.0001);
                pidLift = liftController.calculate(liftPos, liftTarget);

                drive.leftVerticalSlide.setPower(Range.clip(pidLift, -1, 1) * 0.6);
                drive.rightVerticalSlide.setPower(Range.clip(pidLift, -1, 1) * 0.6);

                // Switch States
                if (liftController.atSetPoint()) {
                    liftState = LiftState.LIFT_MANUAL;
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
                break;

            case CYCLE:
                // Starts cycle command
                switch (cycleState) {
                    case START:
                        // Saves reset pos
//                        cycleReset = extensionPos;
                        cycletime.reset();
                        intakeLock = false;
                        cycleState = CycleState.INTAKE_UP;
                        break;

                    case INTAKE_UP:
                        double cycleDelay = cycleReset / 600;
                        if (!clawLock) {
                            clawClose();
                            telemetry.addData("Status", "Closed");
                            clawLock = true;
                        }
                        // Bring intake up
                        if (cycletime.seconds() >= 0.3) {
                            setExtension(0);
                            intakeUp();
                            if (cycletime.seconds() >= 1.5 + cycleDelay) {
                                clawOpen();
                                if (cycletime.seconds() >= 1.83 + cycleDelay) {
                                    cycletime.reset();
                                    intakeLock = false;
                                    cycleState = CycleState.DEPOSIT;
                                }
                            }

                        }
                        break;

                    case DEPOSIT:
                        clawLock = false;
                        if (cycletime.seconds() >= 1.25) {
                            // Bring lift down
                            setLiftSLow(7);
                            if (!intakeLock) {
                                intakeDown();
                                intakeLock = true;
                            }
                            if (liftPos < 560) {
                                drive.stabilizer.setPosition(stabilizer2);
                            }
                            setExtension(cycleReset);
                            if (extensionPos > 325) {
                                drive.intakeAngle.setPosition(intakeAngle1);
                            }
                        } else {
                            drive.intakeAngle.setPosition(intakeAngle3);
                            drive.stabilizer.setPosition(stabilizer1);
                            // Bring lift up
                            setLift(highPole);
                        }
                        if (cycletime.seconds() >= 1.7) {
                            cycletime.reset();
                            cycleState = CycleState.INTAKE_UP;
                            liftState = LiftState.LIFT_MANUAL;
                        }
                        break;
                }
                break;
        }

        if (liftState != LiftState.CYCLE) {
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
                    toggleClaw = "false";
                    setExtension(0);
                    intakeUp();
                    if (extensionPos < 10) {
                        if (transfertime.seconds() > 0.2) {
                            drive.clawAngle.setPosition(claw2 + 0.23);
                        }
                        if (transfertime.seconds() > 0.3) {
                            extendState = ExtendState.EXTEND_MANUAL;
                        }
                    } else {
                        drive.clawAngle.setPosition(clawAngle2);
                        transfertime.reset();
                    }
                    break;
                case LOW_POLE:
                    setExtension(0);
                    if (extensionPos < 40) {
                        lowPole();
                        if (transfertime.seconds() > 0.5) {
                            extendState = ExtendState.EXTEND_MANUAL;
                        }
                    } else {
                        transfertime.reset();
                    }
                    break;
            }
        }
        // Override the lift auto state
        if (gamepad2.right_trigger > 0.05) {
            liftState = LiftState.LIFT_MANUAL;
        }

        // Reset cycle extension position
        if (gamepad2.left_stick_button) {
            cycleReset = drive.leftHorizontalSlide.getCurrentPosition();
        }

        // Cycle Command
        if (gamepad2.y) {
            clawLock = false;
            cycletime.reset();
            liftState = LiftState.CYCLE;
        }

        // INTAKE
        if (gamepad1.right_bumper) {
            transfertime.reset();
            extendState = ExtendState.TRANSFER;
        } else if (gamepad2.dpad_down || gamepad1.a) {
            intakeDown();
            extendState = ExtendState.EXTEND_MANUAL;
        }

            // Bottom Pole
        if (gamepad2.a) {
            drive.intakeAngle.setPosition(intakeAngle3);
            liftState = LiftState.LIFT_AUTO_SLOW;
            liftTarget = 0;
            // Mid Pole
        } else if (gamepad2.b) {
            toggleClaw = "true";
            clawOpen();
            drive.stabilizer.setPosition(stabilizer1);
            liftState = LiftState.LIFT_AUTO;
            liftTarget = midPole;
            // High Pole
        } else if (gamepad2.x) {
            toggleClaw = "true";
            clawOpen();
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
        if (liftState != LiftState.CYCLE) {
            if (toggleClaw == "true") {
                clawOpen();
                telemetry.addData("Claw", claw1);

            } else if (toggleClaw == "false") {
                clawClose();
                telemetry.addData("Claw", claw2);
            }
        }

        // LOW POLE
        if (gamepad1.y) {
            extendState = ExtendState.LOW_POLE;
        }

        // STABILIZER
        if (liftState == LiftState.LIFT_MANUAL) {
            if (-gamepad2.right_stick_y < 0.6) {
                drive.stabilizer.setPosition(stabilizer2);
            } else {
                if (liftPos > stabilizerVertical) {
                    drive.stabilizer.setPosition(stabilizer1);
                } else {
                    drive.stabilizer.setPosition(stabilizer2);
                }
            }
        }

        // FLIPPER
        if (currentGamepad1.y && !previousGamepad1.y) {
            if (toggleFlipper == "false" || toggleFlipper == "init") {
                toggleFlipper = "true";
            } else {
                toggleFlipper = "false";
            }
        }
        if (toggleFlipper == "true") {
            drive.leftFlipper.setPosition(leftFlipper2);
            drive.rightFlipper.setPosition(rightFlipper2);
        } else if (toggleFlipper == "false") {
            drive.leftFlipper.setPosition(leftFlipper1);
            drive.rightFlipper.setPosition(rightFlipper1);
        }

//        // DELETE SOON
//
//        // CLAW ANGLE
//        if ((currentGamepad1.a && !previousGamepad1.a) || (currentGamepad1.a && !previousGamepad1.a)) {
//            if (toggleClawAngle == "false" || toggleClawAngle == "init") {
//                toggleClawAngle = "true";
//            } else {
//                toggleClawAngle = "false";
//            }
//        }
//        if (liftState != LiftState.CYCLE) {
//            if (toggleClawAngle == "true") {
//                drive.clawAngle.setPosition(clawAngle1);
//                telemetry.addData("Claw Angle", clawAngle1);
//
//            } else if (toggleClawAngle == "false") {
//                drive.clawAngle.setPosition(clawAngle2);
//                telemetry.addData("Claw Angle", clawAngle2);
//            }
//        }
//
//        // INTAKE ANGLE
//        if ((currentGamepad1.b && !previousGamepad1.b) || (currentGamepad1.b && !previousGamepad1.b)) {
//            if (toggleIntakeAngle == "false" || toggleIntakeAngle == "init") {
//                toggleIntakeAngle = "true";
//            } else {
//                toggleIntakeAngle = "false";
//            }
//        }
//        if (liftState != LiftState.CYCLE) {
//            if (toggleIntakeAngle == "true") {
//                drive.intakeAngle.setPosition(intakeAngle1);
//                telemetry.addData("Intake Angle", intakeAngle1);
//
//            } else if (toggleIntakeAngle == "false") {
//                drive.intakeAngle.setPosition(intakeAngle2);
//                telemetry.addData("Intake Angle", intakeAngle2);
//            }
//        }

        // TELEMETRY
        telemetry.addData("Lift State", liftState);
        telemetry.addData("Cycle State", cycleState);
        telemetry.addData("Extend State", extendState);
        telemetry.addData("Lift Pos", liftPos);
        telemetry.addData("Extend Pos", extensionPos);

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
    }

    public void setLift(int target) {
        liftController.setPID(pL, iL, dL);
        if (liftTarget > liftMax) {
            liftTarget = liftMax;
        } else if (liftTarget < 0) {
            liftTarget = 0;
        }
        pidLift = liftController.calculate(drive.rightVerticalSlide.getCurrentPosition(), target);

        drive.leftVerticalSlide.setPower(Range.clip(pidLift, -1, 1) * 0.85);
        drive.rightVerticalSlide.setPower(Range.clip(pidLift, -1, 1) * 0.85);
    }

    public void setLiftSLow(int target) {
        liftController.setPID(0.015, 0.0001, 0.0001);
        if (liftTarget > liftMax) {
            liftTarget = liftMax;
        } else if (liftTarget < 0) {
            liftTarget = 0;
        }

        pidLift = liftController.calculate(drive.rightVerticalSlide.getCurrentPosition(), target);

        drive.leftVerticalSlide.setPower(Range.clip(pidLift, -1, 1) * 0.5);
        drive.rightVerticalSlide.setPower(Range.clip(pidLift, -1, 1) * 0.5);
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
