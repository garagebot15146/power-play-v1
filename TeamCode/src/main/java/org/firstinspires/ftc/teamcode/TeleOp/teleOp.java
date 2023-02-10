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

    // TOGGLE
    String toggleClaw = "init";
    String toggleClawAngle = "init";
    String toggleIntakeAngle = "init";
    String toggleClawRotate = "init";
    String toggleIntake = "init";
    String toggleFlipper = "init";
    String toggleStabilizer = "init";


    // THRESHOLDS
    public static int highPole = 595;
    public static int midPole = 360;
    public int stabilizerVertical = 110;

    // SERVO POSITIONS
    public static double claw1 = 1;
    public static double claw2 = 0.7;

    public static double clawAngle1 = 0.02;
    public static double clawAngle2 = 0.71;
    public static double clawAngle3 = 0.27;

    public static double intakeAngle1 = 0.85;
    public static double intakeAngle2 = 0.13;
    public static double intakeAngle3 = 0.31;

    public static double clawRotate1 = 1;
    public static double clawRotate2 = 0.23;

    public static double leftFlipper1 = 1;
    public static double leftFlipper2 = 0.5;

    public static double rightFlipper1 = 0;
    public static double rightFlipper2 = 0.5;

    public static double stabilizer1 = 0;
    public static double stabilizer2 = 0.2;

    // STATE MACHINES
    public enum LiftState {
        LIFT_AUTO,
        LIFT_AUTO_SLOW,
        LIFT_MANUAL,
        CYCLE
    }

    public enum CycleState {
        START,
        INTAKE_UP,
        DEPOSIT
    }

    LiftState liftState = LiftState.LIFT_MANUAL;
    CycleState cycleState = CycleState.START;

    int cycleReset;
    boolean clawLock = false;
    double pidLift = 0;

    // CLOCK
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime cycletime = new ElapsedTime();
    boolean timerLock = false;
    double timeAmount = 0;

    //PID
    PIDController liftController;
    public int liftMax = highPole;
    public int liftTarget = 0;
    public static double pL = 0.0144, iL = 0.001, dL = 0.0001;

    PIDController extendController;
    public static int extendMax = 1000;
    public int extendTarget = 0;
    public static double pE = 0.02, iE = 0.001, dE = 0.0001;

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
        // ENCODERS
        int liftPos = drive.rightVerticalSlide.getCurrentPosition();
        int extensionPos = drive.leftHorizontalSlide.getCurrentPosition();

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

        // STATE MACHINES
        switch (liftState) {
            case LIFT_AUTO:
                // Uses lift PID
                liftController.setPID(pL, iL, dL);
                pidLift = liftController.calculate(liftPos, liftTarget);

                drive.leftVerticalSlide.setPower(pidLift);
                drive.rightVerticalSlide.setPower(pidLift);

                // Switch States
                if (liftController.atSetPoint()) {
                    liftState = LiftState.LIFT_MANUAL;
                }
                break;

            case LIFT_AUTO_SLOW:
                // Slows lift PID
                liftController.setPID(0.015, 0.0001, 0.0001);
                pidLift = liftController.calculate(liftPos, liftTarget);

                drive.leftVerticalSlide.setPower(Range.clip(pidLift, -1, 1) * 0.35);
                drive.rightVerticalSlide.setPower(Range.clip(pidLift, -1, 1) * 0.35);

                // Switch States
                if (liftController.atSetPoint()) {
                    liftState = LiftState.LIFT_MANUAL;
                }
                break;

            case LIFT_MANUAL:
                // Uses joystick no PID
                double liftPower = -gamepad2.right_stick_y;
                if (liftPower > 0) {
                    drive.leftVerticalSlide.setPower(liftPower * 0.63);
                    drive.rightVerticalSlide.setPower(liftPower * 0.63);
                } else {
                    drive.leftVerticalSlide.setPower(liftPower * 0.33);
                    drive.rightVerticalSlide.setPower(liftPower * 0.33);
                }
//                if (liftPower > 0) {
//                    drive.leftVerticalSlide.setPower(liftPower * 1);
//                    drive.rightVerticalSlide.setPower(liftPower * 1);
//                } else {
//                    drive.leftVerticalSlide.setPower(liftPower * 0.9);
//                    drive.rightVerticalSlide.setPower(liftPower * 0.9);
//                }
                break;

            case CYCLE:
                // Starts cycle command
                switch (cycleState) {
                    case START:
                        // Saves reset pos
                        cycleReset = extensionPos;
                        cycletime.reset();
                        cycleState = CycleState.INTAKE_UP;
                        break;

                    case INTAKE_UP:
                        double cycleDelay = cycleReset / 1500;
                        if (!clawLock) {
                            clawClose();
                            telemetry.addData("Status", "Closed");
                            clawLock = true;
                        }
                        // Bring intake up
                        if (cycletime.seconds() >= 0.3) {
                            setExtension(0);
                            if (extensionPos < 10) {
                                if (cycletime.seconds() >= 0.6 + cycleDelay) {
                                    intakeUp();
                                    if (cycletime.seconds() >= 1.6 + cycleDelay) {
                                        clawOpen();
                                        if (cycletime.seconds() >= 2 + cycleDelay) {
                                            cycletime.reset();
                                            cycleState = CycleState.DEPOSIT;
                                        }
                                    }
                                }
                            }
                        }
                        break;

                    case DEPOSIT:
                        clawLock = false;
                        if (cycletime.seconds() >= 1.1) {
                            // Bring lift down
                            setLiftSLow(7);
                            if (liftPos < 400) {
                                intakeDown();
                                setExtension(cycleReset);
                            }
                        } else {
                            // Bring lift up
                            setLift(highPole);
                        }
                        if (cycletime.seconds() >= 2) {
                            cycletime.reset();
                            cycleState = CycleState.INTAKE_UP;
                            liftState = LiftState.LIFT_MANUAL;
                        }
                        break;
                }
                break;
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
            cycletime.reset();
            liftState = LiftState.CYCLE;
        }
        // Voltage Check
        if (voltage < 10) {
            liftState = LiftState.LIFT_MANUAL;
            cycleState = CycleState.INTAKE_UP;
        }

        // Low Pole
        if (gamepad2.a) {
            liftState = LiftState.LIFT_AUTO_SLOW;
            liftTarget = 20;
            // Mid Pole
        } else if (gamepad2.b) {
            liftState = LiftState.LIFT_AUTO;
            liftTarget = midPole;
            // High Pole
        } else if (gamepad2.x) {
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

        // EXTEND
        if (liftState != LiftState.CYCLE) {
            drive.leftHorizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.rightHorizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            double extendPower = -gamepad2.left_stick_y;
            drive.leftHorizontalSlide.setPower(extendPower * 0.6);
            drive.rightHorizontalSlide.setPower(extendPower * 0.6);
        }


        // CLAW
        if ((currentGamepad2.left_bumper && !previousGamepad2.left_bumper) || (currentGamepad1.left_bumper && !previousGamepad1.left_bumper)) {
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
            intakeUp();
        } else if (gamepad2.dpad_down) {
            intakeDown();
        }

        // LOW POLE
        if (gamepad2.right_bumper) {
            lowPole();
        }

        // STABILIZER
        if (liftPos > stabilizerVertical) {
            drive.stabilizer.setPosition(stabilizer1);
        } else {
            drive.stabilizer.setPosition(stabilizer2);
        }

        // FLIPPER
        if (liftPos > 250) {
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

//                telemetry.addData("Left Flipper", leftFlipper2);
//                telemetry.addData("Right Flipper", rightFlipper2);
            } else if (toggleFlipper == "false") {
                drive.leftFlipper.setPosition(leftFlipper1);
                drive.rightFlipper.setPosition(rightFlipper1);
//
//                telemetry.addData("Left Flipper", leftFlipper1);
//                telemetry.addData("Right Flipper", rightFlipper1);
            }
        }

        // TELEMETRY
        telemetry.addData("Lift State", liftState);
        telemetry.addData("Cycle State", cycleState);
        telemetry.addData("Claw Lock", clawLock);
        telemetry.addData("Cycle Time", cycletime.seconds());
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
        drive.clawAngle.setPosition(clawAngle2);
    }

    public void setLift(int target) {
        liftController.setPID(pL, iL, dL);
        if (liftTarget > liftMax) {
            liftTarget = liftMax;
        } else if (liftTarget < 0) {
            liftTarget = 0;
        }
        pidLift = liftController.calculate(drive.rightVerticalSlide.getCurrentPosition(), target);

        drive.leftVerticalSlide.setPower(pidLift);
        drive.rightVerticalSlide.setPower(pidLift);
    }

    public void setLiftSLow(int target) {
        liftController.setPID(0.015, 0.0001, 0.0001);
        if (liftTarget > liftMax) {
            liftTarget = liftMax;
        } else if (liftTarget < 0) {
            liftTarget = 0;
        }

        pidLift = liftController.calculate(drive.rightVerticalSlide.getCurrentPosition(), target);

        drive.leftVerticalSlide.setPower(Range.clip(pidLift, -1, 1) * 0.35);
        drive.rightVerticalSlide.setPower(Range.clip(pidLift, -1, 1) * 0.35);
    }

    public void setExtension(int target) {
        drive.leftHorizontalSlide.setTargetPosition(target);
        drive.rightHorizontalSlide.setTargetPosition(target);

        drive.leftHorizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightHorizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.leftHorizontalSlide.setVelocity(3000);
        drive.rightHorizontalSlide.setVelocity(3000);
    }

    public double getTime() {
        if (!timerLock) {
            timeAmount = runtime.seconds();
            timerLock = true;
        }
        return timeAmount;
    }

    public void resetTime() {
        timerLock = false;
    }

}
