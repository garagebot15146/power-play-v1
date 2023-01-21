package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Settings.drive.HWMap;

@TeleOp(name = "teleOp", group = "Iterative Opmode")
//@Disabled
public class teleOp extends OpMode {

    // Initialize
    static HWMap drive;

    // GAMEPAD
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    // VERTICAL SLIDES
    int verticalSlideTarget = 0;
    int stabilizerVertical = 715;
    int maxVertical = 1150;
    private int liftTarget = 0;

    // RANGES
    int liftSlow = 500;

    // HORIZONTAL SLIDES
    int horizontalSlideTarget = 0;
    int maxHorizontal = 1120;

    // TOGGLE
    String toggleClaw = "init";
    String toggleClawAngle = "init";
    String toggleIntakeAngle = "init";
    String toggleClawRotate = "init";
    String toggleIntake = "init";
    String toggleFlipper = "init";
    String toggleStabilizer = "init";


    // SERVO POS
    double claw1 = 1;
    double claw2 = 0.7;

    double clawAngle1 = 0.95;
    double clawAngle2 = 0.37;
    double clawAngle3 = 0.79;

    double intakeAngle1 = 0.9;
    double intakeAngle2 = 0.1;
    double intakeAngle3 = 0.17;

    double clawRotate1 = 0.74;
    double clawRotate2 = 0;

    double leftFlipper1 = 1;
    double leftFlipper2 = 0.5;

    double rightFlipper1 = 0;
    double rightFlipper2 = 0.5;

    double stabilizer1 = 0;
    double stabilizer2 = 0.2;

    // CLOCK
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        drive = new HWMap(hardwareMap);

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
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        // GAMEPAD A

        // WHEELS
        double normal = 0.6;
        double slow = 0.3;
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

        // HORIZONTAL SLIDES
        horizontalSlideTarget -= gamepad2.left_stick_y * 15;

        drive.leftHorizontalSlide.setTargetPosition(horizontalSlideTarget);
        drive.rightHorizontalSlide.setTargetPosition(horizontalSlideTarget);

        if (horizontalSlideTarget < 0) {
            horizontalSlideTarget = 0;
        } else if (horizontalSlideTarget > maxHorizontal) {
            horizontalSlideTarget = maxHorizontal;
        }

        drive.leftHorizontalSlide.setPower(1);
        drive.rightHorizontalSlide.setPower(1);

        drive.leftHorizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightHorizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        telemetry.addData("Horizontal Slides Target", horizontalSlideTarget);
        telemetry.addData("Horizontal Encoder", drive.leftHorizontalSlide.getCurrentPosition());
//        telemetry.addData("Right Horizontal Encoder", drive.rightHorizontalSlide.getCurrentPosition());


        // VERTICAL SLIDES
//        verticalSlideTarget -= gamepad2.right_stick_y * 35;
//
//        drive.leftVerticalSlide.setTargetPosition(verticalSlideTarget);
//        drive.rightVerticalSlide.setTargetPosition(verticalSlideTarget);
//
//        if (verticalSlideTarget < 0) {
//            verticalSlideTarget = 0;
//        } else if (verticalSlideTarget > maxVertical) {
//            verticalSlideTarget = maxVertical;
//        }
//
//        drive.leftVerticalSlide.setPower(1);
//        drive.rightVerticalSlide.setPower(1);
//
//        drive.leftVerticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        drive.rightVerticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double liftPower = 0;

        //Lift (Negative spoolPower is up)

        //Check if there's an input
        if (gamepad2.right_stick_y < -0.05 || gamepad2.right_stick_y > 0.05) {

            //If down is pressed
            if (gamepad2.right_stick_y > 0.05) {

                //If it's not at bottom
                if (drive.leftVerticalSlide.getCurrentPosition() > 0) {

                    //Slow down when position gets close to bottom
                    if (Math.abs(drive.leftVerticalSlide.getCurrentPosition()) < liftSlow) {
                        liftPower = gamepad2.right_stick_y * 0.6;
                    } else {
                        liftPower = gamepad2.right_stick_y * 0.8;
                    }

                    //If at bottom, turn off power and reset position
                } else {
                    liftPower = 0;
                    drive.leftVerticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    drive.leftVerticalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftTarget = 0;
                }
                //If joystick is not pressed down
            } else {
                liftPower = gamepad2.right_stick_y * 1;
            }

            //If there's a controller input, set liftTarget
            liftTarget = drive.leftVerticalSlide.getCurrentPosition();

            //If no input, hold the arm up
        }

        //Send power to lift motor
        drive.leftVerticalSlide.setPower(-liftPower);
        drive.rightVerticalSlide.setPower(-liftPower);
        telemetry.addData("Vertical Encoder", drive.leftVerticalSlide.getCurrentPosition());


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
            telemetry.addData("Claw", claw2);

        } else if (toggleClaw == "false") {
            clawClose();
            telemetry.addData("Claw", claw1);
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
        if(gamepad2.right_bumper){
            lowPole();
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

        // STABILIZER

        if (drive.leftVerticalSlide.getCurrentPosition() > stabilizerVertical) {
            drive.stabilizer.setPosition(stabilizer1);
        } else {
            drive.stabilizer.setPosition(stabilizer2);
        }


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
