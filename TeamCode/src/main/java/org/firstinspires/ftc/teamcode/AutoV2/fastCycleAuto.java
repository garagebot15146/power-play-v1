package org.firstinspires.ftc.teamcode.AutoV2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Settings.drive.HWMap;
import org.firstinspires.ftc.teamcode.Settings.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.TeleOp.teleOp;

@Config
@Autonomous(name = "Cycle Auto", group = "auto")
//@Disabled
public class fastCycleAuto extends OpMode {
    HWMap drive;

    // CLOCK
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime cycletime = new ElapsedTime();


    // PID
    PIDController liftController;
    public static double pL = 0.02, iL = 0.001, dL = 0.0001;
    double pidLift = 0;

    PIDController extendController;
    public static double pE = 0.02, iE = 0, dE = 0.0001;

    // Cone Stack
    public static double base = 0.85;
    public static double inc = 0.04;
    public static double[] intakeAngles = {0, 0.715, 0.67, 0.6, 0.5, 0.4};
    public static double[] clawAngles = {0, 0.02, 0.02, 0.02, 0.04, 0.04};
    public static int[] extensions = {970, 970, 970, 1000, 1150, 1170};

    public static int cycleReset = 1010;

    // THRESHOLDS
    public static int highPole = 595;
    public static int midPole = 360;
    public int stabilizerVertical = 350;

    // Servo Positions
    public static double claw1 = 1;
    public static double claw2 = 0.7;

    public static double clawAngle1 = 0.02;
    public static double clawAngle2 = 0.66;
    public static double clawAngle3 = 0.4;
    public static double clawAngle4 = 0.6;

    public static double intakeAngle1 = 0.85;
    public static double intakeAngle2 = 0.25;
    public static double intakeAngle3 = 0.31;
    public static double intakeAngle4 = 0.2;

    public static double clawRotate1 = 1;
    public static double clawRotate2 = 0.23;

    public static double leftFlipper1 = 1;
    public static double leftFlipper2 = 0.5;

    public static double rightFlipper1 = 0;
    public static double rightFlipper2 = 0.5;

    public static double stabilizer1 = 0;
    public static double stabilizer2 = 0.37;

    // STATE MACHINES
    public enum CycleState {
        START,
        INTAKE,
        DEPOSIT,
        PARK,
        NOTHING
    }

    CycleState cycleState = CycleState.START;
    boolean clawLock = false;
    boolean clawAngleLock = false;
    boolean startLock = false;
    boolean parkLock = false;
    boolean timeLock = false;
    boolean distanceLock = false;

    // AUTO
    int cones = 5;
    public static String signal;

    TrajectorySequence toPole;
    TrajectorySequence parkLeft;
    TrajectorySequence parkCenter;
    TrajectorySequence parkRight;

    public static double toPoleBack = 48;
    public static double toPoleLineX = 34;
    public static double toPoleLineY = -4;
    public static double toPoleLineH = -22;

    public static double parkCenterLineX = 33;
    public static double parkCenterLineY = -13;
    public static double parkCenterLineH = 0;

    @Override
    public void init() {
        drive = new HWMap(hardwareMap);

        // TRAJECTORIES
        Pose2d startPose = new Pose2d(34, -72 + (15.5 / 2), Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        toPole = drive.trajectorySequenceBuilder(startPose)
                .back(toPoleBack)
                .lineToLinearHeading(new Pose2d(toPoleLineX, toPoleLineY, Math.toRadians(toPoleLineH)))
                .build();

        parkLeft = drive.trajectorySequenceBuilder(toPole.end())
                .lineToLinearHeading(new Pose2d(parkCenterLineX, parkCenterLineY, Math.toRadians(parkCenterLineH)))
                .back(15)
                .turn(Math.toRadians(90))
                .build();

        parkCenter = drive.trajectorySequenceBuilder(toPole.end())
                .lineToLinearHeading(new Pose2d(parkCenterLineX, parkCenterLineY, Math.toRadians(parkCenterLineH)))
                .turn(Math.toRadians(90))
                .build();

        parkLeft = drive.trajectorySequenceBuilder(toPole.end())
                .lineToLinearHeading(new Pose2d(parkCenterLineX, parkCenterLineY, Math.toRadians(parkCenterLineH)))
                .forward(15)
                .turn(Math.toRadians(90))
                .build();

        // Horizontal Slides
        drive.leftHorizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightHorizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.leftHorizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightHorizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Vertical Slides
        drive.leftVerticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightVerticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.leftVerticalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightVerticalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // SERVOS
        drive.intakeAngle.setPosition(intakeAngle4);
        drive.clawRotate.setPosition(clawRotate1);
        drive.clawAngle.setPosition(clawAngle4);
        clawOpen();

        // PID
        liftController = new PIDController(pL, iL, dL);
        extendController = new PIDController(pE, iE, dE);

        liftController.setTolerance(20);
        extendController.setTolerance(20);

        // Detection
        signal = "CENTER";

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Auto", "Init");
        telemetry.update();
        runtime.reset();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        double liftPos = drive.leftVerticalSlide.getCurrentPosition();
        double extensionPos = drive.leftHorizontalSlide.getCurrentPosition();
        double distance = drive.distanceSensor.getDistance(DistanceUnit.CM);

        // Starts cycle command
        switch (cycleState) {
            case START:
                if (!startLock) {
//                    drive.followTrajectorySequence(toPole);
                    runtime.reset();
                    cycletime.reset();
                    startLock = true;
                }
                cycleState = CycleState.DEPOSIT;
                break;

            case INTAKE:
                if(!timeLock){
                    cycletime.reset();
                    timeLock = true;
                }
                drive.stabilizer.setPosition(stabilizer2);
                setLiftSLow(7);
                // Bring intake up
                if (cycletime.seconds() >= 0.4) {
                    if (!clawLock) {
                        clawClose();
                        clawLock = true;
                    }
                    if (cycletime.seconds() >= 0.65) {
                        if (!clawAngleLock) {
                            drive.clawAngle.setPosition(clawAngle3);
                            drive.intakeAngle.setPosition(intakeAngle2);
                            clawAngleLock = true;
                        }
                        if (cycletime.seconds() >= 1.1) {
                            setExtension(0);
                            drive.clawRotate.setPosition(clawRotate2);
                            if (cycletime.seconds() >= 1.85) {
                                drive.clawAngle.setPosition(clawAngle2);
                                if (cycletime.seconds() >= 2.15) {
                                    clawOpen();
                                    if (cycletime.seconds() >= 2.4) {
                                        cycletime.reset();
                                        cycleState = CycleState.DEPOSIT;
                                    }
                                }
                            }
                        }
                    }
                }
                break;

            case DEPOSIT:
                clawLock = false;
                clawAngleLock = false;
                timeLock = false;

                // Change to claw reset
                if (cones != 0) {
                    intakeDown(cones);
                }

                drive.stabilizer.setPosition(stabilizer1);

                // Move Slides
                depositUp(cones == 0 ? 0 : extensions[cones], highPole);

                // Check
                if (cycletime.seconds() >= 1.4) {
                    cones -= 1;
                    if (cones == -1) {
                        cycletime.reset();
                        cycleState = CycleState.PARK;
                    } else {
                        cycleState = CycleState.INTAKE;
                    }
                }
                break;

            case PARK:
                setLiftSLow(7);
                drive.stabilizer.setPosition(stabilizer2);
                if (cycletime.seconds() >= 1) {
//                    if (!parkLock) {
//                        switch (signal){
//                            case "LEFT":
//                                drive.followTrajectorySequence(parkLeft);
//                                parkLock = true;
//                                break;
//                            case "CENTER":
//                                drive.followTrajectorySequence(parkCenter);
//                                parkLock = true;
//                                break;
//                            case "RIGHT":
//                                drive.followTrajectorySequence(parkRight);
//                                parkLock = true;
//                                break;
//                        }
//                    } else {
                        telemetry.addData("Auto", "Parking");
                        requestOpModeStop();
//                    }
                }
                break;

            case NOTHING:
                telemetry.addData("Auto", "Paused");
                break;
        }

        // TELEMETRY
        telemetry.addData("State", cycleState);
        telemetry.addData("Claw Lock", clawLock);
        telemetry.addData("Claw Angle Lock", clawAngleLock);
        telemetry.addData("Cycle Time", cycletime.seconds());
        telemetry.addData("Run Time", runtime.seconds());
        telemetry.addData("Lift Pos", liftPos);
        telemetry.addData("Extend Pos", extensionPos);
    }


    public void clawOpen() {
        drive.claw.setPosition(claw1);
    }

    public void clawClose() {
        drive.claw.setPosition(claw2);
    }

    public void intakeDown(int cones) {
        drive.clawAngle.setPosition(clawAngles[cones]);
        drive.clawRotate.setPosition(clawRotate1);
        drive.intakeAngle.setPosition(intakeAngles[cones]);
    }

//    public void intakeUp() {
//        drive.intakeAngle.setPosition(intakeAngle2);
//        drive.clawRotate.setPosition(clawRotate2);
//    }

    public void depositUp(int extendTarget, int liftTarget) {
        // INIT
        liftController.setPID(pL, iL, dL);
        pidLift = liftController.calculate(drive.rightVerticalSlide.getCurrentPosition(), liftTarget);

        drive.leftHorizontalSlide.setTargetPosition(extendTarget);
        drive.rightHorizontalSlide.setTargetPosition(extendTarget);

        drive.leftHorizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightHorizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // RUN
        drive.leftVerticalSlide.setPower(pidLift);
        drive.rightVerticalSlide.setPower(pidLift);

        drive.leftHorizontalSlide.setVelocity(3200);
        drive.rightHorizontalSlide.setVelocity(3200);

    }

    public void setLift(int target) {
        liftController.setPID(pL, iL, dL);
        pidLift = liftController.calculate(drive.rightVerticalSlide.getCurrentPosition(), target);

        drive.leftVerticalSlide.setPower(pidLift);
        drive.rightVerticalSlide.setPower(pidLift);
    }

    public void setLiftSLow(int target) {
        liftController.setPID(0.015, 0.0001, 0.0001);
        pidLift = liftController.calculate(drive.rightVerticalSlide.getCurrentPosition(), target);

        drive.leftVerticalSlide.setPower(Range.clip(pidLift, -1, 1) * 0.32);
        drive.rightVerticalSlide.setPower(Range.clip(pidLift, -1, 1) * 0.32);
    }

    public void setExtension(int target) {
        drive.leftHorizontalSlide.setTargetPosition(target);
        drive.rightHorizontalSlide.setTargetPosition(target);

        drive.leftHorizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightHorizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.leftHorizontalSlide.setVelocity(4000);
        drive.rightHorizontalSlide.setVelocity(4000);
    }

}