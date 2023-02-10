package org.firstinspires.ftc.teamcode.AutoV2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Settings.drive.HWMap;
import org.firstinspires.ftc.teamcode.Settings.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.TeleOp.teleOp;

@Config
@Autonomous(name = "Fast Cycle", group = "auto")
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
//    public static double[] intakeAngleList = {base, base - inc, base - inc * 2, base - inc * 3, base - inc * 4};
    public static double[] intakeAngleList = {0, 0.81, 0.76, 0.67, 0.63, 0.57};

    public static double[] clawAngleList = {0.02, 0.02, 0.02, 0.02, 0.02};
    public static int cycleReset = 670;

    // THRESHOLDS
    public static int highPole = 494;
    public static int midPole = 360;

    // Servo Positions
    public static double claw1 = 1;
    public static double claw2 = 0.7;

    public static double clawAngle1 = 0.02;
    public static double clawAngle2 = 0.71;
    public static double clawAngle3 = 0.3;

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

    // AUTO
    int cones = 6;
    TrajectorySequence toPole;
    Trajectory forward;
    Trajectory backward;

    @Override
    public void init() {
        drive = new HWMap(hardwareMap);

        // TRAJECTORIES
        Pose2d startPose = new Pose2d(-34, -72 + (15.5 / 2), Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        toPole = drive.trajectorySequenceBuilder(startPose)
                .back(48)
                .lineToLinearHeading(new Pose2d(-31.8, -4, Math.toRadians(188)))
                .build();

        forward = drive.trajectoryBuilder(new Pose2d())
                .forward(10)
                .build();

        backward = drive.trajectoryBuilder(forward.end())
                .back(10)
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
        drive.intakeAngle.setPosition(intakeAngle3);
        drive.clawRotate.setPosition(clawRotate1);
        drive.clawAngle.setPosition(clawAngle3);
        clawOpen();

        // PID
        liftController = new PIDController(pL, iL, dL);
        extendController = new PIDController(pE, iE, dE);

        liftController.setTolerance(20);
        extendController.setTolerance(20);

        // Detection
        String conePos = "LEFT";

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Autp", "Init");
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

        // Starts cycle command
        switch (cycleState) {
            case START:
                intakeDown();
                clawOpen();
//                if (!startLock) {
//                    drive.followTrajectory(forward);
//                    runtime.reset();
//                    startLock = true;
//                }
//                intakeDown();
//                clawOpen();
//                if(runtime.seconds() > 2){
//                    cycleState = CycleState.DEPOSIT;
//                }
                cycletime.reset();
                cycleState = CycleState.DEPOSIT;
                break;

            case INTAKE:
                if (!clawLock) {
                    clawClose();
                    clawLock = true;
                }
                if (cycletime.seconds() >= 0.2) {
                    if (!clawAngleLock) {
                        drive.clawAngle.setPosition(clawAngle3);
                        clawAngleLock = true;
                    }
                }
                setLiftSLow(7);
                // Bring intake up
                if (cycletime.seconds() >= 0.5) {
                    setExtension(0);
                    if (extensionPos < 10) {
                        if (cycletime.seconds() >= 0.9) {
                            intakeUp();
                            if (cycletime.seconds() >= 1.5) {
                                drive.clawAngle.setPosition(clawAngle2);
                                if (cycletime.seconds() >= 2) {
                                    clawOpen();
                                    if (cycletime.seconds() >= 2.3) {
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

                // Change to claw reset
                intakeDown();

                // Move Slides
                depositUp(cycleReset, highPole);

                // Check
                if(cycletime.seconds() >= 1.5){
                    cycletime.reset();
                    cones -= 1;
                    if (cones == 0) {
                        cycleState = CycleState.PARK;
                    } else {
                        cycleState = CycleState.INTAKE;
                    }
                }
                break;

            case PARK:
                telemetry.addData("Auto", "Parking");
                requestOpModeStop();
                break;

            case NOTHING:
                telemetry.addData("Auto", "Paused");
                break;
        }

        // TELEMETRY
        telemetry.addData("State", cycleState);
        telemetry.addData("Claw Lock", clawLock);
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

    public void intakeDown() {
        drive.clawAngle.setPosition(clawAngle1);
        drive.clawRotate.setPosition(clawRotate1);
        drive.intakeAngle.setPosition(intakeAngle1);
    }

    public void intakeUp() {
        drive.intakeAngle.setPosition(intakeAngle2);
        drive.clawRotate.setPosition(clawRotate2);
    }

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

        drive.leftHorizontalSlide.setVelocity(3000);
        drive.rightHorizontalSlide.setVelocity(3000);

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

        drive.leftVerticalSlide.setPower(Range.clip(pidLift, -1, 1) * 0.35);
        drive.rightVerticalSlide.setPower(Range.clip(pidLift, -1, 1) * 0.35);
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