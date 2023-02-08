package org.firstinspires.ftc.teamcode.AutoV2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Settings.drive.HWMap;
import org.firstinspires.ftc.teamcode.Settings.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "rightAuto", group = "auto")
//@Disabled
public class rightAuto extends OpMode {
    HWMap drive;

    // Runtime
    private ElapsedTime totalTime = new ElapsedTime();
    private ElapsedTime totalTimeSlide = new ElapsedTime();
    private ElapsedTime slideTime = new ElapsedTime();
    private ElapsedTime liftTime = new ElapsedTime();
    private ElapsedTime extendTime = new ElapsedTime();
    private ElapsedTime threshold = new ElapsedTime();
    boolean timerLock = false;
    double timeAmount = 0;
    boolean timerSlideLock = false;
    double timeSlideAmount = 0;
    double transferTime;


    //PID
    PIDController liftController;
    public static double pL = 0.02, iL = 0.001, dL = 0.0001;

    PIDController extendController;
    public static double pE = 0.02, iE = 0, dE = 0.0001;

    // Cone Stack
    double[] intakeAngleList = {0.69, 0.64, 0.55, 0.46, 0.4};
    double[] clawAngleList = {0.925, 0.92, 0.91, 0.91, 0.91};

    // Servo Positions
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

    public enum CycleState {
        START,
        DEPOSIT_UP,
        DEPOSIT_DOWN,
        HIT
    }

    public enum TransferState {
        NOTHING,
        TRANSFER
    }

    CycleState cycleState = CycleState.START;
    TransferState transferState = TransferState.NOTHING;
    boolean clawLock = false;
    boolean clawAngleLock = false;

    // How many cones are left
    int cones = 5;

    double pidLift = 0;
    double pidExtend = 0;

    TrajectorySequence toPole;

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
        totalTime.reset();
        liftTime.reset();
        extendTime.reset();
        slideTime.reset();
        threshold.reset();
        totalTimeSlide.reset();
    }

    @Override
    public void start() {
//        drive.followTrajectorySequence(toPole);
    }

    @Override
    public void loop() {
        double liftPos = drive.leftVerticalSlide.getCurrentPosition();
        double extendPos = drive.leftHorizontalSlide.getCurrentPosition();

        switch (cycleState) {
            case START:
                cycleState = CycleState.DEPOSIT_UP;
                break;

            case DEPOSIT_UP:
                if (transferState == TransferState.NOTHING) {
                    intakeDown();
                    depositUp(1000, 1100);
                }
                break;

            case DEPOSIT_DOWN:
                resetSlideTime();
                resetTime();
                depositDown(2.5);
                break;
        }

        switch (transferState) {
            case NOTHING:
                resetTime();
                clawLock = false;
                clawAngleLock = false;
                break;

            case TRANSFER:
                transferTime = getTime();
                if (!clawLock) {
                    clawClose();
                    if (totalTime.seconds() > transferTime + 0.3) {
                        drive.clawAngle.setPosition(clawAngle3);
                        clawLock = true;
                    }
                }
                if (totalTime.seconds() > transferTime + 0.6) {
                    setExtension(0);
                    intakeUp();
                    if(extendPos < 10 && totalTime.seconds() > transferTime + 2){
                        transferState = TransferState.NOTHING;
                    }

                }
                break;
        }
        telemetry.addData("Cycle State", cycleState);
        telemetry.addData("Transfer State", transferState);
        telemetry.addData("Lift", liftPos + " " + liftController.atSetPoint());
        telemetry.addData("Extend", extendPos + " " + extendController.atSetPoint());
        telemetry.update();
    }


    public void depositUp(int extendTarget, int liftTarget) {
        drive.stabilizer.setPosition(stabilizer1);
        pidLift = liftController.calculate(drive.leftVerticalSlide.getCurrentPosition(), liftTarget);
        pidExtend = extendController.calculate(drive.leftHorizontalSlide.getCurrentPosition(), extendTarget);

        drive.leftVerticalSlide.setPower(pidLift);
        drive.rightVerticalSlide.setPower(pidLift);

        drive.leftHorizontalSlide.setPower(pidExtend);
        drive.rightHorizontalSlide.setPower(pidExtend);

        if ((liftController.atSetPoint() && extendController.atSetPoint()) || totalTime.seconds() > getTime() + 2) {
            if(totalTimeSlide.seconds() > getSlideTime() + 0.7){
                drive.stabilizer.setPosition(stabilizer2);
                cycleState = CycleState.DEPOSIT_DOWN;
            }
        }
    }

    public void depositDown(double timeout) {
        drive.stabilizer.setPosition(stabilizer1);
        pidLift = liftController.calculate(drive.leftVerticalSlide.getCurrentPosition(), 7);

        drive.leftVerticalSlide.setPower(Range.clip(pidLift, -1, 1) * 0.55);
        drive.rightVerticalSlide.setPower(Range.clip(pidLift, -1, 1) * 0.55);

        if (drive.leftVerticalSlide.getCurrentPosition() < 900) {
            transferState = TransferState.TRANSFER;
        }
        if (liftController.atSetPoint() || totalTimeSlide.seconds() > getSlideTime() + 1.5) {
            resetSlideTime();
            cycleState = CycleState.DEPOSIT_UP;
        }
    }

    public void setExtension(int target) {
        pidExtend = extendController.calculate(drive.leftHorizontalSlide.getCurrentPosition(), target);

        drive.leftHorizontalSlide.setPower(pidExtend);
        drive.rightHorizontalSlide.setPower(pidExtend);
    }

    public void clawOpen() {
        drive.claw.setPosition(claw1);
    }

    public void clawClose() {
        drive.claw.setPosition(claw2);
    }


    public void clawReset(int cones) {
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

    public double getTime() {
        if (!timerLock) {
            timeAmount = totalTime.seconds();
            timerLock = true;
        }
        return timeAmount;
    }

    public void resetTime() {
        timerLock = false;
    }

    public double getSlideTime() {
        if (!timerSlideLock) {
            timeSlideAmount = totalTimeSlide.seconds();
            timerSlideLock = true;
        }
        return timeSlideAmount;
    }

    public void resetSlideTime() {
        timerSlideLock = false;
    }

    public void intakeDown(){
        drive.clawAngle.setPosition(clawAngle1);
        drive.clawRotate.setPosition(clawRotate1);
        drive.intakeAngle.setPosition(intakeAngle1);
    }

    public void intakeUp(){
        drive.intakeAngle.setPosition(intakeAngle2);
        drive.clawRotate.setPosition(clawRotate2);
        drive.clawAngle.setPosition(clawAngle2);
    }

}