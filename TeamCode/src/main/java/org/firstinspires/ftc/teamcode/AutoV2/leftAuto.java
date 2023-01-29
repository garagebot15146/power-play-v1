package org.firstinspires.ftc.teamcode.AutoV2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Settings.drive.HWMap;
import org.firstinspires.ftc.teamcode.Settings.trajectorysequence.TrajectorySequence;

@Autonomous(name = "leftAuto V2", group = "auto")
//@Disabled
public class leftAuto extends LinearOpMode {
    HWMap drive;

    // Runtime
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime threshold = new ElapsedTime();


    //PID
    PIDController liftController;
    public static double pL = 0.01, iL = 0, dL = 0;

    PIDController extendController;
    public static double pE = 0.01, iE = 0, dE = 0;

    // Cone Stack
    double[] intakeAngleList = {0.69, 0.64, 0.55, 0.46, 0.4};
    double[] clawAngleList = {0.925, 0.92, 0.91, 0.91, 0.91};

    // Servo Positions
    double claw1 = 1;
    double claw2 = 0.7;

    double clawAngle1 = 0.95;
    double clawAngle2 = 0.37;
    double clawAngle3 = 0.5;


    double intakeAngle1 = 0.9;
    double intakeAngle2 = 0.1;
    double intakeAngle3 = 0.17;

    double clawRotate1 = 0.74;
    double clawRotate2 = 0;

    @Override
    public void runOpMode() {
        drive = new HWMap(hardwareMap);

        Pose2d startPose = new Pose2d(-34, -72 + (15.5 / 2), Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        String conePos = "LEFT";

        TrajectorySequence toPole = drive.trajectorySequenceBuilder(startPose)
                .back(48)
                .lineToLinearHeading(new Pose2d(-31.8, -4, Math.toRadians(188)))
                .build();


        // Horizontal Slides
        drive.leftHorizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightHorizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.leftHorizontalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.rightHorizontalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Vertical Slides
        drive.leftVerticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightVerticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.leftVerticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.rightVerticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // SERVOS
        drive.intakeAngle.setPosition(intakeAngle3);
        drive.clawRotate.setPosition(clawRotate1);
        drive.clawAngle.setPosition(clawAngle3);
        clawOpen();


        liftController = new PIDController(pL, iL, dL);
        extendController = new PIDController(pE, iE, dE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;
        telemetry.update();

        cycle();
    }

    public void cycle(){
        deposit(6);

        intake();
        deposit(5);
        sleep(200);

        intake();
        deposit(4);
        sleep(200);

        intake();
        deposit(3);
        sleep(200);

        intake();
        deposit(2);
        sleep(200);

        intake();
        deposit(1);
    }

    public void intake(){
        lift(0, 2.5);
        transfer();
    }

    public void deposit(int cones){
        drive.stabilizer.setPosition(0);
        if(cones == 6){
            slides(5, 1000, 1190, 3.5);
        } else if (cones == 1){
            lift(1190, 3);
            drive.stabilizer.setPosition(0.2);
            sleep(200);
            lift(0, 3);
        } else {
            slides(cones - 1, 1000, 1190, 3.5);
        }
        drive.stabilizer.setPosition(0.2);
    }

    public void extend(int extendTarget, double timeout) {
        threshold.reset();

        extendController.setPID(pE, iE, dE);
        extendController.setSetPoint(extendTarget);

        boolean lock = false;
        while (!extendController.atSetPoint() && (threshold.seconds() < timeout)) {
            int extendPos = drive.leftHorizontalSlide.getCurrentPosition();
            double pidExtend = extendController.calculate(extendPos, extendTarget);

            drive.leftHorizontalSlide.setPower(pidExtend);
            drive.rightHorizontalSlide.setPower(pidExtend);


            telemetry.addData("Extend Pos", extendPos);
            telemetry.addData("Extend Target", extendTarget);
            telemetry.addData("Extend Set Point", extendController.atSetPoint());
            telemetry.update();
            if (Math.abs(extendTarget - extendPos) < 15) {
                if (!lock) {
                    runtime.reset();
                    lock = true;
                }
                if (runtime.seconds() > 0.1) {
                    break;
                }
            }
        }
        drive.leftHorizontalSlide.setPower(0);
        drive.rightHorizontalSlide.setPower(0);
    }

    public void lift(int liftTarget, double timeout) {
        threshold.reset();

        liftController.setPID(pL, iL, dL);
        liftController.setSetPoint(liftTarget);

        boolean lock = false;
        while (!liftController.atSetPoint() && (threshold.seconds() < timeout)) {
            int liftPos = drive.leftVerticalSlide.getCurrentPosition();
            double pidLift = liftController.calculate(liftPos, liftTarget);

            drive.leftVerticalSlide.setPower(pidLift);
            drive.rightVerticalSlide.setPower(pidLift);

            telemetry.addData("Lift Pos", liftPos);
            telemetry.addData("Lift Target", liftTarget);
            telemetry.addData("Lift Set Point", liftController.atSetPoint());
            telemetry.update();

            if (Math.abs(liftTarget - liftPos) < 15) {
                if (!lock) {
                    runtime.reset();
                    lock = true;
                }
                if (runtime.seconds() > 0.1) {
                    break;
                }
            }
        }
        drive.leftVerticalSlide.setPower(0);
        drive.rightVerticalSlide.setPower(0);
    }

    public void slides(int cones, int extendTarget, int liftTarget, double timeout) {
        threshold.reset();

        extendController.setPID(pE, iE, dE);
        extendController.setSetPoint(extendTarget);

        liftController.setPID(pL, iL, dL);
        liftController.setSetPoint(liftTarget);

        boolean lock = false;
        while (!(extendController.atSetPoint() && liftController.atSetPoint()) && (threshold.seconds() < timeout)) {
            int extendPos = drive.leftHorizontalSlide.getCurrentPosition();
            int liftPos = drive.leftVerticalSlide.getCurrentPosition();

            double pidExtend = extendController.calculate(extendPos, extendTarget);
            double pidLift = liftController.calculate(liftPos, liftTarget);

            drive.leftHorizontalSlide.setPower(pidExtend);
            drive.rightHorizontalSlide.setPower(pidExtend);

            drive.leftVerticalSlide.setPower(pidLift);
            drive.rightVerticalSlide.setPower(pidLift);

            clawReset(cones);

            telemetry.addData("Extend Pos", extendPos);
            telemetry.addData("Extend Target", extendTarget);
            telemetry.addData("Lift Pos", liftPos);
            telemetry.addData("Lift Target", liftTarget);
            telemetry.addData("Extend Set Point", extendController.atSetPoint());
            telemetry.addData("Lift Set Point", liftController.atSetPoint());
            telemetry.update();
            if (Math.abs(liftTarget - liftPos) < 15) {
                if (!lock) {
                    runtime.reset();
                    lock = true;
                }
                if (runtime.seconds() > 0.1) {
                    break;
                }
            }
        }
        drive.leftHorizontalSlide.setPower(0);
        drive.rightHorizontalSlide.setPower(0);
        drive.leftVerticalSlide.setPower(0);
        drive.rightVerticalSlide.setPower(0);
    }

    public void clawOpen() {
        drive.claw.setPosition(claw1);
    }

    public void clawClose() {
        drive.claw.setPosition(claw2);
    }

    public void transfer() {
        clawClose();
        sleep(300);
        drive.clawAngle.setPosition(clawAngle3);
        sleep(600);
        drive.intakeAngle.setPosition(intakeAngle2);
        drive.clawRotate.setPosition(clawRotate2);
        extend(0, 2);
        drive.clawAngle.setPosition(clawAngle2);
        sleep(100);
        clawOpen();
        sleep(150);
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

}