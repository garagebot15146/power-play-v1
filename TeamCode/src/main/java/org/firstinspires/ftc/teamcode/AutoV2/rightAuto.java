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
import org.openftc.easyopencv.OpenCvCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Settings.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.Settings.OpenCVPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous(name = "Right Auto", group = "auto")
//@Disabled
public class rightAuto extends OpMode {
    HWMap drive;

    // CLOCK
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime visiontime = new ElapsedTime();
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
    public static double[] intakeAngles = {0, 0.715, 0.67, 0.6, 0.585, 0.45};
    public static double[] clawAngles = {0, 0.02, 0.02, 0.02, 0.04, 0.04};
    public static int[] extensions = {970, 970, 970, 970, 1000, 1110};

    public static int cycleReset = 1010;

    // THRESHOLDS
    public static int highPole = 610;
    public static int midPole = 360;
    public static int stabilizerVertical = 550;

    // Servo Positions
    public static double claw1 = 1;
    public static double claw2 = 0.7;

    public static double clawAngle1 = 0.02;
    public static double clawAngle2 = 0.66;
    public static double clawAngle3 = 0.3;
    public static double clawAngle4 = 0.6;

    public static double intakeAngle1 = 0.85;
    public static double intakeAngle2 = 0.25;
    public static double intakeAngle3 = 0.34;
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
        LIFT_AGAIN
    }

    CycleState cycleState = CycleState.START;
    boolean clawLock = false;
    boolean clawAngleLock = false;
    boolean startLock = false;
    boolean parkLock = false;
    boolean timeLock = false;
    boolean colorFail = false;
    boolean colorLock = false;
    boolean pipelineLock = false;

    // VISION
    OpenCvCamera camera;
    String CVconePos = "CENTER";
    String ATconePos = "NOT_SET";
    String signal = "CENTER";
    //    public static String signal;
    boolean ATLock = true;

    // AUTO
    int cones = 5;

    TrajectorySequence toPole;
    TrajectorySequence parkLeft;
    TrajectorySequence parkCenter;
    TrajectorySequence parkRight;

    public static double toPoleBack = 48;
    public static double toPoleLineX = 34;
    public static double toPoleLineY = -4;
    public static double toPoleLineH = -22;

    public static double parkCenterLineX = 33;
    public static double parkCenterLineY = -15;
    public static double parkCenterLineH = 0;

    public static double parkLeftMove = 23;
    public static double parkLeftTurn = 90;

    public static double parkRightMove = 23;
    public static double parkRightTurn = 90;

    @Override
    public void init() {
        drive = new HWMap(hardwareMap);

        //COLOR SENSOR
        drive.colorSensor.enableLed(true);

        // Camera Init
        int cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );

        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Loading Pipeline
        OpenCVPipeline visionPipeline = new OpenCVPipeline();
        AprilTagPipeline aprilTagDetectionPipeline = new AprilTagPipeline();

        // Start Streaming
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        // Stream Camera Onto Dash
        FtcDashboard.getInstance().startCameraStream(camera, 30);

        // Output Log
        telemetry.addData("Status", "Pipeline Initializing");
        telemetry.update();

        // TRAJECTORIES
        Pose2d startPose = new Pose2d(34, -72 + (15.5 / 2), Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        toPole = drive.trajectorySequenceBuilder(startPose)
                .back(toPoleBack)
                .lineToLinearHeading(new Pose2d(toPoleLineX, toPoleLineY, Math.toRadians(toPoleLineH)))
                .build();

        parkLeft = drive.trajectorySequenceBuilder(toPole.end())
                .lineToLinearHeading(new Pose2d(parkCenterLineX, parkCenterLineY, Math.toRadians(parkCenterLineH)))
                .back(parkLeftMove)
                .turn(Math.toRadians(parkLeftTurn))
                .back(10)
                .build();

        parkCenter = drive.trajectorySequenceBuilder(toPole.end())
                .lineToLinearHeading(new Pose2d(parkCenterLineX, parkCenterLineY, Math.toRadians(parkCenterLineH)))
                .turn(Math.toRadians(90))
                .back(10)
                .build();

        parkRight = drive.trajectorySequenceBuilder(toPole.end())
                .lineToLinearHeading(new Pose2d(parkCenterLineX, parkCenterLineY, Math.toRadians(parkCenterLineH)))
                .forward(parkRightMove)
                .turn(Math.toRadians(parkRightTurn))
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
        drive.intakeAngle.setPosition(intakeAngle4);
        drive.clawRotate.setPosition(clawRotate1);
        drive.clawAngle.setPosition(clawAngle4);

        clawOpen();

        // PID
        liftController = new PIDController(pL, iL, dL);

        extendController = new PIDController(pE, iE, dE);

        liftController.setTolerance(7);
        extendController.setTolerance(7);

        // Detection
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Auto", "Init");
        telemetry.update();

//        while (visiontime.seconds() <= 40) {
//            if (!pipelineLock) {
//                runtime.reset();
//                pipelineLock = true;
//            }
//
//            if (runtime.seconds() <= 0.3) {
//                // Start Auto
//                camera.setPipeline(visionPipeline);
//                CVconePos = visionPipeline.getPosition().name();
//                telemetry.addData("CV Position", visionPipeline.getPosition());
//                telemetry.addData("CV Analysis", visionPipeline.getAnalysis());
//            } else if (runtime.seconds() <= 0.6 && runtime.seconds() > 0.3) {
//                camera.setPipeline(aprilTagDetectionPipeline);
//                ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
//                if (ATLock) {
//                    if (detections != null) {
//                        if (detections.size() != 0) {
//                            int aprilTagID = detections.get(0).id;
//                            switch (aprilTagID) {
//                                case 1:
//                                    ATconePos = "LEFT";
//                                    break;
//                                case 2:
//                                    ATconePos = "CENTER";
//                                    break;
//                                default:
//                                    ATconePos = "RIGHT";
//                                    break;
//                            }
////                            ATLock = false;
//                        }
//                    }
//                }
//                telemetry.addData("AT Position", ATconePos);
//                telemetry.update();
//            } else {
//                pipelineLock = false;
//            }
//
//            if (ATconePos == "NOT_SET") {
//                signal = CVconePos;
//            } else {
//                signal = ATconePos;
//            }
//
//            telemetry.addData("Signal", signal);
//            telemetry.addData("Analysis", visionPipeline.getAnalysis());
//            telemetry.update();
//        }
//        telemetry.addData("Loop Done", "Yes");
//        telemetry.update();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        double liftPos = drive.leftVerticalSlide.getCurrentPosition();
        double extensionPos = drive.leftHorizontalSlide.getCurrentPosition();
        double colorBlue = drive.colorSensor.blue();
        double colorRed = drive.colorSensor.red();

        // Starts cycle command
        switch (cycleState) {
            case START:
                FtcDashboard.getInstance().stopCameraStream();
                if (!startLock) {
                    drive.followTrajectorySequence(toPole);
                    runtime.reset();
                    cycletime.reset();
                    startLock = true;
                }
                cycleState = CycleState.DEPOSIT;
                break;

            case INTAKE:
                if (!timeLock) {
                    cycletime.reset();
                    timeLock = true;
                }
                if (liftPos < stabilizerVertical) {
                    drive.stabilizer.setPosition(stabilizer2);
                }
                setLiftSLow(0);
                // Bring intake up
                if (cycletime.seconds() >= 0.4) {
                    if (!clawLock) {
                        clawClose();
                        clawLock = true;
                    }
                    if (cycletime.seconds() >= 0.62) {
                        if (!clawAngleLock) {
                            //clawangle3, intakeangle3
                            drive.clawAngle.setPosition(0.27);
                            drive.intakeAngle.setPosition(0.31);
                            clawAngleLock = true;
                        }
                        if (cycletime.seconds() >= 1) {
                            setExtension(-5);
                            drive.clawRotate.setPosition(clawRotate2);
                            if (cycletime.seconds() >= 1.6) {
                                drive.clawAngle.setPosition(clawAngle2);
                                drive.intakeAngle.setPosition(intakeAngle2);
                                if (cycletime.seconds() >= 2) {
                                    clawOpen();
                                    if (cycletime.seconds() >= 2.3) {
                                        cycletime.reset();
                                        cycleState = CycleState.DEPOSIT;
                                    }
                                }
                            } else {
                                if(!colorLock){
                                    if (colorBlue > 500 || colorRed > 500) {
                                        colorFail = true;
                                        colorLock = true;
                                        cycletime.reset();
                                        cycleState = CycleState.PARK;
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
                if (cycletime.seconds() >= 1.1) {
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
                if(colorFail == true) {
                    drive.intakeAngle.setPosition(intakeAngle3);
                    drive.intakeAngle.setPosition(clawAngle2);
                    if (cycletime.seconds() >= 1) {
                        if (!parkLock) {
                            switch (signal) {
                                case "LEFT":
                                    drive.followTrajectorySequence(parkLeft);
                                    parkLock = true;
                                    break;
                                case "CENTER":
                                    drive.followTrajectorySequence(parkCenter);
                                    parkLock = true;
                                    break;
                                case "RIGHT":
                                    drive.followTrajectorySequence(parkRight);
                                    parkLock = true;
                                    break;
                            }
                        } else {
                            telemetry.addData("Auto", "Parking");
                            requestOpModeStop();
                        }
                    }
                } else {
                    setLiftSLow(0);
                    drive.intakeAngle.setPosition(intakeAngle3);
                    drive.stabilizer.setPosition(stabilizer2);
                    if (cycletime.seconds() >= 1) {
                        if (!parkLock) {
                            switch (signal) {
                                case "LEFT":
                                    drive.followTrajectorySequence(parkLeft);
                                    parkLock = true;
                                    break;
                                case "CENTER":
                                    drive.followTrajectorySequence(parkCenter);
                                    parkLock = true;
                                    break;
                                case "RIGHT":
                                    drive.followTrajectorySequence(parkRight);
                                    parkLock = true;
                                    break;
                            }
                        } else {
                            telemetry.addData("Auto", "Parking");
                            requestOpModeStop();
                        }
                    }
                }
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
        telemetry.addData("Red", drive.colorSensor.red());
        telemetry.addData("Blue", drive.colorSensor.blue());
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

        drive.leftVerticalSlide.setPower(Range.clip(pidLift, -1, 1) * 0.84);
        drive.rightVerticalSlide.setPower(Range.clip(pidLift, -1, 1) * 0.84);
    }

    public void setLiftSLow(int target) {
        liftController.setPID(0.015, 0.0001, 0.0001);
        pidLift = liftController.calculate(drive.rightVerticalSlide.getCurrentPosition(), target);

        drive.leftVerticalSlide.setPower(Range.clip(pidLift, -1, 1) * 0.4);
        drive.rightVerticalSlide.setPower(Range.clip(pidLift, -1, 1) * 0.4);
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