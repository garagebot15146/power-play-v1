package org.firstinspires.ftc.teamcode.tata.OpenCVDetector;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


public class OpenCVDetectorHW {

    private OpenCvWebcam webcamMain;
    private OpenCvWebcam webcamIntake;
    private Telemetry telemetry;
    private CapstoneDetector detector = null;
    private IntakeCapstoneDetector intakeDetector = null;

    private tataAutonomousBase.SideColor currSide;
    private OpenCVDetectorDriver.RobotCamera camera_type;

    public void init(HardwareMap ahwMap, OpenCVDetectorDriver.RobotCamera cameraType, tataAutonomousBase.SideColor sc, Telemetry t) {
        telemetry = t;
        camera_type = cameraType;
        if (cameraType == OpenCVDetectorDriver.RobotCamera.MAIN) {
            RobotLog.ii("SHANK", "OpenCVInit - enter, MAIN");
            //Main Camera that detects the team markers
            int cameraMonitorViewId = ahwMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                    "id",ahwMap.appContext.getPackageName());
            webcamMain = OpenCvCameraFactory.getInstance().createWebcam(ahwMap.get(WebcamName.class, "mainWC"), cameraMonitorViewId);
            detector = new CapstoneDetector(telemetry, sc);
            webcamMain.setPipeline(detector);
            webcamMain.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcamMain.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                }
            });
        } else {
            RobotLog.ii("SHANK", "OpenCVInit - enter, INTAKE ");
            //Intake camera
            webcamIntake = OpenCvCameraFactory.getInstance().createWebcam(ahwMap.get(WebcamName.class, "intakeWC"));
            intakeDetector = new IntakeCapstoneDetector(telemetry);
            webcamIntake.setPipeline(intakeDetector);
            webcamIntake.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcamIntake.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                }
            });
        }

        currSide = sc;
        RobotLog.ii("C1234", "OpenCVInit - exit");

    }

    public void stop(){
        if (camera_type == OpenCVDetectorDriver.RobotCamera.MAIN) {
            RobotLog.ii("C1234", "OpenCVInit - Stop enter MAIN");
            webcamMain.stopStreaming();

        } else {
            RobotLog.ii("C1234", "OpenCVInit - Stop enter INTAKE");
            webcamIntake.stopStreaming();
        }

    }

    public int getLocation() {

        int markerLoc = 0;
        if (camera_type == OpenCVDetectorDriver.RobotCamera.MAIN) {
            if (currSide == tataAutonomousBase.SideColor.Red) {
                switch (detector.getLocation()) {
                    case pos1:
                        markerLoc = 1;
                        break;
                    case pos2:
                        markerLoc = 2;
                        break;
                    case pos3:
                        markerLoc = 3;
                        break;
                }

            } else {
                //Blue
                switch (detector.getLocation()) {
                    case pos1:
                        markerLoc = 2;
                        break;
                    case pos2:
                        markerLoc = 3;
                        break;
                    case pos3:
                        markerLoc = 1;
                        break;
                }
            }
        } else {
            //Intake camera
            switch (intakeDetector.getIntakeStatus()) {
                case FULL:
                    markerLoc = 1;
                    break;
                case EMPTY:
                    markerLoc = 0;
                    break;
            }
        }
        //RobotLog.ii("C1234", "OpenCVInit - markerloc %d", markerLoc);

        return markerLoc;
    }


}

