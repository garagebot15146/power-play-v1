package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Settings.OpenCVPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "detectionCV", group = "Tests")
//@Disabled
public class detectionCV extends LinearOpMode {

    OpenCvCamera camera;

    public String stack = "";


    @Override
    public void runOpMode() {
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

        // Loading pipeline
        OpenCVPipeline visionPipeline = new OpenCVPipeline();
        camera.setPipeline(visionPipeline);

        // Start Streaming
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        // Stream Camera
        FtcDashboard.getInstance().startCameraStream(camera, 30);


        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("CV Position", visionPipeline.getPosition());
            telemetry.addData("CV Analysis", visionPipeline.getAnalysis());
            telemetry.update();
        }

        FtcDashboard.getInstance().stopCameraStream();

    }

}


