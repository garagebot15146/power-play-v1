package org.firstinspires.ftc.teamcode.tata.OpenCVDetector;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;

@TeleOp(name = "testOpenCVIntakeDetector", group = "Test - TATA")
//@Disabled
public class testOpenCVIntakeDetector extends LinearOpMode {
    public OpenCVDetectorDriver driver;
    public int markerPos = 0;

    @Override
    public void runOpMode() {
        driver = new OpenCVDetectorDriver(hardwareMap, 200, OpenCVDetectorDriver.RobotCamera.INTAKE, tataAutonomousBase.SideColor.Blue, telemetry);
        Thread driverThread = new Thread(driver);
        driverThread.start();

        // Wait for the game to begin
        while (!isStopRequested() && !isStarted()) {
            markerPos = driver.getMarkerPos();

        }
        driver.stop();


        waitForStart();
        telemetry.addData(">", "Intake: Marker Found %d", markerPos);
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        sleep(10000);
        idle();
    }
}

