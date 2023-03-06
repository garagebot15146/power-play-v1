package org.firstinspires.ftc.teamcode.tata.OpenCVDetector;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;
import org.firstinspires.ftc.teamcode.tata.RobotCarousel.RC.RobotCaroselDriver;

@TeleOp(name = "testOpenCVDetector", group = "Test - TATA")
//@Disabled
public class testOpenCVDetector extends LinearOpMode {
    public OpenCVDetectorDriver driver;
    public int markerPos = 0;

    @Override
    public void runOpMode() {
        driver = new OpenCVDetectorDriver(hardwareMap, 200, OpenCVDetectorDriver.RobotCamera.MAIN, tataAutonomousBase.SideColor.Red, telemetry);
        //driver = new OpenCVDetectorDriver(hardwareMap, 200, OpenCVDetectorDriver.RobotCamera.MAIN, tataAutonomousBase.SideColor.Red, telemetry);
        Thread driverThread = new Thread(driver);
        driverThread.start();
        RobotLog.ii("C1234", "test - am here 1");

        // Wait for the game to begin
        while (!isStopRequested() && !isStarted()) {
            markerPos = driver.getMarkerPos();
            telemetry.addData(">", "Main: Marker position Found %d", markerPos);
            telemetry.update();

        }
        RobotLog.ii("C1234", "test - am here 2");


        waitForStart();
        RobotLog.ii("C1234", "test - am here 3");

        telemetry.addData(">", "Main: Marker position Found %d", markerPos);
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        sleep(10000);
        driver.stop();
        idle();
    }
}

