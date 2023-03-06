package org.firstinspires.ftc.teamcode.tata.OpenCVDetector;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;
import org.firstinspires.ftc.teamcode.tata.RobotCarousel.RC.RobotCarouselHW;
import org.firstinspires.ftc.teamcode.tata.RobotCarousel.RC.RobotCarouselParams;


public class OpenCVDetectorDriver implements Runnable{
    public enum RobotCamera {
        MAIN,
        INTAKE
    }
    private OpenCVDetectorHW hw = new OpenCVDetectorHW();

    private boolean is_done  = true;

    //Thead run condition
    private boolean isRunning = true;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    private int markerPos = 0;
    private RobotCamera cameraType;

    public OpenCVDetectorDriver(HardwareMap ahwMap, int threadSleepDelay, RobotCamera ct, tataAutonomousBase.SideColor sc, Telemetry t){
        hw.init(ahwMap, ct, sc, t);
        sleepTime = threadSleepDelay;
        cameraType = ct;
    }

    private void markerPosUpdate(){

        markerPos = hw.getLocation();
    }

    public int getMarkerPos(){
        return markerPos;
    }

    public void stop(){
        isRunning = false;
        hw.stop();
    }

    @Override
    public void run() {
        while(isRunning) {
            markerPosUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
