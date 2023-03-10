package org.firstinspires.ftc.teamcode.Commands.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DistanceSensor;

public class DistanceSensorSubsystem extends SubsystemBase {
    private DistanceSensor ds = null;

    //Need to convert distance in inches to tick count used by extend sub system
    private static final double TICK_COUNTS_PER_INCH = (13.7 * 28) / (1 * 3.1415);

    public DistanceSensorSubsystem(HardwareMap hardwareMap) {
        ds = hardwareMap.get(DistanceSensor.class, "distanceSensor");
    }

    public void loop() {
        position = ds.getDistance(DistanceUnit.INCH);
    }

    public int position() {
        return ds.getDistance(DistanceUnit.INCH);
    }

    public int getPosInTicks() {
        return (ds.getDistance(DistanceUnit.INCH) * TICK_COUNTS_PER_INCH);
    }


    public boolean isReached() {
        return true;
    }
}
