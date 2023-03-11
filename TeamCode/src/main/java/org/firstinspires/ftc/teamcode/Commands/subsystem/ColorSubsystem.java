package org.firstinspires.ftc.teamcode.Commands.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorSubsystem extends SubsystemBase {
    public ColorSensor colorSensor;
    private static int coneVal;

    public ColorSubsystem(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
    }

    public boolean hasCone(){
        if(Math.abs(coneVal - colorSensor.alpha()) < 100){
            return true;
        } else {
            return false;
        }
    }

    public int color() {
        return colorSensor.alpha();
    }

}
