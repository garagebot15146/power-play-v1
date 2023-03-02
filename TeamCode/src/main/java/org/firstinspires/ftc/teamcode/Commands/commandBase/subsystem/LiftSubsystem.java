package org.firstinspires.ftc.teamcode.Commands.commandBase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class LiftSubsystem extends SubsystemBase  {
    public DcMotorEx leftVerticalSlide, rightVerticalSlide;

    PIDController controller;
    private final int highPole = 645;
    private final int midPole = 348;
    private int position = 0;
    private final double pL = 0.05;
    private final double iL = 0.001;
    private final double dL = 0.001;

    public LiftSubsystem(HardwareMap hardwareMap){
        leftVerticalSlide = hardwareMap.get(DcMotorEx.class, "leftVerticalSlide");
        rightVerticalSlide = hardwareMap.get(DcMotorEx.class, "rightVerticalSlide");

        leftVerticalSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightVerticalSlide.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void loop() {
        controller = new PIDController(pL, iL, dL);
        controller.setTolerance(5);
        double power = controller.calculate(leftVerticalSlide.getCurrentPosition(), position);
        if(power > 0){
            leftVerticalSlide.setPower(power);
            rightVerticalSlide.setPower(power);
        } else {
            leftVerticalSlide.setPower(Range.clip(power, -1,1) * 0.5);
            rightVerticalSlide.setPower(Range.clip(power, -1,1) * 0.5);
        }
    }

    public void setTarget(int target){
        position = target;
    }

    public int position(){
        return leftVerticalSlide.getCurrentPosition();
    }


    public boolean isReached(){
        return Math.abs(position - leftVerticalSlide.getCurrentPosition()) < 5;
    }
}
