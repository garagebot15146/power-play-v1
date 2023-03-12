package org.firstinspires.ftc.teamcode.Commands.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class LiftSubsystem extends SubsystemBase {
    public DcMotorEx leftVerticalSlide, rightVerticalSlide;
    public Servo stabilizer;

    PIDController controller;
    private int position = 0;
    public static double pL = 0.03, iL = 0.001, dL = 0.0004;

    public LiftSubsystem(HardwareMap hardwareMap) {
        leftVerticalSlide = hardwareMap.get(DcMotorEx.class, "leftVerticalSlide");
        rightVerticalSlide = hardwareMap.get(DcMotorEx.class, "rightVerticalSlide");
        stabilizer = hardwareMap.get(Servo.class, "stabilizer");

        leftVerticalSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightVerticalSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        leftVerticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVerticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftVerticalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightVerticalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stabilizer.setPosition(0.15);

        controller = new PIDController(pL, iL, dL);
        controller.setTolerance(5);
    }

    public void loop() {
        if(position > 50){
//            controller.setPID(pL, iL, dL);
        } else {
            controller.setPID(0.015, 0.0001, 0.0001);
        }
        double power = controller.calculate(leftVerticalSlide.getCurrentPosition(), position);
        if(position > 50){
            leftVerticalSlide.setPower(power);
            rightVerticalSlide.setPower(power);
        } else {
            leftVerticalSlide.setPower(Range.clip(power, -1, 1) * 0.8);
            rightVerticalSlide.setPower(Range.clip(power, -1, 1) * 0.8);
        }
    }

    public void setTarget(String pole) {
        switch (pole) {
            case "BOTTOM":
                stabilizer.setPosition(0.15);
                position = 5;
                break;
            case "MEDIUM":
                stabilizer.setPosition(0);
                position = 520;
                break;
            case "HIGH":
                stabilizer.setPosition(0);
                position = 876;
                break;
        }
    }

    public int position() {
        return leftVerticalSlide.getCurrentPosition();
    }

    public void pullDown(){
        position = 10;
    }

    public boolean isReached() {
        return Math.abs(position - leftVerticalSlide.getCurrentPosition()) < 10;
    }
}
