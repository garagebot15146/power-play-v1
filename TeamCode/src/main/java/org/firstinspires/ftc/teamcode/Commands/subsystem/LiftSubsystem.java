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
    private final double pL = 0.04;
    private final double iL = 0.001;
    private final double dL = 0.001;

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
    }

    public void loop() {
        controller = new PIDController(pL, iL, dL);
        controller.setTolerance(10);
        double power = controller.calculate(leftVerticalSlide.getCurrentPosition(), position);
        if (power > 0) {
            leftVerticalSlide.setPower(power);
            rightVerticalSlide.setPower(power);
        } else {
            leftVerticalSlide.setPower(Range.clip(power, -1, 1) * 0.65);
            rightVerticalSlide.setPower(Range.clip(power, -1, 1) * 0.65);
        }
    }

    public void setTarget(String pole) {
        switch (pole) {
            case "BOTTOM":
                stabilizer.setPosition(0.08);
                position = 5;
                break;
            case "MEDIUM":
                stabilizer.setPosition(0);
                position = 350;
                break;
            case "HIGH":
                stabilizer.setPosition(0);
                position = 655;
                break;
        }
    }

    public int position() {
        return leftVerticalSlide.getCurrentPosition();
    }


    public boolean isReached() {
        return Math.abs(position - leftVerticalSlide.getCurrentPosition()) < 10;
    }
}
