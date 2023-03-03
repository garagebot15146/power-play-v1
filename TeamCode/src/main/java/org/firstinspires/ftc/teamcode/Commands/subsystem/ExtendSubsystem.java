package org.firstinspires.ftc.teamcode.Commands.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class ExtendSubsystem extends SubsystemBase {
    public DcMotorEx leftHorizontalSlide, rightHorizontalSlide;

    PIDController controller;
    private int position = 0;
    private final double pL = 0.02;
    private final double iL = 0;
    private final double dL = 0.0001;


    public ExtendSubsystem(HardwareMap hardwareMap) {
        leftHorizontalSlide = hardwareMap.get(DcMotorEx.class, "leftHorizontalSlide");
        rightHorizontalSlide = hardwareMap.get(DcMotorEx.class, "rightHorizontalSlide");

        leftHorizontalSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightHorizontalSlide.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop() {
        controller = new PIDController(pL, iL, dL);
        controller.setTolerance(5);
        double power = controller.calculate(leftHorizontalSlide.getCurrentPosition(), position);
        leftHorizontalSlide.setPower(power);
        rightHorizontalSlide.setPower(power);
    }

    public void setTarget(int target) {
        position = target;
    }

    public int position() {
        return leftHorizontalSlide.getCurrentPosition();
    }


    public boolean isReached() {
        return Math.abs(position - leftHorizontalSlide.getCurrentPosition()) < 5;
    }
}
