package org.firstinspires.ftc.teamcode.Commands.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class ExtendSubsystem extends SubsystemBase {
    public DcMotorEx leftHorizontalSlide, rightHorizontalSlide;

    PIDController controller;
    private int position = 0;
    private final double pL = 0.04;
    private final double iL = 0.001;
    private final double dL = 0.0004;

    public ExtendSubsystem(HardwareMap hardwareMap) {
        leftHorizontalSlide = hardwareMap.get(DcMotorEx.class, "leftHorizontalSlide");
        rightHorizontalSlide = hardwareMap.get(DcMotorEx.class, "rightHorizontalSlide");

        leftHorizontalSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightHorizontalSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftHorizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHorizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftHorizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightHorizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void loop() {
        controller = new PIDController(pL, iL, dL);
        if(position < 300){
            controller.setTolerance(10);
        } else {
            controller.setTolerance(5);
        }
        double power = controller.calculate(leftHorizontalSlide.getCurrentPosition(), position);
        if(position < 300){
            leftHorizontalSlide.setPower(Range.clip(power, -1, 1) * 0.75);
            rightHorizontalSlide.setPower(Range.clip(power, -1, 1) * 0.75);
        } else {
            leftHorizontalSlide.setPower(Range.clip(power, -1, 1) * 0.72);
            rightHorizontalSlide.setPower(Range.clip(power, -1, 1) * 0.72);
        }
    }

    public void setTarget(int target) {
        position = target;
    }

    public int position() {
        return leftHorizontalSlide.getCurrentPosition();
    }


    public boolean isReached() {
        return Math.abs(position - leftHorizontalSlide.getCurrentPosition()) < 10;
    }
}
