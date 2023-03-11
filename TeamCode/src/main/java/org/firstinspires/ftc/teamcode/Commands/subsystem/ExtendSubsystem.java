package org.firstinspires.ftc.teamcode.Commands.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ExtendSubsystem extends SubsystemBase {
    public DcMotorEx leftHorizontalSlide, rightHorizontalSlide;

    private DistanceSensor distanceSensor;
    boolean use_ds = false;

    PIDController controller;
    private int position = 0;
    private final double pL = 0.02;
    private final double iL = 0;
    private final double dL = 0.0001;

    public ExtendSubsystem(HardwareMap hardwareMap) {
        leftHorizontalSlide = hardwareMap.get(DcMotorEx.class, "leftHorizontalSlide");
        rightHorizontalSlide = hardwareMap.get(DcMotorEx.class, "rightHorizontalSlide");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        leftHorizontalSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightHorizontalSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftHorizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHorizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftHorizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightHorizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void loop() {
        controller = new PIDController(pL, iL, dL);
        if (position < 300) {
            controller.setTolerance(10);
        } else {
            controller.setTolerance(5);
        }
        double power = controller.calculate(leftHorizontalSlide.getCurrentPosition(), position);
        if (position < 300) {
            leftHorizontalSlide.setPower(Range.clip(power, -1, 1) * 0.9);
            rightHorizontalSlide.setPower(Range.clip(power, -1, 1) * 0.9);
        } else {
            leftHorizontalSlide.setPower(Range.clip(power, -1, 1) * 0.72);
            rightHorizontalSlide.setPower(Range.clip(power, -1, 1) * 0.72);
        }
    }

    public void setTarget(int target) {
        position = target;
        use_ds = false;
    }

    public int position() {
        return leftHorizontalSlide.getCurrentPosition();
    }

    public void pullIn(){
        position = 10;
    }

    public double distance() {
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }

    public boolean isReached() {
        boolean reached = false;

        if (use_ds == false) {
            //Coarse level measurement
            reached = Math.abs(position - leftHorizontalSlide.getCurrentPosition()) < 5;
            if (reached == true) {
                //switch to fine level measurements of using distance sensor
                use_ds = true;

                //if you are retracting then dont use distance sensor
                if (position < 300) {
                    use_ds = false;
                }
            }
        }
        if (use_ds == true) {
            //Check the distance sensor
            double dist = distanceSensor.getDistance(DistanceUnit.INCH);
            if (dist < 0.90 || position > 935) {
                reached = true;
            } else {
                position += 18;
                reached = false;
            }
        }

        return reached;
    }
}