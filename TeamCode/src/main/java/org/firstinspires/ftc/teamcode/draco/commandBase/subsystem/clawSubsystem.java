package org.firstinspires.ftc.teamcode.draco.commandBase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class clawSubsystem extends SubsystemBase {

    // Servos in the claw subsystem
    private Servo elbowServo;    //??
    private Servo wristServo;      //??
    private Servo rotatorServo;     //Rotate the clow
    private Servo clawServo;       //Used to open or close claw


    //Servo angles for Elbow Joint of the claw subsystem
    private static double[] elbowServoPickPos = {0.2, 0.715, 0.67, 0.6, 0.585, 0.45};
    private static double[] elbowServoDropPos = {0.0, 0.715, 0.67, 0.6, 0.585, 0.45};

    //States for Elbow Joint of the claw subsystem
    public enum ElbowPos {
        INIT_POS,
        PICK_CONE_1,
        PICK_CONE_2,
        PICK_CONE_3,
        PICK_CONE_4,
        PICK_CONE_5,
        DROP_CONE_1,
        DROP_CONE_2,
        DROP_CONE_3,
        DROP_CONE_4,
        DROP_CONE_5
    }

    public static double[] wristServoPickPos = {0, 0.02, 0.02, 0.02, 0.04, 0.04};
    public static double[] wristServoDropPos = {0, 0.02, 0.02, 0.02, 0.04, 0.04};

    public enum WristPos {
        INIT_POS,
        PICK_CONE_1,
        PICK_CONE_2,
        PICK_CONE_3,
        PICK_CONE_4,
        PICK_CONE_5,
        DROP_CONE_1,
        DROP_CONE_2,
        DROP_CONE_3,
        DROP_CONE_4,
        DROP_CONE_5
    }

    private static double claw_rotator_pick     = 0.23;
    private static double claw_rotator_drop     = 1;

    public enum RotatorState {
        PICK,
        DROP,
    }

    private static double claw_pos_open     = 1;
    private static double claw_pos_closed = 0.7;

    public enum ClawState {
        OPEN,
        CLOSED,
    }

    public clawSubsystem(HardwareMap hardwareMap, boolean isAuto) {

        elbowServo   = hardwareMap.get(Servo.class, "intakeAngle");  //rename this to elbow
        wristServo   = hardwareMap.get(Servo.class, "clawAngle");    //rename this to wrist
        rotatorServo = hardwareMap.get(Servo.class, "clawRotate");   //rename this to rotator
        clawServo    = hardwareMap.get(Servo.class, "claw");         //

        //Initialize the claw subsystem to a default state
        update(ElbowPos.INIT_POS);
        update(WristPos.INIT_POS);
        update(RotatorState.DROP);
        update(ClawState.CLOSED);

    }

    public void update(ElbowPos state) {
        switch (state) {
            case INIT_POS:
                elbowServo.setPosition(elbowServoPickPos[0]);
                break;
            case PICK_CONE_1:
                elbowServo.setPosition(elbowServoPickPos[1]);
                break;
            case PICK_CONE_2:
                elbowServo.setPosition(elbowServoPickPos[2]);
                break;
            case PICK_CONE_3:
                elbowServo.setPosition(elbowServoPickPos[3]);
                break;
            case PICK_CONE_4:
                elbowServo.setPosition(elbowServoPickPos[4]);
                break;
            case PICK_CONE_5:
                elbowServo.setPosition(elbowServoPickPos[5]);
                break;
            case DROP_CONE_1:
                elbowServo.setPosition(elbowServoDropPos[1]);
                break;
            case DROP_CONE_2:
                elbowServo.setPosition(elbowServoDropPos[2]);
                break;
            case DROP_CONE_3:
                elbowServo.setPosition(elbowServoDropPos[3]);
                break;
            case DROP_CONE_4:
                elbowServo.setPosition(elbowServoDropPos[4]);
                break;
            case DROP_CONE_5:
                elbowServo.setPosition(elbowServoDropPos[5]);
                break;
        }
    }

    public void update(WristPos state) {
        switch (state) {
            case INIT_POS:
                wristServo.setPosition(wristServoPickPos[0]);
                break;
            case PICK_CONE_1:
                wristServo.setPosition(wristServoPickPos[1]);
                break;
            case PICK_CONE_2:
                wristServo.setPosition(wristServoPickPos[2]);
                break;
            case PICK_CONE_3:
                wristServo.setPosition(wristServoPickPos[3]);
                break;
            case PICK_CONE_4:
                wristServo.setPosition(wristServoPickPos[4]);
                break;
            case PICK_CONE_5:
                wristServo.setPosition(wristServoPickPos[5]);
                break;
            case DROP_CONE_1:
                wristServo.setPosition(wristServoDropPos[1]);
                break;
            case DROP_CONE_2:
                wristServo.setPosition(wristServoDropPos[2]);
                break;
            case DROP_CONE_3:
                wristServo.setPosition(wristServoDropPos[3]);
                break;
            case DROP_CONE_4:
                wristServo.setPosition(wristServoDropPos[4]);
                break;
            case DROP_CONE_5:
                wristServo.setPosition(wristServoDropPos[5]);
                break;
        }
    }

    public void update(RotatorState state) {
        switch (state) {
            case PICK:
                rotatorServo.setPosition(claw_rotator_pick);
                break;
            case DROP:
                rotatorServo.setPosition(claw_rotator_drop);
                break;
        }
    }

    public void update(ClawState state) {
        switch (state) {
            case OPEN:
                clawServo.setPosition(claw_pos_open);
                break;
            case CLOSED:
                clawServo.setPosition(claw_pos_closed);
                break;
        }
    }



}
