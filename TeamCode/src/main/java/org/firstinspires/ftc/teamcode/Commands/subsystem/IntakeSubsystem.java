package org.firstinspires.ftc.teamcode.Commands.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem extends SubsystemBase {

    // Servos in the claw subsystem
    private Servo elbowServo;
    private Servo wristServo;
    private Servo rotatorServo;
    private Servo clawServo;


    //Servo angles for Elbow Joint of the claw subsystem
    private static double[] elbowServoPickPos = {0.27, 0.9, 0.75, 0.675, 0.62, 0.55};
    private static double elbowServoDropPos = 0.2;
    private static double elbowServoLiftPos = 0.28;

    //States for Elbow Joint of the claw subsystem
    public enum ElbowPos {
        INIT_POS,
        PICK_CONE_1,
        PICK_CONE_2,
        PICK_CONE_3,
        PICK_CONE_4,
        PICK_CONE_5,
        DROP_CONE,
        LIFT_CONE
    }

    public static double[] wristServoPickPos = {0.6, 0.01, 0.03, 0.01, 0.04, 0};
    public static double wristServoDropPos = 0.63;
    public static double wristServoMidPos = 0.54;
    public static double wristServoLiftPos = 0.4;

    public enum WristPos {
        INIT_POS,
        PICK_CONE_1,
        PICK_CONE_2,
        PICK_CONE_3,
        PICK_CONE_4,
        PICK_CONE_5,
        DROP_CONE,
        LIFT_CONE,
        MID_CONE
    }

    private static double claw_rotator_pick = 1;
    private static double claw_rotator_drop = 0.23;

    public enum RotatorState {
        PICK,
        DROP,
    }

    private static double claw_pos_open = 1;
    private static double claw_pos_closed = 0.4;

    public enum ClawState {
        OPEN,
        CLOSED,
    }

    public IntakeSubsystem(HardwareMap hardwareMap) {

        elbowServo = hardwareMap.get(Servo.class, "intakeAngle");  //rename this to elbow
        wristServo = hardwareMap.get(Servo.class, "clawAngle");    //rename this to wrist
        rotatorServo = hardwareMap.get(Servo.class, "clawRotate");   //rename this to rotator
        clawServo = hardwareMap.get(Servo.class, "claw");         //

        //Initialize the claw subsystem to a default state
        update(RotatorState.DROP);
        update(ElbowPos.INIT_POS);
        update(WristPos.INIT_POS);

    }

    public void down(int cones) {
        switch (cones) {
            case 0:
                update(ElbowPos.INIT_POS);
                update(WristPos.INIT_POS);
                break;

            case 1:
                update(ClawState.OPEN);
                update(RotatorState.PICK);
                update(ElbowPos.PICK_CONE_1);
                update(WristPos.PICK_CONE_1);
                break;

            case 2:
                update(ClawState.OPEN);
                update(RotatorState.PICK);
                update(ElbowPos.PICK_CONE_2);
                update(WristPos.PICK_CONE_2);
                break;

            case 3:
                update(ClawState.OPEN);
                update(RotatorState.PICK);
                update(ElbowPos.PICK_CONE_3);
                update(WristPos.PICK_CONE_3);
                break;

            case 4:
                update(ClawState.OPEN);
                update(RotatorState.PICK);
                update(ElbowPos.PICK_CONE_4);
                update(WristPos.PICK_CONE_4);
                break;

            case 5:
                update(ClawState.OPEN);
                update(RotatorState.PICK);
                update(ElbowPos.PICK_CONE_5);
                update(WristPos.PICK_CONE_5);
                break;

            case 6:
                update(ElbowPos.DROP_CONE);
                update(WristPos.DROP_CONE);
                break;
        }
    }

    public void lift(){
        update(WristPos.LIFT_CONE);
        update(ElbowPos.LIFT_CONE);
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
            case DROP_CONE:
                elbowServo.setPosition(elbowServoDropPos);
                break;
            case LIFT_CONE:
                elbowServo.setPosition(elbowServoLiftPos);
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
            case DROP_CONE:
                wristServo.setPosition(wristServoDropPos);
                break;
            case LIFT_CONE:
                wristServo.setPosition(wristServoLiftPos);
                break;
            case MID_CONE:
                wristServo.setPosition(wristServoMidPos);
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
