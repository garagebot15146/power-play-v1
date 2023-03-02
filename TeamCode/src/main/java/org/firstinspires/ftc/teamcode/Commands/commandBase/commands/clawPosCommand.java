package org.firstinspires.ftc.teamcode.Commands.commandBase.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Commands.commandBase.subsystem.ClawSubsystem;

public class clawPosCommand extends CommandBase {

    private final ClawSubsystem clawSubsystem;

    public clawPosCommand(ClawSubsystem subsystem) {
        clawSubsystem = subsystem;
        addRequirements(clawSubsystem);
    }


    @Override
    public void initialize() {
        clawSubsystem.update(ClawSubsystem.ElbowPos.PICK_CONE_1);
        clawSubsystem.update(ClawSubsystem.WristPos.PICK_CONE_1);
        clawSubsystem.update(ClawSubsystem.RotatorState.PICK);
        clawSubsystem.update(ClawSubsystem.ClawState.OPEN);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
