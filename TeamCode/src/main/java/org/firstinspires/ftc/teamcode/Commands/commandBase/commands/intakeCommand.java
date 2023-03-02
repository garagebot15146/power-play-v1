package org.firstinspires.ftc.teamcode.Commands.commandBase.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Commands.commandBase.subsystem.IntakeSubsystem;

public class intakeCommand extends CommandBase {

    private final IntakeSubsystem clawSubsystem;

    public intakeCommand(IntakeSubsystem subsystem) {
        clawSubsystem = subsystem;
        addRequirements(clawSubsystem);
    }


    @Override
    public void initialize() {
        clawSubsystem.update(IntakeSubsystem.ElbowPos.PICK_CONE_1);
        clawSubsystem.update(IntakeSubsystem.WristPos.PICK_CONE_1);
        clawSubsystem.update(IntakeSubsystem.RotatorState.PICK);
        clawSubsystem.update(IntakeSubsystem.ClawState.OPEN);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
