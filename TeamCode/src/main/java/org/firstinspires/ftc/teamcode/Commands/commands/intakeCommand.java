package org.firstinspires.ftc.teamcode.Commands.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Commands.subsystem.IntakeSubsystem;

public class intakeCommand extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;
    private int cones = 0;

    public intakeCommand(IntakeSubsystem subsystem, int cones) {
        intakeSubsystem = subsystem;
        this.cones = cones;
        addRequirements(intakeSubsystem);
    }


    @Override
    public void initialize() {
        intakeSubsystem.down(cones);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
