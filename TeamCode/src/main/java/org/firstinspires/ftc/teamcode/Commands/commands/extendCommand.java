package org.firstinspires.ftc.teamcode.Commands.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Commands.subsystem.ExtendSubsystem;

public class extendCommand extends CommandBase {

    private final ExtendSubsystem extendSubsystem;
    private int target;

    public extendCommand(ExtendSubsystem subsystem, int target) {
        this.target = target;
        extendSubsystem = subsystem;
        addRequirements(extendSubsystem);
    }


    @Override
    public void initialize() {
        extendSubsystem.setTarget(target);
    }

    @Override
    public boolean isFinished() {
        return extendSubsystem.isReached();
    }

}
