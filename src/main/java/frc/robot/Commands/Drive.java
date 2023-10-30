package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class Drive extends CommandBase {
    DriveSubsystem drivetrain;
    DoubleSupplier fwdSupplier;
    DoubleSupplier rotSupplier;

    public Drive(DriveSubsystem drivetrain, DoubleSupplier fwdSupplier, DoubleSupplier rotSupplier) {
        this.drivetrain = drivetrain;
        this.fwdSupplier = fwdSupplier;
        this.rotSupplier = rotSupplier;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        drivetrain.arcadeDrive(fwdSupplier.getAsDouble(), rotSupplier.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.arcadeDrive(0, 0);
    }
}
