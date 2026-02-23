package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {

    public Swerve() {

    }

    public Command driveJoystick (DoubleSupplier forwardpercent, DoubleSupplier sidetoside, DoubleSupplier rotation) {
        return this.run(() -> {

        }).withName("Swerve.driveJoystick");
    }

}

