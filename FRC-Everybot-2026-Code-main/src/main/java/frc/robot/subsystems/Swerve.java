package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {

    public Swerve() {
        
    }

    public Command driveJoystick (double forwardpercent, double sidetoside, double rotation) {
        return this.run(() -> {

        }).withName("Swerve.driveJoystick");
    }

}

