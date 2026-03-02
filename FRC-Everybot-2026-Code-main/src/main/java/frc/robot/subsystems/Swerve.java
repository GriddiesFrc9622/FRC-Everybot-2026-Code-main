package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.stream.Collector.Characteristics;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private final double TRACK_WIDTH_INCHES = 24.5;
    private final double WHEEL_BASE_INCHES = 23;

    private final double TRACK_WIDTH_METERS = Units.inchesToMeters(TRACK_WIDTH_INCHES);
    private final double WHEEL_BASE_METERS = Units.inchesToMeters(WHEEL_BASE_INCHES);

    private final SwerveDriveKinematics Kinematics = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
            new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / -2),
            new Translation2d(WHEEL_BASE_METERS / -2, TRACK_WIDTH_METERS / 2),
            new Translation2d(WHEEL_BASE_METERS / -2, TRACK_WIDTH_METERS / -2));
    private final double STEERING_MOTOR_REDUCTION = 9424.0 / 203.0;
    private final double DRIVE_MOTOR_PINION_TEETH = 13;
    private final double DRIVE_MOTOR_REDUCTION = (22.0 / DRIVE_MOTOR_PINION_TEETH) * (45.0 / 15.0);
    private final double DRIVE_WHEEL_DIAMETER_INCHES = 3;
    private final double DRIVE_WHEEL_CIRCUMFERANCE = Math.PI * DRIVE_WHEEL_DIAMETER_INCHES;
    private final double DRIVE_WHEEL_CIRCUMFERANCE_METERS = Units.inchesToMeters(DRIVE_WHEEL_CIRCUMFERANCE);
    private final double MAX_VELOCITY_METERS_PER_SEC = 4.46 * 0.9;
    private double MAX_ROTATION = 2 * Math.PI;
    private SwerveModule frontleft;
    private SwerveModule frontright;
    private SwerveModule backleft;
    private SwerveModule backright;
    private Pigeon2 imuPigeon2;

    public Swerve() {
        frontleft = new SwerveModule(11, 21, Math.toRadians(-90));
        frontright = new SwerveModule(12, 22, Math.toRadians(0));
        backleft = new SwerveModule(13, 23, Math.toRadians(180));
        backright = new SwerveModule(14, 24, Math.toRadians(90));
        imuPigeon2 = new Pigeon2(0);

        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            // Configure AutoBuilder last
            AutoBuilder.configure(
                    this::getPose, // Robot pose supplier
                    this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given
                                                                          // ROBOT
                                                                          // RELATIVE ChassisSpeeds. Also optionally
                                                                          // outputs
                                                                          // individual module feedforwards
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller
                                                    // for
                                                    // holonomic drive trains
                            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                    ),
                    config, // The robot configuration
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red
                        // alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
    }

    private Pose2d getPose() {
        return null;

    }

    private void resetPose(Pose2d Pose) {

    }

    private ChassisSpeeds getRobotRelativeSpeeds() {
        return null;


    }

    private void driveRobotRelative(ChassisSpeeds speeds) {
           SwerveModuleState[] states = Kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(speeds, 1.0 / 50.0));
            SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SEC);

            frontleft.SetDesiredState(states[0]);
            frontright.SetDesiredState(states[1]);
            backleft.SetDesiredState(states[2]);
            backright.SetDesiredState(states[3]);
        
    }

    public Command driveJoystick(DoubleSupplier forwardpercent, DoubleSupplier sidetoside, DoubleSupplier rotation) {
        return this.run(() -> {
            double forwardspeedMetersPerSecond = forwardpercent.getAsDouble() * MAX_VELOCITY_METERS_PER_SEC;
            double sidetosidespeedMetersPerSecond = sidetoside.getAsDouble() * MAX_VELOCITY_METERS_PER_SEC;
            double rotationperseconds = rotation.getAsDouble() * MAX_ROTATION;

            ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forwardspeedMetersPerSecond,
                    sidetosidespeedMetersPerSecond, rotationperseconds, imuPigeon2.getRotation2d());
           driveRobotRelative(speeds);
        }).withName("Swerve.driveJoystick");
    }

}
