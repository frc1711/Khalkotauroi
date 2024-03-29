package frc.robot.commands.auton.base;

import java.util.function.Function;

import claw.math.Vector;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveTeleopAccelerationConstraints;

public class DriveToPosition extends CommandBase {
    
    // Distances measured in meters, time in seconds, angles in radians
    private static final double
        MAX_STRAFE_VELOCITY_AT_END = 0.05,
        MAX_ANGULAR_VELOCITY_AT_END = 0.2,
        MAX_DIST_FROM_ENDPOINT = 0.02,
        MAX_RADIANS_FROM_ENDPOINT = 0.01;
    
    private final Swerve swerve;
    private final Function<Pose2d, Pose2d> robotPoseToTargetPose;
    private final PIDController
        xController = new PIDController(6, 0, 0),
        yController = new PIDController(6, 0, 0),
        thetaRadiansController = new PIDController(3, 0, 0);
    
    private static final double MAX_STRAFE_VELOCITY = 4.5, MAX_ANGULAR_VELOCITY = 4.5;
    private final SwerveTeleopAccelerationConstraints constraints = new SwerveTeleopAccelerationConstraints(14.5, 9);
    
    private Pose2d targetPose = new Pose2d();
    private ChassisSpeeds lastCommandedSpeeds = new ChassisSpeeds();
    
    /**
     * Drives the swerve drive to a final target pose
     */
    public DriveToPosition (Swerve swerve, Pose2d targetPose) {
        this(swerve, robotPose -> targetPose);
    }
    
    /**
     * Drives the swerve drive across the transform
     */
    public DriveToPosition (Swerve swerve, Transform2d transform) {
        this(swerve, robotPose -> robotPose.plus(transform));
    }
    
    private DriveToPosition (Swerve swerve, Function<Pose2d, Pose2d> robotPoseToTargetPose) {
        this.swerve = swerve;
        this.robotPoseToTargetPose = robotPoseToTargetPose;
        addRequirements(swerve);
    }
    
    @Override
    public void initialize () {
        targetPose = robotPoseToTargetPose.apply(swerve.getPose());
        lastCommandedSpeeds = new ChassisSpeeds();
        xController.reset();
        yController.reset();
        thetaRadiansController.reset();
        constraints.reset();
    }
    
    @Override
    public void execute () {
        ChassisSpeeds desiredSpeeds = getPIDDesiredChassisSpeeds();
        ChassisSpeeds constrainedSpeeds = applyConstraintsToSpeeds(desiredSpeeds);
        ChassisSpeeds robotRelSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(constrainedSpeeds, swerve.getRobotRotation());
        swerve.moveRobotRelative(robotRelSpeeds);
        lastCommandedSpeeds = robotRelSpeeds;
    }
    
    /**
     * Gets field-relative speeds
     */
    private ChassisSpeeds getPIDDesiredChassisSpeeds () {
        Pose2d currentPose = swerve.getPose();
        
        double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
        
        double thetaOffset = MathUtil.inputModulus(
            targetPose.getRotation().minus(currentPose.getRotation()).getRadians(),
            -Math.PI, Math.PI
        );
        
        double thetaSpeed = thetaRadiansController.calculate(0, thetaOffset);
        
        return new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
    }
    
    private ChassisSpeeds applyConstraintsToSpeeds (ChassisSpeeds inputSpeeds) {
        Vector<N2> strafeVelocity = Vector.from(inputSpeeds.vxMetersPerSecond, inputSpeeds.vyMetersPerSecond);
        if (strafeVelocity.getMagnitude() > MAX_STRAFE_VELOCITY) {
            strafeVelocity = strafeVelocity.scaleToMagnitude(MAX_STRAFE_VELOCITY);
        }
        
        double angularVelocity = inputSpeeds.omegaRadiansPerSecond;
        if (angularVelocity > MAX_ANGULAR_VELOCITY) {
            angularVelocity = MAX_ANGULAR_VELOCITY;
        } else if (angularVelocity < -MAX_ANGULAR_VELOCITY) {
            angularVelocity = -MAX_ANGULAR_VELOCITY;
        }
        
        return constraints.applyToSpeeds(new ChassisSpeeds(
            strafeVelocity.getX(),
            strafeVelocity.getY(),
            angularVelocity
        ));
    }
    
    @Override
    public void end (boolean interrupted) {
        swerve.stop();
    }
    
    @Override
    public boolean isFinished () {
        Vector<N2> strafeVelocity = Vector.from(lastCommandedSpeeds.vxMetersPerSecond, lastCommandedSpeeds.vyMetersPerSecond);
        double angularVelocity = lastCommandedSpeeds.omegaRadiansPerSecond;
        
        Transform2d fromEndpoint = targetPose.minus(swerve.getPose());
        Vector<N2> distFromEndpoint = Vector.from(fromEndpoint.getX(), fromEndpoint.getY());
        double radiansFromEndpoint = fromEndpoint.getRotation().getRadians();
        
        return
            (strafeVelocity.getMagnitude() < MAX_STRAFE_VELOCITY_AT_END) &&
            (Math.abs(angularVelocity) < MAX_ANGULAR_VELOCITY_AT_END) &&
            (distFromEndpoint.getMagnitude() < MAX_DIST_FROM_ENDPOINT) &&
            (Math.abs(radiansFromEndpoint) < MAX_RADIANS_FROM_ENDPOINT);
    }
    
}
