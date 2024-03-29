package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    
    private static Vision visionInstance;
    public static Vision getInstance() {
        if (visionInstance == null) visionInstance = new Vision();
        return visionInstance;
    }

    private final NetworkTable limelight;
    private NetworkTableEntry 
            tv,
            tx,
            ty,
            tid,
            targetPose;
    public double maxOffsetDegrees = 1;

    private Vision () {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        tv = limelight.getEntry("tv");
        tx = limelight.getEntry("tx");
        ty = limelight.getEntry("ty");
        tid = limelight.getEntry("tid");
        targetPose = limelight.getEntry("targetpose_robotspace");
        limelight.getEntry("ledMode").setDouble(0); //Sets the LED to desired status. 0 to follow current pipeline, 1 for off, 2 for blinking, 3 for on
        limelight.getEntry("camMode").setDouble(0); //Enables or disables vision processing. 0 for processing, 1 for drive camera
        limelight.getEntry("pipeline").setDouble(0); //Changes the pipeline of the Limelight
    }

    public boolean isTargetFriendly () {
        if (getTag() >= 1 && getTag() <= 4 && DriverStation.getAlliance() == Alliance.Red) return true;
        else if (getTag() >= 5 && getTag() <= 8 && DriverStation.getAlliance() == Alliance.Blue) return true;
        else return false;
    }
    
    //Checks if Limelight sees a target, returns false if the value is zero, true if it is 1
    public boolean seesTarget () {
        return tv.getDouble(0) == 1;
    }

    public double setPipeline (int pipeline) {
        limelight.getEntry("pipeline").setDouble(pipeline);
        return limelight.getEntry("pipeline").getDouble(0);
    }

    //Returns the horizontal offset of the camera to the target (-27 degrees to 27 degrees)
    public double getHorizontalOffset () {
        return tx.getDouble(0);
    }

    //Returns the vertical offset of the camera to the target (-20.5 degrees to 20.5 degrees)
    public double getVerticalOffset () {
        return ty.getDouble(0);
    }

    public double getTag () {
        return tid.getDouble(0);
    }

    public double getDistance () {
        return targetPose.getDoubleArray(new double[5])[2]; //TODO: Measure robot and set proper specs in limelight pipeline
    }

  @Override
  public void periodic() {}
}