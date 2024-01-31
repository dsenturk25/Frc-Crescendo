package frc.robot.subsystems;

import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutonomousConstants;

public class PoseSubsystem extends SubsystemBase {

    PhotonCamera photonCamera;
    DriveSubsystem driveSubsystem;
    public double x;
    public double y;
    public double angle;

    private static final List<Pose3d> targetPoses = Collections.unmodifiableList(List.of(
        new Pose3d(0, 0, 0, new Rotation3d(0, 0, 3)),
        new Pose3d(0, 1, 0, new Rotation3d(0, 0, 3))
    ));

    public PoseSubsystem(PhotonCamera photonCamera, DriveSubsystem driveSubsystem) {
        this.photonCamera = photonCamera;
        this.driveSubsystem = driveSubsystem;
    }

    public boolean exampleCondition() {
        return false;
    }

    @Override
    public void periodic() {
        var result = photonCamera.getLatestResult();
        // var resultTimeStamp = result.getTimestampSeconds();
        if (result.hasTargets()) {
            var target = result.getBestTarget();
            var fiducialId = target.getFiducialId();  // index
            if (target.getPoseAmbiguity() <= 0.2 && fiducialId >= 0 && fiducialId < 2) {
                var targetPose = targetPoses.get(fiducialId);
                Transform3d camToTarget = target.getBestCameraToTarget();
                Pose3d camPose = targetPose.transformBy(camToTarget);

                var visionMeasurement = camPose.transformBy(AutonomousConstants.CAM_TO_ROBOT_CENTER);
                Pose2d pose2d = visionMeasurement.toPose2d();
                
                this.x = pose2d.getX();
                this.y = pose2d.getY();
                this.angle = pose2d.getRotation().getRadians();  // in radians

            }
        }
    }

    @Override
    public void simulationPeriodic() {
    }

    public void poseDrive() {
        driveSubsystem.drivePID(x, y, angle);
    }

}
