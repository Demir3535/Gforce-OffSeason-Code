package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.RobotConstants.LimelightConstants;

public class AutoPositionToTagCommand extends Command {
    private final LimelightSubsystem limelight;
    private final DriveSubsystem drive;
    private final int targetTagID; // if -1 its connect to any tag

    /**
     * Creates a command that automatically positions the robot to an AprilTag.
     * 
     * @param limelight   The limelight subsystem
     * @param drive       The drive subsystem
     * @param targetTagID The specific AprilTag ID to target, or -1 for any visible
     *                    tag
     */
    public AutoPositionToTagCommand(LimelightSubsystem limelight, DriveSubsystem drive, int targetTagID) {
        this.limelight = limelight;
        this.drive = drive;
        this.targetTagID = targetTagID; // specific id or any id for -1
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // Log that we're starting the command
        System.out.println("Starting AutoPositionToTag command" +
                (targetTagID == -1 ? " for any tag" : " for tag ID: " + targetTagID));
    }
    @Override
public void execute() {
    if (limelight.hasTargets()) {
        // Check if we're targeting a specific tag and that's what we see
        if (targetTagID != -1 && limelight.getTargetID() != targetTagID) {
            // We see a tag, but not the one we want - stop
            drive.drive(0, 0, 0, true, true);
            return;
        }
        
        // Calculate steering adjustment to center on target
        double steer = limelight.getSteer();
        
        // Yatay hizalama hatasını kontrol et
        double txError = Math.abs(limelight.getTx());
        boolean isAligned = txError < LimelightConstants.ACCEPTABLE_TX_ERROR;
        
        // Calculate drive speed based on distance to target
        double driveSpeed = 0;
        
        // Önce hizalan, sonra yaklaş
        if (isAligned) {
            // Hizalanmış durumdayız, şimdi mesafeyi kontrol et
            double targetArea = limelight.getTa();
            
            // If we're not close enough, move forward
            if (targetArea < LimelightConstants.DESIRED_TARGET) {
                // Scale drive speed based on how far we are
                double distanceRatio = 1.0 - (targetArea / LimelightConstants.DESIRED_TARGET);
                driveSpeed = LimelightConstants.MAX_DRIVE_SPEED * distanceRatio;
                
                // Ensure we don't go too slow - minimum 10% of max speed
                driveSpeed = Math.max(driveSpeed, LimelightConstants.MAX_DRIVE_SPEED * 0.1);
            }
        } else {
            // Hizalanmadıysak, sadece dönüş yap, ileri gitme
            driveSpeed = 0;
        }
        
        // Use rotation and drive together
        drive.drive(driveSpeed, 0, steer * LimelightConstants.MAX_TURN_SPEED, true, true);
    } else {
        // No targets - stop the robot
        drive.drive(0, 0, 0, true, true);
    }
}

    @Override
    public boolean isFinished() {
        // Command is finished when:
        // 1. We have a target
        // 2. We're aligned with it (tx is close to 0)
        // 3. We're at the desired distance (area is at or above target)
        // 4. If targeting specific tag, we see that tag

        if (!limelight.hasTargets()) {
            return false; // No targets, keep running
        }

        if (targetTagID != -1 && limelight.getTargetID() != targetTagID) {
            return false; // We see a tag, but not the one we want
        }

        boolean alignedWithTarget = Math.abs(limelight.getTx()) < LimelightConstants.ACCEPTABLE_TX_ERROR;
        boolean atDesiredDistance = limelight.getTa() >= LimelightConstants.DESIRED_TARGET;

        return alignedWithTarget && atDesiredDistance;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        drive.drive(0, 0, 0, true, true);

        if (interrupted) {
            System.out.println("AutoPositionToTag command was interrupted");
        } else {
            System.out.println("AutoPositionToTag command completed successfully");
        }
    }
}