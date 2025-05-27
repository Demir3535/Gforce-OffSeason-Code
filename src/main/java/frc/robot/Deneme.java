package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.RobotConstants.LimelightConstants;

public class AutoPositionToTagCommand extends Command {
    private final LimelightSubsystem limelight;
    private final DriveSubsystem drive;
    private final int targetTagID;
    private final Joystick joystick;
    
    // PID değişkenleri
    private double lastTxError = 0;
    private double lastHeadingError = 0;
    private double txIntegral = 0;
    private double headingIntegral = 0;

    // Reef modu durum takibi
    private enum ReefMode { CENTER, LEFT, RIGHT }
    private ReefMode currentReefMode = ReefMode.CENTER;
    private boolean lastButton5 = false;
    private boolean lastButton6 = false;

    public AutoPositionToTagCommand(LimelightSubsystem limelight, DriveSubsystem drive,
            int targetTagID, Joystick joystick) {
        this.limelight = limelight;
        this.drive = drive;
        this.targetTagID = targetTagID;
        this.joystick = joystick;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        // 1. JOYSTICK KONTROLÜ - Manuel sürüşte reef modunu kapat
        if (joystick != null && (Math.abs(joystick.getX()) > 0.2 || 
                                Math.abs(joystick.getY()) > 0.2 || 
                                Math.abs(joystick.getTwist()) > 0.2)) {
            resetReefMode();
            drive.drive(joystick.getY(), joystick.getX(), joystick.getTwist(), true, true);
            return;
        }

        // 2. REEF MOD KONTROLÜ (Tek tuşla geçiş)
        boolean currentButton5 = joystick != null && joystick.getRawButton(5);
        boolean currentButton6 = joystick != null && joystick.getRawButton(6);

        if (currentButton5 && !lastButton5) {
            currentReefMode = (currentReefMode == ReefMode.RIGHT) ? ReefMode.CENTER : ReefMode.RIGHT;
            applyReefMode();
        }

        if (currentButton6 && !lastButton6) {
            currentReefMode = (currentReefMode == ReefMode.LEFT) ? ReefMode.CENTER : ReefMode.LEFT;
            applyReefMode();
        }
        lastButton5 = currentButton5;
        lastButton6 = currentButton6;

        // 3. OTOMATİK HİZALMA
        if (limelight.hasTargets() && (targetTagID == -1 || limelight.getTargetID() == targetTagID)) {
            double currentHeading = drive.getHeading();
            double normalizedHeading = currentHeading % 360;
            if (normalizedHeading > 180) normalizedHeading -= 360;
            
            // 180° düzeltme
            boolean isFlipped = Math.abs(normalizedHeading) > 135;
            double tx = limelight.getTx() * (isFlipped ? -1 : 1);
            
            // Reef moduna göre ayar
            if (currentReefMode == ReefMode.RIGHT) {
                tx = limelight.getAdjustedTx() * (isFlipped ? -1 : 1);
            } else if (currentReefMode == ReefMode.LEFT) {
                tx = limelight.getAdjustedTx() * (isFlipped ? -1 : 1);
            }

            // PID Hesaplamaları
            double steer = calculateSteering(normalizedHeading);
            double strafe = calculateStrafe(tx);
            double driveSpeed = calculateDriveSpeed(limelight.getTa());

            drive.drive(driveSpeed, strafe, steer, true, true);

            // DEBUG
            updateSmartDashboard(tx, normalizedHeading, isFlipped);
        } else {
            drive.drive(0, 0, 0, true, true);
        }
    }

    private void applyReefMode() {
        limelight.setLeftReefTarget(currentReefMode == ReefMode.LEFT);
        limelight.setRightReefTarget(currentReefMode == ReefMode.RIGHT);
        SmartDashboard.putString("Reef/Mode", currentReefMode.toString());
    }

    private void resetReefMode() {
        if (currentReefMode != ReefMode.CENTER) {
            currentReefMode = ReefMode.CENTER;
            limelight.setLeftReefTarget(false);
            limelight.setRightReefTarget(false);
            SmartDashboard.putString("Reef/Mode", "CENTER (MANUAL OVERRIDE)");
        }
    }

    private double calculateSteering(double heading) {
        double error = heading;
        double derivative = error - lastHeadingError;
        headingIntegral += error * 0.02;
        headingIntegral = Math.max(-1, Math.min(1, headingIntegral));
        
        double output = error * LimelightConstants.ANGULAR_P * 0.3 +
                      headingIntegral * LimelightConstants.ANGULAR_I * 0.1 +
                      derivative * LimelightConstants.ANGULAR_D * 0.4;
        
        lastHeadingError = error;
        return Math.max(-0.3, Math.min(0.3, output));
    }

    private void updateSmartDashboard(double tx, double heading, boolean isFlipped) {
        SmartDashboard.putNumber("AprilTag/TX", tx);
        SmartDashboard.putNumber("AprilTag/Heading", heading);
        SmartDashboard.putBoolean("AprilTag/IsFlipped", isFlipped);
        SmartDashboard.putNumber("AprilTag/TA", limelight.getTa());
    }

    @Override
    public boolean isFinished() {
        return false; // Komutun sürekli çalışması için
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, true, true);
        resetReefMode();
    }
}