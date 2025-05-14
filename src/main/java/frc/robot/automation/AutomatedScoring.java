package frc.robot.automation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.utils.GforceUtils;
import frc.robot.RobotState;
import frc.robot.RobotConstants.ElevatorConstants;
import frc.robot.RobotConstants.ScoringConstants;
import frc.robot.RobotConstants.WristConstants;
import com.pathplanner.lib.util.FlippingUtil;

public class AutomatedScoring {
    static Pose2d targetPose;
    static double xOffset = .2;// left and right
    // double yOffset = -.25;// forward and back

    private static Pose2d pathPlanToHP(int humanPlayerSide) {
        targetPose = ScoringConstants.BlueAlliance.HP_POSES.get(humanPlayerSide); // no -1 since 0 is left and 1 is
                                                                                  // right
                                                                                  // and indexing starts at 0'
        if (GforceUtils.isRedAlliance()) {
            targetPose = FlippingUtil.flipFieldPose(targetPose);
        }
        return targetPose;
    }

    private static Pose2d pathPlanToReef(int reefSide, int position) {
        // System.out.println("Reef Side: " + reefSide.get());
        targetPose = ScoringConstants.BlueAlliance.REEF_SIDE_POSES.get(reefSide - 1);

        if (GforceUtils.isRedAlliance()) {
            targetPose = FlippingUtil.flipFieldPose(targetPose);
            // System.out.println("Flipping pose");
            targetPose = new Pose2d(targetPose.getX(), targetPose.getY(),
                    new Rotation2d(Math.toRadians(targetPose.getRotation().getDegrees() - 90)));
            // // Not sure what to do
            // // about this
        }
        // Field2d field = new Field2d();
        // field.setRobotPose(targetPose);
        // SmartDashboard.putData("e", field);

        // Determine the correct x & y offset(s) based on the position
        double adjustedXOffset = xOffset;
        if (position == 0) {
            adjustedXOffset = -xOffset;
        } else if (position == 1) {
            adjustedXOffset = 0;
        } else {
            adjustedXOffset = xOffset;
        }

        // Create a translation for the offsets
        Translation2d translation = new Translation2d(adjustedXOffset, 0.2);

        // Apply the translation to the target pose
        targetPose = targetPose.transformBy(new Transform2d(translation, targetPose.getRotation()));

        return targetPose;
    }

    public static Command fullReefAutomation(int reefSide, int position,
            int height,
            DriveSubsystem drivesubsystem, ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {

        Pose2d pose = pathPlanToReef(reefSide, position);
        if (position == 1) {
            RobotState.isAlgaeMode = true;
            return new ParallelCommandGroup(

                    new AlignWithPose(drivesubsystem, pose),
                    new SequentialCommandGroup(elevatorSubsystem.goToAlgaeGrabSetpoint(height),
                            wristSubsystem.goToAlgaeGrabSetpoint(height)));
        } else {
            RobotState.isAlgaeMode = false;
            return new ParallelCommandGroup(

                    new AlignWithPose(drivesubsystem, pose),
                    new SequentialCommandGroup(elevatorSubsystem.goToCoralScoreSetpoint(height),
                            wristSubsystem.goToCoralScoreSetpoint(height)));
        }

    }

    public static Command coralShootCommand(
            int height, WristSubsystem wristSubsystem, ElevatorSubsystem elevatorSubsystem,
            ShooterSubsystem shooterSubsystem) {
        return new SequentialCommandGroup(
                wristSubsystem.goToCoralScoreSetpoint(1), new WaitUntilCommand(() -> wristSubsystem.atSetpoint()),
                elevatorSubsystem.goToCoralScoreSetpoint(height),
                new WaitUntilCommand(() -> elevatorSubsystem.atSetpoint()), wristSubsystem.goToCoralScoreSetpoint(height),
                new WaitUntilCommand(() -> wristSubsystem.atSetpoint()));
    }

    public static Command stopShooterCommand(
        ShooterSubsystem shooterSubsystem) {
    return new SequentialCommandGroup(
            new RunCommand(() -> shooterSubsystem.moveAtSpeed(0)));
}
    public static Command shootl1Command(
            double shooterSpeed, ShooterSubsystem shooterSubsystem) {
        return new SequentialCommandGroup(
                new RunCommand(() -> shooterSubsystem.moveAtSpeed(shooterSpeed)));
    }

    public static Command algeaPickUpCommand(
            int height, double shooterSpeed, WristSubsystem wristSubsystem, ElevatorSubsystem elevatorSubsystem,
            ShooterSubsystem shooterSubsystem) {
        return new SequentialCommandGroup(
                wristSubsystem.goToCoralScoreSetpoint(2), new WaitUntilCommand(() -> wristSubsystem.atSetpoint()),
                elevatorSubsystem.goToAlgaeGrabSetpoint(height),
                new WaitUntilCommand(() -> elevatorSubsystem.atSetpoint()),
                wristSubsystem.goToAlgaeGrabSetpoint(height), new WaitUntilCommand(() -> wristSubsystem.atSetpoint()),
                new RunCommand(() -> shooterSubsystem.moveAtSpeed(shooterSpeed)));
    }

    public static Command humanplayerCommand(
            int height, double shooterSpeed, WristSubsystem wristSubsystem, ElevatorSubsystem elevatorSubsystem,
            ShooterSubsystem shooterSubsystem) {
        return new SequentialCommandGroup(
                wristSubsystem.goToAlgaeGrabSetpoint(2), new WaitUntilCommand(() -> wristSubsystem.atSetpoint()),
                elevatorSubsystem.huCommand(1), new WaitUntilCommand(() -> elevatorSubsystem.atSetpoint()));
    }

    public static Command intakeCommand(
            double shooterSpeed, WristSubsystem wristSubsystem, ElevatorSubsystem elevatorSubsystem,
            ShooterSubsystem shooterSubsystem) {
        return new SequentialCommandGroup(
                wristSubsystem.goToCoralScoreSetpoint(1), new WaitUntilCommand(() -> wristSubsystem.atSetpoint()),
                elevatorSubsystem.intakeCommand(0), new WaitUntilCommand(() -> elevatorSubsystem.atSetpoint()),
                wristSubsystem.intakeCommand(0), new WaitUntilCommand(() -> wristSubsystem.atSetpoint()),
                new RunCommand(() -> shooterSubsystem.moveAtSpeed(shooterSpeed))
                        .until(() -> shooterSubsystem.hasGameElement())
                        .andThen(() -> shooterSubsystem.moveAtSpeed(0)));
    }

    public static Command wristThenElevator(double wristSetpoint, double elevatorSetpoint,
            WristSubsystem wristSubsystem, ElevatorSubsystem elevatorSubsystem) {
        return new SequentialCommandGroup(
                // Önce wrist'i hedef encoder değerine getir
                new InstantCommand(() -> wristSubsystem.goToSetpoint(wristSetpoint)),
                // Wrist hedefine ulaşana kadar bekle
                // Sonra elevator'ı hedef yüksekliğe getir
                new InstantCommand(() -> elevatorSubsystem.goToSetpoint(elevatorSetpoint)));
    }

    public static Command intakePosition(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
        return new SequentialCommandGroup(
                // 1. Adım: Elevator'ı intake pozisyonuna indir
                new InstantCommand(() -> elevatorSubsystem.goToSetpoint(ElevatorConstants.HeightSetpoints.HOME)),
                // Elevator hedefine ulaşana kadar bekle

                // 2. Adım: Wrist'i düzelt
                new InstantCommand(() -> wristSubsystem.goToSetpoint(WristConstants.AngleSetpoints.HOME)));
    }

    public static Command scoreWristAlgeaPathing(int height, WristSubsystem wristSubsystem) {
        RobotState.isAlgaeMode = false;

        return new RunCommand(() -> {
            // Sürekli olarak komut gönder
            if (height == 1) {
                wristSubsystem.goToSetpoint(WristConstants.AngleSetpoints.Algae.L2);
            } else if (height == 2) {
                wristSubsystem.goToSetpoint(WristConstants.AngleSetpoints.Algae.L2);
            } else if (height == 3) {
                wristSubsystem.goToSetpoint(WristConstants.AngleSetpoints.Algae.L2);
            } else if (height == 0) {
                wristSubsystem.goToSetpoint(WristConstants.AngleSetpoints.Algae.L2);
            }
        }, wristSubsystem);
    }

    public static Command scoreElevatorAlgea(int height, ElevatorSubsystem elevatorSubsystem) {
        RobotState.isAlgaeMode = false;

        return new RunCommand(() -> {
            // Sürekli olarak komut gönder
            if (height == 2) {
                elevatorSubsystem.goToSetpoint(ElevatorConstants.HeightSetpoints.Algae.L2);
            } else if (height == 3) {
                elevatorSubsystem.goToSetpoint(ElevatorConstants.HeightSetpoints.Algae.L3);
            } else if (height == 5) {
                elevatorSubsystem.goToSetpoint(ElevatorConstants.HeightSetpoints.Algae.L1);
            }
        }, elevatorSubsystem);
    }

    public static Command scoreWristCoralPathing(int height, WristSubsystem wristSubsystem) {
        RobotState.isAlgaeMode = false;

        return new RunCommand(() -> {
            // Sürekli olarak komut gönder
            if (height == 1) {
                wristSubsystem.goToSetpoint(WristConstants.AngleSetpoints.Coral.L1);
            } else if (height == 2) {
                wristSubsystem.goToSetpoint(WristConstants.AngleSetpoints.Coral.L2);
            } else if (height == 3) {
                wristSubsystem.goToSetpoint(WristConstants.AngleSetpoints.Coral.L2);
            } else if (height == 0) {
                wristSubsystem.goToSetpoint(WristConstants.AngleSetpoints.HOME);
            }
        }, wristSubsystem);
    }

    public static Command scoreCoralNoPathing(int height, ElevatorSubsystem elevatorSubsystem,
            WristSubsystem wristSubsystem) {
        RobotState.isAlgaeMode = false;

        return new RunCommand(() -> {
            // Sürekli olarak komut gönder
            if (height == 1) {
                elevatorSubsystem.goToSetpoint(ElevatorConstants.HeightSetpoints.Coral.L1);
            } else if (height == 2) {
                elevatorSubsystem.goToSetpoint(ElevatorConstants.HeightSetpoints.Coral.L2);
            } else if (height == 3) {
                elevatorSubsystem.goToSetpoint(ElevatorConstants.HeightSetpoints.Coral.L3);
            } else if (height == 0) {
                elevatorSubsystem.goToSetpoint(ElevatorConstants.HeightSetpoints.HOME);
            }
        }, elevatorSubsystem);
    }

    public static Command grabAlgaeNoPathing(int height, ElevatorSubsystem elevatorSubsystem,
            WristSubsystem wristSubsystem) {
        RobotState.isAlgaeMode = true;
        return new SequentialCommandGroup(elevatorSubsystem.goToAlgaeGrabSetpoint(height),
                wristSubsystem.goToAlgaeGrabSetpoint(height)); // clawSubsystem.intakeAlgae());
    }

    public static Command homeSubsystems(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
        return new RunCommand(() -> {
            elevatorSubsystem.goToSetpoint(ElevatorConstants.HeightSetpoints.HOME);
            wristSubsystem.goToSetpoint(WristConstants.AngleSetpoints.HOME);
        }, elevatorSubsystem, wristSubsystem);
    }

    public static Command humanPlayerPickup(int humanPlayerSide, DriveSubsystem drivesubsystem,
            ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
        RobotState.isAlgaeMode = false;
        return new ParallelCommandGroup(
                new AlignWithPose(drivesubsystem, pathPlanToHP(humanPlayerSide)),
                elevatorSubsystem.goToHumanPlayerPickup(), wristSubsystem.goToHumanPlayerSetpoint());
    }

    public static Command humanPlayerPickupNoPathing(DriveSubsystem drivesubsystem,
            ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
        RobotState.isAlgaeMode = false;
        return new ParallelCommandGroup(elevatorSubsystem.goToHumanPlayerPickup(),
                wristSubsystem.goToHumanPlayerSetpoint());

    }
}