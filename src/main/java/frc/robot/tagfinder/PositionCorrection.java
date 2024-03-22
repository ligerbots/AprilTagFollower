package frc.robot.tagfinder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class PositionCorrection extends Command {
    private final DriveTrain m_driveTrain;
    public PositionCorrection(DriveTrain driveTrain) {
        super();
        m_driveTrain = driveTrain;
    }

    Transform2d getRequiredMovement(Transform3d transform) {
        double targetX = transform.getTranslation().toTranslation2d().getX();
        double targetZ = transform.getTranslation().toTranslation2d().getY();
        double targetAngle = transform.getRotation().getZ();
        double currentX = m_driveTrain.getPose().getX();
        double currentZ = m_driveTrain.getPose().getY();
        double currentAngle = m_driveTrain.getYaw().getRadians();
        return new Transform2d(
            new Translation2d(targetX - currentX, targetZ - currentZ),
            new Rotation2d(targetAngle - currentAngle)
        );
    }
    ChassisSpeeds getChassisSpeeds(Transform2d transform) {
        return new ChassisSpeeds(
            transform.getX(), transform.getY(),
            transform.getRotation().getRadians()
        );
    }
    void doRequiredMovement(ChassisSpeeds speed) {
        m_driveTrain.drive(speed);
    }
    void doRequiredMovement(Transform3d transform) {
        doRequiredMovement(getChassisSpeeds(getRequiredMovement(transform)));
    }
}
