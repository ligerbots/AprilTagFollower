package frc.robot.tagfinder;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ShooterPivot;

public class PositionCorrection extends Command {
    private final DriveTrain m_driveTrain;
    private final ShooterPivot m_shooterPivot;
    private final Supplier<Transform3d> m_targetSupplier;
    
    private static final double SHOOTER_ANGLE_THRESHOLD = Math.toRadians(10);
    private static final double ROBOT_ANGLE_THRESHOLD = Math.toRadians(10);
    private static final double ROBOT_POSITION_THRESHOLD = 0.3;
    
    private static final Transform2d ROBOT_POSITION_OFFSET = new Transform2d(0, -1.5, new Rotation2d());

    public PositionCorrection(DriveTrain driveTrain, ShooterPivot shooterPivot, Supplier<Transform3d> targetSupplier) {
        super();
        addRequirements(driveTrain, shooterPivot);
        m_driveTrain = driveTrain;
        m_shooterPivot = shooterPivot;
        m_targetSupplier = targetSupplier;
    }
    
    static Translation3d toVector(Rotation3d angle) {
        return new Translation3d(
            Math.cos(-angle.getZ()) * Math.cos(angle.getY()),
            Math.sin(-angle.getZ()) * Math.cos(angle.getY()),
            Math.sin(angle.getY())
        );
    }

    

    @Override
    public void initialize() {
        
    }
    @Override
    public void execute() {
        Transform3d transform = m_targetSupplier.get();
        Transform2d offsetPos = new Transform2d(
            transform.getTranslation().toTranslation2d(),
            transform.getRotation().toRotation2d()
        ).plus(ROBOT_POSITION_OFFSET);
        m_driveTrain.tagFollowerDrive(offsetPos.getX(), offsetPos.getY(), transform.getRotation().getZ());
        m_shooterPivot.setAngle(transform.getRotation().getY(), true);
    }
    @Override
    public boolean isFinished() {
        return Math.abs(m_shooterPivot.getAngleRadians() - m_targetSupplier.get().getRotation().getY()) < SHOOTER_ANGLE_THRESHOLD
            && Math.abs(m_driveTrain.getYaw().getRadians() - m_targetSupplier.get().getRotation().getZ()) < ROBOT_ANGLE_THRESHOLD
            && m_driveTrain.getPose().getTranslation().getDistance(m_targetSupplier.get().getTranslation().toTranslation2d()) < ROBOT_POSITION_THRESHOLD;
    }
    @Override
    public void end(boolean interrupted) {
        
    }
}
