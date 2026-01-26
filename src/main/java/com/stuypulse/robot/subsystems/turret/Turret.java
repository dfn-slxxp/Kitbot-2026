package com.stuypulse.robot.subsystems.turret;

import java.util.Vector;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.HubUtil;
import com.stuypulse.robot.util.TurretVisualizer;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public abstract class Turret extends SubsystemBase {
    public static Turret instance;
    public TurretState state;

    static {
        if (Robot.isReal()) {
            instance = new TurretImpl();
        } else {
            instance = new TurretSim();
        }
    }

    public static Turret getInstance() {
        return instance;
    }

    public Turret() {
        state = TurretState.ZERO;
    }

    public enum TurretState {
        ZERO,
        FERRYING,
        POINT_AT_HUB;
    }

    public Rotation2d getTargetAngle() {
        return switch (getTurretState()) {
            case ZERO -> Robot.isBlue() ? new Rotation2d() : Rotation2d.fromDegrees(180.0); 
            case FERRYING -> getFerryAngle(true);
            case POINT_AT_HUB -> getPointAtHubAngle();
        };
    }

    public void setTurretState(TurretState targetState) {
        state = targetState;
    }

    public TurretState getTurretState() {
        return state;
    }

    public boolean atTargetAngle() {
        return Math.abs(getTurretAngle().minus(getTargetAngle()).getDegrees()) < Settings.Turret.TOLERANCE_DEG;
    }

    public Rotation2d getPointAtHubAngle() {
        return getPointAtTargetAngle(Field.getAllianceHubPose());
    }

    public Rotation2d getFerryAngle(boolean isLeftFerryZone) {
        return getPointAtTargetAngle(Field.getFerryZonePose(isLeftFerryZone));
    }

    public abstract Rotation2d getTurretAngle();

    public abstract SysIdRoutine getSysIdRoutine();

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret/Angle (Deg)", getTurretAngle().getDegrees());

        if (Settings.DEBUG_MODE) {
            if (Settings.EnabledSubsystems.TURRET.get()) {
                TurretVisualizer.getInstance().updateTurretAngle(getTurretAngle(), atTargetAngle());
            }
            else {
                TurretVisualizer.getInstance().updateTurretAngle(new Rotation2d(), false);
            }
        }
    }

    public Rotation2d getPointAtTargetAngle(Pose2d targetPose) {
        Vector2D robot = new Vector2D(CommandSwerveDrivetrain.getInstance().getPose().getTranslation());
        Vector2D target = new Vector2D(targetPose.getX(), targetPose.getY());
        Vector2D robotToTarget = target.sub(robot).normalize();
        Vector2D zeroVector = new Vector2D(1.0, 0.0); // Alliance-relative

        // https://www.youtube.com/watch?v=_VuZZ9_58Wg
        double crossProduct = zeroVector.x * robotToTarget.y - zeroVector.y * robotToTarget.x;
        double dotProduct = zeroVector.dot(robotToTarget);
        double angleRadians = Math.atan2(crossProduct, dotProduct);

        if (!Robot.isBlue()) {
            angleRadians += Math.PI; // Flip target angle for red alliance

            while (angleRadians < 0) angleRadians += 2 * Math.PI;
            while (angleRadians >= 2 * Math.PI) angleRadians -= 2 * Math.PI;
        }

        SmartDashboard.putNumber("Turret/Robot to Target Vector X", robotToTarget.x);
        SmartDashboard.putNumber("Turret/Robot to Target Vector Y", robotToTarget.y);

        Rotation2d targetAngle = Rotation2d.fromRadians(angleRadians);
        return targetAngle;
    }
}
