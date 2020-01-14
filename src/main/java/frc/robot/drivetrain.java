/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.RobotMap;
import frc.robot.Robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpiutil.math.MathUtil;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration; 

/**
 * Add your docs here.
 */
public class drivetrain extends RobotDriveBase implements Sendable, AutoCloseable {
    
    private static drivetrain instance;
    public static drivetrain getInstance(){
        if (instance == null)
            instance = new drivetrain();
        return instance;
    }

    private TalonSRX m_left_front = new TalonSRX(RobotMap.DRIVE_LEFT_FRONT);
    private TalonSRX m_left_middle = new TalonSRX(RobotMap.DRIVE_LEFT_MIDDLE);
    private TalonSRX m_left_back = new TalonSRX(RobotMap.DRIVE_LEFT_BACK);

    private TalonSRX m_right_front = new TalonSRX(RobotMap.DRIVE_RIGHT_FRONT);
    private TalonSRX m_right_middle = new TalonSRX(RobotMap.DRIVE_RIGHT_MIDDLE);
    private TalonSRX m_right_back = new TalonSRX(RobotMap.DRIVE_RIGHT_BACK);

    private DifferentialDriveKinematics mKinematics;
    private DifferentialDriveOdometry mOdometry;

    private Pose2d mRobotPose;
    
    private RamseteController mRamsete;

    public static final double kDefaultQuickStopThreshold = 0.2;
    public static final double kDefaultQuickStopAlpha = 0.1;

    private double m_quickStopThreshold = kDefaultQuickStopThreshold;
    private double m_quickStopAlpha = kDefaultQuickStopAlpha;
    private double m_quickStopAccumulator;
    private double m_rightSideInvertMultiplier = -1.0;
    private boolean m_reported;

    private double jx = 0.0;
    private double jy = 0.0;
    private double ROTATION_SCALE = 0.5;

    private SendableChooser<String> m_drive_chooser = new SendableChooser<String>();
    private SendableChooser<String> m_velocity_chooser = new SendableChooser<String>();

    public drivetrain() {
        configAllMotors();
        
    }

    private void configAllMotors() {
        TalonSRXConfiguration config = new TalonSRXConfiguration();
        config.forwardSoftLimitEnable = false;
        config.reverseSoftLimitEnable = false;
        config.neutralDeadband = 0.01;
        config.nominalOutputForward = 1.0;
        config.nominalOutputReverse = -1.0;
        config.openloopRamp = 0.05;
        config.closedloopRamp = 0.02;
        config.slot0.closedLoopPeriod = 2;
        config.slot0.closedLoopPeakOutput = 0.5;
        config.slot0.kP = 0;
        config.slot0.kI = 0;
        config.slot0.kD = 0;
        config.slot0.kF = 0;
        config.slot0.integralZone = 0;
        config.slot0.maxIntegralAccumulator = 1.0;
        
        m_right_front.configAllSettings(config);
        m_right_middle.configAllSettings(config);
        m_right_back.configAllSettings(config);
        m_right_front.configAllSettings(config);
        m_right_middle.configAllSettings(config);
        m_right_back.configAllSettings(config);
    }

    public void robotInit() {
        m_drive_chooser.setDefaultOption("Arcade", "arcade");
        m_drive_chooser.addOption("Curvature", "curvature");
        m_velocity_chooser.setDefaultOption("Velocity", "velocity");
        m_velocity_chooser.addOption("Percent", "percent");
    }

    public void teleopPeriodic() {
        jy = -1 * Robot.driveController.getRawAxis(RobotMap.LEFT_JOYSTICK_DRIVE_FORWARD);
        jx = Robot.driveController.getRawAxis(RobotMap.RIGHT_JOYSTICK_DRIVE_ROTATE);

        switch (m_drive_chooser.getSelected()) {
            case "arcade":
                arcadeDrive(jy, jx * ROTATION_SCALE, true);
                break;
            case "curvature":
                curvatureDrive(jy, jx * ROTATION_SCALE, Robot.driveController.getStickButton(Hand.kRight));
                break;
            default:
                break;
        }


        
    }

    /**
     * Arcade drive method for differential drive platform.
     *
     * @param xSpeed        The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
     * @param zRotation     The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
     *                      positive.
     * @param squareInputs If set, decreases the input sensitivity at low speeds.
     */
    @SuppressWarnings("ParameterName")
    public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
        if (!m_reported) {
            HAL.report(tResourceType.kResourceType_RobotDrive,
                        tInstances.kRobotDrive2_DifferentialArcade, 2);
            m_reported = true;
        }

        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
        xSpeed = applyDeadband(xSpeed, m_deadband);

        zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
        zRotation = applyDeadband(zRotation, m_deadband);

        // Square the inputs (while preserving the sign) to increase fine control
        // while permitting full power.
        if (squareInputs) {
            xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
            zRotation = Math.copySign(zRotation * zRotation, zRotation);
        }

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

        if (xSpeed >= 0.0) {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0) {
            leftMotorOutput = maxInput;
            rightMotorOutput = xSpeed - zRotation;
            } else {
            leftMotorOutput = xSpeed + zRotation;
            rightMotorOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0) {
            leftMotorOutput = xSpeed + zRotation;
            rightMotorOutput = maxInput;
            } else {
            leftMotorOutput = maxInput;
            rightMotorOutput = xSpeed - zRotation;
            }
        }

        m_left_front.set(ControlMode.PercentOutput, MathUtil.clamp(leftMotorOutput, -1.0, 1.0) * m_maxOutput);
        double maxOutput = m_maxOutput * m_rightSideInvertMultiplier;
        m_right_front.set(ControlMode.PercentOutput, MathUtil.clamp(rightMotorOutput, -1.0, 1.0) * maxOutput);

        feed();
    }

    /**
     * Curvature drive method for differential drive platform.
     *
     * <p>The rotation argument controls the curvature of the robot's path rather than its rate of
     * heading change. This makes the robot more controllable at high speeds. Also handles the
     * robot's quick turn functionality - "quick turn" overrides constant-curvature turning for
     * turn-in-place maneuvers.
     *
     * @param xSpeed      The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
     * @param zRotation   The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
     *                    positive.
     * @param isQuickTurn If set, overrides constant-curvature turning for
     *                    turn-in-place maneuvers.
     */
    @SuppressWarnings({"ParameterName", "PMD.CyclomaticComplexity"})
    public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
        if (!m_reported) {
        HAL.report(tResourceType.kResourceType_RobotDrive,
                    tInstances.kRobotDrive2_DifferentialCurvature, 2);
        m_reported = true;
        }

        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
        xSpeed = applyDeadband(xSpeed, m_deadband);

        zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
        zRotation = applyDeadband(zRotation, m_deadband);

        double angularPower;
        boolean overPower;

        if (isQuickTurn) {
        if (Math.abs(xSpeed) < m_quickStopThreshold) {
            m_quickStopAccumulator = (1 - m_quickStopAlpha) * m_quickStopAccumulator
                + m_quickStopAlpha * MathUtil.clamp(zRotation, -1.0, 1.0) * 2;
        }
        overPower = true;
        angularPower = zRotation;
        } else {
        overPower = false;
        angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator;

        if (m_quickStopAccumulator > 1) {
            m_quickStopAccumulator -= 1;
        } else if (m_quickStopAccumulator < -1) {
            m_quickStopAccumulator += 1;
        } else {
            m_quickStopAccumulator = 0.0;
        }
        }

        double leftMotorOutput = xSpeed + angularPower;
        double rightMotorOutput = xSpeed - angularPower;

        // If rotation is overpowered, reduce both outputs to within acceptable range
        if (overPower) {
        if (leftMotorOutput > 1.0) {
            rightMotorOutput -= leftMotorOutput - 1.0;
            leftMotorOutput = 1.0;
        } else if (rightMotorOutput > 1.0) {
            leftMotorOutput -= rightMotorOutput - 1.0;
            rightMotorOutput = 1.0;
        } else if (leftMotorOutput < -1.0) {
            rightMotorOutput -= leftMotorOutput + 1.0;
            leftMotorOutput = -1.0;
        } else if (rightMotorOutput < -1.0) {
            leftMotorOutput -= rightMotorOutput + 1.0;
            rightMotorOutput = -1.0;
        }
        }

        // Normalize the wheel speeds
        double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
        if (maxMagnitude > 1.0) {
        leftMotorOutput /= maxMagnitude;
        rightMotorOutput /= maxMagnitude;
        }

        m_left_front.set(ControlMode.PercentOutput, leftMotorOutput * m_maxOutput);
        m_right_front.set(ControlMode.PercentOutput, rightMotorOutput * m_maxOutput * m_rightSideInvertMultiplier);

        feed();
    } 

    @Override
    public String getDescription() {
        return "drivetrain";
    }

    @Override
    public void stopMotor() {
        m_left_front.neutralOutput();
        m_left_middle.neutralOutput();
        m_left_back.neutralOutput();
        m_right_front.neutralOutput();
        m_right_middle.neutralOutput();
        m_right_back.neutralOutput();
        feed();
    }

    @Override
    public void close() {
        SendableRegistry.remove(this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Drivetrain");
        builder.setActuator(true);
        builder.setSafeState(this::stopMotor);
        builder.addDoubleProperty("Left Motor %output", () -> m_left_front.getMotorOutputPercent(), null);
        builder.addDoubleProperty("Right Motor %output", () -> m_right_front.getMotorOutputPercent() * m_rightSideInvertMultiplier,null);
    }
}
