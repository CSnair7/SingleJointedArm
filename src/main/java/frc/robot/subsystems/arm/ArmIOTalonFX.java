package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.SubsystemConstants;

public class ArmIOTalonFX implements ArmIO {

    private final TalonFX motor;
    private final CANcoder motorCoder;

    private double positionSetpointDegs;
    private double startAngleDegs;

    private StatusSignal<Angle> motorPositionRotations;
    private final StatusSignal<AngularVelocity> velocityDegsPerSec;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> statorCurrentAmps;
    private final StatusSignal<Current> supplyCurrentAmps;

    public ArmIOTalonFX(int motorID, int canCoderID) {
        CANcoderConfiguration coderConfig = new CANcoderConfiguration();
        coderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        // OFFSET IS IN ROTATIONS
        // coderConfig.MagnetSensor.withMagnetOffset(Units.degreesToRotations(58));

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = SubsystemConstants.ArmConstants.CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable
                = SubsystemConstants.ArmConstants.CURRENT_LIMIT_ENABLED;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = 1;
        motor = new TalonFX(motorID);
        motorCoder = new CANcoder(canCoderID);

        motor.getConfigurator().apply(config);
        motorCoder.getConfigurator().apply(coderConfig);

        if (motorCoder.isConnected()) {
            motor.setPosition(
                    (motorCoder.getAbsolutePosition().getValueAsDouble() - Units.degreesToRotations(57 - 12))
                    * SubsystemConstants.ArmConstants.ARM_GEAR_RATIO);
        } else {
            motor.setPosition(
                    Units.degreesToRotations(SubsystemConstants.ArmConstants.STOW_SETPOINT_DEG)
                    * SubsystemConstants.ArmConstants.ARM_GEAR_RATIO);
        }

        motorPositionRotations = motor.getPosition();
        velocityDegsPerSec = motor.getVelocity();
        appliedVolts = motor.getMotorVoltage();
        statorCurrentAmps = motor.getStatorCurrent();
        supplyCurrentAmps = motor.getSupplyCurrent();

        // leader.get
        positionSetpointDegs = SubsystemConstants.ArmConstants.STOW_SETPOINT_DEG;

        Logger.recordOutput("start angle", startAngleDegs);

        motor.optimizeBusUtilization();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                motorPositionRotations,
                velocityDegsPerSec,
                appliedVolts,
                statorCurrentAmps,
                supplyCurrentAmps);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                motorPositionRotations,
                velocityDegsPerSec,
                appliedVolts,
                statorCurrentAmps,
                supplyCurrentAmps);

        inputs.positionDegs
                = Units.rotationsToDegrees(motorPositionRotations.getValueAsDouble())
                / SubsystemConstants.ArmConstants.ARM_GEAR_RATIO;

        inputs.velocityDegsPerSec = Units.rotationsToDegrees(velocityDegsPerSec.getValueAsDouble());
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
        inputs.positionSetpointDegs = positionSetpointDegs;

        Logger.recordOutput(
                "cancoder arm position degrees",
                Units.rotationsToDegrees(
                        motorCoder.getAbsolutePosition().getValueAsDouble()
                        - Units.degreesToRotations(57 - 12)));
    }

    @Override
    public void setBrakeMode(boolean bool) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        if (bool) {
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }

        motor.getConfigurator().apply(config);
    }

    @Override
    public void setPositionSetpointDegs(double positionDegs, double ffVolts) {
        this.positionSetpointDegs = positionDegs;

        motor.setControl(
                new PositionVoltage(
                        Units.degreesToRotations(positionDegs)
                        * SubsystemConstants.ArmConstants.ARM_GEAR_RATIO)
                        .withFeedForward(ffVolts));
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void stop() {
        positionSetpointDegs = motorPositionRotations.getValueAsDouble();
        motor.stopMotor();
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        Slot0Configs config = new Slot0Configs();

        config.kP = kP;
        config.kI = kI;
        config.kD = kD;

        motor.getConfigurator().apply(config);
    }
}
