package frc.team5115.subsystems.climber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.team5115.Constants;

public class ClimberIOSparkMax implements ClimberIO {

    public final CANSparkMax leftClimb;
    public final CANSparkMax rightClimb;
    public final RelativeEncoder leftClimbEncoder;
    public final RelativeEncoder rightClimbEncoder;

    public ClimberIOSparkMax() {
        leftClimb = new CANSparkMax(Constants.CLIMBER_LEFT_MOTOR_ID, MotorType.kBrushless);
        // left climber is inverted!
        leftClimb.setInverted(true);
        leftClimb.setIdleMode(IdleMode.kBrake);
        leftClimb.setSmartCurrentLimit(45);

        rightClimb = new CANSparkMax(Constants.CLIMBER_RIGHT_MOTOR_ID, MotorType.kBrushless);
        rightClimb.setInverted(true);
        rightClimb.setIdleMode(IdleMode.kBrake);
        rightClimb.setSmartCurrentLimit(45);

        leftClimbEncoder = leftClimb.getEncoder();
        leftClimbEncoder.setPositionConversionFactor(1.0 / 16.0);

        rightClimbEncoder = rightClimb.getEncoder();
        rightClimbEncoder.setPositionConversionFactor(1.0 / 16.0);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.leftAngle = Rotation2d.fromDegrees(leftClimbEncoder.getPosition());
        inputs.rightAngle = Rotation2d.fromDegrees(rightClimbEncoder.getPosition());
        inputs.leftVelocityRPM = leftClimbEncoder.getVelocity();
        inputs.rightVelocityRPM = rightClimbEncoder.getVelocity();
        inputs.leftCurrentAmps = leftClimb.getOutputCurrent();
        inputs.rightCurrentAmps = rightClimb.getOutputCurrent();
        inputs.leftAppliedVolts = leftClimb.getAppliedOutput();
        inputs.rightAppliedVolts = rightClimb.getAppliedOutput();
    }

    @Override
    public void setClimberVoltages(double leftVoltage, double rightVoltage) {
        leftClimb.setVoltage(leftVoltage);
        rightClimb.setVoltage(rightVoltage);
    }
}
