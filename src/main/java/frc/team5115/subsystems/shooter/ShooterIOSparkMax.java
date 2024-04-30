package frc.team5115.subsystems.shooter;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.team5115.Constants;

public class ShooterIOSparkMax implements ShooterIO {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final CANSparkMax intakeMotor;
    private final CANSparkMax auxMotor;
    private final CANSparkMax amperMotor;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    private final RelativeEncoder intakeEncoder;
    private final RelativeEncoder auxEncoder;
    private final AbsoluteEncoder amperEncoder;

    private final DigitalInput sensor;

    public ShooterIOSparkMax() {
        sensor = new DigitalInput(Constants.SHOOTER_SENSOR_ID);

        leftMotor = new CANSparkMax(Constants.SHOOTER_LEFT_MOTOR_ID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Constants.SHOOTER_RIGHT_MOTOR_ID, MotorType.kBrushless);
        auxMotor = new CANSparkMax(Constants.SHOTER_AUX_MOTOR_ID, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        amperMotor = new CANSparkMax(Constants.SNOWBLOWER_MOTOR_ID, MotorType.kBrushed);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();
        auxEncoder = auxMotor.getEncoder();
        intakeEncoder = intakeMotor.getEncoder();
        amperEncoder = amperMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        amperEncoder.setPositionConversionFactor(360);

        // Intake motor configs
        intakeMotor.setSmartCurrentLimit(60, 80);
        intakeMotor.setIdleMode(IdleMode.kCoast);

        // Shooter motor configs
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        auxMotor.restoreFactoryDefaults();

        leftMotor.setClosedLoopRampRate(0.1);
        rightMotor.setClosedLoopRampRate(0.1);
        auxMotor.setClosedLoopRampRate(0.1);

        leftMotor.setInverted(false);
        rightMotor.setInverted(true);
        auxMotor.setInverted(false);

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
        auxMotor.setIdleMode(IdleMode.kBrake);

        leftMotor.setSmartCurrentLimit(40);
        rightMotor.setSmartCurrentLimit(40);
        auxMotor.setSmartCurrentLimit(40);

        leftMotor.burnFlash();
        rightMotor.burnFlash();
        auxMotor.burnFlash();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftVelocityRPM = leftEncoder.getVelocity();
        inputs.leftAppliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
        inputs.leftCurrentAmps = leftMotor.getOutputCurrent();

        inputs.rightVelocityRPM = rightEncoder.getVelocity();
        inputs.rightAppliedVolts = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
        inputs.rightCurrentAmps = rightMotor.getOutputCurrent();

        inputs.auxVelocityRPM = auxEncoder.getVelocity();
        inputs.auxAppliedVolts = auxMotor.getAppliedOutput() * auxMotor.getBusVoltage();
        inputs.auxCurrentAmps = auxMotor.getOutputCurrent();
        inputs.auxPositionRotations = auxEncoder.getPosition();

        inputs.intakeVelocityRPM = intakeEncoder.getVelocity();
        inputs.intakeAppliedVolts = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
        inputs.intakeCurrentAmps = intakeMotor.getOutputCurrent();

        inputs.amperAppliedVolts = amperMotor.getAppliedOutput() * amperMotor.getBusVoltage();
        inputs.amperCurrentAmps = amperMotor.getOutputCurrent();
        inputs.amperPosition = Rotation2d.fromDegrees(amperEncoder.getPosition());

        inputs.noteDetected = !sensor.get();
    }

    @Override
    public void setAmperPercent(double percent) {
        amperMotor.set(percent);
    }

    @Override
    public void setAuxPercent(double percent) {
        auxMotor.set(percent);
    }

    @Override
    public void setAuxVoltage(double volts) {
        auxMotor.setVoltage(volts);
    }

    @Override
    public void setIntakePercent(double percent) {
        intakeMotor.set(percent);
    }

    @Override
    public void setLeftPercent(double percent) {
        leftMotor.set(percent);
    }

    @Override
    public void setLeftVoltage(double volts) {
        leftMotor.setVoltage(volts);
    }

    @Override
    public void setRightPercent(double percent) {
        rightMotor.set(percent);
    }

    @Override
    public void setRightVoltage(double volts) {
        rightMotor.setVoltage(volts);
    }

    @Override
    public void setAmperVoltage(double volts) {
        amperMotor.setVoltage(volts);
    }

    @Override
    public void setIntakeVoltage(double volts) {
        intakeMotor.setVoltage(volts);
    }
}
