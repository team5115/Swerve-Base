package frc.team5115.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.team5115.Constants;

public class ShooterIOSim implements ShooterIO {
    private final FlywheelSim leftSim;
    private final FlywheelSim rightSim;
    private final FlywheelSim auxSim;
    private final DCMotorSim intakeSim;
    private final DCMotorSim amperSim;

    private double leftAppliedVolts;
    private double rightAppliedVolts;
    private double auxAppliedVolts;
    private double intakeAppliedVolts;
    private double amperAppliedVolts;

    public ShooterIOSim() {
        leftSim = new FlywheelSim(DCMotor.getNEO(1), 1.0, 0.0002);
        rightSim = new FlywheelSim(DCMotor.getNEO(1), 1.0, 0.0002);
        auxSim = new FlywheelSim(DCMotor.getNEO(1), 1.0, 0.0002);
        intakeSim = new DCMotorSim(DCMotor.getNEO(1), 1.0, 0.0002);
        amperSim = new DCMotorSim(new DCMotor(+12.0, +7.909, +24.0, +5.0, +10.472, +1), +1.0, 0.0002);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        leftSim.update(Constants.LOOP_PERIOD_SECS);
        rightSim.update(Constants.LOOP_PERIOD_SECS);
        auxSim.update(Constants.LOOP_PERIOD_SECS);
        intakeSim.update(Constants.LOOP_PERIOD_SECS);
        amperSim.update(Constants.LOOP_PERIOD_SECS);

        inputs.leftVelocityRPM = leftSim.getAngularVelocityRPM();
        inputs.leftAppliedVolts = leftAppliedVolts;
        inputs.leftCurrentAmps = Math.abs(leftSim.getCurrentDrawAmps());

        inputs.rightVelocityRPM = rightSim.getAngularVelocityRPM();
        inputs.rightAppliedVolts = rightAppliedVolts;
        inputs.rightCurrentAmps = Math.abs(rightSim.getCurrentDrawAmps());

        inputs.auxVelocityRPM = auxSim.getAngularVelocityRPM();
        inputs.auxAppliedVolts = auxAppliedVolts;
        inputs.auxCurrentAmps = Math.abs(auxSim.getCurrentDrawAmps());

        inputs.intakeVelocityRPM = intakeSim.getAngularVelocityRPM();
        inputs.intakeAppliedVolts = intakeAppliedVolts;
        inputs.intakeCurrentAmps = Math.abs(intakeSim.getCurrentDrawAmps());

        inputs.amperPosition = Rotation2d.fromRadians(amperSim.getAngularPositionRad());
        inputs.amperAppliedVolts = amperAppliedVolts;
        inputs.amperCurrentAmps = Math.abs(amperSim.getCurrentDrawAmps());

        inputs.noteDetected = false;
    }

    @Override
    public void setAuxVoltage(double volts) {
        auxAppliedVolts = MathUtil.clamp(volts, -12.0, +12.0);
        auxSim.setInputVoltage(auxAppliedVolts);
    }

    @Override
    public void setRightVoltage(double volts) {
        rightAppliedVolts = MathUtil.clamp(volts, -12.0, +12.0);
        rightSim.setInputVoltage(rightAppliedVolts);
    }

    @Override
    public void setLeftVoltage(double volts) {
        leftAppliedVolts = MathUtil.clamp(volts, -12.0, +12.0);
        leftSim.setInputVoltage(leftAppliedVolts);
    }

    @Override
    public void setIntakeVoltage(double volts) {
        intakeAppliedVolts = MathUtil.clamp(volts, -12.0, +12.0);
        intakeSim.setInputVoltage(intakeAppliedVolts);
    }

    @Override
    public void setAmperVoltage(double volts) {
        amperAppliedVolts = MathUtil.clamp(volts, -12.0, +12.0);
        amperSim.setInputVoltage(amperAppliedVolts);
    }

    @Override
    public void setIntakePercent(double percent) {
        intakeAppliedVolts = MathUtil.clamp(percent * 12, -12.0, +12.0);
        intakeSim.setInputVoltage(intakeAppliedVolts);
    }

    @Override
    public void setAmperPercent(double percent) {
        amperAppliedVolts = MathUtil.clamp(percent * 12, -12.0, +12.0);
        amperSim.setInputVoltage(amperAppliedVolts);
    }

    @Override
    public void setAuxPercent(double percent) {
        auxAppliedVolts = MathUtil.clamp(percent * 12, -12.0, +12.0);
        auxSim.setInputVoltage(auxAppliedVolts);
    }

    @Override
    public void setLeftPercent(double percent) {
        leftAppliedVolts = MathUtil.clamp(percent * 12, -12.0, +12.0);
        leftSim.setInputVoltage(leftAppliedVolts);
    }

    @Override
    public void setRightPercent(double percent) {
        rightAppliedVolts = MathUtil.clamp(percent * 12, -12.0, +12.0);
        rightSim.setInputVoltage(rightAppliedVolts);
    }
}
