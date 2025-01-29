package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private SparkMax leftFlywheelMotor = new SparkMax(Constants.Shooter.leftLeaderFlywheelMotor,
            MotorType.kBrushless);
    private SparkMax rightFlywheelMotor = new SparkMax(Constants.Shooter.rightLeaderFlywheelMotor,
            MotorType.kBrushless);

    private boolean PIDEnabled = false;

    private PIDController leftPID = new PIDController(Constants.Shooter.kLeftShooterkP, Constants.Shooter.kLeftShooterkI,
            Constants.Shooter.kLeftShooterkD);
    private PIDController rightPID = new PIDController(Constants.Shooter.kRightShooterkP, Constants.Shooter.kRightShooterkI,
            Constants.Shooter.kRightShooterkD);
    private SimpleMotorFeedforward rightFF = new SimpleMotorFeedforward(Constants.Shooter.kRightShooterks,
            Constants.Shooter.kRightShooterkv, Constants.Shooter.kRightShooterkA);
    private SimpleMotorFeedforward leftFF = new SimpleMotorFeedforward(Constants.Shooter.kLeftShooterks,
            Constants.Shooter.kLeftShooterkv, Constants.Shooter.kLeftShooterkA);
    

    public SysIdRoutine routine = new SysIdRoutine(new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism((Voltage voltage) -> setFlywheelVoltage(voltage.in(Units.Volts), 0),
                    log ->
                    // Record a frame for the shooter motor.i
                    log.motor("Flywheel")
                            .voltage(
                                    Units.Volts.of(
                                            leftFlywheelMotor.getAppliedOutput() * leftFlywheelMotor.getBusVoltage()))
                            .angularPosition(Units.Rotations.of(leftFlywheelMotor.getEncoder().getPosition()))
                            .angularVelocity(Units.RotationsPerSecond.of(getLeftFlywheelSpeed())),
                    this));

    public ShooterSubsystem() {
        leftFlywheelMotor.setInverted(Constants.Shooter.kleftMotorInverted);
        rightFlywheelMotor.setInverted(Constants.Shooter.krightMotorInverted);
        leftPID.setTolerance(Constants.Shooter.PID_TOLERANCE);
        rightPID.setTolerance(Constants.Shooter.PID_TOLERANCE);

    }

    public void setSpeed(double leftrpm, double rightrpm) {
        leftPID.setSetpoint(leftrpm); // Set the setpoint of the PID controller
        rightPID.setSetpoint(rightrpm);
    }

    public void setSpeed(double rpm) {
        setSpeed(rpm, rpm);
    }

    public void setFlywheelVoltage(double leftVoltage, double rightVoltage) {
        leftFlywheelMotor.setVoltage(leftVoltage); // Set the voltage of the motor
        rightFlywheelMotor.setVoltage(rightVoltage);
        SmartDashboard.putNumber("Left Flywheel", leftVoltage);
        SmartDashboard.putNumber("Right Flywheel", rightVoltage);
    }

    public void setIdleSpeed() {
        leftFlywheelMotor.set(1000);
        rightFlywheelMotor.set(1000);
    }

    public double getLeftFlywheelSpeed() {
        return leftFlywheelMotor.getEncoder().getVelocity() * Constants.Shooter.CONVERSION_FACTOR; // Get th // motor
    }

    public double getRightFlywheelSpeed() {
        return rightFlywheelMotor.getEncoder().getVelocity() * Constants.Shooter.CONVERSION_FACTOR; // Get the // motor
    }

    public void periodic() {
        SmartDashboard.putNumber("Left PID", leftPID.getSetpoint());
        SmartDashboard.putNumber("Right PID", rightPID.getSetpoint());
        SmartDashboard.putNumber("Left Current", getLeftFlywheelSpeed());
        SmartDashboard.putNumber("Right Current", getRightFlywheelSpeed());
         

        double leftVoltage = leftPID.calculate(getLeftFlywheelSpeed()) + leftFF.calculate(leftPID.getSetpoint());
        double rightVoltage = rightPID.calculate(getRightFlywheelSpeed()) + rightFF.calculate(rightPID.getSetpoint());

        if (PIDEnabled) {
            setFlywheelVoltage(leftVoltage, rightVoltage);
        }

    }

    public boolean isBusy() {
        return !(leftPID.atSetpoint() && rightPID.atSetpoint());

    }

    public void enable() {
        PIDEnabled = true;
    }

    public void disable() {
        PIDEnabled = false;
        setFlywheelVoltage(0, 0);
    }

    public boolean getEnabled(){
        return PIDEnabled;
    }

    public Command sysIdQuasistaticc(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
}
