package frc.robot.subsystems;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants;

// Controls the Arm and Shooter motors and sensors, and contains all Arm-, Shooter-, and Climber-related commands
public class ArmSubsystem extends SubsystemBase {
    private SparkMax rightArmMotorOne = new SparkMax(Constants.Arm.rightMotorOneID, MotorType.kBrushless);
    private SparkMax rightArmMotorTwo = new SparkMax(Constants.Arm.rightMotorTwoID, MotorType.kBrushless);

    private SparkMax leftArmMotorOne = new SparkMax(Constants.Arm.leftMotorOneID, MotorType.kBrushless);
    private SparkMax leftArmMotorTwo = new SparkMax(Constants.Arm.leftMotorTwoID, MotorType.kBrushless);

    private DutyCycleEncoder armEncoder = new DutyCycleEncoder(Constants.Arm.armEncoderPortS);
    private Encoder armIncrementalEncoder = new Encoder(Constants.Arm.armEncoderPortA, Constants.Arm.armEncoderPortB);

    private double initialArmPosition;

    private PIDController PID = new PIDController(Constants.Arm.armkP, Constants.Arm.armkI, Constants.Arm.armkD);
    private ArmFeedforward FF = new ArmFeedforward(Constants.Arm.armks, Constants.Arm.armkg, Constants.Arm.armkv);

    private boolean enabled = false;

    private TrapezoidProfile profile = new TrapezoidProfile(Constants.Arm.kArmMotionConstraint);

    private TrapezoidProfile.State goalState, setpointState;

    public SysIdRoutine routine = new SysIdRoutine(new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism((Voltage voltage) -> setVoltage(voltage.in(Units.Volts)), log ->
            // Record a frame for the shooter motor.
            log.motor("arm")
                    .voltage(
                            Units.Volts.of(leftArmMotorOne.getAppliedOutput() * leftArmMotorOne.getBusVoltage()))
                    .angularPosition(Units.Rotations.of(getPosition()))
                    .angularVelocity(Units.RotationsPerSecond.of(getVelocity())), this));

    // Constructor
    public ArmSubsystem() {

        SparkMaxConfig lConfig = new SparkMaxConfig();
        SparkMaxConfig fConfig = new SparkMaxConfig();
        SparkMaxConfig rConfig = new SparkMaxConfig();
        lConfig.smartCurrentLimit(60).idleMode(IdleMode.kBrake).inverted(false).disableFollowerMode();
        fConfig.smartCurrentLimit(60).idleMode(IdleMode.kBrake).follow(leftArmMotorOne, false);
        rConfig.smartCurrentLimit(60).idleMode(IdleMode.kBrake).follow(leftArmMotorOne, true);

        
        leftArmMotorOne.configure(lConfig, null, null);
        leftArmMotorTwo.configure(fConfig, null, null);
        rightArmMotorOne.configure(rConfig, null, null);
        rightArmMotorTwo.configure(rConfig, null, null);

        
        //armEncoder.setPositionOffset(Constants.Arm.horizontalArmOffset);
        
        armIncrementalEncoder.setDistancePerPulse(1.0 / Constants.Arm.armTicksPerRevolution);
        armIncrementalEncoder.setReverseDirection(true);
        initialArmPosition = armEncoder.get();
        // goalState = new TrapezoidProfile.State(getPosition(), getVelocity());
        setpointState = new TrapezoidProfile.State(getPosition(), getVelocity());
        setPosition(Constants.Arm.SetPointPositions.kStowPosition);
    }

    public void setVoltage(double voltage) {
        leftArmMotorOne.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Arm Quadrature Encoder",
        // armIncrementalEncoder.getDistance());
        SmartDashboard.putNumber("Duty Cycle Encoder", armEncoder.get());

        // if (profile.isFinished(0.2)) {
        // setpointState = new TrapezoidProfile.State(getPosition(), 0);
        // }
        
        setpointState = profile.calculate(0.02, setpointState, goalState);
        double voltage = PID.calculate(getPosition(), setpointState.position)
                + FF.calculate(setpointState.position * 2 * Math.PI, setpointState.velocity);
        SmartDashboard.putNumber("voltage", voltage);
        SmartDashboard.putNumber("Arm Angle", armEncoder.get());
        if (enabled) {
            setVoltage(voltage);
        }

        

    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
    }

    public double getPosition() {
        return armEncoder.get();
    }

    public double getVelocity() {
        return armIncrementalEncoder.getRate();
    }

    public void setPosition(double position) {
        goalState = new TrapezoidProfile.State(position, 0);
    }

    public boolean isBusy() {
        return !PID.atSetpoint();
    }

    public Command sysIdQuasistaticc(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

}
