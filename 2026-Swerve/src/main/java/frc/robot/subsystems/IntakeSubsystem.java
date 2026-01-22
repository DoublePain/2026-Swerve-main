package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import edu.wpi.first.wpilibj.smartdashboard.*;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.NovaWrapper;
import yams.motorcontrollers.local.SparkWrapper;


public class IntakeSubsystem  extends SubsystemBase {
    
     private final SparkMax                IntakearmMotor    = new SparkMax(Constants.IDConstants.IntakearmMotor_ID, MotorType.kBrushless);
      private final SparkMax               IntakeMotor    = new SparkMax(Constants.IDConstants.IntakeMotor_ID, MotorType.kBrushless);
  
  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(1, 0, 0, DegreesPerSecond.of(50), DegreesPerSecondPerSecond.of(30))
      .withSoftLimit(Degrees.of(5), Degrees.of(90))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
      .withControlMode(ControlMode.CLOSED_LOOP);
  private final SmartMotorController       motor            = new SparkWrapper(IntakearmMotor,
                                                                              DCMotor.getNEO(1),
                                                                              motorConfig);
  
  private       ArmConfig m_config = new ArmConfig(motor)
      .withLength(Meters.of(0.135))
      .withHardLimit(Degrees.of(0), Degrees.of(100))
      .withTelemetry("IntakeArm", TelemetryVerbosity.HIGH)
      .withMass(Pounds.of(8))
      .withStartingPosition(Degrees.of(0));

  private final Arm       intakeArm      = new Arm(m_config);

  public IntakeSubsystem()
  {
  }

  public void periodic()
  {
    intakeArm.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    intakeArm.simIterate();
  }

  public Command armCmd(double dutycycle)
  {
    return intakeArm.set(dutycycle);
  }

  public Command sysId()
  {
    return intakeArm.sysId(Volts.of(3), Volts.of(3).per(Second), Second.of(30));
  }

  public Command setAngle(Angle angle)
  {
    return intakeArm.setAngle(angle);
  }
  
 // Method to set the speed of both motors
    public void setSpeed(double speed) {
        IntakeMotor.set(speed);

    }

public void Intake() {
        setSpeed(Constants.ShooterConstants.Shooter_Speed);
    }
    public Command IntakeCommand(double speed){
        return run(() -> Intake());
    }

}



