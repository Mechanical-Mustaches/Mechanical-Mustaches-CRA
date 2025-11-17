// package frc.robot.subsystems;

// import frc.robot.Constants;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkBase;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkLowLevel;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkBaseConfig;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Elevator extends SubsystemBase {

//     public enum Level {
//         LDefault(-3, 0),
//         LIntake(-51, 0.38956),
//         LSource(0.75, 0),
//         L1(0, 0.07575),
//         L2(-7, 0.07575),
//         L3(-24, 0.07575),
//         L4(-60, 0.07575);

//         public final double elevatorEncoderValue;
//         public final double pivotEncoderValue;

//         private Level(double elevatorEncoder, double pivotEncoder) {
//             this.elevatorEncoderValue = elevatorEncoder;
//             this.pivotEncoderValue = pivotEncoder;

//         }
//     }

//     private RelativeEncoder eleEncoder;
//     private double requestedPosition;
//     private double elevatorDutyCycle;
//     private PIDController elevatorPID;
//     private final double gravityDutyCycle = 0.0;
//     private SparkMax leftMotor;
//     private SparkMax rightMotor;
//     private SparkMax backMotor;
//     private boolean initialized;

//     public Elevator() {
//         initialized = false;
//         leftMotor = new SparkMax(Constants.Elevator.LEFT_MOTOR, SparkLowLevel.MotorType.kBrushless);
//         rightMotor = new SparkMax(Constants.Elevator.RIGHT_MOTOR, SparkLowLevel.MotorType.kBrushless);

//         SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
//         SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
//         leftMotorConfig
//             .idleMode(SparkBaseConfig.IdleMode.kBrake)
//             .encoder.positionConversionFactor(Constants.Elevator.RADIANS_PER_REVOLUTION);
//         leftMotorConfig.closedLoop.apply(Constants.Elevator.CLOSED_LOOP_CONFIG);
//         rightMotorConfig
//             .apply(leftMotorConfig)
//             .follow(leftMotor, true);
//         leftMotor.configure(leftMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
//         rightMotor.configure(rightMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
//     }

//      public Command idle() {
//        return this.runOnce(() -> {initialized = false; });
//     }


//     public void periodic(){
//        if (!initialized) {
//            elevatorDutyCycle = 0.0;
//            requestedPosition = eleEncoder.getPosition();
//            elevatorPID = new PIDController(3.5, 0, 0);
//            elevatorPID.reset();
//            initialized = true;
//        }
//         elevatorDutyCycle = elevatorPID.calculate(eleEncoder.getPosition(), requestedPosition) + gravityDutyCycle;
//         elevatorDutyCycle = MathUtil.clamp(elevatorDutyCycle, -0.5, 0.5);
//         leftMotor.set(elevatorDutyCycle);
//     }

//     public Command L1(){
//         Level targteLevel;
//         return new InstantCommand((Level targetLevel) -> {
//             this.targetLevel = targetLevel;
//            requestedPosition = leftMotor.getClosedLoopController().setReference(targetLevel.elevatorEncoderValue, ControlType.kPosition);
//        }, this).repeatedly();
//    }




// }
