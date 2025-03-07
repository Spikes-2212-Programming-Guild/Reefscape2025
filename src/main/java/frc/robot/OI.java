package frc.robot;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.PlaystationControllerWrapper;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeFromFeeder;
import frc.robot.commands.ReleaseCoral;
import frc.robot.commands.RotateStorage;
import frc.robot.subsystems.*;

import java.util.function.Supplier;

public class OI /*GEVALD*/ {

    public enum Side {

        LEFT, RIGHT;
    }

    private static final RootNamespace namespace = new RootNamespace("oi");
    private static final Supplier<Double> power = namespace.addConstantDouble("power", 2);

        private final PlaystationControllerWrapper driverJoystick = new PlaystationControllerWrapper(0);
    //    private final PlaystationControllerWrapper driverJoystick = new PlaystationControllerWrapper(1);
    private final PlaystationControllerWrapper navigatorJoystick = new PlaystationControllerWrapper(1);
    private final Joystick leftJoystick = new Joystick(1);
    private final Joystick rightJoystick = new Joystick(2);

        private final Elevator elevator = Elevator.getInstance();
    private final Storage storage = Storage.getInstance();
    //    private final CoralJoint coralJoint = CoralJoint.getInstance();
    private final CoralJoint coralJoint = CoralJoint.getInstance();
    private final AlgaeJoint algaeJoint = AlgaeJoint.getInstance();
    private final Drivetrain drivetrain = Drivetrain.getInstance();

    private boolean inAlgaeMode;

    private double currentSetpoint;

    public OI() {
//        navigatorJoystick.getL1Button().onTrue(new ConditionalCommand(
//                new PlaceCoralAndTakeAlgae(elevator, algaeJoint, gripper,
//                        drivetrain, coralJoint, storage,
//                        Elevator.ElevatorLevel.L1),
//                new PlaceOnReef(elevator,
//                        coralJoint, storage, Elevator.ElevatorLevel.L1),
//                () -> inAlgaeMode));
        navigatorJoystick.getL1Button().onTrue(new RotateStorage(coralJoint, CoralJoint.StoragePose.L1));
        navigatorJoystick.getR1Button().onTrue(new RotateStorage(coralJoint, CoralJoint.StoragePose.L2));
//        navigatorJoystick.getLeftButton().onTrue(new InstantCommand(coralJoint::finish, coralJoint));

//        IntakeCoral intakeCoral = new IntakeCoral(storage);
//        navigatorJoystick.getTriangleButton().onTrue(new InstantCommand(() -> {
//            if (intakeCoral.isScheduled()) intakeCoral.end(true);
//            else intakeCoral.schedule();
//        }));

//        navigatorJoystick.getTriangleButton().onTrue(new IntakeCoral(storage));
        navigatorJoystick.getTriangleButton().onTrue(new IntakeFromFeeder(elevator, coralJoint, storage));
        navigatorJoystick.getCircleButton().onTrue(new ReleaseCoral(storage));

        navigatorJoystick.getUpButton().whileTrue(new MoveGenericSubsystem(coralJoint,
                CoralJoint.CORAL_JOINT_FORWARD_SPEED) {
            @Override
            public void end(boolean interrupted) {
                double setpoint = coralJoint.getPosition();
                new RotateStorage(coralJoint, () -> setpoint).schedule();
            }
        });
        navigatorJoystick.getDownButton().whileTrue(new MoveGenericSubsystem(coralJoint,
                CoralJoint.CORAL_JOINT_BACKWARD_SPEED) {
//            @Override
//            public void end(boolean interrupted) {
//                double setpoint = coralJoint.getPose();
//                new District2RotateStorage(coralJoint, () -> setpoint).schedule();
//            }
        });

        navigatorJoystick.getSquareButton().onTrue(new RotateStorage(coralJoint,
                CoralJoint.StoragePose.INTAKE));
        navigatorJoystick.getRightButton().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
        driverJoystick.getR1Button().onTrue(new InstantCommand(drivetrain::resetGyro));
        new JoystickButton(rightJoystick, 1).onTrue(new InstantCommand(drivetrain::resetGyro));
//
//        navigatorJoystick.getR1Button().onTrue(new ConditionalCommand(
//                new PlaceCoralAndTakeAlgae(elevator, algaeJoint, gripper,
//                        drivetrain, coralJoint, storage,
//                        Elevator.ElevatorLevel.L2),
//                new PlaceOnReef(elevator,
//                        coralJoint, storage, Elevator.ElevatorLevel.L2),
//                () -> inAlgaeMode));
//
//        navigatorJoystick.getL2Button().onTrue(new ConditionalCommand(
//                new PlaceCoralAndTakeAlgae(elevator, algaeJoint, gripper,
//                        drivetrain, coralJoint, storage,
//                        Elevator.ElevatorLevel.L3),
//                new PlaceOnReef(elevator,
//                        coralJoint, storage, Elevator.ElevatorLevel.L3),
//                () -> inAlgaeMode));
//
//        navigatorJoystick.getR2Button().onTrue(new PlaceOnReef(elevator, coralJoint,
//                storage, Elevator.ElevatorLevel.L4));

//        navigatorJoystick.getTriangleButton().onTrue(new InstantCommand(() -> inAlgaeMode = !inAlgaeMode));
//        navigatorJoystick.getSquareButton().onTrue(new IntakeCoral(storage));
//        navigatorJoystick.getCircleButton().onTrue(new ReleaseAlgae(gripper));
//
//        navigatorJoystick.getRightStickButton().onTrue(new Reset(elevator, coralJoint,
//                algaeJoint));
//
//        navigatorJoystick.getUpButton().whileTrue(new MoveGenericSubsystem(elevator,
//                Elevator.ELEVATOR_FORWARD_SPEED));
//        navigatorJoystick.getDownButton().whileTrue(new MoveGenericSubsystem(elevator,
//                Elevator.ELEVATOR_BACKWARD_SPEED));
//        navigatorJoystick.getRightButton().whileTrue(new MoveGenericSubsystem(coralJoint,
//                CoralJoint.CORAL_JOINT_FORWARD_SPEED));
//        navigatorJoystick.getLeftButton().whileTrue(new MoveGenericSubsystem(coralJoint,
//                CoralJoint.CORAL_JOINT_BACKWARD_SPEED));
//
//        navigatorJoystick.getLeftStickButton().onTrue(new ReleaseCoral(storage));
//
//        navigatorJoystick.getShareButton().onTrue(new RotateAlgaeJointToBottom(algaeJoint));
//        navigatorJoystick.getOptionsButton().onTrue(new IntakeAlgae(gripper));
//
//        driverJoystick.getR1Button().onTrue(new InstantCommand(drivetrain::resetGyro));
    }

    public double getLeftX() {
        double val = driverJoystick.getLeftX();
//        double val = leftJoystick.getX();
        return Math.signum(val) * Math.pow(val, power.get());
    }

    public double getLeftY() {
        double val = driverJoystick.getLeftY();
//        double val = leftJoystick.getY();
        return Math.signum(val) * Math.pow(val, power.get());
    }

    public double getRightX() {
        double val = driverJoystick.getRightX();
//        double val = rightJoystick.getX();
        return Math.signum(val) * Math.pow(val, power.get());
    }

    public double getRightY() {
        double val = driverJoystick.getRightY();
//        double val = rightJoystick.getY();
        return Math.signum(val) * Math.pow(val, power.get());
    }
}
