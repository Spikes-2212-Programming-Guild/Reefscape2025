package frc.robot;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.util.PlaystationControllerWrapper;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class OI /*GEVALD*/ {

    private final PlaystationControllerWrapper driverJoystick = new PlaystationControllerWrapper(0);
    private final PlaystationControllerWrapper navigatorJoystick = new PlaystationControllerWrapper(1);
    
    private final Elevator elevator = Elevator.getInstance();
    private final Storage storage = Storage.getInstance();
    private final Gripper gripper = Gripper.getInstance();
    private final CoralJoint coralJoint = CoralJoint.getInstance();
    private final AlgaeJoint algaeJoint = AlgaeJoint.getInstance();
    private final Drivetrain drivetrain = Drivetrain.getInstance();

    private boolean inAlgaeMode;

    public OI() {
        navigatorJoystick.getL1Button().onTrue(new ConditionalCommand(
                new PlaceCoralAndTakeAlgae(elevator, algaeJoint, gripper,
                        drivetrain, coralJoint, storage,
                        Elevator.ElevatorLevel.L1),
                new PlaceOnReef(elevator,
                        coralJoint, storage, Elevator.ElevatorLevel.L1),
                () -> inAlgaeMode));

        navigatorJoystick.getR1Button().onTrue(new ConditionalCommand(
                new PlaceCoralAndTakeAlgae(elevator, algaeJoint, gripper,
                        drivetrain, coralJoint, storage,
                        Elevator.ElevatorLevel.L2),
                new PlaceOnReef(elevator,
                        coralJoint, storage, Elevator.ElevatorLevel.L2),
                () -> inAlgaeMode));

        navigatorJoystick.getL2Button().onTrue(new ConditionalCommand(
                new PlaceCoralAndTakeAlgae(elevator, algaeJoint, gripper,
                        drivetrain, coralJoint, storage,
                        Elevator.ElevatorLevel.L3),
                new PlaceOnReef(elevator,
                        coralJoint, storage, Elevator.ElevatorLevel.L3),
                () -> inAlgaeMode));

        navigatorJoystick.getR2Button().onTrue(new PlaceOnReef(elevator, coralJoint,
                storage, Elevator.ElevatorLevel.L4));

        navigatorJoystick.getTriangleButton().onTrue(new InstantCommand(() -> inAlgaeMode = !inAlgaeMode));
        navigatorJoystick.getSquareButton().onTrue(new IntakeCoral(storage));
        navigatorJoystick.getCircleButton().onTrue(new ReleaseAlgae(gripper));

        navigatorJoystick.getRightStickButton().onTrue(new Reset(elevator, coralJoint,
                algaeJoint));

        navigatorJoystick.getUpButton().whileTrue(new MoveGenericSubsystem(elevator,
                Elevator.ELEVATOR_FORWARD_SPEED));
        navigatorJoystick.getDownButton().whileTrue(new MoveGenericSubsystem(elevator,
                Elevator.ELEVATOR_BACKWARD_SPEED));
        navigatorJoystick.getRightButton().whileTrue(new MoveGenericSubsystem(coralJoint,
                CoralJoint.CORAL_JOINT_FORWARD_SPEED));
        navigatorJoystick.getLeftButton().whileTrue(new MoveGenericSubsystem(coralJoint,
                CoralJoint.CORAL_JOINT_BACKWARD_SPEED));

        navigatorJoystick.getLeftStickButton().onTrue(new ReleaseCoral(storage));

        navigatorJoystick.getShareButton().onTrue(new RotateAlgaeJointToBottom(algaeJoint));
        navigatorJoystick.getOptionsButton().onTrue(new IntakeAlgae(gripper));

        driverJoystick.getR1Button().onTrue(new InstantCommand(drivetrain::resetGyro));
    }

    public double getLeftX() {
        return driverJoystick.getLeftX();
    }

    public double getLeftY() {
        return driverJoystick.getLeftY();
    }

    public double getRightX() {
        return driverJoystick.getRightX();
    }

    public double getRightY() {
        return driverJoystick.getRightY();
    }
}
