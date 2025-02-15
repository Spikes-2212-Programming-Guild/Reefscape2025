package frc.robot;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.util.PlaystationControllerWrapper;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class OI /*GEVALD*/{

    private final PlaystationControllerWrapper driverJoystick = new PlaystationControllerWrapper(0);
    private final PlaystationControllerWrapper navigatorJoystick = new PlaystationControllerWrapper(1);
    private boolean inAlgaeMode;

    public OI() {
        navigatorJoystick.getL1Button().onTrue(new ConditionalCommand(
                new PlaceCoralAndTakeAlgae(Elevator.getInstance(), AlgaeJoint.getInstance(), Gripper.getInstance(),
                        Drivetrain.getInstance(), CoralJoint.getInstance(), Storage.getInstance(),
                        Elevator.ElevatorLevel.L1), new PlaceOnReef(Elevator.getInstance(),
                CoralJoint.getInstance(), Storage.getInstance(), Elevator.ElevatorLevel.L1), () -> inAlgaeMode));

        navigatorJoystick.getR1Button().onTrue(new ConditionalCommand(
                new PlaceCoralAndTakeAlgae(Elevator.getInstance(), AlgaeJoint.getInstance(), Gripper.getInstance(),
                        Drivetrain.getInstance(), CoralJoint.getInstance(), Storage.getInstance(),
                        Elevator.ElevatorLevel.L2), new PlaceOnReef(Elevator.getInstance(),
                CoralJoint.getInstance(), Storage.getInstance(), Elevator.ElevatorLevel.L2), () -> inAlgaeMode));

        navigatorJoystick.getL2Button().onTrue(new ConditionalCommand(
                new PlaceCoralAndTakeAlgae(Elevator.getInstance(), AlgaeJoint.getInstance(), Gripper.getInstance(),
                        Drivetrain.getInstance(), CoralJoint.getInstance(), Storage.getInstance(),
                        Elevator.ElevatorLevel.L3), new PlaceOnReef(Elevator.getInstance(),
                CoralJoint.getInstance(), Storage.getInstance(), Elevator.ElevatorLevel.L3), () -> inAlgaeMode));

        navigatorJoystick.getR2Button().onTrue(new PlaceOnReef(Elevator.getInstance(), CoralJoint.getInstance(),
                Storage.getInstance(), Elevator.ElevatorLevel.L4));

        navigatorJoystick.getTriangleButton().onTrue(new InstantCommand(() -> inAlgaeMode = !inAlgaeMode));
        navigatorJoystick.getSquareButton().onTrue(new IntakeCoral(Storage.getInstance()));
        navigatorJoystick.getCircleButton().onTrue(new ReleaseAlgae(Gripper.getInstance()));

        navigatorJoystick.getRightStickButton().onTrue(new Reset(Elevator.getInstance(), CoralJoint.getInstance(),
                AlgaeJoint.getInstance()));

        navigatorJoystick.getUpButton().whileTrue(new MoveGenericSubsystem(Elevator.getInstance(),
                Elevator.ELEVATOR_FORWARD_SPEED));
        navigatorJoystick.getDownButton().whileTrue(new MoveGenericSubsystem(Elevator.getInstance(),
                Elevator.ELEVATOR_BACKWARD_SPEED));
        navigatorJoystick.getRightButton().whileTrue(new MoveGenericSubsystem(CoralJoint.getInstance(),
                CoralJoint.CORAL_JOINT_FORWARD_SPEED));
        navigatorJoystick.getLeftButton().whileTrue(new MoveGenericSubsystem(CoralJoint.getInstance(),
                CoralJoint.CORAL_JOINT_BACKWARD_SPEED));

        navigatorJoystick.getLeftStickButton().onTrue(new ReleaseCoral(Storage.getInstance()));

        navigatorJoystick.getShareButton().onTrue(new RotateAlgaeJointToBottom(AlgaeJoint.getInstance()));
        navigatorJoystick.getOptionsButton().onTrue(new IntakeAlgae(Gripper.getInstance()));
    }
}
