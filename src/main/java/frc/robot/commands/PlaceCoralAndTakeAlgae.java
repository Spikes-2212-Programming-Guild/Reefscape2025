package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

public class PlaceCoralAndTakeAlgae extends SequentialCommandGroup {

    private static final double speed = -0.2;

    public PlaceCoralAndTakeAlgae(Elevator elevator, AlgaeJoint algaeJoint, Gripper gripper,
                                  Drivetrain drivetrain, CoralJoint coralJoint, Storage storage,
                                  Elevator.ElevatorLevel level) {
        addCommands(new ParallelCommandGroup(
                        new MoveToHeight(elevator, level),
                        new RotateStorage(coralJoint, CoralJoint.StoragePose.PLACEMENT),
                        new RotateAlgaeJointToBottom(algaeJoint)),
                new ParallelCommandGroup(
                        new ReleaseCoral(storage), new IntakeAlgae(gripper)),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new Drive(drivetrain, () -> speed, () -> 0.0, () -> 0.0, false, false,
                                                false).withTimeout(0.5),
                                        new RotateStorage(coralJoint, CoralJoint.StoragePose.RESTING)
                                ),
                                new MoveToHeight(elevator, Elevator.ElevatorLevel.PROCESSOR)
                        ),
                        new ParallelCommandGroup(
                                new RotateAlgaeJointToTop(algaeJoint),
                                new RotateStorage(coralJoint, CoralJoint.StoragePose.INTAKE),
                                new MoveToHeight(elevator, Elevator.ElevatorLevel.BOTTOM)
                        ), gripper::hasAlgae));

    }
}
