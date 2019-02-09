package frc.team972.robot.lib;

public interface State<S> extends Interpolable<S> {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}