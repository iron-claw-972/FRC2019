package frc.team972.robot.lib;

public interface Interpolable<T> {
    public T interpolate(T other, double x);
}