package frc.robot.subsystems.DMS;

public class DriveInfo<T> {
    public T FL;
    public T FR;
    public T RL;
    public T RR;

    public DriveInfo(T value) {
        FL = value;
        FR = value;
        RL = value;
        RR = value;
    }

    public DriveInfo(T fl, T fr, T rl, T rr) {
        FL = fl;
        FR = fr;
        RL = rl;
        RR = rr;
    }

    @Override
    public String toString() {
        return String.format("DriveInfo(FL=%s, FR=%s, RL=%s, RR=%s)", FL, FR, RL, RR);
    }

}
