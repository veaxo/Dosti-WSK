package frc.robot.gamepad;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ExampleSubsystem;

/**
 * Пульт с ноутбука (MapperX → PitX «🎮 Пульт») через NetworkTables.
 *
 * MapperX/pad = double[11]:
 * [seq, fwd, strafe, turn, liftRate, armRate, clawRate, liftT, armT, clawT, speed]
 * rate -1..1, T &lt; 0 = нет цели, speed 0..1.
 * Пульт активен, пока seq меняется (watchdog 0.3 с): UI жив — робот едет,
 * UI закрыт/завис — seq стоит, робот сам останавливается.
 */
public class RemotePad {
  private static final int PAD_LEN = 11;
  private static final double WATCHDOG_SEC = 0.3;
  private static final double SERVO_RATE_DEG = 2.0;   // °/цикл при rate 1.0
  private static final double SERVO_TARGET_STEP = 3.0; // °/цикл к цели

  private final NetworkTableEntry padEntry;
  private final NetworkTableEntry stateEntry;

  private double lastSeq = Double.NaN;
  private double lastChangeAt = -100;
  private boolean active = false;
  private boolean justDeactivated = false;

  private final double[] v = new double[PAD_LEN];
  // lift, arm, claw: последняя цель из кадра и цель, к которой едем сейчас
  private final double[] lastT = {-1, -1, -1};
  private final double[] goalT = {-1, -1, -1};

  public RemotePad() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("MapperX");
    padEntry = table.getEntry("pad");
    stateEntry = table.getEntry("padState");
  }

  /** Раз в цикл: читает NT, ведёт watchdog, пишет padState. Моторы не трогает. */
  public void update() {
    double now = Timer.getFPGATimestamp();
    double[] raw = padEntry.getDoubleArray((double[]) null);
    boolean valid = raw != null && raw.length >= PAD_LEN;
    if (valid) {
      for (int i = 0; i < PAD_LEN; i++) {
        v[i] = Double.isFinite(raw[i]) ? raw[i] : (i >= 7 && i <= 9 ? -1 : 0);
      }
      if (Double.isNaN(lastSeq)) {
        lastSeq = v[0];   // первый кадр после старта — только база (в NT может лежать старое)
      } else if (v[0] != lastSeq) {
        lastSeq = v[0];
        lastChangeAt = now;
      }
    }
    boolean nowActive = valid && (now - lastChangeAt) < WATCHDOG_SEC;
    if (active && !nowActive) {
      justDeactivated = true;
      lastT[0] = lastT[1] = lastT[2] = -1;
      goalT[0] = goalT[1] = goalT[2] = -1;
    }
    active = nowActive;
    stateEntry.setString(active ? "active" : "stale");
  }

  public boolean isActive() {
    return active;
  }

  /** true один раз после active→inactive: вызвавший должен остановить моторы. */
  public boolean consumeJustDeactivated() {
    boolean r = justDeactivated;
    justDeactivated = false;
    return r;
  }

  private static double clamp(double x, double lo, double hi) {
    return Math.max(lo, Math.min(hi, x));
  }

  /** Применяет последний кадр: езда и сервы (вызывать только при isActive()). */
  public void apply(ExampleSubsystem s) {
    double speed = clamp(v[10], 0, 1);
    s.driveOmni(clamp(v[1], -1, 1) * speed, clamp(v[2], -1, 1) * speed,
        clamp(v[3], -1, 1) * speed);

    for (int i = 0; i < 3; i++) {
      double rate = clamp(v[4 + i], -1, 1);
      double target = v[7 + i] >= 0 ? v[7 + i] : -1;
      if (target != lastT[i]) {
        lastT[i] = target;
        if (target >= 0) {
          goalT[i] = target;        // новая цель; кадр без цели (-1) её не сбрасывает
        }
      }
      if (rate != 0) {
        goalT[i] = -1;              // ручной ход перебивает цель
      }
      double cur = i == 0 ? s.getServoLiftAngle()
          : i == 1 ? s.getServoArmAngle() : s.getServoClawAngle();
      if (goalT[i] >= 0) {
        boolean reached = i == 0 ? s.moveServoLiftToward(goalT[i], SERVO_TARGET_STEP)
            : i == 1 ? s.moveServoArmToward(goalT[i], SERVO_TARGET_STEP)
            : s.moveServoClawToward(goalT[i], SERVO_TARGET_STEP);
        if (reached) {
          goalT[i] = -1;
        }
      } else if (rate != 0) {
        double next = cur + rate * SERVO_RATE_DEG;
        if (i == 0) s.setServoLift(next);
        else if (i == 1) s.servo_Hook_Hand(next);
        else s.servo_Hook(next);
      }
    }
  }
}
