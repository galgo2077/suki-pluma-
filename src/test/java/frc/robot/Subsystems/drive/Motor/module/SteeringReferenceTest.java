package frc.robot.Subsystems.drive.Motor.module;

import static org.junit.jupiter.api.Assertions.*;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;

class SteeringReferenceTest {
  @Test
  void manualZeroWritesZeroExactlyOnceDuringInitialization() {
    var writes = new AtomicInteger();
    var position = new AtomicReference<Double>();
    ModuleIOSpark.initializeSteeringReference(
        SteeringReferenceMode.MANUAL_ZERO,
        value -> {
          writes.incrementAndGet();
          position.set(value);
        });
    assertEquals(1, writes.get());
    assertEquals(0.0, position.get());
  }

  @Test
  void absoluteReferenceDoesNotManuallyZero() {
    var writes = new AtomicInteger();
    ModuleIOSpark.initializeSteeringReference(
        SteeringReferenceMode.ABSOLUTE_ENCODER, value -> writes.incrementAndGet());
    assertEquals(0, writes.get());
  }
}
