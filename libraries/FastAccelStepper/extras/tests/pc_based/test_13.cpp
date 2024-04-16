#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include "FastAccelStepper.h"
#include "StepperISR.h"

char TCCR1A;
char TCCR1B;
char TCCR1C;
char TIMSK1;
char TIFR1;
unsigned short OCR1A;
unsigned short OCR1B;

StepperQueue fas_queue[NUM_QUEUES];

void inject_fill_interrupt(int mark) {}
void noInterrupts() {}
void interrupts() {}

#include "RampChecker.h"

class FastAccelStepperTest {
 public:
  void init_queue() {
    fas_queue[0].read_idx = 0;
    fas_queue[1].read_idx = 0;
    fas_queue[0].next_write_idx = 0;
    fas_queue[1].next_write_idx = 0;
  }

  void ramp(uint32_t accel, uint32_t speed_us, uint32_t steps,
            bool reach_coasting) {
    init_queue();
    FastAccelStepper s = FastAccelStepper();
    s.init(NULL, 0, 0);
    RampChecker rc = RampChecker();
    assert(0 == s.getCurrentPosition());

    assert(s.isQueueEmpty());
    s.setSpeedInUs(speed_us);
    s.setAcceleration(accel);
    s.fill_queue();
    assert(s.isQueueEmpty());
    s.move(steps);
    s.fill_queue();
    assert(!s.isQueueEmpty());
    float old_planned_time_in_buffer = 0;

    char fname[100];
    sprintf(fname, "test_12.gnuplot");
    rc.start_plot(fname);
    bool coast = false;
    for (int i = 0; i < 100 * steps; i++) {
      if (true) {
        printf(
            "Loop %d: Queue read/write = %d/%d    Target pos = %d, Queue End "
            "pos = %d  QueueEmpty=%s\n",
            i, fas_queue[0].read_idx, fas_queue[0].next_write_idx,
            s.targetPos(), s.getPositionAfterCommandsCompleted(),
            s.isQueueEmpty() ? "yes" : "no");
      }
      if (!s.isRampGeneratorActive()) {
        break;
      }
      s.fill_queue();
      uint32_t from_dt = rc.total_ticks;
      while (!s.isQueueEmpty()) {
        rc.increase_ok = true;
        rc.decrease_ok = true;
        rc.check_section(
            &fas_queue[0].entry[fas_queue[0].read_idx & QUEUE_LEN_MASK]);
        fas_queue[0].read_idx++;
      }
      if ((s.rampState() & RAMP_STATE_MASK) == RAMP_STATE_COAST) {
        coast = true;
      }
      uint32_t to_dt = rc.total_ticks;
      float planned_time = (to_dt - from_dt) * 1.0 / 16000000;
      printf("planned time in buffer: %.6fs\n", planned_time);
      // This must be ensured, so that the stepper does not run out of commands
      assert((i == 0) || (old_planned_time_in_buffer > 0.005));
      old_planned_time_in_buffer = planned_time;
    }
    while (!s.isQueueEmpty()) {
      rc.increase_ok = true;
      rc.decrease_ok = true;
      rc.check_section(
          &fas_queue[0].entry[fas_queue[0].read_idx & QUEUE_LEN_MASK]);
      fas_queue[0].read_idx++;
    }
    rc.finish_plot();
    test(!s.isRampGeneratorActive(), "too many commands created");
    test(s.getCurrentPosition() == steps, "has not reached target position");
    test(coast == reach_coasting, "coasting target not met");
    printf("Total time  %f\n", rc.total_ticks / 16000000.0);

#if (TEST_CREATE_QUEUE_CHECKSUM == 1)
    printf("CHECKSUM for %d/%d/%d: %d\n", steps, travel_dt, accel, s.checksum);
#endif
  }
};
int main() {
  FastAccelStepperTest test;
  for (uint16_t s = 1; s <= 255; s++) {
    printf("test with steps s=%d\n", s);
    test.ramp(INT32_MAX, 50, s, false);
    puts("");
  }
  printf("TEST_13 PASSED\n");
  return 0;
}
