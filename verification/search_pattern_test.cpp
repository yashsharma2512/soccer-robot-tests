#include <cassert>
#include "SearchPattern.h"
int main() {
  const unsigned boundaries[] = {0, 349, 350, 599, 600, 1299, 1300, 1549, 1550, 2249, 2250, 2499, 2500, 2849, 2850, 5000};
  for (int side = -1; side <= 1; side += 2) {
    for (unsigned elapsed : boundaries) {
      search::Step step = search::at(elapsed, side, 60);
      assert(step.motion.forward >= 0);
      assert(step.motion.rotation == 0);
      assert(step.finished == (elapsed >= search::TOTAL_MS));
    }
    assert(search::at(0, side, 60).motion.strafe == side * 60);
    assert(search::at(600, side, 60).motion.strafe == -side * 60);
  }
}
