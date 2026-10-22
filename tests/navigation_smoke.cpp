#include "../practice_tank/Navigation.h"
#include <cassert>
#include <cmath>

int main() {
  using namespace Navigation;
  assert(std::fabs(angleError(-3.13f, 3.13f)) < 0.03f); // wrap at pi
  assert(step({0.5f, 0.5f, 0.0f, false}, 1.0f, 1.0f, 0.5f).status == Command::INVALID_POSE);
  assert(step({0.5f, 0.5f, 0.0f, true}, 0.0f, 1.0f, 0.5f).status == Command::INVALID_RANGE);
  assert(step({0.5f, 0.5f, 0.0f, true}, 0.19f, 1.0f, 0.5f).status == Command::BLOCKED);
  assert(step({1.0f, 1.0f, 0.0f, true}, 1.0f, 1.02f, 1.02f).status == Command::ARRIVED);
  Command forward = step({0.5f, 0.5f, 0.0f, true}, 1.0f, 1.0f, 0.5f);
  assert(forward.status == Command::MOVING && forward.left == forward.right && forward.left > 0);
  Command left = step({0.5f, 0.5f, 0.0f, true}, 1.0f, 0.5f, 1.0f);
  assert(left.left < 0 && left.right > 0);
}
