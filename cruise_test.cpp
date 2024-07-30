#include <iomanip>
#include <iostream>
#include <math.h>

using namespace std;

const int MAX_CM_UNTIL_IMPACT = 800;
const int MAX_SPEED = 100;

int precomputedValues[MAX_CM_UNTIL_IMPACT + 1][MAX_SPEED + 1];
int cm_buffer = 5;

bool cruise_control_enabled = true;

/**
 * Calculates the estimated cruise control speed based on impact conditions.
 *
 * @param cm_until_impact: Distance until impact in centimeters.
 * @param intAccel: Acceleration value.
 * @return Estimated average speed after adjusting for impact and acceleration.
 */
int calculateCruiseControl(int cm_until_impact, int intAccel) {
  // Calculate the estimated new speed based on time to impact
  float new_speed = sqrt(0.5 * cm_until_impact * intAccel);

  // Ensure new_speed does not exceed max_speed
  if (new_speed > MAX_SPEED) {
    new_speed = MAX_SPEED;
  }
  // Prevents tailgaiting by cm_buffer amount
  else if (new_speed <= cm_buffer) {
    new_speed -= cm_buffer;

    // No negative speed allowed
    if (new_speed < 0)
      new_speed = 0;
  }

  // Return the average speed
  return (new_speed + intAccel + 1) * 0.5;
}

/**
 * Precomputes and stores cruise control values for all possible combinations
 * of impact distance and acceleration.
 *
 * Uses calculateCruiseControl() to compute and populate precomputedValues[][].
 */
void precomputeCruiseControlValues() {
  for (int cm_until_impact = 0; cm_until_impact <= MAX_CM_UNTIL_IMPACT;
       ++cm_until_impact) {
    for (int intAccel = 0; intAccel <= MAX_SPEED; ++intAccel) {
      // Calculate cruise control value and store it
      precomputedValues[cm_until_impact][intAccel] =
          calculateCruiseControl(cm_until_impact, intAccel);
    }
  }
}

// Retrieves precomputed cruise control value based on impact distance and
// acceleration.
int cruiseControl(int cm_until_impact, int intAccel) {
  // Keep values within bounds
  if (cm_until_impact > MAX_CM_UNTIL_IMPACT)
    cm_until_impact = MAX_CM_UNTIL_IMPACT;
  if (intAccel > MAX_SPEED)
    intAccel = MAX_SPEED;

  // Return value
  return precomputedValues[cm_until_impact][intAccel];
}

void test(int distance = MAX_CM_UNTIL_IMPACT, int accel = 1) {
  for (int d = distance; d > 0; d--) {
    cout << "Distance: " << d << "cm" << setw(20) << "Acceleration: " << accel
         << setw(15);
    accel = cruiseControl(d, accel);
    cout << "Cruise: " << accel << endl;
  }
}

int main() {
  precomputeCruiseControlValues();

  // All distance values starting from distance 800 and accel 1
  if (cruise_control_enabled) {
    test();
  }

  // Manually test cruise control
  int dist_loop;
  int accel_loop;
  while (true) {
    cout << "Look up with cm_until_impact (0-800): ";
    cin >> dist_loop;
    cout << "Look up with acceleration (0-100): ";
    cin >> accel_loop;
    test(dist_loop, accel_loop);
  }
}