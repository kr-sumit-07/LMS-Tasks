
#include <iostream>
using namespace std;

const int SENSOR_COUNT = 5;
const int MAX_SENSOR_READING = 4095;

// Part A: Estimate line position using weighted centroid.
// Returns false if input is invalid or all readings are zero.
bool estimateLinePosition(const int readings[],
                          int count,
                          double &position) {
    const int weights[SENSOR_COUNT] = {-2, -1, 0, 1, 2};

    // Validate array pointer and number of readings.
    if (readings == nullptr || count != SENSOR_COUNT) {
        return false;
    }

    long long weightedSum = 0;
    long long totalReading = 0;

    for (int i = 0; i < SENSOR_COUNT; i++) {
        // Sensor readings must be within 0-4095.
        if (readings[i] < 0 ||
            readings[i] > MAX_SENSOR_READING) {
            return false;
        }

        weightedSum +=
            static_cast<long long>(readings[i]) * weights[i];

        totalReading += readings[i];
    }

    // Avoid division by zero.
    if (totalReading == 0) {
        return false;
    }

    position = static_cast<double>(weightedSum)
               / totalReading;

    return true;
}

// Part B: Clamp a motor command while preserving its sign.
// Returns false if the maximum magnitude is invalid.
bool clampMotorCommand(int command,
                       int maxMagnitude,
                       int &limitedCommand) {
    // A negative maximum magnitude is invalid.
    if (maxMagnitude < 0) {
        return false;
    }

    // Clamp positive commands.
    if (command > maxMagnitude) {
        limitedCommand = maxMagnitude;
    }
    // Clamp negative commands.
    else if (command < -maxMagnitude) {
        limitedCommand = -maxMagnitude;
    }
    // Command is already within the permitted range.
    else {
        limitedCommand = command;
    }

    return true;
}

int main() {
    cout << "=== Part A: Line Position ===\n";

    // Test 1: Normal readings.
    int readings1[5] = {0, 100, 300, 200, 0};
    double position = 0.0;

    if (estimateLinePosition(readings1, 5, position)) {
        cout << "Normal position: " << position << '\n';
    } else {
        cout << "Invalid readings or line unavailable\n";
    }

    // Test 2: All readings are zero.
    int readings2[5] = {0, 0, 0, 0, 0};

    if (!estimateLinePosition(readings2, 5, position)) {
        cout << "All-zero case handled\n";
    }

    // Test 3: Negative sensor reading.
    int readings3[5] = {100, -5, 300, 200, 0};

    if (!estimateLinePosition(readings3, 5, position)) {
        cout << "Negative reading rejected\n";
    }

    // Test 4: Reading exceeds the assumed ADC range.
    int readings4[5] = {100, 200, 5000, 200, 100};

    if (!estimateLinePosition(readings4, 5, position)) {
        cout << "Out-of-range reading rejected\n";
    }

    // Test 5: Wrong number of readings.
    if (!estimateLinePosition(readings1, 4, position)) {
        cout << "Incorrect sensor count rejected\n";
    }

    cout << "\n=== Part B: Motor Commands ===\n";

    const int limit = 255;
    int result;

    // Test 6: Positive command exceeds limit.
    if (clampMotorCommand(300, limit, result)) {
        cout << "300 -> " << result << '\n';
    }

    // Test 7: Negative command exceeds limit.
    if (clampMotorCommand(-300, limit, result)) {
        cout << "-300 -> " << result << '\n';
    }

    // Test 8: Command is within limit.
    if (clampMotorCommand(100, limit, result)) {
        cout << "100 -> " << result << '\n';
    }

    // Test 9: Zero maximum magnitude.
    if (clampMotorCommand(-50, 0, result)) {
        cout << "Limit 0: " << result << '\n';
    }

    // Test 10: Invalid negative maximum magnitude.
    if (!clampMotorCommand(100, -1, result)) {
        cout << "Negative limit rejected\n";
    }

    // Test 11: Extreme negative command.
    if (clampMotorCommand(-2147483647 - 1,
                          limit, result)) {
        cout << "Extreme negative command -> "
             << result << '\n';
    }

    return 0;
}
