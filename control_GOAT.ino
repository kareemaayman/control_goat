// PID control with soft start using Exponential Smoothing Filter for motor speed control
// and implementation of a general PID class in Arduino IDE

// Define PID constants (you can tune these for your specific system)
float kp = 2.0;  // Proportional gain
float ki = 0.5;  // Integral gain
float kd = 0.1;  // Derivative gain

// Smoothing factor for the exponential smoothing filter (between 0 and 1)
float alpha = 0.1;  // Smoothing factor for soft start

// Desired speed setpoint (this can be RPM or any other speed unit depending on the sensor)
float setPoint = 100;

// Variables to hold motor speed, PID output, and filtered output
float input = 0;       // Current speed (from sensor)
float output = 0;      // PID output (before filtering)
float filteredOutput = 0;  // Output after applying soft start filter

// Motor control pin (PWM output)
const int motorPin = 9;  // Pin connected to motor driver

// PID class for modularity
class PID {
  private:
    float kp, ki, kd;    // PID gains
    float integral, lastError;  // Variables for integral and derivative terms

  public:
    // Constructor to initialize PID gains
    PID(float p, float i, float d) {
      kp = p;
      ki = i;
      kd = d;
      integral = 0;
      lastError = 0;
    }

    // Method to compute PID output
    float compute(float setPoint, float input) {
      // Calculate error between desired setpoint and current input
      float error = setPoint - input;

      // Calculate integral term by accumulating error over time
      integral += error;

      // Calculate derivative term as the change in error
      float derivative = error - lastError;

      // Save current error for the next iteration
      lastError = error;

      // Calculate PID output using the standard PID formula
      return (kp * error) + (ki * integral) + (kd * derivative);
    }
};

// Create a PID object with specified gains
PID pid(kp, ki, kd);

// Function to simulate reading motor speed (replace this with actual sensor code)
float readMotorSpeed() {
  // Example: reading from an analog pin (this could be an encoder or other sensor)
  return analogRead(A0);  // Read from a potentiometer or encoder
}

// Arduino setup function, runs once when the program starts
void setup() {
  // Set the motor control pin as output
  pinMode(motorPin, OUTPUT);

  // Initialize serial communication for debugging purposes
  Serial.begin(9600);
}

// Arduino loop function, runs repeatedly
void loop() {
  // Read current motor speed (replace this with actual sensor reading)
  input = readMotorSpeed();

  // Calculate the PID output using the PID class
  output = pid.compute(setPoint, input);

  // Apply exponential smoothing filter to the PID output for soft start
  filteredOutput = alpha * output + (1 - alpha) * filteredOutput;

  // Ensure the filtered output is within valid range (0-255 for PWM)
  filteredOutput = constrain(filteredOutput, 0, 255);

  // Write the filtered output to the motor driver (using PWM signal)
  analogWrite(motorPin, filteredOutput);

  // Print debugging information (setpoint, input, filtered output) to Serial Monitor
  Serial.print("Setpoint: ");
  Serial.print(setPoint);
  Serial.print(" | Input: ");
  Serial.print(input);
  Serial.print(" | Filtered Output: ");
  Serial.println(filteredOutput);

  // Delay for stability (adjust the loop frequency as needed)
  delay(100);
}
