#include <Stepper.h>

#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38
#define X_MIN_PIN           3
#define X_MAX_PIN           2

#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19

#define step_rev 400

#define step1_dir 1
#define step2_dir 1
#define step3_dir -1

#define speed_tmp 480
#define reduction 24

//AccelStepper stepper1(AccelStepper::FULL2WIRE, X_DIR_PIN, X_STEP_PIN);
Stepper stepper1(step_rev, X_DIR_PIN, X_STEP_PIN);
Stepper stepper2(step_rev, Y_DIR_PIN, Y_STEP_PIN);
Stepper stepper3(step_rev, Z_DIR_PIN, Z_STEP_PIN);

float motor_pos[3] = {15, 12, 15};
float data_in[3] = { -1, -1, -1};

void setup()
{
  Serial.begin(115200);
  Serial3.begin(115200);

  pinMode(X_MIN_PIN, INPUT_PULLUP);
  pinMode(Y_MIN_PIN, INPUT_PULLUP);
  pinMode(Z_MIN_PIN, INPUT_PULLUP); //end stops setup

  pinMode(X_ENABLE_PIN, OUTPUT);
  digitalWrite(X_ENABLE_PIN, LOW); //LOW TO ENABLE
  pinMode(Y_ENABLE_PIN, OUTPUT);
  digitalWrite(Y_ENABLE_PIN, LOW); //LOW TO ENABLE
  pinMode(Z_ENABLE_PIN, OUTPUT);
  digitalWrite(Z_ENABLE_PIN, LOW); //LOW TO ENABLE

  auto_home(300);
  delay(500);
  sync_motion(300, 400, 1);
  delay(500);

  stepper1.setSpeed(speed_tmp);
  stepper2.setSpeed(speed_tmp);
  stepper3.setSpeed(speed_tmp);
}

void auto_home(int home_speed)
{

  stepper1.setSpeed(home_speed);
  stepper2.setSpeed(home_speed);
  stepper3.setSpeed(home_speed);
  while (true)
  {
    if (digitalRead(X_MIN_PIN))
      stepper1.step(-1 * step1_dir);
    if (digitalRead(Y_MIN_PIN))
      stepper2.step(-1 * step2_dir);
    if (digitalRead(Z_MIN_PIN))
      stepper3.step(-1 * step3_dir);
    if (!digitalRead(X_MIN_PIN) && !digitalRead(Y_MIN_PIN) && !digitalRead(Z_MIN_PIN))
      break;
  }
}

void sync_motion(int motion_speed, int angle, int dir)
{
  stepper1.setSpeed(motion_speed);
  stepper2.setSpeed(motion_speed);
  stepper3.setSpeed(motion_speed);

  for (int s = 0; s < angle; s++)
  {
    stepper1.step(dir * step1_dir);
    stepper2.step(dir * step2_dir);
    stepper3.step(dir * step3_dir);
  }
}

void serial_read(float *data)
{
  if (Serial.available())
  {
    while (Serial.read() != 'h') {}
    for (int i = 0; i < 3; i++)
      *(data + i) = Serial.parseFloat();
  }
}

int find_max(int *data, int len)
{
  int max_tmp = data[0];
  for (int s = 0; s < len; s++)
  {
    if (max_tmp < data[s])
      max_tmp = data[s];
  }
  return max_tmp;
}

void loop()
{
  serial_read(data_in);

  int step_move[3] = {0};
  int speed_move[3] = {0};

  if (data_in[0] > 0 && data_in[1] > 0 && data_in[2] > 0)
  {
    for (int s = 0; s < 3; s++)
      step_move[s] = int(((data_in[s] - motor_pos[s]) / 0.9f) * reduction);

    int max_step = abs(find_max(step_move, 3));
    double step1 = abs(double(max_step) / step_move[0]);
    double step2 = abs(double(max_step) / step_move[1]);
    double step3 = abs(double(max_step) / step_move[2]);

    double s1 = 0;
    double s2 = 0;
    double s3 = 0;

    for (int s = 0; s < max_step; s++)
    {
      if (s1 < max_step)
      {
        stepper1.step((step_move[0] / abs(step_move[0]))*step1_dir);
        s1 += step1;
      }
      if (s2 < max_step)
      {
        stepper2.step((step_move[1] / abs(step_move[1]))*step2_dir);
        s2 += step2;
      }
      if (s3 < max_step)
      {
        stepper3.step((step_move[2] / abs(step_move[2]))*step3_dir);
        s3 += step3;
      }
    }

    for (int s = 0; s < 3; s++)
      motor_pos[s] = data_in[s];
  }

  /*Serial3.print(step_move[0]); Serial3.print(" ");
    Serial3.print(step_move[1]); Serial3.print(" ");
    Serial3.print(step_move[2]); Serial3.print(" ");
    Serial3.println(" ");*/
  //Serial.println("cycle");
}
