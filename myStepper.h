
#ifndef myStepper_h
#define myStepper_h

// library interface description
class myStepper {
  public:
    // constructors:
    myStepper();
    void initStepper(int numSteps, int imotor_pin_1, int imotor_pin_2, int imotor_pin_3, int imotor_pin_4);

    // speed setter method:
    void setSpeed(long whatSpeed);

    // mover method:
    void step(int number_of_steps);

    int version(void);

  private:
    void stepMotor(int this_step);

    int direction;            // Direction of rotation
    unsigned long step_delay; // delay between steps, in ms, based on speed
    int number_of_steps;      // total number of steps this motor can take
    int pin_count;            // how many pins are in use.
    int step_number;          // which step the motor is on

    // motor pin numbers:
    int motor_pin_1;
    int motor_pin_2;
    int motor_pin_3;
    int motor_pin_4;
    int motor_pin_5;          // Only 5 phase motor

    unsigned long last_step_time; // time stamp in us of when the last step was taken
};

#endif
