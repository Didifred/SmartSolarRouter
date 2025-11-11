#ifndef LIB_PID
#define LIB_PID

#define PID_LIBRARY_VERSION "1.2.1"

#include <math.h>


enum class PidProportionalOption
{
    ON_MEASUREMENT = 0,
    ON_ERROR
};

enum class PidMode
{
    AUTOMATIC = 0,
    MANUAL
};

enum class PidDirection
{
    DIRECT = 0,
    REVERSE
};

class PID
{

public:


  // commonly used functions **************************************************************************
  
  /**
   * @brief Constructs and initializes a PID controller instance.
   *
   * Creates a PID controller that will read the current process value from *Input,
   * compare it to the desired target *Setpoint, and write the computed control value
   * to *Output using the provided tuning parameters and controller options.
   *
   * @param Input  The controller reads the value pointed to by this pointer during compute cycles.
   * @param Output Pointer to the variable where the controller writes the computed output.
   * @param Setpoint Pointer to the target value the controller should achieve.
   * @param Kp Proportional gain.
   * @param Ki Integral gain.
   * @param Kd Derivative gain.
   * @param ProportionalOption Specifies how the proportional term is applied (for
   *                           example, proportional-on-error vs proportional-on-measurement).
   *                           Type: PidProportionalOption.
   * @param ControllerDirection Sets the controller action direction (e.g. DIRECT or REVERSE).
   *                            Type: PidDirection.
   */
  PID(float_t* Input, float_t* Output, float_t* Setpoint,
      float_t Kp, float_t Ki, float_t Kd, 
      PidProportionalOption ProportionalOption, PidDirection ControllerDirection);

  /**
   * @brief  Constructs and initializes a PID controller instance with proportional-on-error option.
   *
   * Creates a PID controller that will read the current process value from *Input,
   * compare it to the desired target *Setpoint, and write the computed control value
   * to *Output using the provided tuning parameters and controller options.
   *
   * @param Input  The controller reads the value pointed to by this pointer during compute cycles.
   * @param Output Pointer to the variable where the controller writes the computed output.
   * @param Setpoint Pointer to the target value the controller should achieve.
   * @param Kp Proportional gain.
   * @param Ki Integral gain.
   * @param Kd Derivative gain.
   * @param ControllerDirection Sets the controller action direction (e.g. DIRECT or REVERSE).
   *                            Type: PidDirection.
   *
   */
  PID(float_t* Input, float_t* Output, float_t* Setpoint,
      float_t Kp, float_t Ki, float_t Kd, PidDirection ControllerDirection);


  /**
   * @brief Sets the mode of operation for the PID controller
   * @param Mode The desired mode of operation (Manual or Auto)
   * 
   * Sets the PID controller to operate in either Manual mode where output is directly
   * set by the user, or Auto mode where output is calculated based on PID algorithm.
   */
  void SetMode(PidMode Mode);


  /**
   * @brief Perform the PID control calculation and update the output.
   *
   * @return true if a new output was computed and applied (controller in
   * automatic mode); false if no calculation was performed 
   * (controller in manual mode).
   */
  bool Compute(void); 


  /**
   * @brief Configure the allowable range for the controller output.
   *
   * Sets the minimum and maximum output values produced by the PID controller.
   * The controller's output is clamped to this range and the internal integral
   * term is adjusted as needed to prevent integral wind-up when the limits change.
   *
   * @param Min The lower bound of the output (inclusive).
   * @param mMx The upper bound of the output (inclusive).
   * 
   * @return true if success (Min < Max)
   */
  bool SetOutputLimits(float_t Min, float_t Max);

  /**
   * @brief Configure the PID controller tuning parameters.
   *
   * Sets the proportional, integral and derivative gains. 
   *
   * @param Kp Proportional gain.
   * @param Ki Integral gain.
   * @param Kd Derivative gain.
   * 
   * @return true if success (parameters needs to be positives)
   */
  bool SetTunings(float_t Kp, float_t Ki, float_t Kd);

  /**
   * @brief Configure the PID controller tuning parameters.
   *
   * Sets the proportional, integral and derivative gains and selects how the
   * proportional action is applied and the controller action direction.
   *
   * @param Kp Proportional gain.
   * @param Ki Integral gain.
   * @param Kd Derivative gain.
   * @param ProportionalOption Option specifying where the proportional term is applied.
   * @param controllerDirection Controller action direction (e.g., DIRECT or REVERSE).
   * 
   * @return true if success (parameters needs to be positives)
   */
  bool SetTunings(float_t Kp, float_t Ki, float_t Kd, 
                  PidProportionalOption ProportionalOption,
                  PidDirection controllerDirection);

  
  /**
   * @brief Set the PID controller's sample time (control loop period).
   *
   * Configure the interval at which the controller's Compute() method is intended
   * to run, expressed in milliseconds. 
   *
   * @param NewSampleTime New sample time in milliseconds.
   * 
   */
  void SetSampleTime(uint32_t NewSampleTime);



private:
  void Initialize();

  void Clamp(float* value);

  float_t m_kp; /** (P)roportional Tuning Parameter */
  float_t m_ki; /** (I)ntegral Tuning Parameter */
  float_t m_kd; /** (D)erivative Tuning Parameter */

  /** The PID will either be connected to a DIRECT acting process (+Output leads
    * to +Input) or a REVERSE acting process(+Output leads to -Input.)  
    */
  PidProportionalOption m_proportionalOption;
  PidMode m_mode;
  PidDirection m_controllerDirection;

  /** Pointers to the Input, Output, and Setpoint variables */
  float_t *m_input;    
  float_t *m_output;   
  float_t *m_setpoint; 

  float_t m_outputSum, m_lastInput;
  float_t m_outMin, m_outMax;
  uint32_t m_sampleTime;
};
#endif /* LIB_PID */
