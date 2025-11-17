#ifndef LIB_PID
#define LIB_PID

#define PID_LIBRARY_VERSION "1.2.1"

#include <math.h>


enum class PidProportionalOption
{
    ON_MEASUREMENT = 0,
    ON_ERROR
};

enum class PidDirection
{
    DIRECT = 0,
    REVERSE
};

class PID
{

public:

  /**
   * @brief Constructs and initializes a PID controller instance.
   *
   * Creates a PID controller using the provided tuning parameters and controller options.
   *
   * @param Kp Proportional gain.
   * @param Ki Integral gain.
   * @param Kd Derivative gain.
   * @param ProportionalOption Specifies how the proportional term is applied (for
   *                           example, proportional-on-error vs proportional-on-measurement).
   *                           Type: PidProportionalOption.
   * @param ControllerDirection Sets the controller action direction (e.g. DIRECT or REVERSE).
   *                            Type: PidDirection.
   */
  PID(float_t Kp, float_t Ki, float_t Kd, 
      PidProportionalOption ProportionalOption, PidDirection ControllerDirection);

  /**
   * @brief  Constructs and initializes a PID controller instance with proportional-on-error option.
   *
   * Creates a PID controller using the provided tuning parameters and controller options.
   *
   * @param Kp Proportional gain.
   * @param Ki Integral gain.
   * @param Kd Derivative gain.
   * @param ControllerDirection Sets the controller action direction (e.g. DIRECT or REVERSE).
   *                            Type: PidDirection.
   *
   */
  PID(float_t Kp, float_t Ki, float_t Kd, PidDirection ControllerDirection);


  /**
   * @brief Perform the PID control calculation and update the output.
   * 
   * @param Input  New input value for the PID controller.
   * @param Setpoint The target value the PID controller should achieve.
   * 
   * @return PID filtered output
   */
  float_t Compute(float_t Input, float_t Setpoint); 


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

  /**
   * @brief Initialize internal state of the PID controller.
   * @param Input Latest input
   * @param Output Latest output (forced)
   * 
   * This method seeds the integral accumulator and the last-input value from the
   * current Input, and enforces the configured output limits to prevent integral windup on startup.
   */
  void Initialize(float_t Input, float_t Output);


private:

  void Clamp(float* value);

  float_t m_kp; /** (P)roportional Tuning Parameter */
  float_t m_ki; /** (I)ntegral Tuning Parameter */
  float_t m_kd; /** (D)erivative Tuning Parameter */

  /**  Either apply proportional on error or on measurement */
  PidProportionalOption m_proportionalOption;

  /** The PID will either be connected to a DIRECT acting process (+Output leads
    * to +Input) or a REVERSE acting process(+Output leads to -Input.)  
    */
  PidDirection m_controllerDirection; 

  /** Internal variables of the PID controller */
  float_t m_outputSum;
  float_t m_lastInput;

  /** In order to clamp output */
  float_t m_outMin, m_outMax;

  /* Sample time in milliseconds*/
  uint32_t m_sampleTime;
};
#endif /* LIB_PID */
