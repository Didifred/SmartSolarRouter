/**
 * @file libPID.cpp
 * @author Freddy Boutin (https://github.com/Didifred/)
 *         Original version by: Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 **/
#include <stdint.h>
#include <math.h>
#include "libPID.h"
#include "Logger.h"


PID::PID(float_t Kp, float_t Ki, float_t Kd,
         PidProportionalOption ProportionalOption,
         PidDirection ControllerDirection)
{
    m_lastInput = 0.0f;
    m_outputSum = 0.0f;
    m_lastInput = 0.0f;

    // default output limit corresponds to the arduino pwm limits
    PID::SetOutputLimits(0, 255);				
								
    //default Controller Sample Time is 0.1 seconds
    m_sampleTime = 100;
    						
    PID::SetTunings(Kp, Ki, Kd, ProportionalOption, ControllerDirection);
}

PID::PID(float_t Kp, float_t Ki, float_t Kd, PidDirection ControllerDirection)
    :PID::PID( Kp, Ki, Kd, PidProportionalOption::ON_ERROR, ControllerDirection)
{

}

float_t PID::Compute(float_t Input, float_t Setpoint)
{
   float_t error = (Setpoint - Input);
   float_t dInput = (Input - m_lastInput);
   float_t output = 0.0f;
   float_t saturationError = 0.0;

   // Integral term (accumulate error over time)
   m_outputSum += (m_ki * error);

   // Proportional term
   if (m_proportionalOption == PidProportionalOption::ON_ERROR)
   {
      output = (m_kp * error);
   }
   else 
   {
      // PidProportionalOption::ON_MEASUREMENT
      m_outputSum = -(m_kp * dInput);
   }
   // Calculate total output (P + I + D)
   // Note: to avoid derivative kick on setpoint change, derivative is done on measurement
   output += m_outputSum - (m_kd * dInput);
   saturationError = Clamp(&output, m_outMin, m_outMax);

   // Back-calculation : compensate  integral due to output saturation
   m_outputSum -= (m_kt * saturationError);
   Clamp(&m_outputSum, m_outMin, m_outMax);
   Logger::plot("Dimmer.outputSum", String(m_outputSum), "W");
   
   // Remember some variables for next time
   m_lastInput = Input;

   return (output);
}

bool PID::SetTunings(float_t Kp, float_t Ki, float_t Kd, 
                     PidProportionalOption ProportionalOption,
                     PidDirection ControllerDirection)
{
   bool success = false;

   if (Kp>=0.0f && Ki>=0.0f && Kd>=0.0f)
   {
      m_userKp = Kp;
      m_userKi = Ki; 
      m_userKd = Kd;
      
      m_proportionalOption = ProportionalOption;
      m_controllerDirection = ControllerDirection;

      float_t SampleTimeInSec = ((float_t)m_sampleTime)/1000;
      m_kp = Kp;
      m_ki = Ki * SampleTimeInSec;
      m_kd = Kd / SampleTimeInSec;

      if (m_kp > 0.0f)
      {
         // Anti-windup standard tuning
         m_kt = m_ki / Kp; 
      }
      else
      {
         m_kt = 1.0f;
      }
      Logger::log(LogLevel::DEBUG, "PID: SetTunings Kp=%.3f Ki=%.3f Kd=%.3f Kt=%.3f",
                  Kp, Ki, Kd, m_kt);

      if(ControllerDirection == PidDirection::REVERSE)
      {
         m_kp = -m_kp;
         m_ki = -m_ki;
         m_kd = -m_kd;
      }

      success = true;
   }

   return success;
}


bool PID::SetTunings(float_t Kp, float_t Ki, float_t Kd)
{
    return (SetTunings(Kp, Ki, Kd, m_proportionalOption,  m_controllerDirection)); 
}

float_t PID::GetKp() const
{
   return m_userKp;
}

float_t PID::GetKi() const
{
   return m_userKi;
}

float_t PID::GetKd() const
{
   return m_userKd;
}  


void PID::SetSampleTime(uint32_t NewSampleTime)
{
   float_t ratio  = (float_t)NewSampleTime / m_sampleTime;
   
   m_ki *= ratio;
   m_kd /= ratio;

   m_sampleTime = NewSampleTime;
}

uint32_t PID::GetSampleTime() const
{
   return m_sampleTime;
}

bool PID::SetOutputLimits(float_t Min, float_t Max)
{
   bool success = false;

   if (Min < Max)
   {
      m_outMin = Min;
      m_outMax = Max;

      success = true;
   }

   return (success);
}

void PID::Initialize(float_t Input, float_t Output)
{
   m_lastInput = Input;

   Clamp(&Output, m_outMin, m_outMax);
   m_outputSum = Output;
}

/**
 * @brief Clamp the parameter between min and max
 * 
 * @param Value 
 */
float_t PID::Clamp(float_t* Value, float_t Min, float_t Max)
{
   float_t saturationError = 0.0f;

   if (*Value > Max)
   {
      saturationError = *Value - Max;
      *Value = Max;
   }
   else if (*Value < Min) 
   {
      saturationError = *Value - Min;
      *Value = Min;
   }

   return saturationError;
}



