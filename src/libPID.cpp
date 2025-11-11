/**********************************************************************************************
 * Arduino PID Library - Version 1.2.1
 * original version by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 * Code re-writen by Didifred (https://github.com/Didifred)
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#include <stdint.h>
#include <math.h>
#include "libPID.h"


PID::PID(float_t* Input, float_t* Output, float_t* Setpoint,
         float_t Kp, float_t Ki, float_t Kd, 
         PidProportionalOption ProportionalOption, PidDirection ControllerDirection)
{
    m_output = Output;
    m_input = Input;
    m_setpoint = Setpoint;

    m_mode = PidMode::AUTOMATIC;
    m_lastInput = 0.0f;

    //default output limit corresponds to the arduino pwm limits
    PID::SetOutputLimits(0, 255);				
								
    //default Controller Sample Time is 0.1 seconds
    m_sampleTime = 100;
    						
    PID::SetTunings(Kp, Ki, Kd, ProportionalOption, ControllerDirection);
}


PID::PID(float_t* Input, float_t* Output, float_t* Setpoint,
        float_t Kp, float_t Ki, float_t Kd, PidDirection ControllerDirection)
    :PID::PID(Input, Output, Setpoint, Kp, Ki, Kd, PidProportionalOption::ON_ERROR, ControllerDirection)
{

}

bool PID::Compute(void)
{
   if (m_mode == PidMode::AUTOMATIC)
   {
      // Compute all the working error variables
      float_t input = *m_input;
      float_t error = *m_setpoint - input;
      float_t dInput = (input - m_lastInput);
      float_t output = 0.0f;
      
      m_outputSum += (m_ki * error);

      // Add Proportional on Measurement, if PidProportionalOption::ON_MEASUREMENT is specified
      if (m_proportionalOption == PidProportionalOption::ON_MEASUREMENT)
      {
         m_outputSum -= m_kp * dInput;
      }

      Clamp(&m_outputSum);

      // Add Proportional on Error, if PidProportionalOption::ON_ERROR is specified
      if (m_proportionalOption == PidProportionalOption::ON_ERROR)
      {
         output = m_kp * error;
      }

      // Compute Rest of PID Output
      output += m_outputSum - m_kd * dInput;

      Clamp(&output);
	   
      *m_output = output;

      // Remember some variables for next time
      m_lastInput = input;
   }
	 
   return (m_mode == PidMode::AUTOMATIC);
}

bool PID::SetTunings(float_t Kp, float_t Ki, float_t Kd, 
                     PidProportionalOption ProportionalOption,
                     PidDirection ControllerDirection)
{
   bool success = false;

   if (Kp>=0 && Ki>=0 && Kd>=0)
   {
      m_proportionalOption = ProportionalOption;
      m_controllerDirection = ControllerDirection;

      float_t SampleTimeInSec = ((float_t)m_sampleTime)/1000;
      m_kp = Kp;
      m_ki = Ki * SampleTimeInSec;
      m_kd = Kd / SampleTimeInSec;

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
    return (SetTunings(Kp, Ki, Kd, m_proportionalOption, m_controllerDirection)); 
}


void PID::SetSampleTime(uint32_t NewSampleTime)
{

   float_t ratio  = (float_t)NewSampleTime / m_sampleTime;
   m_ki *= ratio;
   m_kd /= ratio;

   m_sampleTime = NewSampleTime;
}

bool PID::SetOutputLimits(float_t Min, float_t Max)
{
   bool success = false;

   if (Min < Max)
   {
      m_outMin = Min;
      m_outMax = Max;

      if(m_mode == PidMode::AUTOMATIC)
      {
         Clamp(m_output);
         Clamp(&m_outputSum);
      }

      success = true;
   }

   return (success);
}


void PID::SetMode(PidMode Mode)
{
    if ((Mode == PidMode::AUTOMATIC) && (m_mode == PidMode::MANUAL))
    {
      // we just went from manual to auto
      PID::Initialize();
    }
    m_mode = Mode;
}



/**
 * @brief Initialize internal state of the PID controller.
 *
 * This method seeds the integral accumulator and the last-input value from the
 * current controller I/O values, and enforces the configured output limits to
 * prevent integral windup on startup.
 */
void PID::Initialize()
{
   m_outputSum = *m_output;
   m_lastInput = *m_input;

   Clamp(&m_outputSum);
}

/**
 * @brief Clamp the parameter between min and max
 * 
 * @param Value 
 */
void PID::Clamp(float* Value)
{
   if (*Value > m_outMax)
   {
      *Value = m_outMax;
   }
   else if(*Value < m_outMin) 
   {
      *Value = m_outMin;
   }
}



