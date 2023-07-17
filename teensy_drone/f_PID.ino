void set_pid_offsets() {
  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0;
  
  //We need a little dead band of 16us for better results.
  //deadband:
  // a set of values in the domain of a function that returns 0 value
  // deadband here is from 1492 - 1508
 
  if (receiver_input_channel_3 > 1050)  {
    if (receiver_input_channel_1 > 1508)
      pid_roll_setpoint = receiver_input_channel_1 - 1508;
    else if (receiver_input_channel_1 < 1492)
      pid_roll_setpoint = receiver_input_channel_1 - 1492;

    pid_roll_setpoint -= roll_level_adjust; //Subtract the angle correction from the standardized receiver roll input value.
    pid_roll_setpoint /= 3.0;               //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.
  }

  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_3 > 1050)  {
    if (receiver_input_channel_2 > 1508)
      pid_pitch_setpoint = 1508 - receiver_input_channel_2;
    else if (receiver_input_channel_2 < 1492)
      pid_pitch_setpoint = 1492 - receiver_input_channel_2;

    pid_pitch_setpoint -= pitch_level_adjust; //Subtract the angle correction from the standardized receiver pitch input value.
    pid_pitch_setpoint /= 3.0;                //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.
  }

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_3 > 1050)  { //Do not yaw when turning off the motors.
    if (receiver_input_channel_4 > 1508) {
      pid_yaw_setpoint = (receiver_input_channel_4 - 1508) / 3.0;

#ifdef HEADING_HOLD_MODE
      prev_heading = heading;
      heading_hold = false;
#endif
    } 
    
    
    else if (receiver_input_channel_4 < 1492) {                     // if stick is not at centre,
      pid_yaw_setpoint = (receiver_input_channel_4 - 1492) / 3.0;   // new setpoint = new stick location / 3

#ifdef HEADING_HOLD_MODE
      prev_heading = heading;
      heading_hold = false;
#endif
    } 
    
    
    
    else {                                                            //Yaw stick at center --> No transmitter input
#ifdef HEADING_HOLD_MODE
      pid_yaw_setpoint = prev_heading;
      heading_hold = true;
#endif
    }
  }

#ifndef DEBUG_PID_SETPOINT
  Serial.print(pid_roll_setpoint);
  Serial.print(" ");
  Serial.print(pid_pitch_setpoint);
  Serial.print(" ");
  Serial.print(pid_yaw_setpoint);
  Serial.println();
#endif
}

void calculate_pid() {
  //* ========================================= Roll calculations =========================================
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;                                   //calculate error = what you detect - what u want (how much you are deviating by)
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;                                     //accumulated (integral) memory = error * i-gain
  if (pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;                        //if accumulated error > maximum error, set the maximum error as the accumulated error
  else if (pid_i_mem_roll < pid_max_roll * -1) pid_i_mem_roll = pid_max_roll * -1;        //do the same as ^ for the opposite direction

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  
  //PID output for roll = p-gain * error + accumulated error + d-gain * (current error - previous error) [change in error]
  
  if (pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if (pid_output_roll < pid_max_roll * -1) pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;           //set current error as previous error

  //* ========================================= Pitch calculations =========================================
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;                                  //calculate error = what u detect - what u want (deviation from setpoint)
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;                                    //accumulated error = error * i-gain
  if (pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;                     //check if accumulated error is too big or too small
  else if (pid_i_mem_pitch < pid_max_pitch * -1) pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);

  //PID output for pitch = p-gain * error + accumulated error + d-gain * change in error
  
  if (pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1) pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;        //set current error as previous error

  //* ========================================= Yaw calculations =========================================
#ifdef HEADING_HOLD_MODE
//if HEADING_HOLD_MODE is defined, execute loop
  if (heading_hold) {
    float diff = heading - pid_yaw_setpoint;                                //error (diff) = current heading - heading that you want

    //use this error ^ to calculate if positive value obtained
    
    if (diff > 180) pid_yaw_setpoint += 360;                                //if error>180, means drone is facing opposite direction, adjust yaw setpoint
    else if (diff < -180) pid_yaw_setpoint -= 360;                          //so that values outputted is positive
    pid_yaw_error_temp = pid_yaw_setpoint - heading;                            //error = how much u are deviating from setpoint
    if (abs(pid_yaw_error_temp) > 2) pid_yaw_error_temp *= heading_hold_gain;       //if error is too big, multiply by 3?
    heading_hold = false;                                                   //switch it off
  } else {
    pid_yaw_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  }
#endif
#ifndef HEADING_HOLD_MODE
  pid_yaw_error_temp = gyro_yaw_input - pid_yaw_setpoint;
#endif

  pid_i_mem_yaw += pid_i_gain_yaw * pid_yaw_error_temp;                        //accumulated error = i-gain * error
  if (pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;             //adjust accordingly if error is too big or too small
  else if (pid_i_mem_yaw < pid_max_yaw * -1) pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_yaw_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_yaw_error_temp - pid_last_yaw_d_error);

  //PID output for yaw = p-gain * error + accumulated error + d-gain * (change in error)
  
  if (pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;            //if output is too huge, use maximum output
  else if (pid_output_yaw < pid_max_yaw * -1) pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_yaw_error_temp;                                    //set current error as previous error

#ifndef DEBUG_PID_OUTPUT
  Serial.print(" ");
  Serial.print(pid_p_gain_roll);
  Serial.print(", ");
  Serial.print(pid_d_gain_roll);
  Serial.print(", ");
  Serial.print(pid_p_gain_pitch);
  Serial.print(", ");
  Serial.print(pid_d_gain_pitch);
  Serial.println();
/*  Serial.print(" ");
  Serial.print(pid_output_roll);
  Serial.print(", ");
  Serial.print(pid_output_pitch);
  Serial.print(", ");
  Serial.print(pid_output_yaw);
  Serial.println();*/
#endif
}
