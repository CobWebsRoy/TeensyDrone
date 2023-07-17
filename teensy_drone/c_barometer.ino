//BAROMETER VARIABLES
int starting_pressure=0, pressurediff=0, isitstarting=0;
double heightchange=0;



//Pressure variables.
float pid_error_gain_altitude, pid_throttle_gain_altitude;
uint16_t C[7];
uint8_t barometer_counter, temperature_counter, average_temperature_mem_location;
int64_t OFF, OFF_C2, SENS, SENS_C1, P;
uint32_t raw_pressure, raw_temperature, temp, raw_temperature_rotating_memory[6], raw_average_temperature_total;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
float ground_pressure, altutude_hold_pressure;
int32_t dT, dT_C5;



//Altitude PID variables
float pid_i_mem_altitude, pid_altitude_setpoint, pid_altitude_input, pid_output_altitude, pid_last_altitude_d_error;

uint8_t parachute_rotating_mem_location;
int32_t parachute_buffer[35], parachute_throttle;
float pressure_parachute_previous;

int32_t pressure_rotating_mem[50], pressure_total_avarage;
uint8_t pressure_rotating_mem_location;
float pressure_rotating_mem_actual;
uint32_t loop_timer;
float top_line, bottom_line;

uint8_t MS5611_address = 0x77;



void setup_barometer(){
for (start = 1; start <= 6; start++) {
      Wire.beginTransmission(MS5611_address);                    //Start communication with the MPU-6050.
      Wire.write(0xA0 + start * 2);                              //Send the address that we want to read.
      Wire.endTransmission();                                    //End the transmission.

      Wire.requestFrom(MS5611_address, 2);                       //Request 2 bytes from the MS5611.
      C[start] = Wire.read() << 8 | Wire.read();                //Add the low and high byte to the C[x] calibration variable.
    }
    start = 0;
    //Print the 6 calibration values on the screen.
    Serial.print("C1 = ");
    Serial.println(C[1]);
    Serial.print("C2 = ");
    Serial.println(C[2]);
    Serial.print("C3 = ");
    Serial.println(C[3]);
    Serial.print("C4 = ");
    Serial.println(C[4]);
    Serial.print("C5 = ");
    Serial.println(C[5]);
    Serial.print("C6 = ");
    Serial.println(C[6]);

    OFF_C2 = C[2] * pow(2, 16);                                   //This value is pre-calculated to offload the main program loop.
    SENS_C1 = C[1] * pow(2, 15);                                  //This value is pre-calculated to offload the main program loop.
  }


void read_barometer(){
  barometer_counter ++;

  if (barometer_counter == 1) {
    if (temperature_counter == 0) {
      //Get temperature data from MS-5611
      Wire.beginTransmission(MS5611_address);
      Wire.write(0x00);
      Wire.endTransmission();
      Wire.requestFrom(MS5611_address, 3);

      raw_average_temperature_total -= raw_temperature_rotating_memory[average_temperature_mem_location];
      raw_temperature_rotating_memory[average_temperature_mem_location] = Wire.read() << 16 | Wire.read() << 8 | Wire.read();
      raw_average_temperature_total += raw_temperature_rotating_memory[average_temperature_mem_location];
      average_temperature_mem_location++;
      if (average_temperature_mem_location == 5)average_temperature_mem_location = 0;
      raw_temperature = raw_average_temperature_total / 5;
    }
    else {
      //Get pressure data from MS-5611
      Wire.beginTransmission(MS5611_address);
      Wire.write(0x00);
      Wire.endTransmission();
      Wire.requestFrom(MS5611_address, 3);
      raw_pressure = Wire.read() << 16 | Wire.read() << 8 | Wire.read();
    }

    temperature_counter ++;
    if (temperature_counter == 20) {
      temperature_counter = 0;
      //Request temperature data
      Wire.beginTransmission(MS5611_address);
      Wire.write(0x58);
      Wire.endTransmission();
    }
    else {
      //Request pressure data
      Wire.beginTransmission(MS5611_address);
      Wire.write(0x48);
      Wire.endTransmission();
    }
  }
  if (barometer_counter == 2) {
    //Calculate pressure as explained in the datasheet of the MS-5611.
    dT = C[5];
    dT <<= 8;
    dT *= -1;
    dT += raw_temperature;
    OFF = OFF_C2 + ((int64_t)dT * (int64_t)C[4]) / pow(2, 7);
    SENS = SENS_C1 + ((int64_t)dT * (int64_t)C[3]) / pow(2, 8);
    P = ((raw_pressure * SENS) / pow(2, 21) - OFF) / pow(2, 15);

    //Let's use a rotating mem
    pressure_total_avarage -= pressure_rotating_mem[pressure_rotating_mem_location];    //Subtract the current memory position to make room for the new value.
    pressure_rotating_mem[pressure_rotating_mem_location] = P;                          //Calculate the new change between the actual pressure and the previous measurement.
    pressure_total_avarage += pressure_rotating_mem[pressure_rotating_mem_location];    //Add the new value to the long term avarage value.
    pressure_rotating_mem_location++;                                                   //Increase the rotating memory location.
    if (pressure_rotating_mem_location == 20)pressure_rotating_mem_location = 0;        //Start at 0 when the memory location 20 is reached.
    actual_pressure_fast = (float)pressure_total_avarage / 20.0;                        //Calculate the avarage pressure value of the last 20 readings.
    if(millis() < 5000)actual_pressure_slow = actual_pressure_fast;                     //Keep the slow and fast avareges the same for the first 5 seconds.

    actual_pressure_slow = actual_pressure_slow * (float)0.985 + actual_pressure_fast * (float)0.015;
    
    actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;
    if (actual_pressure_diff > 8)actual_pressure_diff = 8;
    if (actual_pressure_diff < -8)actual_pressure_diff = -8;
    if (actual_pressure_diff > 1 || actual_pressure_diff < -1)actual_pressure_slow -= actual_pressure_diff / 6.0;
    actual_pressure = actual_pressure_slow;
  }

  if (barometer_counter == 3) {    
    
    if (isitstarting < 200 ){

      isitstarting++;
      barometer_counter = 0;
    }

    else if (isitstarting > 199 && isitstarting < 300){
      starting_pressure += actual_pressure;
      isitstarting++;
      barometer_counter = 0;
    }
    
    else if (isitstarting == 300){
      starting_pressure = starting_pressure/100;
      Serial.print("Starting Pressure: ");
      Serial.println(starting_pressure);
      isitstarting++;
      barometer_counter = 0;
    }
    
//    When the barometer counter is 3
    else if (isitstarting == 301){
    Serial.print("Pressure: ");
    Serial.print(actual_pressure);
    Serial.print(",");
    pressurediff = actual_pressure - starting_pressure;
    Serial.print("Change in height: ");
    //1 metre change in height = 11.7 pascal change in pressure
    heightchange = pressurediff/11.7;
    Serial.println(-1*heightchange);
    pressurediff = 0;
    heightchange = 0;
    barometer_counter = 0;     
    }                                                              //Set the barometer counter to 0 for the next measurements
  }
  if (millis() > 5000 && start == 0) {
    start = 1;
    top_line = actual_pressure + 20;
    bottom_line = actual_pressure - 20;
  }

  while (loop_timer > micros());
  loop_timer = micros() + 4000;
}
