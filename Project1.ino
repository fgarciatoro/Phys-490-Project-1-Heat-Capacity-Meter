
double IRTempInitial;
double IRTempNow;
double thermTempInitial;
double thermTempNow;
int loopTimer = 0;


int thermistorPin = 23;
double temp = 0;

//int heatingPadPin = 5;

//IR stuff
#include <Wire.h>

#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

const byte MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640

#define TA_SHIFT 8 //Default shift for MLX90640 in open air

static float mlx90640To[768];
paramsMLX90640 mlx90640;
//End of IR stuff

void setup() {
  // put your setup code here, to run once:

  //pinMode(heatingPadPin, OUTPUT);

  pinMode(thermistorPin, OUTPUT);


// initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  //IR stuff
  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  Serial.begin(9600);
  while (!Serial); //Wait for user to open terminal
  Serial.println("MLX90640 IR Array Example");

  if (isConnected() == false)
  {
    Serial.println("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
    while (1);
  }
  Serial.println("MLX90640 online!");

  //Get device parameters - We only have to do this once
  int status;
  uint16_t eeMLX90640[832];
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (status != 0)
    Serial.println("Failed to load system parameters");

  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0)
    Serial.println("Parameter extraction failed");


}

// the loop routine runs over and over again forever:
void loop() {

  loopTimer = loopTimer + 1;

  //Power heating pad:
  //digitalWrite(heatingPadPin, HIGH);

  
  //Thermistor code:
  digitalWrite(thermistorPin, HIGH);
  
  int sensorValue = analogRead(A0);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  double voltage = sensorValue * (3.3 / 1023.0);

  //Serial.println(voltage);
  temp = readThermistor(voltage);

  Serial.print("Thermistor temp: ");
  Serial.println(temp);

  //IR camera code
  for (byte x = 0 ; x < 2 ; x++) //Read both subpages
  {
    uint16_t mlx90640Frame[834];
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    if (status < 0)
    {
      Serial.print("GetFrame Error: ");
      Serial.println(status);
    }

    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
    float emissivity = 0.95;

    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
  }

  for (int x = 0 ; x < 10 ; x++)
  {
    Serial.print("Pixel ");
    Serial.print(x);
    Serial.print(": ");
    Serial.print(mlx90640To[x], 2);
    Serial.print("C");
    Serial.println();
  }
  //End of IR

  
  if(loopTimer == 1){

  double tempTemp = 0;

    for (int x = 0 ; x < 10 ; x++){
      tempTemp = mlx90640To[x] + tempTemp;
    }

    IRTempInitial = tempTemp/10;

    thermTempInitial = temp;

    Serial.print("Initial IR temp: ");
    Serial.println(IRTempInitial);

    Serial.print("Initial Therm temp: ");
    Serial.println(thermTempInitial);
    
  }

  //calculate heat capacity
  if (loopTimer > 1) {

    double tempTemp = 0;

    for (int x = 0 ; x < 10 ; x++){
      tempTemp = mlx90640To[x] + tempTemp;
    }

    IRTempNow = tempTemp/10;
    thermTempNow = temp;
    
    double power = 53.64;
    double timeOn = 10*(loopTimer-1);

    double q = (power * timeOn)/1000;

    double IRTempDelta = (IRTempNow - IRTempInitial);
    double thermTempDelta = (thermTempNow - thermTempInitial);
  
    double deltaT = (thermTempDelta - IRTempDelta);

    double mass = 0.000572;
    
    double heatCapacityIR = q/(IRTempDelta*mass);
    double heatCapacityTherm = q/(thermTempDelta*mass);

    double heatCapacity = q/(deltaT*mass);

    Serial.print("Heat Capacity IR: ");
    Serial.println(heatCapacityIR);

    Serial.print("Heat Capacity Thermistor: ");
    Serial.println(heatCapacityTherm);

    Serial.print("Heat Capacity: ");
    Serial.println(heatCapacity);
  }
  
  delay(10000);
  
}

//Code for converting thermistor readings to temperature
double readThermistor(double voltage) 
{
  // variables
  double rTherm = (9910*voltage)/(3.3-voltage);    //Thermistor resistance value
  double tempKelvin = 0;    // Temperature in Kelvin (calculated)
  double tempCel = 0;    // Temperature in celsius
  double temp = 0;

  //Constants for readThermistor();
  const double a = 0.001125308852122;
  const double b = 0.000234711863267;
  const double c = 0.000000085663516;
  
  double partA = a;

  double partB = b*log(rTherm);

  double partC = c * log(rTherm) * log(rTherm) * log(rTherm);

  temp = partA + partB + partC;

  tempKelvin = 1/temp;
  
  tempCel = tempKelvin - 273;

  return tempCel;
  
  //Serial.println (rTherm);

}

//Returns true if the MLX90640 is detected on the I2C bus
boolean isConnected()
{
  Wire.beginTransmission((uint8_t)MLX90640_address);
  if (Wire.endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}
