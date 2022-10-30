/*----------------------------------------------------------------------------*/
/*                                 Includes                                   */
/*----------------------------------------------------------------------------*/
//#include <Arduino_FreeRTOS.h>

/*----------------------------------------------------------------------------*/
/*                               Local defines                                */
/*----------------------------------------------------------------------------*/
#define ECHO_PIN    (2u)    /* Digital output PIN_2   */
#define TRIGGER_PIN (3u)    /* Digital input PIN_3    */
#define BLUE_PIN    (4u)    /* Digital output PIN_4   */
#define GREEN_PIN   (5u)    /* Digital output PIN_5   */
#define RED_PIN     (6u)    /* Digital output PIN_6   */

#define TEMT_PIN    (A0)    /* Analog input PIN_0    */

#define MAX_DISTANCE (100u)
#define MIN_LIGHT_INTENSITY (50u)
#define NUMBER_OF_SAMPLES_AVERAGE (5u)
/*----------------------------------------------------------------------------*/
/*                              Local data types                              */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                             Global data at RAM                             */
/*----------------------------------------------------------------------------*/
long distance_cm;
long light_intensity;
/*----------------------------------------------------------------------------*/
/*                       Declaration of local functions                       */
/*----------------------------------------------------------------------------*/
/**
 * \brief     This function measures the distance;
 * \param     trigger_Pin: represents the HC-SR04 sensor trigger pin
 * \param     echo_Pin: represents the HC-SR04 sensor echo pin
 * \return    The measured distance in cm 
 */
 int SDTR_MeasureDistance(int trigger_Pin, int echo_Pin);
  /**
 * \brief     This function is used for reading an analog value;
 * \param     input_Pin: represents the pin used for reading the ADC value
 * \return    Light intensity
 */
 long SDTR_FindLightIntensity(int input_Signal_Pin);
  /**
 * \brief     This function is used for turning on/off a led;
 * \param     pin_Led: represents the IO microcontroller pin
 * \param     target_distance: threshold
 * \return    void 
 */
 void SDTR_BlinkLed(int red_pin, int green_pin, int blue_pin, int threshold_distance, int threshold_light_intensity);

/*----------------------------------------------------------------------------*/
/*                     Implementation of global functions                     */
/*----------------------------------------------------------------------------*/
/**
 * \brief     This function represents the init function;
 * \param     None
 * \return    None 
 */
void setup() {
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT); 
  pinMode(TEMT_PIN, INPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);    
  Serial.begin(9600);
}

void loop() {
  distance_cm = SDTR_MeasureDistance(TRIGGER_PIN, ECHO_PIN);
  Serial.print("Distance is: ");
  Serial.println(distance_cm);  
  Serial.println();  
  light_intensity = SDTR_FindLightIntensity(TEMT_PIN);
  Serial.print("Light intensity is: ");
  Serial.println(light_intensity);  
  Serial.println();
  SDTR_BlinkLed(RED_PIN, GREEN_PIN, BLUE_PIN, MAX_DISTANCE, MIN_LIGHT_INTENSITY);
  delay(100);
}

/*----------------------------------------------------------------------------*/
/*                     Implementation of local functions                      */
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/* SDTR_MeasureDistance                                                      */
/*----------------------------------------------------------------------------*/
int SDTR_MeasureDistance(int trigger_Pin, int echo_Pin) 
{
  long pulseWidthInMicro = 0;
  long distance = 0;
  
  digitalWrite(trigger_Pin, HIGH);
  delayMicroseconds(10);  
  digitalWrite(trigger_Pin, LOW);
  pulseWidthInMicro = pulseIn(echo_Pin, HIGH);
  distance = (pulseWidthInMicro*340/10000)/2; //cm
  //distanta = (durata impuls*0,034secunde)/2
  return distance;
}
/*----------------------------------------------------------------------------*/
/* SDTR_MeasureDistance                                                      */
/*----------------------------------------------------------------------------*/
long SDTR_FindLightIntensity(int input_Signal_Pin)
{
  long light_intensity_pin = 0; //pin light intensity
  int light_intensity_percentage = 0;
  long sum = 0;
  int adc_sample = 0;

  for(adc_sample = 0; adc_sample < NUMBER_OF_SAMPLES_AVERAGE; adc_sample++)
  {
    sum = sum + analogRead(input_Signal_Pin);
  }

  light_intensity_pin = sum / NUMBER_OF_SAMPLES_AVERAGE; 
  light_intensity_percentage = light_intensity_pin * 0.0977;

  return light_intensity_percentage;
}
/*----------------------------------------------------------------------------*/
/* SDTR_BlinkLed                                                             */
/*----------------------------------------------------------------------------*/
 void SDTR_BlinkLed(int red_pin, int green_pin, int blue_pin, int threshold_distance, int threshold_light_intensity)
 {
  if((distance_cm > MAX_DISTANCE) && (light_intensity < MIN_LIGHT_INTENSITY)) 
  {
    digitalWrite(RED_PIN, LOW);
    digitalWrite(BLUE_PIN, HIGH);
    digitalWrite(GREEN_PIN, HIGH);
  }
  else 
  {
    digitalWrite(RED_PIN, LOW);
    digitalWrite(BLUE_PIN, LOW);
    digitalWrite(GREEN_PIN, LOW);    
  }
 }
