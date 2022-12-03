/*----------------------------------------------------------------------------*/
/*                                 Includes                                   */
/*----------------------------------------------------------------------------*/
#include <Arduino_FreeRTOS.h>

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
long light_intensity_percentage;
bool system_controlled_led_status = false;
bool cloud_controlled_les_status = false;

/*----------------------------------------------------------------------------*/
/*                       Declaration of local functions                       */
/*----------------------------------------------------------------------------*/
/**
 * \brief     This function measures the distance;
 * \param     trigger_Pin: represents the HC-SR04 sensor trigger pin
 * \param     echo_Pin: represents the HC-SR04 sensor echo pin
 * \return    The measured distance in cm 
 */
void SDTR_MeasureDistance(void *pvParameters);
  /**
 * \brief     This function is used for reading an analog value;
 * \param     input_Pin: represents the pin used for reading the ADC value
 * \return    Light intensity
 */
void SDTR_FindLightIntensity(void *pvParameters);
  /**
 * \brief     This function is used for turning on/off a led;
 * \param     pin_Led: represents the IO microcontroller pin
 * \param     target_distance: threshold
 * \return    void 
 */
 void SDTR_BlinkLed(void *pvParameters);
 void SDTR_GetDistance();
 void SDTR_GetIntenisty();
void SDTR_Blink();

/*----------------------------------------------------------------------------*/
/*                     Implementation of global functions                     */
/*----------------------------------------------------------------------------*/
/**
 * \brief     This function represents the init function;
 * \param     None
 * \return    None 
 */
void setup() {

  // Initialize serial and wait for port to open:
  Serial.begin(9600);

  //Pin settings 
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT); 
  pinMode(TEMT_PIN, INPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);    

  // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
  vTaskDelay(1500); 

  //Task creation
 /* xTaskCreate(
    SDTR_UpdateCloud,            // Task function
    "SDTR_UpdateCloud",          // Name 
    10000,                       // Stack size of the task
    NULL,                        // Parameters
    1,                           // Priority
    NULL);                       // Task handle
  */
  xTaskCreate(
    SDTR_MeasureDistance,        // Task function
    "SDTR_MeasureDistance",      // Name 
    10000,                       // Stack size of the task
    NULL,                        // Parameters
    1,                           // Priority
    NULL);                       // Task handle
    
  xTaskCreate(
    SDTR_FindLightIntensity,        // Task function
    "SDTR_FindLightIntensity",      // Name 
    10000,                       // Stack size of the task
    NULL,                        // Parameters
    1,                           // Priority
    NULL);                       // Task handle

  xTaskCreate(
    SDTR_BlinkLed,        // Task function
    "SDTR_BlinkLed",      // Name 
    10000,                       // Stack size of the task
    NULL,                        // Parameters
    1,                           // Priority
    NULL);                       // Task handle
}

void loop() {
  //empty
}

/*----------------------------------------------------------------------------*/
/*                     Implementation of local functions                      */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* SDTR_MeasureDistance                                                      */
/*----------------------------------------------------------------------------*/

void SDTR_MeasureDistance(void *pvParameters) {
  (void) pvParameters;

  // A task shall never return or exit
  for(;;)

  Serial.println("READING DISTANCE SENSOR VALUE");
  SDTR_GetDistance();
}

/*----------------------------------------------------------------------------*/
/* SDTR_GetDistance                                                     */
/*----------------------------------------------------------------------------*/
void SDTR_GetDistance() {
  long pulseWidthInMicro = 0;
  
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);  
  digitalWrite(TRIGGER_PIN, LOW);
  pulseWidthInMicro = pulseIn(ECHO_PIN, HIGH);
  distance_cm = (pulseWidthInMicro*340/10000)/2; //cm
  //distanta = (durata impuls*0,034secunde)/2
  Serial.print("Distance: ");
  Serial.print(distance_cm);
  Serial.print(" cm");
}

/*----------------------------------------------------------------------------*/
/* SDTR_FindLightIntensity                                                     */
/*----------------------------------------------------------------------------*/
void SDTR_FindLightIntensity(void *pvParameters) {
  (void) pvParameters;

    // A task shall never return or exit
  for(;;)

  Serial.println("READING LIGHT INTENSITY SENSOR VALUE");
  SDTR_GetIntensity();
}

/*----------------------------------------------------------------------------*/
/* SDTR_GetIntensity                                                     */
/*----------------------------------------------------------------------------*/
void SDTR_GetIntensity() {
  long light_intensity_pin = 0; //pin light intensity
  long sum = 0;
  int adc_sample = 0;

  for(adc_sample = 0; adc_sample < NUMBER_OF_SAMPLES_AVERAGE; adc_sample++)
  {
    sum = sum + analogRead(TEMT_PIN);
  }

  light_intensity_pin = sum / NUMBER_OF_SAMPLES_AVERAGE; 
  light_intensity_percentage = light_intensity_pin * 0.0977;
}

/*----------------------------------------------------------------------------*/
/* SDTR_BlinkLed                                                    */
/*----------------------------------------------------------------------------*/
void SDTR_BlinkLed(void *pvParameters) {
  (void) pvParameters;

  // A task shall never return or exit
  for(;;)

  SDTR_Blink();
}

/*----------------------------------------------------------------------------*/
/* SDTR_Blink                                                 */
/*----------------------------------------------------------------------------*/
void SDTR_Blink() {
  if((distance_cm > MAX_DISTANCE) && (light_intensity_percentage < MIN_LIGHT_INTENSITY)) 
  {
    system_controlled_led_status = true;
    digitalWrite(RED_PIN, LOW);
    digitalWrite(BLUE_PIN, HIGH);
    digitalWrite(GREEN_PIN, HIGH);
  }
  else 
  {
    system_controlled_led_status = false;
    digitalWrite(RED_PIN, LOW);
    digitalWrite(BLUE_PIN, LOW);
    digitalWrite(GREEN_PIN, LOW);    
  }
  vTaskDelay(20);
}
