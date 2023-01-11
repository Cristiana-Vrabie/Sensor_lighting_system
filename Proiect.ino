#include "arduino_secrets.h"
/*----------------------------------------------------------------------------*/
/*                                 Includes                                   */
/*----------------------------------------------------------------------------*/
#include "thingProperties.h"
/*----------------------------------------------------------------------------*/
/*                               Local defines                                */
/*----------------------------------------------------------------------------*/
#define ECHO_PIN    (21u)    /* Digital output PIN_21   */
#define TRIGGER_PIN (22u)    /* Digital input PIN_22    */
#define BLUE_PIN    (12u)    /* Digital output PIN_12   */


#define TEMT_PIN    (36u)    /* Analog input PIN_36    */

#define MAX_DISTANCE (100u)
#define MIN_LIGHT_INTENSITY (50u)
#define NUMBER_OF_SAMPLES_AVERAGE (5u)
/*----------------------------------------------------------------------------*/
/*                             Global data at RAM                             */
/*----------------------------------------------------------------------------*/
bool user_cntrl = false;
bool led_on = false;

xSemaphoreHandle xSemaphore;
/*----------------------------------------------------------------------------*/
/*                       Declaration of local functions                       */
/*----------------------------------------------------------------------------*/

void SDTR_UpdateCloud(void *pvParameters);

void SDTR_MeasureDistance(void *pvParameters);

void SDTR_MeasureLightIntensity(void *pvParameters);

void SDTR_BlinkLed(void *pvParameters);

/*----------------------------------------------------------------------------*/
/*                     Implementation of global functions                     */
/*----------------------------------------------------------------------------*/
void setup() {
  // Initialize serial and wait for port to open:
  Serial.begin(9600);

  // Defined in thingProperties.h
  initProperties();

  xSemaphore = xSemaphoreCreateBinary();

  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);

  //Pin settings
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TEMT_PIN, INPUT);
  pinMode(BLUE_PIN, OUTPUT);

  //Task creation
  xTaskCreatePinnedToCore(
    SDTR_UpdateCloud,            // Task function
    "SDTR_UpdateCloud",          // Name
    1024 * 7,                       // Stack size of the task
    NULL,                        // Parameters
    1,                           // Priority
    NULL,                       // Task handle
    0);                         // Core

  xTaskCreatePinnedToCore(
    SDTR_MeasureDistance,        // Task function
    "SDTR_MeasureDistance",      // Name
    1024 * 7,                       // Stack size of the task
    NULL,                        // Parameters
    1,                           // Priority
    NULL,                       // Task handle
    0);                         //Core

  xTaskCreatePinnedToCore(
    SDTR_MeasureLightIntensity,        // Task function
    "SDTR_MeasureLightIntensity",      // Name
    1024 * 7,                       // Stack size of the task
    NULL,                        // Parameters
    1,                           // Priority
    NULL,                       // Task handle
    0);                         //Core

  xTaskCreatePinnedToCore(
    SDTR_BlinkLed,        // Task function
    "SDTR_BlinkLed",      // Name
    1024 * 7,                       // Stack size of the task
    NULL,                        // Parameters
    1,                           // Priority
    NULL,                       // Task handle
    0);                         //Core


  xSemaphoreGive(xSemaphore);
}

void loop() {
  //
  // Your code here


}



/*----------------------------------------------------------------------------*/
/*                     Implementation of local functions                      */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* SDTR_UpdateCloud                                                      */
/*----------------------------------------------------------------------------*/

void SDTR_UpdateCloud(void *pvParameters) {
  (void) pvParameters;
  // A task shall never return or exit
  for (;;) {
    xSemaphoreTake(xSemaphore, portMAX_DELAY);
    Serial.println("Start WEB service...");
    ArduinoCloud.update();
    xSemaphoreGive(xSemaphore);
    vTaskDelay(100 / portTICK_PERIOD_MS); //delay of 100ms
  }
}


/*----------------------------------------------------------------------------*/
/* SDTR_MeasureDistance                                                      */
/*----------------------------------------------------------------------------*/

void SDTR_MeasureDistance(void *pvParameters) {
  (void) pvParameters;
  // A task shall never return or exit
  for (;;) {
    SDTR_GetDistance();
    vTaskDelay(100 / portTICK_PERIOD_MS); //delay of 100ms
  }
}

/*----------------------------------------------------------------------------*/
/* SDTR_GetDistance                                                     */
/*----------------------------------------------------------------------------*/
void SDTR_GetDistance() {
  long pulseWidthInMicro = 0;
  int measured_distance = 0;

  digitalWrite(TRIGGER_PIN, HIGH);
  //10 us wait = 0.01ms/portTICK_PERIOD_MS
  vTaskDelay(0.01 / portTICK_PERIOD_MS);
  digitalWrite(TRIGGER_PIN, LOW);
  pulseWidthInMicro = pulseIn(ECHO_PIN, HIGH);
  measured_distance = (pulseWidthInMicro * 340 / 10000) / 2; //cm
  //distanta = (durata impuls*0,034secunde)/2
  xSemaphoreTake(xSemaphore, portMAX_DELAY);
  distance_cm = measured_distance;
  Serial.print("Distance: ");
  Serial.print(distance_cm);
  Serial.print(" cm");
  Serial.println();
  xSemaphoreGive(xSemaphore);
}

/*----------------------------------------------------------------------------*/
/* SDTR_MeasureLightIntensity                                                     */
/*----------------------------------------------------------------------------*/
void SDTR_MeasureLightIntensity(void *pvParameters) {
  (void) pvParameters;

  // A task shall never return or exit
  for (;;) {
    SDTR_GetIntensity();
    vTaskDelay(100 / portTICK_PERIOD_MS ); //delay of 100ms
  }
}

/*----------------------------------------------------------------------------*/
/* SDTR_GetIntensity                                                     */
/*----------------------------------------------------------------------------*/
void SDTR_GetIntensity() {
  long light_intensity_pin = 0; //pin light intensity
  long sum = 0;
  int adc_sample = 0;
  int measured_intensity;

  for (adc_sample = 0; adc_sample < NUMBER_OF_SAMPLES_AVERAGE; adc_sample++)
  {
    sum = sum + analogRead(TEMT_PIN);
  }

  light_intensity_pin = sum / NUMBER_OF_SAMPLES_AVERAGE;
  measured_intensity = (light_intensity_pin * 100) / 4096;
  xSemaphoreTake(xSemaphore, portMAX_DELAY);
  light_intensity = measured_intensity;
  Serial.print("Light intensity: ");
  Serial.print(light_intensity);
  Serial.print("%");
  Serial.println();
  xSemaphoreGive(xSemaphore);
}

/*----------------------------------------------------------------------------*/
/* SDTR_BlinkLed                                                    */
/*----------------------------------------------------------------------------*/
void SDTR_BlinkLed(void *pvParameters) {
  (void) pvParameters;

  // A task shall never return or exit
  for (;;) {
    SDTR_Blink();
    vTaskDelay(100 / portTICK_PERIOD_MS ); //delay of 100ms
  }
}

/*----------------------------------------------------------------------------*/
/* SDTR_Blink                                                 */
/*----------------------------------------------------------------------------*/
void SDTR_Blink() {
  xSemaphoreTake(xSemaphore, portMAX_DELAY);
  if (user_cntrl == false)
  {
    if ((distance_cm > MAX_DISTANCE) && (light_intensity < MIN_LIGHT_INTENSITY))
    {
      led_status = true;
      Serial.println("Turning on led... ");
      // digitalWrite(RED_PIN, LOW);
      digitalWrite(BLUE_PIN, HIGH);
      //digitalWrite(GREEN_PIN, HIGH);
      xSemaphoreGive(xSemaphore);
    }
    else
    {
      led_status = false;
      Serial.println("Turning off led...");
      // digitalWrite(RED_PIN, LOW);
      digitalWrite(BLUE_PIN, LOW);
      // digitalWrite(GREEN_PIN, LOW);
      xSemaphoreGive(xSemaphore);
    }
  }
  else
  {
    if (led_on == true)
    {
      // digitalWrite(RED_PIN, LOW);
      digitalWrite(BLUE_PIN, HIGH);
      //  digitalWrite(GREEN_PIN, HIGH);

    }
    else
    {
      //  digitalWrite(RED_PIN, LOW);
      digitalWrite(BLUE_PIN, LOW);
      //  digitalWrite(GREEN_PIN, LOW);
    }
    xSemaphoreGive(xSemaphore);
  }
}


/*
  Since LedTurnOn is READ_WRITE variable, onLedTurnOnChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onLedTurnOnChange()  {
  led_on = led_turn_on;
}

/*
  Since UserControl is READ_WRITE variable, onUserControlChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onUserControlChange()  {
  user_cntrl = user_control;
}