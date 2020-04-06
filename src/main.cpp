/* 
* Bike Dashboard
* The program is supposed to read from all of the sensors and update the character display
*/

#include "Arduino.h"

//Pins
//#define ENCODER_PIN_1 2 //Needs interrupts!
//#define ENCODER_PIN_2 3

#define LCD_RS 4
#define LCD_EN 5
#define LCD_D4 6
#define LCD_D5 7
#define LCD_D6 8
#define LCD_D7 9

#define HALL_EFFECT_PIN 2

// All the component libraries
#include <DHT.h>
//#include <Encoder.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <RtcDS3231.h>

// Library setups
DHT temp_humid_sensor;
//Encoder encoder(ENCODER_PIN_1, ENCODER_PIN_2);
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
RtcDS3231<TwoWire> Rtc(Wire);

// Basic variables
unsigned long last_temp_update = -2000; // Keep track to update the sensor when possible
int current_readout = 0; // State of previous LCD readouts, for scrolling
const int scroll_time = 1000; // Time till screen scrolls
int ticks = 0; // Ticks of hall effect sensor
float humidity = 0;
float temperature_C = 0;
float temperature_F = 0;
String buffer_1;
String buffer_2;


// Writes the two strings to the LCD
void writeToLCD(String first_string, String second_string)
{
    lcd.flush();
    lcd.home();
    lcd.print(first_string);
    lcd.setCursor(0, 1);
    lcd.print(second_string);
}

#define countof(a) (sizeof(a) / sizeof(a[0]))
// Gets the curent time in the format of HH:MM:SS
String current_clock_time()
{
    RtcDateTime current_time = Rtc.GetDateTime();
    char datestring[10];

    snprintf_P(datestring, 
            countof(datestring),
            PSTR("%02u:%02u:%02u"),
            current_time.Hour(),
            current_time.Minute(),
            current_time.Second() );

    String converted_date = datestring;
    return converted_date;
}

void updateTicks()
//This is the interrupt subroutine that increments ticks counts for each HES response.
{ 
    ticks++; 
}


void setup()
{
    // Serial
    Serial.begin(115200);

    // Clock
    Rtc.Begin();
    Rtc.Enable32kHzPin(false);    
    Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone); 

    // LCD
    lcd.begin(16,2);
    lcd.home();
    lcd.print(PSTR("I'm working!"));
    lcd.setCursor(0, 1);
    lcd.print(PSTR("Need more data!"));

    // Hall effect sensor
    pinMode(HALL_EFFECT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(HALL_EFFECT_PIN), updateTicks, RISING); 

    // DHT Temp + Humidity Sensor
    humidity = temp_humid_sensor.getHumidity();
    temperature_C = temp_humid_sensor.getTemperature();
    temperature_F = temp_humid_sensor.toFahrenheit(temperature_C);
    
}

void loop()
{
    signed long time_since_update = (millis() - last_temp_update);
    // Check if it's time to update the temperature sensors
    if (time_since_update > temp_humid_sensor.getMinimumSamplingPeriod()) {
        // Time to update!
        humidity = temp_humid_sensor.getHumidity();
        temperature_C = temp_humid_sensor.getTemperature();
        temperature_F = temp_humid_sensor.toFahrenheit(temperature_C);
        last_temp_update = millis();
    }
    
    // LCD update switch
    switch(current_readout){
        case -1:
            // Nothing on screen, just got started
            current_readout = 0;
            break;
        case 0:
            // Start with temp and humidity

            // Temperature buffer
            buffer_1 = F("Temperature: ");
            buffer_1 += String(temperature_F, 2);
            buffer_1 += F("°F (");
            buffer_1 += String(temperature_C, 2);
            buffer_1 += F("°C)");

            // Humidity buffer
            buffer_2 = F("Humidity: ");
            buffer_2 = String(humidity, 2);

            // Write buffers to LCD
            writeToLCD(buffer_1, buffer_2);
            delay(scroll_time);
            current_readout = 1;
            break;
        case 1:
            // Temperature and humidity displayed, moving to current time and temperature
 
            // Temperature buffer
            buffer_2 = F("Temperature: ");
            buffer_2 += String(temperature_F, 2);
            buffer_2 += F("°F (");
            buffer_2 += String(temperature_C, 2);
            buffer_2 += F("°C)");

            // Write buffers to LCD
            writeToLCD("Current Time: " + current_clock_time(), buffer_2);
            delay(scroll_time);
            current_readout = 2;
            break;
        case 2:
            // Current time and temperature displayed, move to RPM and time
            delay(scroll_time);
            current_readout = 3;
            break;
        case 3:
            // RPM and time are displayed, move to MPH and RPM
            delay(scroll_time);
            current_readout = 4;
            break;
        case 4:
            // MPH and RPM are displayed, move to humidity and MPH
            delay(scroll_time);
            current_readout = 0;
            break;
    }

}



