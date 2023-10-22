/*
	GPSQM.ino

	(c) 2023 F.Lesage

	0.1.0 - Prototype version, derived from AstroWeatherStation 2.0.1. This is still WIP: basic functions work (GPS, Display, SQM) but that's all ...

	This program is free software: you can redistribute it and/or modify it
	under the terms of the GNU General Public License as published by the
	Free Software Foundation, either version 3 of the License, or (at your option)
	any later version.

	This program is distributed in the hope that it will be useful, but
	WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
	or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
	more details.

	You should have received a copy of the GNU General Public License along
	with this program. If not, see <https://www.gnu.org/licenses/>.
*/

#include <ESP32Time.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "Adafruit_TSL2591.h"
#include "Adafruit_Si7021.h"
#include "NoiascaHt16k33.h"
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include "time.h"
#include "ESP32OTAPull.h"
#include "FS.h"
#include "SPIFFS.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "GPSQM.h"

#define REV "0.1.0"
#define BUILD_DATE "20230801"
#define BUILD "01"

#define DEBUG_MODE 1

// A lot of global stuff for the moment as I still get panics when I do otherwise :-)

WebServer web_server( 80 );

bool debug_mode;
Noiasca_ht16k33_hw_14 display = Noiasca_ht16k33_hw_14();
StaticJsonDocument<384> values;

ESP32Time rtc(0);

extern "C" { uint8_t temprature_sens_read(); }

void setup()
{
	float	battery_level = 0.0;
	bool	web_ok;
//	char	string[64];

	Adafruit_TSL2591	tsl = Adafruit_TSL2591( 2591 );
	Adafruit_Si7021		si7021 = Adafruit_Si7021();
	JsonObject runtime_config;
	TinyGPSPlus gps;
	SoftwareSerial gps_serial( GPS_RX, GPS_TX );

	TaskHandle_t server_task_handle;
	TaskHandle_t sqm_task_handle;

	Serial.begin( 115200 );
	delay( 100 );

	if ( debug_mode )
		Serial.printf( "Starting...\n" );

	Wire.begin();
	display.begin( DISPLAY_I2C_ADDR, 1 );
	display.print( "BOOT" );
             

	if (( web_ok = start_hotspot() )) {

			Serial.printf( "Started AP on SSID [%s] with server @ IP=192.168.168.1/24\n", CONFIG_SSID );
			web_server.on( "/setconfig", HTTP_POST, set_configuration );
			web_server.on( "/resetparam", HTTP_POST, reset_config_parameter );
			web_server.on( "/reboot", HTTP_GET, reboot );
			web_server.on( "/msas", HTTP_GET, get_msas );
			web_server.on( "/status", HTTP_GET, get_sqm_status );
			web_server.onNotFound( web_server_not_found );
			web_server.begin();
	}
	
	pinMode( GPIO_DEBUG, INPUT );
	debug_mode = static_cast<byte>( 1-gpio_get_level( GPIO_DEBUG ) ) | DEBUG_MODE;
		
	if ( !read_runtime_config( &runtime_config, debug_mode ) ) {

		Serial.printf( "Panic: something weird happened, configuration does not fit into allocated space. Stopping here.\n" );
	}

	setenv("TZ",get_config_parameter( &runtime_config, "tzname" ),1);  //  Now adjust the TZ.  Clock settings are adjusted to show the new local time
	tzset();

	if ( debug_mode )
		displayBanner( runtime_config );

	gps_serial.begin( GPS_SPEED );
		
	battery_level = get_battery_level( debug_mode );
	values[ "battery_level" ] = battery_level;

	initialise_sensors( &tsl, &si7021, debug_mode );

	sqm_parameters_t sqm_parameters;
	sqm_parameters.debug_mode = debug_mode;
	sqm_parameters.tsl = &tsl;
	sqm_parameters.si7021 = &si7021;
	sqm_parameters.config = &runtime_config;
	sqm_parameters.values = &values;
	sqm_parameters.gps = &gps;
	sqm_parameters.gps_serial = &gps_serial;
	
	xTaskCreatePinnedToCore( sqm_task, "SQMTask", 10000,  &sqm_parameters, 10, &sqm_task_handle, 0 );
	xTaskCreatePinnedToCore( server_task, "ServerTask", 10000, NULL, 10, &server_task_handle, 1 );

}

static void feed_gps_and_wait( TinyGPSPlus *gps, SoftwareSerial *gps_serial, unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (gps_serial->available())
      gps->encode(gps_serial->read());
     delay( 5 );					// Otherwise we would starve the idle task
  } while (millis() - start < ms);
}
    
void loop()
{
	delay( 100000 );
}

void server_task( void *task_parameter )
{
	while( true ) web_server.handleClient();
}

void sqm_task( void *task_parameter )
{
	char string[6];
	float	msas, nelm;
	sqm_parameters_t *params = (sqm_parameters_t *)task_parameter;
	Adafruit_TSL2591 *tsl = params->tsl;
	Adafruit_Si7021 *si7021 = params->si7021;
	JsonObject *config = params->config;
	JsonDocument *values = params->values;
	TinyGPSPlus *gps = params->gps;
	SoftwareSerial *gps_serial = params->gps_serial;
	
	while( true ) {

		retrieve_sensor_data( config, values, tsl,  si7021, gps, &msas, &nelm, params->debug_mode );
		display.clear();
		sprintf( string, "%02.2f", msas );
		display.print( string );
		Serial.printf( "DATETIME=%02d-%02d-%02d %02d:%02d:%02d\n", gps->date.year(), gps->date.month(), gps->date.day(), gps->time.hour(), gps->time.minute(), gps->time.second() );
  		Serial.printf( "SATS=[%2d] LAT=[%02.8f] LONG=[%02.8f] ALT=[%4.2f] SPEED=[%6.2f]\n", gps->satellites.value(), gps->location.lat(),gps->location.lng(), gps->altitude.meters(), gps->speed.kmph() );
		if (millis() > 5000 && gps->charsProcessed() < 10)
    		Serial.println( "No GPS data" );
  		feed_gps_and_wait( gps, gps_serial, 10000 );
	}
}

/*
   --------------------------------

    Data handling

   --------------------------------
*/
void retrieve_sensor_data( JsonObject *config, JsonDocument *values, Adafruit_TSL2591 *tsl, Adafruit_Si7021 *si7021, TinyGPSPlus *gps, float *msas, float *nelm, byte debug_mode )
{
	float		temp,
				msas_calibration_offset;

	uint16_t	ch0,
				ch1,
				gain,
				int_time;

	char		datetime[55];
	struct tm 	ts = {0};
	
	time_t		now;
	
	if ( gps->time.isValid () ) {
		
		sprintf( datetime, "%04d-%02d-%02dT%02d:%02d:%02d:%03dZ", gps->date.year(), gps->date.month(), gps->date.day(), gps->time.hour(), gps->time.minute(), gps->time.second(), gps->time.centisecond() );
		strptime( datetime,"%Y-%m-%dT%H:%M:%S:%03dZ", &ts );
	}

	now = mktime( &ts );
	localtime( &now );
	(*values)[ "timestamp" ] = now;

	(*values)[ "temp" ] = temp = read_Si7021( si7021, debug_mode );
	Serial.printf( "TEMP=%f\n", temp );
	
	msas_calibration_offset = strtof( get_config_parameter( config, "msas_calibration_offset" ), NULL );
	read_SQM( tsl, debug_mode, msas_calibration_offset, temp, msas, nelm, &ch0, &ch1, &gain, &int_time );
	(*values)[ "msas" ] = *msas;
	(*values)[ "nelm" ] = *nelm;
	(*values)[ "gain" ] = gain;
	(*values)[ "int_time" ] = int_time;
	(*values)[ "ch0" ] = ch0;
	(*values)[ "ch1" ] = ch1;

	(*values)[ "mytemp" ] = ( temprature_sens_read() - 32 ) / 1.8;
}


/*
   --------------------------------

    Sensor intialisation

   --------------------------------
*/
void initialise_sensors( Adafruit_TSL2591 *tsl, Adafruit_Si7021 *si7021, byte debug_mode )
{
	initialise_Si7021( si7021, debug_mode );
	initialise_TSL( tsl, debug_mode );
	
}

void initialise_Si7021( Adafruit_Si7021 *si7021, byte debug_mode )
{
	if ( !si7021->begin() ) {

		if ( debug_mode )
			Serial.println( "Could not find Si7021" );
	
	} else {

		if ( debug_mode )
			Serial.println( "Found Si7021" );

	}
}
void initialise_TSL( Adafruit_TSL2591 *tsl, byte debug_mode )
{
	if ( !tsl->begin() ) {

		if ( debug_mode )
			Serial.println( "Could not find TSL2591" );

	} else {

		if ( debug_mode )
			Serial.println( "Found TSL2591" );

		tsl->setGain( TSL2591_GAIN_LOW );
		tsl->setTiming( TSL2591_INTEGRATIONTIME_100MS );
	}
}

/*
   --------------------------------

	WebServer functions

   --------------------------------
  */

void get_msas( void )
{
	char json[400];
	serializeJson( values, json );

	web_server.send( 200, "application/json", json );
}

void get_sqm_status( void )
{
	char json[400];
	serializeJson( values, json );

	web_server.send( 200, "application/json", json );

}
/*
   --------------------------------

    Utility functions

   --------------------------------
*/
//
// Formulas inferred from TSL2591 datasheet fig.15, page 9. Data points graphically extracted (yuck!).
// Thanks to Marco Gulino <marco.gulino@gmail.com> for pointing this graph to me!
// I favoured a power function over an affine because it better followed the points on the graph
// It may however be overkill since the points come from a PDF graph: the source error is certainly not negligible :-)
//
float ch0_temperature_factor( float temp )
{
	return 0.9759F + 0.00192947F*pow( temp, 0.783129F );
}

float ch1_temperature_factor( float temp )
{
	return 1.05118F - 0.0023342F*pow( temp, 0.958056F );
}

void change_gain( Adafruit_TSL2591 *tsl, byte upDown,  tsl2591Gain_t *gain_idx )
{
	tsl2591Gain_t	g = static_cast<tsl2591Gain_t>( *gain_idx + 16*upDown );

	if ( g < TSL2591_GAIN_LOW )
		g = TSL2591_GAIN_LOW;
	else if ( g > TSL2591_GAIN_MAX )
		g = TSL2591_GAIN_MAX;

	if ( g != *gain_idx ) {

		tsl->setGain( g );
		*gain_idx = g;
	}	
}

void change_integration_time( Adafruit_TSL2591 *tsl, byte upDown, tsl2591IntegrationTime_t *int_time_idx )
{
	tsl2591IntegrationTime_t	t = static_cast<tsl2591IntegrationTime_t>( *int_time_idx + 2*upDown );

	if ( t < TSL2591_INTEGRATIONTIME_100MS )
		t = TSL2591_INTEGRATIONTIME_100MS;
	else if ( t > TSL2591_INTEGRATIONTIME_600MS )
		t = TSL2591_INTEGRATIONTIME_600MS;

	if ( t != *int_time_idx ) {

		tsl->setTiming( t );
		*int_time_idx = t;
	}	
}

void displayBanner( JsonObject &config )
{
	char	string[96];
	byte	i;

	delay(2000);
	Serial.println( "\n##############################################################################################" );
	Serial.println( "# SQM                                                                                        #" );
	Serial.println( "#  (c) Lesage Franck - lesage@datamancers.net                                                #" );
	snprintf( string, 96, "# REV %8s-%8s-%-3s", REV, BUILD_DATE, BUILD );
	for ( i = strlen( string ); i < 93; string[i++] = ' ' );
	strcat( string, "#\n" );
	Serial.printf( string );
	Serial.println( "#--------------------------------------------------------------------------------------------#" );
	Serial.println( "# GPIO PIN CONFIGURATION                                                                     #" );
	Serial.println( "#--------------------------------------------------------------------------------------------#" );
	memset( string, 0, 96 );
	snprintf( string, 93, "# DEBUG     : %d", GPIO_DEBUG );
	for ( i = strlen( string ); i < 93; string[i++] = ' ' );
	strcat( string, "#\n" );
	Serial.printf( string );
	memset( string, 0, 96 );
	snprintf( string, 93, "# BAT LVL   : SW=%d ADC=%d", GPIO_BAT_ADC_EN, GPIO_BAT_ADC );
	for ( i = strlen( string ); i < 93; string[i++] = ' ' );
	strcat( string, "#\n" );
	Serial.printf( string );
	Serial.println( "#--------------------------------------------------------------------------------------------#" );
	Serial.println( "# RUNTIME CONFIGURATION (*:firmware default)                                                 #" );
	Serial.println( "#--------------------------------------------------------------------------------------------#" );
	print_config( config );
	Serial.println( "##############################################################################################" );
}

void reboot()
{
	Serial.printf( "Rebooting...\n" );
	web_server.send( 200, "text/plain", "OK\n" );
	delay( 500 );
	ESP.restart();	
}

/*
   --------------------------------

    Sensor reading

   --------------------------------
*/
float get_battery_level( byte debug_mode )
{
	int		adc_value = 0;
	float	battery_level,
			adc_v_in,
			bat_v;
	
	if ( debug_mode )
		Serial.print( "Battery level: " );

	digitalWrite( GPIO_BAT_ADC_EN, HIGH );
	delay( 500 );
	for( byte i = 0; i < 5; i++ ) {

		adc_value += analogRead( GPIO_BAT_ADC );
		delay( 100 );
	}
	adc_value = static_cast<int>( adc_value / 5 );
	
	battery_level = map( adc_value, ADC_V_MIN, ADC_V_MAX, 0, 100 );

	if ( debug_mode ) {

		adc_v_in = static_cast<float>(adc_value) * VCC / ADC_V_MAX;
		bat_v = adc_v_in * ( V_DIV_R1 + V_DIV_R2 ) / V_DIV_R2;
		Serial.printf( "%03.2f%% (ADC value=%d, ADC voltage=%1.3fV, battery voltage=%1.3fV)\n", battery_level, adc_value, adc_v_in / 1000.F, bat_v / 1000.F );

	}

	digitalWrite( GPIO_BAT_ADC_EN, LOW );
	return battery_level;
}

float read_Si7021( Adafruit_Si7021 *si7021, byte debug_mode )
{
	return si7021->readTemperature();
}

void read_SQM( Adafruit_TSL2591 *tsl, byte debug_mode, float msas_calibration_offset, float ambient_temp, float *msas, float *nelm, uint16_t *ch0, uint16_t *ch1, uint16_t *gain, uint16_t *int_time )
{
	tsl->setGain( TSL2591_GAIN_LOW );
	tsl->setTiming( TSL2591_INTEGRATIONTIME_100MS );
	while ( !SQM_get_msas_nelm( tsl, debug_mode, msas_calibration_offset, ambient_temp, msas, nelm, ch0, ch1, gain, int_time ));
}

byte SQM_get_msas_nelm( Adafruit_TSL2591 *tsl, byte debug_mode, float msas_calibration_offset, float ambient_temp, float *msas, float *nelm, uint16_t *ch0, uint16_t *ch1, uint16_t *gain, uint16_t *int_time )
{
	uint32_t	lum;
	uint16_t	ir,
				full,
				visible;
	byte		iterations = 1;

	tsl2591Gain_t				gain_idx;
	tsl2591IntegrationTime_t 	int_time_idx;

	const uint16_t	gain_factor[4]		= { 1, 25, 428, 9876 },
					integration_time[6]	= { 100, 200, 300, 400, 500, 600 };

	gain_idx = tsl->getGain();
	int_time_idx = tsl->getTiming();
	lum = tsl->getFullLuminosity();
	ir = lum >> 16;
	full = lum & 0xFFFF;
	ir = static_cast<uint16_t>( static_cast<float>(ir) * ch1_temperature_factor( ambient_temp ) );
	full = static_cast<uint16_t>( static_cast<float>(full) * ch0_temperature_factor( ambient_temp ) );
	visible = full - ir;

	// On some occasions this can happen, leading to high values of "visible" although it is dark (as the variable is unsigned), giving erroneous msas
	if ( full < ir )
		return 0;

	if ( debug_mode )
		Serial.printf( "SQM: gain=0x%02x (%dx) integ=0x%02x (%dms)/ temp=%f / ir=%d full=%d vis=%d\n", gain_idx, gain_factor[ gain_idx >> 4 ], int_time_idx, integration_time[ int_time_idx ], ambient_temp, ir, full, visible );

	// Auto gain and integration time, increase time before gain to avoid increasing noise if we can help it, decrease gain first for the same reason
	if ( visible < 128 ) {

		if ( int_time_idx != TSL2591_INTEGRATIONTIME_600MS ) {

				if ( debug_mode )
					Serial.println( "SQM: Increasing integration time." );

				change_integration_time( tsl, UP, &int_time_idx );
        		return 0;
		}

		if ( gain_idx == TSL2591_GAIN_MAX ) {

			if ( int_time_idx != TSL2591_INTEGRATIONTIME_600MS ) {

				if ( debug_mode )
					Serial.println( "SQM: Increasing integration time." );

				change_integration_time( tsl, UP, &int_time_idx );
        		return 0;

			} else {

				iterations = SQM_read_with_extended_integration_time( tsl, debug_mode, ambient_temp, &ir, &full, &visible );
				if ( full < ir )
					return 0;
			}

		} else {

				if ( debug_mode )
					Serial.println( "Increasing gain." );

				change_gain( tsl, UP, &gain_idx );
        		return 0;
		}

	} else if (( full == 0xFFFF ) || ( ir == 0xFFFF )) {

		if ( gain_idx != TSL2591_GAIN_LOW ) {

			if ( debug_mode )
				Serial.printf( "Decreasing gain." );

			change_gain( tsl, DOWN, &gain_idx );
			return 0;

		} else {

			if ( debug_mode )
				Serial.println( "Decreasing integration time." );

			change_integration_time( tsl, DOWN, &int_time_idx );
			return 0;
		}
	}

	// Comes from Adafruit TSL2591 driver
	float cpl = static_cast<float>( gain_factor[ gain_idx >> 4 ] ) * static_cast<float>( integration_time[ int_time_idx ] ) / 408.F;
	float lux = ( static_cast<float>(visible) * ( 1.F-( static_cast<float>(ir)/static_cast<float>(full))) ) / cpl;

	// About the MSAS formula, quoting http://unihedron.com/projects/darksky/magconv.php:
	// This formula was derived from conversations on the Yahoo-groups darksky-list
	// Topic: [DSLF]  Conversion from mg/arcsec^2 to cd/m^2
	// Date range: Fri, 1 Jul 2005 17:36:41 +0900 to Fri, 15 Jul 2005 08:17:52 -0400
	
	// I added a calibration offset to match readings from my SQM-LE
	*msas = ( log10( lux / 108000.F ) / -0.4F ) + msas_calibration_offset;
	if ( *msas < 0 )
		*msas = 0;
	*nelm = 7.93F - 5.F * log10( pow( 10, ( 4.316F - ( *msas / 5.F ))) + 1.F );
	
	*int_time = integration_time[ int_time_idx ];
	*gain = gain_factor[ gain_idx >> 4 ];
	*ch0 = full;
	*ch1 = ir;
	if ( debug_mode )
		Serial.printf("GAIN=[0x%02hhx/%ux] TIME=[0x%02hhx/%ums] iter=[%d] VIS=[%d] IR=[%d] MPSAS=[%f] NELM=[%f]\n", gain_idx, gain_factor[ gain_idx >> 4 ], int_time_idx, integration_time[ int_time_idx ], iterations, visible, ir, *msas, *nelm );

	return 1;
}


byte SQM_read_with_extended_integration_time( Adafruit_TSL2591 *tsl, byte debug_mode, float ambient_temp, uint16_t *cumulated_ir, uint16_t *cumulated_full, uint16_t *cumulated_visible )
{
	uint32_t	lum;
	uint16_t	ir,
				full;
	byte		iterations = 1;

	while (( *cumulated_visible < 128 ) && ( iterations <= 32 )) {

		iterations++;
		lum = tsl->getFullLuminosity();
		ir = lum >> 16;
		full = lum & 0xFFFF;
		ir = static_cast<uint16_t>( static_cast<float>(ir) * ch1_temperature_factor( ambient_temp ));
		full = static_cast<uint16_t>( static_cast<float>(full) * ch0_temperature_factor( ambient_temp ));
		*cumulated_full += full;
		*cumulated_ir += ir;
		*cumulated_visible = *cumulated_full - *cumulated_ir;

		delay( 50 );
	}

	return iterations;
}

/*
   --------------------------------------

    Runtime configuration management
    
   --------------------------------------
*/

char *build_config_json_string( )
{
	// \"key\":\"value\" means 4x\" + : -> 9 chars
	const size_t keys_size = 9+strlen( "ssid" ) + 9+strlen( "password" ) + 9+strlen( "root_ca" ) + 9+strlen( "msas_calibration_offset" );
	// MSAS_CALIBRATION_OFFSET is [+/-] %02.f long -> 6 chars
	const size_t values_size = strlen( CONFIG_SSID ) + strlen( CONFIG_SSID_PASSWORD ) + strlen( ROOT_CA ) + 6;
	// key/values + two brackets + commas + final 0
	char *jsonString = (char *)malloc( 2 + sizeof( configuration_items ) + keys_size + values_size );

	memset( jsonString, 0, 2 + sizeof( configuration_items ) + keys_size + values_size );
	snprintf( jsonString, 2 + sizeof( configuration_items ) + keys_size + values_size, "{\"ssid\":\"%s\",\"password\":\"%s\",\"root_ca\":\"%s\",\"msas_calibration_offset\":\"%02.2f\"}",
		CONFIG_SSID, CONFIG_SSID_PASSWORD, ROOT_CA, MSAS_CALIBRATION_OFFSET );

	return jsonString;
}

const char *get_config_parameter( const JsonObject *config, const char *item )
{
	if ( config->containsKey( item ))
		return (const char *)((*config)[ item ]);
		
	for( byte i = 0; i <  (int)( sizeof( configuration_items ) / sizeof( configuration_items[ 0 ] )); i++ )
		if ( !strcmp( configuration_items[ i ], item ))
			return default_configuration[i];

	return "";
}

void print_config( const JsonObject &config )
{
	char		string[96];
	const char	*parameter;
	int			offset,
				len,
				room,
				s;
	byte		pad,
				i,
				j;
		
	for( i = 0; i < (int)( sizeof( configuration_items ) / sizeof( configuration_items[ 0 ] )); i++ ) {

		memset( string, 0, 96 );
		snprintf( string, 93, "# %-24s", configuration_items[ i ] );
		offset = 0;

		if ( config.containsKey( configuration_items[ i ] )) {

			strcat( string, "  : " );
			parameter = (const char *)config[ configuration_items[ i ] ];

		} else {
			
			strcat( string, "* : " );
			parameter = default_configuration[ i ];
		}
		
		len = strlen( parameter );
		while ( offset < len ) {

			s = strlen( string );
			room = 96 - s - 5;
			strncat( string, parameter+offset, room );
			offset += strlen( string ) - s;
			for ( pad = strlen( string ); pad < 92; string[ pad++ ] = ' ' );
			for ( j = 0; j < strlen( string ); j++ )
				if ( string[j] == '\n' ) string[j] = ' ';
			strcat( string, " #\n" );
			Serial.printf( string );
			memset( string, 0, 96 );
			sprintf( string, "# " );
		}
	}
}

bool read_runtime_config( JsonObject *runtime_config, byte debug_mode )
{
	static StaticJsonDocument<160>	json_config;	// runtime_config will be a reference to this, keep it in memory
	char							*jsonString;
	int								offset = 0,
									s = 0;
	if ( debug_mode )
		Serial.printf( "Reading config file: " );

	if ( !SPIFFS.begin( true )) {

		if ( debug_mode )
			Serial.printf( "could not access flash filesystem, creating config from firmware defaults.\n" );
		jsonString = build_config_json_string();

	} else {
	
		File file = SPIFFS.open( "/aws.conf", FILE_READ );
		if ( !file || !( s = file.size() )) {

			jsonString = build_config_json_string();

			if ( debug_mode )
				Serial.printf( "could not read config file or file is empty, creating config from firmware defaults.\n" );

		} else {

			jsonString = (char *)malloc( s );
			memset( jsonString, 0, s );
		
			while( file.available() ) {
				offset += file.readBytes( jsonString+offset, 64 );
			}
			file.close();
			if ( debug_mode )
				Serial.printf( "OK.\n");
		}
	}
	
	if ( DeserializationError::Ok == deserializeJson( json_config, jsonString )) {

		*runtime_config = json_config.as<JsonObject>();
		return true;

	}
	return false;
}

void reset_config_parameter( void )
{
	JsonObject	*runtime_config = NULL;
	char		parameter[32],
				json_string[2000];
	bool		ok;
	
	snprintf( parameter, 32, web_server.arg( "plain" ).c_str() );

	ok = read_runtime_config( runtime_config, 1 );
	if ( ok ) {

		serializeJson( *runtime_config, json_string );
		if ( runtime_config->containsKey( parameter )) {

			runtime_config->remove( parameter );
			serializeJson( *runtime_config, json_string, 2000 );
			save_configuration( json_string );
			web_server.send( 200, "text/plain", "OK\n" );
			delay( 1000 );
			return;
		}
		web_server.send( 400, "text/plain", "Parameter not found." );
		return;
	}
	web_server.send( 500, "text/plain", "Cannot build config." );
}

const char *save_configuration( const char *jsonString )
{
	if ( !SPIFFS.begin( true )) {
		
		Serial.printf( "Error: could not access flash filesystem.\n" );
		return "Error: could not access flash filesystem.";
	}
	File file = SPIFFS.open( "/aws.conf", FILE_WRITE );
	file.println( jsonString );
	file.close();
	return "";
}

void set_configuration()
{
	StaticJsonDocument<160> json_config;
	String jsonString = web_server.arg( "plain" );
	DeserializationError code;
	
	if ( ( code = deserializeJson( json_config, jsonString )) == DeserializationError::Ok ) {

		save_configuration( jsonString.c_str() );
		web_server.send( 200, "text/plain", "OK\n" );
		return;
	}
	web_server.send( 400, "text/plain", code.c_str() );
}

bool start_hotspot()
{
	bool x;
	if (( x = WiFi.softAP( CONFIG_SSID, CONFIG_SSID_PASSWORD ))) {

		const IPAddress my_IP( 192, 168, 168, 1 );
		const IPAddress gateway( 192, 168, 168, 1 );
		const IPAddress subnet( 255, 255, 255, 0 );
		WiFi.softAPConfig( my_IP, gateway, subnet );
	}
	return x;
}

void web_server_not_found()
{
	web_server.send( 404, "text/plain", "Not found\n" );
}
