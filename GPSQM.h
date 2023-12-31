/*
	SQM.h

	(c) 2023 F.Lesage

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

// Battery level
#define GPIO_BAT_ADC			GPIO_NUM_13			// FIXME
#define GPIO_BAT_ADC_EN			GPIO_NUM_12			// FIXME
#define LOW_BATTERY_COUNT_MIN	5
#define LOW_BATTERY_COUNT_MAX	10
#define BAT_V_MAX				4200	// in mV
#define BAT_V_MIN				3000	// in mV
#define BAT_LEVEL_MIN			33		// in %, corresponds to ~3.4V for a typical Li-ion battery
#define VCC						3300	// in mV
#define V_DIV_R1				100000	// voltage divider R1 in ohms
#define V_DIV_R2				300000	// voltage divider R2 in ohms
#define ADC_MAX					4096	// 12 bits resolution
#define V_MAX_IN				( BAT_V_MAX*V_DIV_R2 )/( V_DIV_R1+V_DIV_R2 )	// in mV
#define V_MIN_IN				( BAT_V_MIN*V_DIV_R2 )/( V_DIV_R1+V_DIV_R2 )	// in mV
#define ADC_V_MAX				( V_MAX_IN*ADC_MAX / VCC )
#define ADC_V_MIN				( V_MIN_IN*ADC_MAX / VCC )

// SQM
#define	DOWN	-1
#define	UP		1
#define	MSAS_CALIBRATION_OFFSET	-0.55F	// Taken from a comparison with a calibrated SQM-L(E/U)

// LED DISPLAY
#define	DISPLAY_I2C_ADDR	0x70

// GPS
#define	GPS_RX		GPIO_NUM_23
#define	GPS_TX		GPIO_NUM_5
#define	GPS_SPEED	9600

// Misc
#define GPIO_DEBUG		GPIO_NUM_34			// FIXME

#define TSL_SENSOR			0x01
#define SI7201_SENSOR		0x02
#define	GPS_SENSOR			0x04
#define ALL_SENSORS			( TSL_SENSOR | SI7201_SENSOR | GPS_SENSOR )

// Runtime configuration
#define FORMAT_SPIFFS_IF_FAILED true
#define	CONFIG_SSID				"GPSQM"
#define	CONFIG_SSID_PASSWORD	"GPSQM2023!"
#define	TZNAME	"CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00"

// Datamancers.net ROOT CA
#define ROOT_CA \
"-----BEGIN CERTIFICATE-----\n" \
"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\n" \
"-----END CERTIFICATE-----\n"


const char *configuration_items[] = { "ssid", "password", "root_ca", "tzname", "msas_calibration_offset" };
const char *default_configuration[] = { CONFIG_SSID, CONFIG_SSID_PASSWORD, ROOT_CA, TZNAME, "-0.55" };

struct sqm_parameters_t {

	bool	debug_mode;
	Adafruit_TSL2591 *tsl;
	Adafruit_Si7021 *si7021;
	JsonObject *config;
	JsonDocument *values;
	TinyGPSPlus *gps;
	SoftwareSerial *gps_serial;
	
};

void displayBanner( JsonObject & );
void get_msas( void );
bool read_runtime_config( JsonObject *, byte );
float read_Si7021( Adafruit_Si7021 *, byte );
void reboot( void );
void reset_config_parameter( void );
void set_configuration( void );
bool start_hotspot( void );
void web_server_not_found( void );
