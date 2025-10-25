/**
 * @file RAK_Multi_Sensor_Node_SIMPLE.ino
 * @brief Simplified version with proper unconfirmed message handling
 */

#include <Arduino.h>
#include <Wire.h>
#include <LoRaWan-RAK4630.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <CayenneLPP.h>

#define LORAWAN_APP_INTERVAL 300000  // 5 minutes
#define LORAWAN_APP_DATA_BUFF_SIZE 51

uint8_t nodeAppEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t nodeDeviceEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t nodeAppKey[16] = {0x00, 0x00, 0x00, 0x00 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

bool doOTAA = true;

Adafruit_BME680 bme;
SFE_UBLOX_GNSS myGNSS;
CayenneLPP lpp(LORAWAN_APP_DATA_BUFF_SIZE);

TimerEvent_t appTimer;
static uint32_t tx_period = LORAWAN_APP_INTERVAL;

float temp_c = 0.0;
float humidity_rh = 0.0;
float pressure_hpa = 0.0;
float gas_ohms = 0.0;
long latitude = 0;
long longitude = 0;
long altitude = 0;
bool gnss_fix_avail = false;
bool bme680_available = false;

static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0};

void send_lorawan_payload(void);
static void tx_timer_callback(void);
static uint32_t timers_init(void);

void lorawan_has_joined_handler(void)
{
  Serial.println("Join OK!");
  TimerSetValue(&appTimer, tx_period);
  TimerStart(&appTimer);
}

void lorawan_rx_handler(lmh_app_data_t *app_data)
{
  Serial.println("RX OK");
}

void lorawan_confirm_class_handler(DeviceClass_t Class)
{
  m_lora_app_data.buffsize = 0;
  m_lora_app_data.port = 0;
  lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);
}

void lorawan_join_failed_handler(void)
{
  Serial.println("Join FAIL");
  delay(30000);
  lmh_join();
}

static lmh_callback_t lora_callbacks = {
    BoardGetBatteryLevel,
    BoardGetUniqueId,
    BoardGetRandomSeed,
    lorawan_rx_handler,
    lorawan_has_joined_handler,
    lorawan_confirm_class_handler,
    lorawan_join_failed_handler
};

static lmh_param_t lora_param_init = {
    LORAWAN_ADR_ON,
    LORAWAN_DEFAULT_DATARATE,
    LORAWAN_PUBLIC_NETWORK,
    1,
    LORAWAN_DEFAULT_TX_POWER,
    LORAWAN_DUTYCYCLE_OFF
};

void init_lora()
{
  Serial.println("Init LoRa...");
  lora_rak4630_init();
  timers_init();
  lmh_setDevEui(nodeDeviceEUI);
  lmh_setAppEui(nodeAppEUI);
  lmh_setAppKey(nodeAppKey);
  lmh_init(&lora_callbacks, lora_param_init, doOTAA, CLASS_A, LORAMAC_REGION_US915);
  lmh_join();
  Serial.println("LoRa OK");
}

void init_bme680()
{
  Serial.println("Init BME680...");
  if (bme.begin(0x76) || bme.begin(0x77))
  {
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150);
    bme680_available = true;
    Serial.println("BME680 OK");
  }
  else
  {
    Serial.println("BME680 NOT FOUND");
    bme680_available = false;
  }
}

void init_gnss()
{
  Serial.println("Init GNSS...");
  if (myGNSS.begin())
  {
    myGNSS.setI2COutput(COM_TYPE_UBX);
    myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
    Serial.println("GNSS OK");
  }
  else
  {
    Serial.println("GNSS NOT FOUND");
  }
}

void read_bme680_data()
{
  if (!bme680_available) return;
  
  if (bme.performReading())
  {
    temp_c = bme.temperature;
    pressure_hpa = bme.pressure / 100.0;
    humidity_rh = bme.humidity;
    gas_ohms = bme.gas_resistance;
    
    // Convert to Fahrenheit
    float temp_f = (temp_c * 9.0/5.0) + 32.0;
    
    // Classify air quality
    const char* air_quality;
    if (gas_ohms > 50000) air_quality = "GREAT";
    else if (gas_ohms > 20000) air_quality = "BETTER";
    else if (gas_ohms > 10000) air_quality = "OK";
    else if (gas_ohms > 5000) air_quality = "POOR";
    else air_quality = "BAD";
    
    Serial.println("===== Environmental Data =====");
    Serial.print("Temperature: ");
    Serial.print((int)temp_f);
    Serial.print(".");
    Serial.print((int)((temp_f - (int)temp_f) * 10));
    Serial.println("F");
    
    Serial.print("Humidity: ");
    Serial.print((int)humidity_rh);
    Serial.print(".");
    Serial.print((int)((humidity_rh - (int)humidity_rh) * 10));
    Serial.println("%");
    
    Serial.print("Pressure: ");
    Serial.print((int)pressure_hpa);
    Serial.println(" hPa");
    
    Serial.print("Gas: ");
    Serial.print((int)gas_ohms);
    Serial.println(" Ohms");
    
    Serial.print("Air Quality: ");
    Serial.println(air_quality);
    Serial.println("==============================");
  }
}

void read_gnss_data()
{
  gnss_fix_avail = false;
  Serial.println("Getting GPS...");
  unsigned long startTime = millis();
  
  while ((millis() - startTime) < 30000)  // 30 seconds timeout
  {
    if (myGNSS.getPVT())
    {
      if (myGNSS.getFixType() > 0)
      {
        latitude = myGNSS.getLatitude();
        longitude = myGNSS.getLongitude();
        altitude = myGNSS.getAltitude();
        gnss_fix_avail = true;
        Serial.println("GPS FIX OK");
        break;
      }
    }
    delay(1000);
  }
  
  if (!gnss_fix_avail)
  {
    Serial.println("GPS NO FIX");
  }
}

void build_lorawan_payload()
{
  lpp.reset();
  lpp.addTemperature(1, temp_c);
  lpp.addRelativeHumidity(2, humidity_rh);
  lpp.addBarometricPressure(3, pressure_hpa);
  lpp.addAnalogInput(4, gas_ohms);
  
  if (gnss_fix_avail)
  {
    lpp.addGPS(5, (float)latitude / 10000000.0, (float)longitude / 10000000.0, (float)altitude / 1000.0);
  }
}

void send_lorawan_payload()
{
  // Check if we're joined
  if (lmh_join_status_get() != LMH_SET)
  {
    Serial.println("Not joined - skipping TX");
    return;
  }

  build_lorawan_payload();
  m_lora_app_data.buffsize = lpp.getSize();
  m_lora_app_data.port = 1;
  memcpy(m_lora_app_data.buffer, lpp.getBuffer(), m_lora_app_data.buffsize);

  Serial.print("Sending ");
  Serial.print(m_lora_app_data.buffsize);
  Serial.println(" bytes...");

  // Use UNCONFIRMED messages to avoid duty cycle issues
  lmh_error_status result = lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);
  
  if (result == LMH_SUCCESS)
  {
    Serial.println("TX OK - Packet queued for transmission");
  }
  else
  {
    Serial.print("TX FAIL - Error code: ");
    Serial.println(result);
    
    // Print helpful debug info
    if (result == LMH_BUSY)
    {
      Serial.println("Radio is busy - may be in duty cycle restriction");
    }
    else if (result == LMH_ERROR)
    {
      Serial.println("General error occurred");
    }
  }
}

static void tx_timer_callback(void)
{
  Serial.println("\n=== CYCLE START ===");
  Serial.print("Uptime: ");
  Serial.print(millis() / 1000);
  Serial.println(" seconds");
  
  // Power up sensors
  digitalWrite(WB_IO2, HIGH);
  delay(500);
  
  // Read sensor data (sensors already initialized in setup)
  read_bme680_data();
  read_gnss_data();
  
  // Send the data
  send_lorawan_payload();
  
  // Power down sensors
  digitalWrite(WB_IO2, LOW);
  
  Serial.println("=== CYCLE END ===\n");
  
  // CRITICAL: Restart the timer for the next transmission
  TimerSetValue(&appTimer, tx_period);
  TimerStart(&appTimer);
}

static uint32_t timers_init(void)
{
  TimerInit(&appTimer, tx_timer_callback);
  return 0;
}

void setup()
{
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n=== RAK4631 START ===");
  
  // Setup power control pin
  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, HIGH);
  delay(500);
  
  // Initialize I2C and sensors ONCE during setup
  Wire.begin();
  init_bme680();
  init_gnss();
  
  // Initialize LoRaWAN (will start timer after join)
  init_lora();
  
  Serial.println("=== SETUP DONE ===");
  Serial.print("Transmit interval: ");
  Serial.print(tx_period / 1000);
  Serial.println(" seconds");
}

void loop()
{
  // Simple delay - the timer handles transmissions
  delay(1000);
}
