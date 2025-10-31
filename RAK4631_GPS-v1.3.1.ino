/**
 * @file RAK4631_GPS-v1.3.1.ino
 * @brief GPS-only LoRaWAN tracker
 * 
 * Hardware: RAK19007 Base + RAK4631 Core + RAK12500 GNSS
 * Region: US915
 */

#include <Arduino.h>
#include <Wire.h>
#include <LoRaWan-RAK4630.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <CayenneLPP.h>

#define LORAWAN_APP_INTERVAL 300000  // 5 minutes
#define LORAWAN_APP_DATA_BUFF_SIZE 51

// LoRaWAN Credentials - Update these in your network server!
uint8_t nodeAppEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t nodeDeviceEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t nodeAppKey[16] = {0x00, 0x00, 0x00, 0x00 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

bool doOTAA = true;

SFE_UBLOX_GNSS myGNSS;
CayenneLPP lpp(LORAWAN_APP_DATA_BUFF_SIZE);

TimerEvent_t appTimer;
static uint32_t tx_period = LORAWAN_APP_INTERVAL;

long latitude = 0;
long longitude = 0;
long altitude = 0;
bool gnss_fix_avail = false;

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
  
  if (gnss_fix_avail)
  {
    lpp.addGPS(1, (float)latitude / 10000000.0, (float)longitude / 10000000.0, (float)altitude / 1000.0);
  }
}

void send_lorawan_payload()
{
  if (lmh_join_status_get() != LMH_SET)
  {
    Serial.println("Not joined");
    return;
  }

  build_lorawan_payload();
  m_lora_app_data.buffsize = lpp.getSize();
  m_lora_app_data.port = 1;
  memcpy(m_lora_app_data.buffer, lpp.getBuffer(), m_lora_app_data.buffsize);

  if (lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG) == LMH_SUCCESS)
  {
    Serial.println("TX OK");
  }
  else
  {
    Serial.println("TX FAIL");
  }
}

static void tx_timer_callback(void)
{
  Serial.println("\n=== CYCLE START ===");
  Serial.print("Uptime: ");
  Serial.print(millis() / 1000);
  Serial.println(" seconds");
  
  digitalWrite(WB_IO2, HIGH);
  delay(500);
  
  read_gnss_data();
  send_lorawan_payload();
  
  digitalWrite(WB_IO2, LOW);
  
  Serial.println("=== CYCLE END ===\n");
  
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
  
  Serial.println("\n=== RAK4631 GPS TRACKER ===");
  
  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, HIGH);
  delay(500);
  
  Wire.begin();
  init_gnss();
  init_lora();
  
  Serial.println("=== SETUP DONE ===");
  Serial.print("Transmit interval: ");
  Serial.print(tx_period / 1000);
  Serial.println(" seconds");
}

void loop()
{
  delay(1000);
}
