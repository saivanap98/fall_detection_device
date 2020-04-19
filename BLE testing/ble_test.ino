#include <ArduinoBLE.h>
#include <DTime.h>

DTime dtime;

uint64_t cycle;

BLEService timerService("1101");
BLEUnsignedCharCharacteristic timeMarkerChar("2101", BLERead | BLENotify);

void setup() {
Serial.begin(9600);
delay(100);
cycle = (uint64_t)millis() + 1000;

while (!Serial);

pinMode(LED_BUILTIN, OUTPUT);
if (!BLE.begin()) 
{
Serial.println("starting BLE failed!");
while (1);
}

BLE.setLocalName("TimeMonitor");
BLE.setAdvertisedService(timerService);
timerService.addCharacteristic(timeMarkerChar);
BLE.addService(timerService);

BLE.advertise();
Serial.println("Bluetooth device active, waiting for connections...");
}

void loop() 
{
BLEDevice central = BLE.central();

if (central) 
{
Serial.print("Connected to central: ");
Serial.println(central.address());
digitalWrite(LED_BUILTIN, HIGH);

while (central.connected()) {

      
      if ((uint64_t)millis() >=  cycle) {
        cycle += 1000;
        dtime.tick();
        sendData();
      }
      delay(200);

}
}
digitalWrite(LED_BUILTIN, LOW);
Serial.print("Disconnected from central: ");
Serial.println(central.address());
}


void sendData() {
    int timer = dtime.timestamp;
    Serial.print("Time is now: ");
    Serial.println(timer);
    timeMarkerChar.writeValue(timer);
  
}
