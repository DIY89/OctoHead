//#include <String.h>
#include <ArduinoJson.h>
#include <DHT.h>

// Setze den Interruptpin auf 2
const byte interruptPin = 2;

#define MAX_MSG_LENGTH 20

#define ON "ON"
#define OFF "OFF"

#define DHT1_PIN 3          // Hier die Pin Nummer eintragen wo der Sensor angeschlossen ist
#define DHT2_PIN 4
#define DHT3_PIN 5
#define DHT4_PIN 6
#define DHT5_PIN 7

#define RELAIS1_PIN 14
#define RELAIS2_PIN 15
#define RELAIS3_PIN 16
#define RELAIS4_PIN 17

StaticJsonDocument<200> doc;

/*Message protocol Arduino <-> Raspi
id : Message Number (must be increase for each message; Respone contains same number
command: r=read; w=write
device_type: "dht": for DHT Sensor data; "switch": for relais; "sensor": for sensor status
device_name: name of the device
data: empty if sensor data is requested; state ON/OFF for switches 
*/
class Message{
  private:
    int id = 0;
    const char* command = "";
    const char* device_type = "";
    const char* device_name = "";
    const char* data = "";

  public: 
    void read_msg(){
      if(check_msg()){
        id = doc["id"].as<int>();
        command = doc["command"];
        device_type = doc["device_type"];
        device_name = doc["device_name"];
        data = doc["data"];
      }
    }
    

    bool check_msg(){
      // Read the JSON document from the "link" serial port
      DeserializationError err = deserializeJson(doc, Serial);

      // Test if parsing succeeded.
      if (err == DeserializationError::Ok){
        return true;
      }else{
        Serial.print("deserializeMsgPack() failed: ");
        //Serial.println(error);
        return true;
      }
    }
};


void alarm(){
  Serial.println("ALARM");
  // Shutdown Printer on Fire Alarm
  digitalWrite(RELAIS1_PIN, HIGH);
}

class Sensor : public DHT{
  private: 
    int sensor_nr;
    int pin_nr;
    int sensor_type;
    String sensor_location;
    bool sensor_state = false;
    float temperatur;
    float humidity;
    bool data_request = false;
    
  public:
    Sensor(int nr = 0,int sensor_pin = 0, int type = 0, String location = "") : DHT(sensor_pin,type), sensor_nr(nr),pin_nr(sensor_pin),sensor_type(type),sensor_location(location){
      sensor_nr = nr;
      pin_nr = sensor_pin;
      sensor_type = type; 
      sensor_location = location;
      begin();
    }
    
    bool check_dht_sensor_data(){
      /*********************( Überprüfen ob alles richtig Ausgelesen wurde )*********************/ 
      //Serial.println("Sensor Data Check");
      //Serial.println(dht.temperatur);
      //Serial.println(dht.humidity);
      if (isnan(temperatur) || isnan(humidity)) {
        // Serial.print("Fehler beim auslesen des Sensors: ");
        // Serial.println(dht.sensor_nr);
        return false;
      }else{
        return true;
      }
    }
  
    void send_dht_sensor_data_uart(){
      if(Sensor::check_dht_sensor_data()){
        // Senden der gemessenen Werte an den PI 
        Serial.println(sensor_nr);                  // Ausgeben der Sensor Nr
        //Serial.print('\n');
        Serial.println(temperatur);                 // Ausgeben der Temperatur
        //Serial.print('\n');
        Serial.println(humidity);                  // Ausgeben der Luftfeuchtigkeit
        //Serial.print("\n");
      }
    }
    
    void read_dht_data(){
      /* ---------------- DHT Sensor 1 ---------------- */                           
      temperatur = DHT::readTemperature();    // Lesen der Luftfeuchtigkeit und speichern in die Variable h1
      humidity = DHT::readHumidity(); // Lesen der Temperatur in °C und speichern in die Variable t1
    }

    void get_dht_command(String command){
      command.trim();
      //String data = command.split();
      // Serial.print("Check DHT command: ");
      // Serial.println(command);
      if(command == "dht"){
        // Serial.println("Set dht data request: true");
        data_request = true;
      }
    }

    String get_available_dht_sensors(){
      if(sensor_location != ""){ //Sends sensor locatin if it's not empty
        Serial.println(sensor_location);
      }
    }
      
};


class Switch{
  private:
    int switch_nr = 0;
    int pin_nr = 0;
    String switch_function = "";
    String switch_state = OFF;
    bool status_changed = false;

  public:
    Switch(int switch_number = 0, int pin_number = 0, String switch_func = "") 
      : switch_nr(switch_number),pin_nr(pin_number), switch_function(switch_func){ // Constructor
      switch_nr = switch_number;
      pin_nr = pin_number;
      switch_function = switch_func;
      pinMode(pin_nr, OUTPUT);
      digitalWrite(pin_nr, LOW);
    }

    bool switch_on(){
        digitalWrite(pin_nr, HIGH);
        switch_state = ON;
        return true;
    }
    
    bool switch_off(){
        digitalWrite(pin_nr, LOW);
        switch_state = OFF;
        return true;
    }

    bool getSwitchState(){
      return switch_state;
    }
};

/********************************( Definieren der Objekte )********************************/                          
//RELAIS relais;

//Create msg-object
  Message msg;

void setup() {
  Serial.begin(9600);
  //Serial.println("DHT11 Testprogramm");

  

  // Output PIN für 3D Drucker
  pinMode(RELAIS1_PIN, OUTPUT);
  pinMode(RELAIS2_PIN, OUTPUT);
  pinMode(RELAIS3_PIN, OUTPUT);
  pinMode(RELAIS4_PIN, OUTPUT);

  // Activate Relais (PIN LOW --> Relais turned on)
  digitalWrite(RELAIS1_PIN, LOW);
  //digitalWrite(RELAIS2_PIN, LOW);
  //digitalWrite(RELAIS3_PIN, LOW);
  //digitalWrite(RELAIS4_PIN, LOW);

  Sensor dht[6] = { Sensor(), //Dummy for non existing Sensor 0
                 Sensor(1, DHT1_PIN, DHT22, "Main-Box"), 
                 Sensor(2, DHT2_PIN, DHT22, "Filament-Box"), 
                 Sensor(3, DHT3_PIN, DHT22, "Outer-Box"),
                 Sensor(4, DHT4_PIN, DHT22),
                 Sensor(5, DHT5_PIN, DHT22)
               };
  
  //for(int i =1; i<=sizeof(dht);i++){
  //  dht[i].begin();
  //}

  // Interrupt PIN
  pinMode(interruptPin, INPUT_PULLUP);
  // Lock for falling edge on interrupt PIN and call alarm
  attachInterrupt(digitalPinToInterrupt(interruptPin), alarm, FALLING);
}

void loop() {
  
  if (Serial.available()) {
    msg.read_msg();
    
    // Serial.print("Empfangene Nachricht: ");
    // Serial.println(received_msg);
  }

  //get_relais_command(received_msg,&relais);
  //set_relais(&relais);
  //get_dht_command(received_msg);
  //send_dht_data(dht_data_request);
  
  // relais[relais[0].relais_nr].status_changed = false;

  // Reset last message
  //*received_msg = '\0';
}
