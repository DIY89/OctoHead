//#include <String.h>
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
    Sensor(int nr,int dht_pin, int type) : DHT(dht_pin,type){
      sensor_nr = nr;
      pin_nr = dht_pin;
      sensor_type = type; 
      //begin();
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
  
    void send_sensor_data_uart(){
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
      // Serial.print("Check DHT command: ");
      // Serial.println(command);
      if(command == "dht"){
        // Serial.println("Set dht data request: true");
        dht_data_request = true;
      }
    }
};


class Switch{
  private:
    int switch_nr=0;
    int pin_nr=0;
    String switch_state = OFF;
    bool status_changed = false;

  public:
    Switch(int switch_num, int pin_num){ // Constructor
      switch_nr = switch_num;
      pin_nr = pin_num;
      pinMode(pin_nr, OUTPUT);
      digitalWrite(pin_nr, LOW);
    }

    bool switch(String state){
      if(state == ON){
        digitalWrite(pin_nr, HIGH);
        switch_state = ON;
        return true;
      }else if(state == OFF){
        digitalWrite(pin_nr, LOW);
        switch_state = OFF;
        return true;
      }else{
         return false; //Return False in case of wrong input
      }
    }

    bool getSwitchState(){
      return switch_state;
    }
    
};

/********************************( Definieren der Objekte )********************************/                          
//RELAIS relais;

String received_msg;

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

  Sensor dht[6] = { Sensor(0, 0, 0), //Dummy for non existing Sensor 0
                    Sensor(1, DHT1_PIN, DHT_TYPE), 
                    Sensor(2, DHT2_PIN, DHT_TYPE), 
                    Sensor(3, DHT3_PIN, DHT_TYPE),
                    Sensor(4, DHT4_PIN, DHT_TYPE),
                    Sensor(5, DHT5_PIN, DHT_TYPE)
                  };
  
  //for(int i =1; i<=sizeof(dht);i++){
  //  dht[i].begin();
  //}

  // Interrupt PIN
  pinMode(interruptPin, INPUT_PULLUP);
  // Lock for falling edge on interrupt PIN and call alarm
  attachInterrupt(digitalPinToInterrupt(interruptPin), alarm, FALLING);
}

void read_data_uart(String &msg){
  while (Serial.available() > 0) {
    //Read the next available byte in the serial receive buffer
    msg = Serial.readStringUntil('\n');
  }
  msg.trim();
}

//void set_relais(RELAIS &relais){
//  // Serial.print("Status 0: ");
//  // Serial.println(relais[relais[0]->relais_nr]->status_changed);
//  if(relais[relais[0]->relais_nr]->status_changed == true){
//    
//    // Serial.print("Relais switched to: ");
//    // Serial.println(relais[relais[0]->relais_nr]->switched_on);
//    if(relais[relais[0]->relais_nr]->switched_on == true){
//      // Serial.print("Relais: ");
//      // Serial.print(relais[0]->relais_nr);
//      // Serial.println(" turned on");
//      relais_state = LOW;
//      // Serial.print("Status 2: ");
//      // Serial.println(relais[relais[0]->relais_nr]->status_changed);
//    }else if(relais[relais[0]->relais_nr]->switched_on == false){
//      // Serial.print("Relais: ");
//      // Serial.print(relais[0]->relais_nr);
//      // Serial.println(" turned off");
//      // Shutdown Printer on Fire Alarm (Output HIGH --> Relais turned off)
//      relais_state = HIGH;
//    }
//    switch(relais[0]->relais_nr){
//      case 1:
//        digitalWrite(RELAIS1_PIN, relais_state);
//      break;
//      case 2:
//        digitalWrite(RELAIS2_PIN, relais_state);
//      break;
//      case 3:
//        digitalWrite(RELAIS3_PIN, relais_state);
//      break;
//      case 4:
//        digitalWrite(RELAIS4_PIN, relais_state);
//      break;
//      default:
//      break;
//    }
//  }
//  relais[relais[0]->relais_nr]->status_changed = false;
//
//  // Serial.print("Status 3: ");
//  // Serial.println(relais.status_changed);
//}

//void get_relais_command(String command, RELAIS *relais) {
//  int FromIndex=0, ToIndex=0;
//  String relais_command, relais_num;
//  int relais_nr=0;
//
//  // Serial.print(command);
//
//  ToIndex = command.indexOf(":");
//  // Serial.print("ToIndex: ");
//  // Serial.println(ToIndex);
//  relais_num = command.substring(1,ToIndex);
//  
//  //trim(relais_nr);
//  relais_nr = atoi(relais_num.c_str());
//  relais[0]->relais_nr = relais_nr;
//  // Serial.print("Relais Nr: ");
//  // Serial.println(relais[0]->relais_nr);
//
//  FromIndex = command.indexOf(":") + 1;
//  // Serial.print("Index from");
//  // Serial.println(FromIndex);
//  ToIndex = command.length();
//  // Serial.print("length");
//  // Serial.println(ToIndex);
//
//  relais_command = command.substring(FromIndex,ToIndex);
//  relais_command.trim();
//
//  // Serial.print("FromIndex: ");
//  // Serial.println(FromIndex);
//  // Serial.print("ToIndex: ");
//  // Serial.println(ToIndex);
//
//  // Serial.println(relais_command);
//
//  if(relais_command == ON && relais[relais_nr]->switched_on == false){
//    // Serial.println("Switching Relais ON");
//    relais[relais_nr]->switched_on = true;
//    relais[relais_nr]->status_changed = true;
//    // Serial.print("Relais: ");
//    // Serial.print(relais_nr);
//    // Serial.print(" switched to: ");
//    // Serial.println(relais[relais_nr]->switched_on);
//  }else if(relais_command == OFF && relais[relais_nr]->switched_on == true){
//    // Serial.println("Switching Relais OFF");
//    relais[relais_nr]->switched_on = false;
//    relais[relais_nr]->status_changed = true;
//    // Serial.print("Relais: ");
//    // Serial.print(relais_nr);
//    // Serial.print(" switched to: ");
//    // Serial.println(relais[relais_nr]->switched_on);
//  }
//}



//void send_dht_data(bool req){
//  
//  if(req == true){
//    /* ---------------- DHT Sensor 1 ---------------- */                           
//    dht_1.sensor_nr = 1;
//    dht_1.temperatur = dht1.readTemperature();    // Lesen der Luftfeuchtigkeit und speichern in die Variable h1
//    dht_1.humidity = dht1.readHumidity(); // Lesen der Temperatur in °C und speichern in die Variable t1
//    send_sensor_data_uart(dht_1);
//
//
//    /* ---------------- DHT Sensor 2 ---------------- */
//    dht_2.sensor_nr = 2;
//    dht_2.temperatur = dht2.readTemperature();    // Lesen der Luftfeuchtigkeit und speichern in die Variable h2
//    dht_2.humidity = dht2.readHumidity(); // Lesen der Temperatur in °C und speichern in die Variable t2
//    send_sensor_data_uart(dht_2);
//
//
//    /* ---------------- DHT Sensor 3 ---------------- */
//    dht_3.sensor_nr = 3;
//    dht_3.temperatur = dht3.readTemperature();    // Lesen der Luftfeuchtigkeit und speichern in die Variable h3
//    dht_3.humidity = dht3.readHumidity(); // Lesen der Temperatur in °C und speichern in die Variable t3
//    send_sensor_data_uart(dht_3);
//
//
//    /* ---------------- DHT Sensor 4 ---------------- */
//    dht_4.sensor_nr = 4;
//    dht_4.temperatur = dht4.readTemperature();    // Lesen der Luftfeuchtigkeit und speichern in die Variable h4
//    dht_4.humidity = dht4.readHumidity(); // Lesen der Temperatur in °C und speichern in die Variable t4
//    send_sensor_data_uart(dht_4);
//
//    /* ---------------- DHT Sensor 5 ---------------- */
//    dht_5.sensor_nr = 5;
//    dht_5.temperatur = dht5.readTemperature();    // Lesen der Luftfeuchtigkeit und speichern in die Variable h5
//    dht_5.humidity = dht5.readHumidity(); // Lesen der Temperatur in °C und speichern in die Variable t5
//    send_sensor_data_uart(dht_5);
//    delay(2000);
//  }
//
//  dht_data_request = false;
//}

void loop() {
  
  if (Serial.available() > 0) {
    read_data_uart(received_msg);

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
