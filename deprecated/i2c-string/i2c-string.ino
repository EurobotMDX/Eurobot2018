#include <Wire.h>
#include <Arduino.h>
#define SLAVE_ADDRESS 0x12

int flag_int_to_send_to_PI = 0;

int flag_int_received_from_PI = 0;

char data_to_send_PI[] = "hello PI"; 

String data_recieved_from_pi = "";

void setup() {

   Wire.begin(SLAVE_ADDRESS);

   Wire.onReceive(receiveData);

   Wire.onRequest(sendData);

   Serial.begin(9600);

   flag_int_to_send_to_PI = 1;


}

void loop() {
}



void receiveData(int byteCount) {

    while(Wire.available()) 
    {
        flag_int_received_from_PI = Wire.read();

        if(flag_int_received_from_PI == 1)
        {
           Serial.println("PI Wants data[].");

           flag_int_to_send_to_PI= 3;


        }  

        if(flag_int_received_from_PI == 3)
        {
            Serial.println("PI Wants To say its ok.");

            //reading a string from pi: Here (How ?)
              while( Wire.available())
              {
                data_recieved_from_pi += (char)Wire.read();

              }

              Serial.print("Data Received From PI:");

              Serial.println(data_recieved_from_pi);

              data_recieved_from_pi = "";
        }

    }
}

void sendData() {

    if(flag_int_to_send_to_PI == 1)
    {
        Serial.println("Conversation begin : sending 1 to PI");

        Wire.write(flag_int_to_send_to_PI);
    }

    if(flag_int_to_send_to_PI == 3) { 

        Wire.write(data_to_send_PI);
    }

    flag_int_to_send_to_PI = 0;

}
