
int motorPin = 2;
boolean fallDetect = false; 
int RESET_BUTTON = 3;
int MANUAL_TRIGGER = 4;
int btVar = 0;
char btVarChar[] = " ";
boolean sendSuccess = false;
boolean sendFail = false;

int rumble = 0;
int count = 0;



void setup() {
   pinMode(motorPin, OUTPUT);
   attachInterrupt(digitalPinToInterrupt(RESET_BUTTON),resetVoid,CHANGE);//CHANGE may need to be replaced with LOW or FALLING depending on button logic
   pinMode(MANUAL_TRIGGER, INPUT);
   Serial.begin(9600);
   while (! Serial);
   Serial.println("Fall Detection Haptic Feedback Script");
}

void loop() {
   if (Serial.available() > 0) {
      //manual testing by serial input. this section is only used for testing
      char testByte = Serial.read();
      Serial.println(rumble);
      Serial.println(fallDetect);
      Serial.println(MANUAL_TRIGGER);
      Serial.println(RESET_BUTTON);
      Serial.println(btVar);
      Serial.println(sendFail);
      Serial.println(sendSuccess);
      
      if (testByte == 'f') {
        fallDetect = true;
      }
      else if (testByte == 'c'){
        //reset all triggers
        fallDetect = false;
        digitalWrite(RESET_BUTTON, LOW);
        digitalWrite(MANUAL_TRIGGER, LOW);
        btVar = 0;
        sendSuccess = false;
        sendFail = false;
      }
      else if (testByte == 'i'){
        //issues containing if again to print values, otherwise does nothing
      }
    }

    
    if (fallDetect == true){
      //type 1 rumble, a fall was detected
      rumble = 1;
      hapticType();
      //btVarChar = "A fall alert has been detected for the user.";
      //btVar = 1;
      //btVarCall();
    }
    else if (MANUAL_TRIGGER == HIGH) {
      //type 2 rumble, 
      rumble = 2;
      hapticType();
      //btVarChar = "A manual health alert has been issued by the user.";
      //btVar = 1;
      //btVarCall();
    }

    
    if (count >= 1){
      //repeats rumble until either reset is hit or successful send
      delay(15000);
      hapticType();
    }
    else if (count >= 4 && rumble == 4){
      delay(60000);
      hapticType();
    }
    
}

void hapticType(){
    if(rumble == 1){
      //indicate to user fall detected
      digitalWrite(motorPin, HIGH);
      delay(5000);
      digitalWrite(motorPin, LOW);
      count++;
      fallDetect = false;
    }
    else if (rumble == 2){
      //indicate to user manual trigger detected
      digitalWrite(motorPin, HIGH);
      delay(3000);
      digitalWrite(motorPin, LOW);
      delay(3000);
      digitalWrite(motorPin, HIGH);
      delay(3000);
      digitalWrite(motorPin, LOW);
      count++;
    }
    else if (rumble == 3){
      //indicate to user that alert was successfully reported over bluetooth
      for(int x = 0; x<4; x++){ 
        digitalWrite(motorPin, HIGH);
        delay(1000);
        digitalWrite(motorPin, LOW);
        delay(500);
      }
      count++;
    }
    else if (rumble == 4){
      //indicate to user that alert was UNsuccessfully reported 
      digitalWrite(motorPin, HIGH);
      delay(6000);
      digitalWrite(motorPin, LOW);
      delay(3000);
      digitalWrite(motorPin, HIGH);
      delay(6000);
      digitalWrite(motorPin, LOW);
      count++;
    }
}

/*void btVarCall(){
    while(sendSuccess == false && btVar == 1){
        if(sendFail == false && sendSuccess == false){
              //do nothing, wait for prompt to action
              Serial.println("Bluetooth notification start. Sending... ");
              
              //send btVarChar to bluetooth script              
        }
        else if(sendFail == true && sendSuccess == false){
              rumble = 4;
              hapticType();
              Serial.println("FAILURE in sending alert. Reattempting...");
              
              //perform bluetooth script again       
        }      
        else if(sendFail == false && sendSuccess == true){
              rumble = 3;
              hapticType();
              Serial.println("SUCCESS in sending alert.");
              delay(1000);
              
              //clear triggers
              fallDetect = false;
              sendSuccess = false;
              rumble = 0;
              btVar = 0;
        }
        else if(sendFail == true && sendSuccess == true){
              //error, or check that the send was truly successful
              Serial.println("ERROR in send receipt, or ERROR in script confirmation of send receipt. Check that phone has received alert.");
              sendFail = false;
              sendSuccess = false;
              btVar = 0;  //force exit of while loop
        } 
    }
}*/

void resetVoid(){
    //reset interrupt for all previous messages, clears vars & prevents bluetooth script from sending alert notif
    count = 0;
    rumble = 0;
    fallDetect = false;
    
    //cancel bluetooth send attempt
    
    //whileCheck = 1;    
    //sendSuccess = false;
    //sendFail = false;
}
