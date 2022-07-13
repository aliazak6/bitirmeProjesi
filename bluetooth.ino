
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>


const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
SoftwareSerial BTserial(9,8); // SRX | STX
char readdata[50];
char x[3] = "X:\0";
char y[3] = "Y:\0";
void setup() {
  Serial.begin(9600);
  BTserial.begin(9600);
  pinMode(A1,OUTPUT);
  pinMode(A0,OUTPUT);
  pinMode(4,OUTPUT);
  
  digitalWrite(A1,50); 
  digitalWrite(A0,LOW); 
  digitalWrite(4,LOW); 
  lcd.begin(16, 2);
  // Print a message to the LCD.
  
  lcd.clear();
  lcd.print(x);
  delay(100);
  lcd.setCursor(0,1); // bottom left
  lcd.print(y);
}
int i = 0;
int a = 0;
void loop()
{
  delay(100);
 while (BTserial.available())
 {
      delay(10);
      char inChar=BTserial.read();
      readdata[a] = inChar;
      ++a;
      //readdata += inChar;,
          
       
 }      
        
        
      if (readdata[0] != '\0'){
        Serial.print(readdata);
        Serial.write("\n");
        lcd.setCursor(2, i);
        

       char * token = strtok(readdata, "-");

        while( token != NULL ) {

        lcd.setCursor(2, i);
        lcd.print(token);
        
        
        for (int j = a+1 ;j < 16 ; j++ ) {
          lcd.setCursor(a,i);
          lcd.write(32);
          }
        
        if (i == 0) i = 1;
        else if (i == 1) i = 0;
    
        token = strtok(NULL, "-");
        
       }
        
        
        //readdata = "";
        memset(&readdata[0], 0, sizeof(readdata));
        a = 0;

        
       
        
        }
        
}
