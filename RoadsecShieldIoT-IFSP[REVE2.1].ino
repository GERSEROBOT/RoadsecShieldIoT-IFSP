/*
*Programa teste do Shield IoT-IFSP [REVE 2.1]
*
*GERSE – Grupo de Estudos em Robótica e Sistemas Embarcados
*http://www.gerserobotica.com
*gerse.robot@gmail.com
*IFSP – Campus Guarulhos
*
*Desenvolvedor: Pedro Igor Borçatti
*
*novenbro de 2017
*/


#include <ESP8266WiFi.h>
#include <Wire.h>                        // BIBLIOTECA PARA I2C
#include <LiquidCrystal_I2C.h>           // BIBLIOTECA PARA LCD 16X2 I2C 
#include <SSD1306.h>                     // BIBLIOTECA PARA LDC OLED 128x64 BASEADO NO SSD1306
#include <SSD1306Wire.h>                 // BIBLIOTECA PARA LDC OLED 128x64 BASEADO NO SSD1306
#include <ESP8266HTTPClient.h>                       

LiquidCrystal_I2C lcd ( 0x27, 16, 2 );   // CONFIGURANDO LCD 16X2
SSD1306 display       ( 0x3c, 4, 5  );   // CONFIGURANDO LCD OLED

#define    A        12                   // LCD 7 SEGMENTO A OU LED A
#define    B        14                   // LCD 7 SEGMENTO B OU LED B
#define    C        0                    // LCD 7 SEGMENTO C OU LED C
#define    D        16                   // LCD 7 SEGMENTO D OU LED D
#define    E        2                    // LCD 7 SEGMENTO 3 OU LED E
#define    F        13                   // LCD 7 SEGMENTO F OU LED F
#define    G        15                   // LCD 7 SEGMENTO G OU LED G
#define    ADC      A0                   // ADC 0 POTENCIOMETRO OU BOTAO 1, 2 E 3


const char* ssid      = "XXXXXXXXXXXX";          // SSID (NOME) DA REDE WIFI
const char* password  = "XXXXXXXXXXXX";          // SENHA DA REDE WIFF

String      apiKey    = "XXXXXXXXXXXXXXXX";      // CHAVE DE ESCRIDA DO CANAL THINGSPEAK
const char* server    = "api.thingspeak.com";

WiFiClient client;

//*******************************************************************************************************************************************************************************//

void PiscaLed( unsigned short int DELAY ) // PISCA LED
{
    digitalWrite ( A,  LOW  );            // APAGAR  LED A
    delay        ( DELAY    );            // ESPERAR
    digitalWrite ( A,  HIGH );            // ACENDER LED A
    delay        ( DELAY    );            // ESPERAR
}

//*******************************************************************************************************************************************************************************//

void Display7Seg( unsigned short int DELAY )
{
    static  byte          NUM   = 0;  // NAVEGADOR DE INDICE
    const   boolean LCD7[10][7] =     // NÚMEROS DE 0 A 9 PARA LCD DE 7 SEGMENTOS 
             {
             // {A,B,C,D,E,F,G}
                {1,1,1,1,1,1,0},    // 0
                {0,1,1,0,0,0,0},    // 1
                {1,1,0,1,1,0,1},    // 2
                {1,1,1,1,0,0,1},    // 3
                {0,1,1,0,0,1,1},    // 4
                {1,0,1,1,0,1,1},    // 5
                {1,0,1,1,1,1,1},    // 6
                {1,1,1,0,0,0,0},    // 7
                {1,1,1,1,1,1,1},    // 8
                {1,1,1,1,0,1,1},    // 9
             // {A,B,C,D,E,F,G}
             };
     
   delay       ( DELAY            ); // ESPERAR
   digitalWrite( A,  LCD7[NUM][0] ); // ESCERVER NUMERO   
   digitalWrite( B,  LCD7[NUM][1] );
   digitalWrite( C,  LCD7[NUM][2] );
   digitalWrite( D,  LCD7[NUM][3] );
   digitalWrite( E,  LCD7[NUM][4] );
   digitalWrite( F,  LCD7[NUM][5] );
   digitalWrite( G,  LCD7[NUM][6] );

   if( NUM == 9 ) NUM = 0;           // ZERAR
   else NUM++;                       // CONTAR
   
}

//*******************************************************************************************************************************************************************************//

int analog( byte DELAY )
{
  unsigned short int  adcValue = analogRead ( ADC );  // AQUISICAO DO VALOR DE ADC
  Serial.print   ( "ADC = "      );                   // ESCREVER NA PORTA UART
  Serial.println ( adcValue      );                   // ESCRVER VALOR DO ADC NA PORTA UART
  delay          ( DELAY         );                   // ESPERAR
  return         ( adcValue      );                   // RETORNAR VALOR DO ADD
}

//*******************************************************************************************************************************************************************************//

// REFE: http://www.arduinoecia.com.br/2014/12/modulo-i2c-display-16x2-arduino.html
// REFE: http://blogmasterwalkershop.com.br/nodemcu/nodemcu-utilizando-com-display-lcd-16x2/
// REFE: https://howtoesp8266.com/connecting-i2c-iic-16x2-display-to-nodemcu-esp-12-using-lua/


void LCDinit()                             // INICIAR DISPLAY LCD 16X2
{
    lcd.init     ();                       // INICIAR DISPLAY LCD 16X2
    lcd.backlight();                       // BACKLIGHT (LUZ DE FINDO) HABILITADA
}

void lcd16x2( float VALUE )                // OPERAR LCD 16X2
{
    VALUE = (VALUE/1024)*3.3;              // CONVERTER VALOR DECIMAL DO ADC PARA TENSAO [V], MIN 0 | MAX 3.30B
    lcd.setCursor ( 0, 0               );  // POSICIONAR CURSOR LINHA 1 COLUNA 0
    lcd.print     ( "GERSE SHIELD IoT" );  // ESCREVE NO DISPLAY LCD
    lcd.setCursor ( 0, 1               );  // POSICIONAR CURSOR LINHA 2 COLUNA 0      
    lcd.print     ( "ADC= "            );  // ESCREVE NO DISPLAY LCD
    lcd.print     ( VALUE              );  // ESCREVE NO DISPLAY LCD
    lcd.print     ( "V      "          );  // ESCREVE NO DISPLAY LCD
    
}
//*******************************************************************************************************************************************************************************//

// REFE: https://github.com/squix78/esp8266-oled-ssd1306
// REFE: http://pedrominatel.com.br/pt/perifericos/oled-display-com-o-esp8266/


void LcdOledInit()                      // INICIAR DISPLAY LCD OLED
{
    display.init();                     // INICIAR DISPLAY LCD OLED
    display.clear();                    // LIMPAR  DISPLAY LCD OLED
}

void LcdOled(float Y)                   // OPERAR  DISPLAY LCD OLED
{
    static byte X = 0;                  // EIXO X VARIAVEL DINAMICA RESRVADA (LEBRA VALOR ANTERIO) 

    Y = int((Y/1024)*63);               // CONVERTER VALOR DECIMAL DO ADC PARA PIXEL, MIN 0 | MAX 63 
                   
    display.setPixel ( X ,Y );          // ESCREVER EM UM PIXEL DO DISPLAY LCD OLED
    display.display  ();                // Escreva o buffer na memória do visor

    if(X == 127)                        // MONITORAR ESTOUTO DE PIXEL HORISONTAL
    {
        display.clear();                // LIMPAR  DISPLAY LCD OLED
        X = 0;                          // LIMPAR VARIAVEL X
    }
    else X++;                           // DESLOCAR PIXEL X (HORIZONTAL)
}

//*******************************************************************************************************************************************************************************//

// REFE: http://nothans.com/measure-wi-fi-signal-levels-with-the-esp8266-and-thingspeak
// REFE: http://pedrominatel.com.br/pt/arduino/integrando-o-sensor-dht11-no-thingspeak-com-o-esp8266/
// REFE: https://www.filipeflop.com/blog/planta-iot-com-esp8266-nodemcu/


void IoTthingspeak(unsigned short int VALUE, unsigned short int DELAY)
{
    delay(DELAY);                     // IMPORTENE ESPERAR UM PERILDO GRANDE RECOMANDA SE 20s
    if (client.connect(server, 80))   // CONECTAR AO THINGSPEAK (TCP)
    {
        // CONSTRUIR PEDIDO DO API
        String body =  "field1=";
               body += String(VALUE); 
               
        // REALIZAR PEDIDO
        client.print("POST /update HTTP/1.1\n");
        client.print("Host: api.thingspeak.com\n");
        client.print("Connection: close\n");
        client.print("X-THINGSPEAKAPIKEY: " + apiKey + "\n");
        client.print("Content-Type: application/x-www-form-urlencoded\n");
        client.print("Content-Length: ");
        client.print(body.length());
        client.print("\n\n");
        client.print(body);
        client.print("\n\n");

  }
client.stop();                      // FEXAR CONEXAO TCP
}

//*******************************************************************************************************************************************************************************//
void setup()
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode      ( A, OUTPUT  );            // CONFIGURANDO PINO 12 COMO SAIDA
  pinMode      ( B, OUTPUT  );            // CONFIGURANDO PINO 14 COMO SAIDA
  pinMode      ( C, OUTPUT  );            // CONFIGURANDO PINO 0  COMO SAIDA
  pinMode      ( D, OUTPUT  );            // CONFIGURANDO PINO 16 COMO SAIDA
  pinMode      ( E, OUTPUT  );            // CONFIGURANDO PINO 2  COMO SAIDA
  pinMode      ( F, OUTPUT  );            // CONFIGURANDO PINO 13 COMO SAIDA
  pinMode      ( G, OUTPUT  );            // CONFIGURANDO PINO 15 COMO SAIDA

  digitalWrite ( A,  LOW    );            // ESCREVER 0 NO PINO 12
  digitalWrite ( B,  LOW    );            // ESCREVER 0 NO PINO 14
  digitalWrite ( C,  LOW    );            // ESCREVER 0 NO PINO 0
  digitalWrite ( D,  LOW    );            // ESCREVER 0 NO PINO 16
  digitalWrite ( E,  LOW    );            // ESCREVER 0 NO PINO 2
  digitalWrite ( F,  LOW    );            // ESCREVER 0 NO PINO 13
  digitalWrite ( G,  LOW    );            // ESCREVER 0 NO PINO 15
  
  Serial.begin ( 9600       );            // CONFIGURANDO UART
  WiFi.begin(ssid, password);             // CONECTAR WIFI

  Serial.println();                       // PULAR LINHA
  while( WiFi.status() != WL_CONNECTED )  // AGUARDAR CONEXAO
  {
    delay       ( 500 );
    Serial.print( "." );
  }
  Serial.println();                       // PULAR LINHA
  Serial.println(WiFi.localIP());         // IP LOCAL

  LCDinit();                              // INICIAR DISPLAY LCD 16X2
  LcdOledInit();                          // INICIAR DISPLAY LCD OLED
  
}

void loop() 
{
  unsigned short int adc  = analog(100);
  static byte        Pled = 0;
  static byte        Dis7 = 0;
  static byte        IoT  = 0;

  
  if(Dis7 == 10) 
  {
        Display7Seg( 1 );
        Dis7 = 0;
  }
  else  Dis7++;

  if(IoT == 200)
  {
        IoTthingspeak ( adc, 1 );
        IoT = 0;
  }
  else  IoT++;
  
    lcd16x2  ( adc  );
    LcdOled  ( adc  );   
//  PiscaLed ( 1000 );

  
}
