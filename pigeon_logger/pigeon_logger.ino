
/*
 * FW Ver: 1.06
 * HW Ver: 5_x
 * Date: 13-Oct-2017
 * 
 * Data_0 -> PC0 PIN_23
 * Data_1 -> PC1 PIN_24
 * Data_2 -> PC2 PIN_25
 * Data_3 -> PC3 PIN_26
 * Data_4 -> PC4 PIN_27
 * Data_5 -> PC5 PIN_28
 * 
 * Data_6 -> PB0 PIN_12
 * Data_7 -> PB1 PIN_14
 * 
 * LedGreenOut -> PD0 PIN_30 
 * BuzzerPinOut -> PD1 PIN_31
 * 
 * StrobePinIn -> PD2 PIN_32/INT0
 * StrobePinOut -> PD5 PIN_9
 * 
 * BusyPinIn -> PD7 PIN_11
 * BusyPinOut -> PD6 PIN_10
 * 
 * AckPinOut -> PD4 PIN_2
 * 
 * STROBE - this signal is sniffed
 * 
 * BUSY -   this signal is sniffed and controlled ( ??mirrored?? )
 *          check timing diagram at https://mediatoget.blogspot.com/2011/02/lpt-printer-parallel-port-part-1.html
 *          to understand how to control BUSY line.
 *          Master will not send new STROBE until BUSY is asserted by uC.
 * ACKNOWLEDGE - this signal is sourced to the PC to replace ACK from printer. Long BUSY duration w/o ACK can be interpret by PC as a some kind of problem with printing.
 *               Optionally can be removed later if ACK signal can be sourced directly from printer to the PC.
 */

// include the SD library:
#include <SPI.h>
#include <SD.h>
#include <util/delay.h>

#define F_CPU 16000000UL  // 16 MHz

// Define log file name depends on board No.
#define LOG_FILE_NAME "log_X10D.akm"

// Firmware and hardware version
#define FW_VER "10D"
#define HW_VER "2_27"

// BuzzerPinOut is shared with TX line for serial.
#define BUZZER_ENABLE

void InitSDcard(void);

// Received data.
volatile byte Data = 0;

volatile byte IdleState = 0;

volatile char DataCharacter; // Variable to store received data converted to ASCII.
volatile byte DataReadyFlag = 0; // Received data ready flag.

volatile unsigned long SimpleChecksum = 0; // Accumulate received bytes values as a simple checksum.
volatile byte CardWasMovedOut = 0; // 1 = indicates that SD card was moved out in runtime.
volatile byte CheckCnt = 0; // Counts try before software reset will be forced.

// Port pins declaration.
#ifdef BUZZER_ENABLE
  const int BuzzerPinOut = 1; //PD1 PIN_31
#endif

const int StrobePinIn = 2; //PD2 PIN_4/INT0
const int StrobePinOut = 5; //PD5 PIN_9 
const int BusyPinIn = 7; //PD7 PIN_11
const int BusyPinOut = 6; //PD6 PIN_10
const int AckPinOut = 4; //PD4 PIN_2
const int LedGreenOut = 0; //PD0 PIN_30
const int CardDetectIn = 3; //PD3 PIN_1

const int chipSelect = 10; // SPI SD CARD chip select PIN. PB2/PIN_14


// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;

File myFile;

// Timer1 is used to count interval time between following print requests. If timer expire, flush data to SD, close file and reopen it to be ready for next printing w/o power cycle whole device.
void SetupTimer1(void);

//*******************************************************************
// Timer1 configuration part.
void SetupTimer1() {
  TCCR1A = 0;
  TCCR1B = 0;
  //OCR1A = 468750;
  OCR1A = 15623;
  TCCR1B |= (1 << WGM12);
  TIMSK1 |= (1 << OCIE1A);
  TCCR1B |= (1 << CS12) | (1 << CS10);
}

//*******************************************************************
// Timer1 ISR
ISR (TIMER1_COMPA_vect) { 
  if(IdleState != 0) {
    IdleState++; 
  }
}
//*******************************************************************
// Main interrupt routine to handle start data 
// transmission and capture byte of data.
ISR(INT0_vect, ISR_NAKED) {
    asm volatile(
    "    SBI %[busy_out],6            \n"  // set 1 on PD6 pin BusyPinOut to speed up setting it into the high state.
    "    rjmp INT0_vect_part_2  \n"  // go to part 2
    :: [busy_out] "I" (_SFR_IO_ADDR(PIND)));
}


ISR(INT0_vect_part_2) {
  // Force HIGH on BusyPinOut to hold-on printing next char until newly arrived will be served.
  // As long as BusyPinOut is HIGH, new data will be not send to print.
  // BusyPinOut need to be set LOW after complete sending data to SD CARD. 
  
  PORTD &= ~(1<<PD5); // Direct access to StrobePinOut pin. Set it LOW and start printing on printer.
  delayMicroseconds(10);  // Wait 10 microseconds and hold on StrobeOut in LOW state (active).   
  PORTD |= (1<<PD5); // Release StrobeOut pin - set it high.
  
  // Get data from the pins and store them in to the single byte.
  Data = 0;
  Data = (PINB & B00000011) << 6; //Bits [7:6]
  Data = Data | (PINC & B00111111); //Bits [5:0]
  SimpleChecksum = SimpleChecksum + long(Data);
  // Set notification flag DataReadyFlag to indicates in loop() that valid data were recived.
  DataReadyFlag = 1;
}

//*******************************************************************
// Interrupt service routine for INT1. SD Card Detect routine.
// This IRQ can be fired only if SD card is ejected in runtime.
ISR(INT1_vect) {
  // Disable INT0 interrupt to hold printing - strobe to printer can't be generated.
  cli();
  byte CheckCardDetectIn = 0;
  EIMSK &= ~(1 << INT0);   // disable INT0 interrupt  
  PORTD |= (1 << PD6); // Direct access to PD6 pin BusyPinOut to speed up setting it into the high state.
  //PORTD |= (1 << PD5);// Disable StrobePinOut - set it HIGH, to block any accidential printing.
  PORTD &= ~(1<<PD0); //Turn OFF LED
  // Disable IdleState to prevent generating IRQ from Timer.
  IdleState = 0;
  DataReadyFlag = 0;
  SimpleChecksum = 0;
  // Turn ON buzzer.
  #ifdef BUZZER_ENABLE
    PORTD |= (1<<PD1);
  #endif
  
  // Wait until card is injected back into the socket - LOW on CardDetectIn.
  // Initialize SD card from the beggining.
  // Poke CardDetectIn. If it is HIGH then SD card is not inserted. Wait here until SD card will be inserted.
  CheckCardDetectIn = (PIND & B00001000);
  while(CheckCardDetectIn == B00001000){
    // TurnOFF/ON LED
    for (byte loop_cnt = 0 ; loop_cnt < 5 ; loop_cnt++) {
      PORTD &= ~(1<<PD0); 
      #ifdef BUZZER_ENABLE
        PORTD &= ~(1<<PD1);
      #endif
      _delay_ms(200);
      
      PORTD |= (1<<PD0); 
      #ifdef BUZZER_ENABLE
        PORTD |= (1<<PD1);
      #endif
      _delay_ms(200);
    }
    _delay_ms(500);
    CheckCardDetectIn = (PIND & B00001000); 
  }
  
  PORTD &= ~(1<<PD0); //Turn OFF LED
  software_Reset();
}

//*******************************************************************
// Restarts program from beginning but does not reset 
// the peripherals and registers.
void software_Reset(){
  cli();
  PORTD |= (1 << PD6); // Direct access to PD6 pin BusyPinOut to speed up setting it into the high state.
  PORTD |= (1 << PD5);// Disable StrobePinOut - set it HIGH, to block any accidential printing.
  // Disable IdleState to prevent generating IRQ from Timer.
  IdleState = 0;
  DataReadyFlag = 0;
  SimpleChecksum = 0;
  PORTD &= ~(1<<PD0); // Turn OFF LED
  // Turn ON buzzer.
  #ifdef BUZZER_ENABLE
    //digitalWrite(BuzzerPinOut, HIGH);
    PORTD |= (1<<PD1);
  #endif
  
  asm volatile ("  jmp 0");
  //sei();
}   

//*******************************************************************
void setup() {
  // StrobePinOut configure as output and set it HIGH
  DDRD = DDRD | B00100000 ;
  PORTD |= (1<<PD5); // StrobeOut pin - set it high.
  
  pinMode(LedGreenOut, OUTPUT);
  PORTD &= ~(1<<PD0); //Turn OFF LED

  #ifdef BUZZER_ENABLE
    pinMode(BuzzerPinOut, OUTPUT);
    // Turn ON buzzer until all setup operation will be completed. This is information to outside to 
    // not start printing before setup complete.
    digitalWrite(BuzzerPinOut, HIGH);
  #endif

  #ifdef BUZZER_ENABLE
    digitalWrite(BuzzerPinOut, HIGH);
  #endif

  // Some extra delay in case of re-insert SD card and forcing software_Reset().
  _delay_ms(200);
  CardWasMovedOut = 0;
 
  // The Data Direction Register of Port B. Port B bits [7:6] configured as inputs.
  
  // Configure as inputs
  // Port B bits [1:0] configured as inputs.
  DDRB = DDRB & B11111100 ;
  // Placeholder. Pullup's might be not required.
  // Set pull-up's
  //PORTB = (1<<PB1)|(1<<PB0);
  // Configure as inputs. The Data Direction Register of Port C. Port C bits [5:0] configured as inputs.
  DDRC = DDRC & B11000000 ;
  // Placeholder. Pullup's might be not required.
  // Set pull-up's
  //PORTC = (1<<PC5)|(1<<PC4)|(1<<PC3)|(1<<PC2)|(1<<PC1)|(1<<PC0);
  
  
  //pinMode(StrobePinIn, INPUT_PULLUP);
  pinMode(StrobePinIn, INPUT);
  //pinMode(StrobePinOut, OUTPUT);
  pinMode(BusyPinIn, INPUT);
  pinMode(BusyPinOut, OUTPUT);
  pinMode(AckPinOut, OUTPUT);
  pinMode(CardDetectIn, INPUT); // PULLUP might be not needed as PCB has pullup.
  
  // Force LOW on BUSY to enable printing. 
  digitalWrite(BusyPinOut, LOW);
  digitalWrite(AckPinOut, HIGH);
  
  // Set initial value. Data  not ready yet to move to SD CARD.
  DataReadyFlag = 0;
  
  // -----------------------------------------
  // SD CARD initialization part.
  // -----------------------------------------

  // we'll use the initialization code from the utility libraries
  // since we're just testing if the card is working!

  // Wait until card is injected back into the socket - LOW on CardDetectIn.
  // Initialize SD card from the beggining.
  // Poke CardDetectIn. If it is HIGH then SD card is not inserted. Wait here until SD card will be inserted.
  CardWasMovedOut = 0;
  while((digitalRead(CardDetectIn) == HIGH)){
    CardWasMovedOut = 1;
    // DEBUG: TurnOFF/ON LED
    for (byte i = 0 ; i<10 ; i++) {
      #ifdef BUZZER_ENABLE
        digitalWrite(BuzzerPinOut, LOW);
      #endif
      PORTD &= ~(1<<PD0); _delay_ms(200);
      #ifdef BUZZER_ENABLE
        digitalWrite(BuzzerPinOut, HIGH);
      #endif
      PORTD |= (1<<PD0); _delay_ms(200);
    }
    _delay_ms(500);
    // END DEBUG
  }

  // Force software reset in case if SD card wasn't present at sturtup time.
  if (CardWasMovedOut == 1){
    PORTD &= ~(1<<PD0);
    CardWasMovedOut=0;
    software_Reset();
  }
  
  // FATAL ERROR
  CheckCnt = 0;
  while(!card.init(SPI_HALF_SPEED, chipSelect)) {
    #ifdef BUZZER_ENABLE
      digitalWrite(BuzzerPinOut, HIGH);
    #endif      
    digitalWrite(LedGreenOut, HIGH);   // sets the LED on
    delay(500);                  // waits for a number of miliseconds      
    #ifdef BUZZER_ENABLE
      digitalWrite(BuzzerPinOut, LOW);
    #endif
    digitalWrite(LedGreenOut, LOW);    // sets the LED off
    delay(500);                  // waits for a number of miliseconds

    CheckCnt++;
    if(CheckCnt > 10) { 
      CheckCnt = 0;
      software_Reset(); // reset 
    }
  }

  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  // FATAL ERROR. Could not find FAT16/FAT32 partition.Make sure you've formatted the card.
  CheckCnt = 0;
  while(!volume.init(card)) {
    #ifdef BUZZER_ENABLE
      digitalWrite(BuzzerPinOut, HIGH);
    #endif      
    digitalWrite(LedGreenOut, HIGH);   // sets the LED on
    delay(500);                  // waits for a number of miliseconds      
    #ifdef BUZZER_ENABLE
      digitalWrite(BuzzerPinOut, LOW);
    #endif
    digitalWrite(LedGreenOut, LOW);    // sets the LED off
    delay(500);                  // waits for a number of miliseconds

    CheckCnt++;
    if(CheckCnt > 10) { 
      CheckCnt = 0;
      software_Reset(); // reset 
    }
  }
  // End of SD CARD initialization part.
  // -----------------------------------------

  // -----------------------------------------
  // Create new or open existed file XXXX.akm
  // -----------------------------------------
  CheckCnt = 0;
  while(!SD.begin(chipSelect)) {
    #ifdef BUZZER_ENABLE
      digitalWrite(BuzzerPinOut, HIGH);
    #endif      
    digitalWrite(LedGreenOut, HIGH);   // sets the LED on
    delay(500);                  // waits for a number of miliseconds      
    #ifdef BUZZER_ENABLE
      digitalWrite(BuzzerPinOut, LOW);
    #endif
    digitalWrite(LedGreenOut, LOW);    // sets the LED off
    delay(500);                  // waits for a number of miliseconds

    CheckCnt++;
    if(CheckCnt > 10) { 
      CheckCnt = 0;
      software_Reset(); // reset 
    }
  }
  
  myFile = SD.open(LOG_FILE_NAME, FILE_WRITE);
  
  
  // if the file opened okay, write to it, otherwise FATAL ERROR
  CheckCnt = 0;
  while(!myFile) {
    #ifdef BUZZER_ENABLE
      digitalWrite(BuzzerPinOut, HIGH);
    #endif      
    digitalWrite(LedGreenOut, HIGH);   // sets the LED on
    delay(500);                  // waits for a number of miliseconds      
    #ifdef BUZZER_ENABLE
      digitalWrite(BuzzerPinOut, LOW);
    #endif
    digitalWrite(LedGreenOut, LOW);    // sets the LED off
    delay(500);                  // waits for a number of miliseconds

    CheckCnt++;
    if(CheckCnt > 10) { 
      CheckCnt = 0;
      software_Reset(); // reset 
    }
  }

  // Write start of LOG indicator at a TOP of logged record. It need to be added to differentiate case for SD card hot plug out.
  myFile.println("");  
  myFile.print("!-!-md5="); myFile.print(SimpleChecksum); 
  myFile.print("-!-!FW="); myFile.print(FW_VER);
  myFile.print("-!-!HW="); myFile.print(HW_VER); 
  myFile.print("-!-!SD="); myFile.print(CardWasMovedOut);
  myFile.print("-!-!START_LOG");
  myFile.print("-!-!");
  myFile.println("");
  myFile.flush();
  _delay_ms(200);
  // -----------------------------------------
  // End Create new or open existed file test.txt
  // -----------------------------------------

  // Turn ON LED to indicate that device starts correctly.
  digitalWrite(LedGreenOut, HIGH);   // sets the LED on

  // Turn OFF buzzer. Setup is completed and printing can be started.
  #ifdef BUZZER_ENABLE
    digitalWrite(BuzzerPinOut, LOW);
  #endif
  
  SetupTimer1();
  // -----------------------------------------
  // Enable INT1 IRQ for CardDetectIn at the end of setup.
  // -----------------------------------------
  EICRA |= (1 << ISC10);  //01 -> Any logical change on INT1 generates an interrupt request.
  EICRA &= ~(1 << ISC11);  //01 -> Any logical change on INT1 generates an interrupt request.
  EIMSK |= (1 << INT1);   // enable INT1 interrupt
  EIFR |= (1 << INTF1);
  // -----------------------------------------
  // Enable INT0 IRQ for StrobePinIn at the end of setup.
  // -----------------------------------------
  EICRA &= ~(1 << ISC00);  // sense falling edge on the INT0 pin
  EICRA |= (1 << ISC01);  // sense falling edge on the INT0 pin
  EIMSK |= (1 << INT0);   // enable INT0 interrupt
  EIFR |= (1 << INTF0);
 
  sei();     // Enable global interrupts by setting global interrupt enable bit in SREG
}

void loop() {

  if(IdleState >= 10) {
     IdleState = 0;
     
     myFile.println("");
     myFile.print("!-!-md5="); myFile.print(SimpleChecksum); 
     myFile.print("-!-!FW="); myFile.print(FW_VER);
     myFile.print("-!-!HW="); myFile.print(HW_VER); 
     myFile.print("-!-!SD="); myFile.print(CardWasMovedOut);
     myFile.print("-!-!END_LOG");
     myFile.print("-!-!");
     myFile.println("");
     myFile.flush();
     
     myFile.close();
     
     #ifdef BUZZER_ENABLE
       digitalWrite(BuzzerPinOut, HIGH);
     #endif
       digitalWrite(LedGreenOut, LOW);   // sets the LED on
       delay (500);
     #ifdef BUZZER_ENABLE
       digitalWrite(BuzzerPinOut, LOW);
     #endif
     digitalWrite(LedGreenOut, HIGH);   // sets the LED off
     
     myFile = SD.open(LOG_FILE_NAME, FILE_WRITE);

     // Reinitialize checksum variable for new data set.
     SimpleChecksum = 0;
     
     if (!myFile) { 
       // FATAL ERROR
       #ifdef BUZZER_ENABLE
         digitalWrite(BuzzerPinOut, HIGH);
       #endif
       cli();

       CheckCnt = 0;
       while(1) {
         digitalWrite(LedGreenOut, HIGH);   // sets the LED on
         delay(300);                  // waits for a number of miliseconds
         digitalWrite(LedGreenOut, LOW);    // sets the LED off
         delay(300);                  // waits for a number of miliseconds

         CheckCnt++;
         if(CheckCnt > 10) { 
           CheckCnt = 0;
           software_Reset(); // reset 
         }
       } 
     }

    myFile.println("");  
    myFile.print("!-!-md5="); myFile.print(SimpleChecksum); 
    myFile.print("-!-!FW="); myFile.print(FW_VER);
    myFile.print("-!-!HW="); myFile.print(HW_VER); 
    myFile.print("-!-!SD="); myFile.print(CardWasMovedOut);
    myFile.print("-!-!RE_START_LOG");
    myFile.print("-!-!");
    myFile.println("");
    myFile.flush();
    
    IdleState = 0;
  }
  
  if(DataReadyFlag == 1) {

    // -----------------------------------------
    // Put here routine for storing data on SD CARD
    // Routine for writing data to SD CARD need to 
    // be out of the INT0 ISR (StrobePinIn) routine getOneCharISR()
    // as some of the IRQ used by SD card routines
    // are not available inside of ISR routine getOneCharISR()
    // -----------------------------------------
    IdleState = 1;
    // For debug purpose, print on serial console received character.
    DataCharacter = char(Data);
    
    if (myFile) {
      myFile.print(DataCharacter);
    } 
    else {
      // FATAL ERROR
      cli();
      CheckCnt = 0;
      while(1) {
        #ifdef BUZZER_ENABLE
           digitalWrite(BuzzerPinOut, HIGH);
        #endif
        
        digitalWrite(LedGreenOut, LOW);   // sets the LED on
        _delay_ms(300);                  // waits for a number of miliseconds
        digitalWrite(LedGreenOut, HIGH);    // sets the LED off
        _delay_ms(300);                  // waits for a number of miliseconds

        CheckCnt++;
        if(CheckCnt > 10) { 
          CheckCnt = 0;
          software_Reset(); // reset 
        }
      } 
    }
    // -----------------------------------------
    // At this point routine for storing data on SD CARD need to be completed.
    // -----------------------------------------
    
    // Wait here until PRINTER sets BUSY line high. Real printer might be still busy and we can't enable next cher printing.    
    while((digitalRead(BusyPinIn))==HIGH) {
      ;
    }

    // Clear DataReadyFlag to enable it for next ISR execution.
    DataReadyFlag = 0;
    
    //digitalWrite(AckPinOut, LOW);
    PORTD &= ~(1<<PD4); // Direct access to AckPinOut pin. Set it LOW.
    delayMicroseconds(6);
    PORTD |= (1 << PD4);  // Direct access to AckPinOut pin. Set it HIGH.
    // If PRINTER is BUSY=0, send BUSY=LOW to the PC. This MUST be last operation in loop() to enable printing next char.
    delayMicroseconds(6);
    PORTD &= ~(1<<PD6); // Direct access to PD6 pin BusyPinOut to speed up setting it into the LOW state.
    
  } //if(DataReadyFlag == 1)
}


