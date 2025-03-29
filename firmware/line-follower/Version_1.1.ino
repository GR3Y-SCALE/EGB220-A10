 //constants
 #define n_sens 8
 #define v0 128

 //ADC Channel MUX
 static char sens[n_sens] = {
  0b00000100,
  0b00000101,
  0b00000110,
  0b00000111,
  0b00100011,
  0b00100010,
  0b00100001,
  0b00100000
 };
 
 //Store results of each sensor
 int result[n_sens];

 int i = 0;
 int Kp = 255/7;

 ISR(ADC_vect){
    result[i] = ADCH;  //save reading of sensors

    //set motor speed based on reading
    OCR0A = 0;
    OCR0B = 0;
    if((result[3]<10) && (result[4]<10)){  //middle sensor is on track
      OCR0A = 255;
      OCR0B = 255;
    }//else if(result[i]<10){     //middle of robot is not on track
     // OCR0A = v0 - (i-3.5) * Kp;
      //OCR0B = v0 + (i-3.5) * Kp;
    //}

    i++;

    if(i == n_sens){
      i = 0;
    }


    ADMUX = (ADMUX & 0xE0) | sens[i] & 0x1F;
    ADCSRB = (ADCSRB & ~(1 << MUX5)) | ((sens[i] >> 5) << MUX5);

    ADCSRA |= (1<<ADSC);
  }



  int main() {

    //Timer 0 setup
    TCCR0A |= (1<<COM0A1) | (1<<COM0B1) | (1<<WGM01) | (1<<WGM00);
    TCCR0B |= (1<<CS01);
    DDRB |= (1<<PB7);
    DDRD |= (1<<PD0);

    //Setup ADC
    ADMUX |= (1<<REFS0) | (1<<ADLAR) | (1<<MUX2);
    ADCSRA |= (1<<ADEN) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
    ADCSRB |= (1<<ADHSM);

    //Start ADC 
    ADCSRA |= (1<<ADSC);

    //Setting initial speed of motor to zero
    OCR0A = 0;

    USBCON = 0;
    sei();

    
  }