#define FLP_2D        (0)
#define FLP_2DD       (1)
#define FLP_2HD       (2)

#define FLP_DR_500K   (0)
#define FLP_DR_1M     (1)


// To determine data rate (bit rate)
#define MEDIA_TYPE FLP_2D

// "2D", "2DD", "2HD"
// To determine which single or double step-pulse to use.
#define FDD_TYPE FLP_2HD

// 0=Normal, 1=Test
#define TEST_MODE     (0)


#if FDD_TYPE==FLP_2DD || FDD_TYPE==FLP_2HD
#define FDD_TRACK_NUM (80)
#else
#define FDD_TRACK_NUM (40)
#endif

#if MEDIA_TYPE==FLP_2DD || MEDIA_TYPE==FLP_2HD
#define MEDIA_TRACK_NUM (80)
#else
#define MEDIA_TRACK_NUM (40)
#endif

#if MEDIA_TRACK_NUM==80 && FDD_TRACK_NUM==160
#define STEP_PULSE (2)
#else
#define STEP_PULSE (1)
#endif

#if MEDIA_TYPE==FLP_2HD
#define DATA_RATE FLP_DR_1M
#else
#define DATA_DATE FLP_DR_500K
#endif

#if MEDIA_TRACK_NUM > FDD_TRACK_NUM
#error Media type and FDD type mismatch.
#endif



// 16bit Timer1 (TC1) : FDD RD signal input capture
// ATmega328P pin assignment
//  ICP1 = 14pin (PB0 (ICP1/CLKO/PCINT0))  -> IO8 on Arduino expansion connector

// Arduino pin assignment
// IO2 = STEP      (FD-20)
// IO3 = HEAD LOAD (FD-4)
// IO4 = M_ON      (FD-16)
// IO5 = SIDE1     (FD-32)
// IO6 = INDEX     (FD-8)
// IO7 = TRK00     (FD-26)
// IO8 = RD        (FD-30)
// IO9 = DIR       (FD-18)


#define FDD_INDEX     (6) /* PD6 */
#define FDD_DIR       (9) /* PB1 */
#define FDD_STEP      (2) /* PD2 */
#define FDD_TRK00     (7) /* PD7 */
#define FDD_RD        (8) /* PB1/ICP1 */
#define FDD_SIDE1     (5) /* PD5 */

#define FDD_HEAD_LOAD (3) /* PD3 */
#define FDD_M_ON      (4) /* PD4 */

#define LED_BUILTIN (13)

bool fdd_is_trk00(void) {
  return digitalRead(FDD_TRK00)==0?true:false;
}

bool fdd_step(void) {
  for(byte i=0; i<STEP_PULSE; i++) {
    digitalWrite(FDD_STEP, LOW);
    delay(5);
    digitalWrite(FDD_STEP, HIGH);
    delay(5);
  }
}

bool fdd_step_in(void) {
  digitalWrite(FDD_DIR, LOW);
  fdd_step();
}

bool fdd_step_out(void) {
  digitalWrite(FDD_DIR, HIGH);
  fdd_step();
}

void fdd_restore(void) {
  while(fdd_is_trk00()==false) {
    fdd_step_out();
  }
}

void fdd_side(uint8_t side_sel) {
  digitalWrite(FDD_SIDE1, side_sel==0?1:0);
}

inline bool fdd_index(void) {
  return digitalRead(FDD_INDEX)==0?true:false;
}

inline void fdd_wait_index(void) {
  while(fdd_index()==true);
  while(fdd_index()==false);
}

void init_io(void) {
  Serial.begin(2e6);              // 2Mbps

  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(FDD_DIR, HIGH);
  digitalWrite(FDD_STEP, HIGH);
  digitalWrite(FDD_SIDE1, HIGH);
  digitalWrite(FDD_HEAD_LOAD, LOW);
  digitalWrite(FDD_M_ON, LOW);

  pinMode(FDD_INDEX,     INPUT_PULLUP);
  pinMode(FDD_TRK00,     INPUT_PULLUP);
  pinMode(FDD_RD,        INPUT_PULLUP);
  pinMode(FDD_DIR,       OUTPUT);
  pinMode(FDD_STEP,      OUTPUT);
  pinMode(FDD_SIDE1,     OUTPUT);
  pinMode(FDD_HEAD_LOAD, OUTPUT);
  pinMode(FDD_M_ON,      OUTPUT);

  UCSR0B |= 0b00001000;       // TXEN=1

  bitWrite(ACSR, ACIC, 0);    // Disable analog comparator input capture
  TCCR1A = 0b00000000;
  // TCCR1B
  // b7   = Input capture 1 noise canceller (0=Disable)
  // b6   = Input capture 1 edge select (0=Falling, 1=Rising)
  // b2:0 = Clock select (001=CLKio/1 (no prescaler), 010=CLKio/8, 011=CLKio/64, 100=CLKio/256, 101=CLKio/1024)
  TCCR1B = 0b00000001;
  TCCR1C = 0b00000000;
  // TImer1 interrupt
  TIMSK1 = 0b00000000;
}



void read_track(void) {
  byte ic_val = 0;
  byte bit_buf = 0;
  byte bit_cnt = 0;
  byte quantized = 0;
  bool index_passed = false;

  byte cell_ofst;
#if DATA_DATE == FLP_DR_500K
  cell_ofst = -8;       // 2D
#else
  cell_ofst = 0;        // 2HD
#endif

  // INDEX=IO6 == PD6
  asm volatile(
    "clr %[v_bit_buf]"                "\n\t"
    "ldi %[v_bit_cnt],0xff"           "\n\t"
    "clr %[v_index_passed]"           "\n\t"

    "cli"                             "\n\t"

    // Wait for the index hole
    "L_IDX0_%=:"                      "\n\t"
    "sbis %[io_pind],%[bit_index]"    "\n\t"
    "rjmp L_IDX0_%="                  "\n\t"
    "L_IDX1_%=:"                      "\n\t"
    "sbic %[io_pind],%[bit_index]"    "\n\t"
    "rjmp L_IDX1_%="                  "\n\t"  // Index hole detected

    // Clear TCNT1
    "sts %[io_TCNT1H],__zero_reg__"   "\n\t"
    "sts %[io_TCNT1L],__zero_reg__"   "\n\t"

    "sbi %[io_tifr1],%[bit_ICF1]"     "\n\t"  // Clear ICF1

    "L_MAIN_LOOP_%=:"                 "\n\t"

    // Wait for input capture flag 1 on timer 1
    "L_WAIT_FOR_ICF1_%=:"             "\n\t"
    "sbis %[io_tifr1],%[bit_ICF1]"    "\n\t"  // TIFR1.ICF1
    "rjmp L_WAIT_FOR_ICF1_%="         "\n\t"

    // Clear TCNT1
    "sts %[io_TCNT1H],__zero_reg__"   "\n\t"
    "sts %[io_TCNT1L],__zero_reg__"   "\n\t"

    "sbi %[io_tifr1],%[bit_ICF1]"     "\n\t"  // Clear ICF1
    "lds %[v_ic_val],%[io_ICR1L]"     "\n\t"  // Read ICR1L

    // Quantize captured value (val+v_cell_ofst)/div
    "add %[v_ic_val],%[v_cell_ofst]"  "\n\t"
#if DATA_DATE == FLP_DR_500K
    "lsr %[v_ic_val]"                 "\n\t"
#endif
    "swap %[v_ic_val]"                "\n\t"  // swap == right shift for 4bits
    "andi %[v_ic_val],0x03"           "\n\t"

    "inc %[v_ic_val]"                 "\n\t"
    "add %[v_bit_cnt],%[v_ic_val]"    "\n\t"  // bit_cnt += quantized_val
    "cpi %[v_bit_cnt],6"              "\n\t"
    "brlo L_skip_%="                  "\n\t"  // if bitcnt < 6, then skip

    // Encode to a printable charactor
    "ldi r17,0x20"                    "\n\t"
    "add %[v_bit_buf],r17"            "\n\t"
    // Wait for UDRE0 (USART0 data register empty)
    "L_WAIT_UDRE0_%=:"                "\n\t"
    "lds r16,%[io_UCSR0A]"            "\n\t"  // x==UCSR0A
    "sbrs r16,%[bit_UDRE0]"           "\n\t"
    "rjmp L_WAIT_UDRE0_%="            "\n\t"
    "sts %[io_UDR0],%[v_bit_buf]"     "\n\t"  // y==UDR0, output encoded data to USART
    "subi %[v_bit_cnt],6"             "\n\t"  // bit_cnt -= 6
    "clr %[v_bit_buf]"                "\n\t"  // bit_buf = 0

    "L_skip_%=:"                      "\n\t"  // Make 1<<bit_cnt value
    "mov r16,%[v_bit_cnt]"            "\n\t"
    "clr r17"                         "\n\t"
    "sec"                             "\n\t"  // Set carry flag
    "L_SHIFT0_%=:"                    "\n\t"
    "rol r17"                         "\n\t"
    "cpi r16,0"                       "\n\t"
    "breq L_SHIFT1_%="                "\n\t"
    "dec r16"                         "\n\t"
    "rjmp L_SHIFT0_%="                "\n\t"
    "L_SHIFT1_%=:"                    "\n\t"
    "or %[v_bit_buf],r17"             "\n\t"  // bit_buf |= 1<<bit_cnt

    "sbic %[io_pind],%[bit_index]"    "\n\t"
    "ori %[v_index_passed],0x01"      "\n\t"
    "andi %[v_index_passed],0x01"     "\n\t"
    "breq L_MAIN_LOOP_%="             "\n\t"
    "sbic %[io_pind],%[bit_index]"    "\n\t"
    "rjmp L_MAIN_LOOP_%="             "\n\t"
    "sei"                             "\n\t"
    : [v_bit_buf]   "+r" (bit_buf),
      [v_bit_cnt]   "+r" (bit_cnt),
      [v_ic_val]    "+r" (ic_val),
      [v_index_passed] "+r" (index_passed)
    : [io_pind]   "I" (_SFR_IO_ADDR(PIND)), 
      [io_tifr1]  "I" (_SFR_IO_ADDR(TIFR1)),
      [io_TCNT1H] "M" (_SFR_MEM_ADDR(TCNT1H)), 
      [io_TCNT1L] "M" (_SFR_MEM_ADDR(TCNT1L)),
      [io_ICR1H]  "M" (_SFR_MEM_ADDR(ICR1H)),
      [io_ICR1L]  "M" (_SFR_MEM_ADDR(ICR1L)),
      [io_UCSR0A] "M" (_SFR_MEM_ADDR(UCSR0A)),
      [io_UDR0]   "M" (_SFR_MEM_ADDR(UDR0)),
      [bit_index] "I" (6),
      [bit_ICF1]  "I" (ICF1),
      [bit_UDRE0] "I" (UDRE0),
      [v_cell_ofst] "r" (cell_ofst)
    : "r16", "r17", "r18"
    );
}


void rt_test(void) 
{
  byte ic_val = 0;
  byte quantized = 0;

  byte buf[128];
  word buf_cnt = 0;

  for(int i=0; i<128; i++) buf[i]=0;

  noInterrupts();
  fdd_wait_index();
  TCNT1 = 0;
  TIFR1 = 1<<ICF1;                         // Clear ICF1
  do {
    while((TIFR1 & (1<<ICF1))==0) ;        // Input Captuer interrupt request flag
    TCNT1 = 0;
    ic_val = ICR1L;                        // Read captured time (ignore high byte)
    TIFR1 = 1<<ICF1;                       // Clear ICF1
    buf[ic_val>>1]++;
    buf_cnt++;
  } while(buf_cnt<2000);
  interrupts();

  for(int i=0; i<128; i++) {
    Serial.print(i<<1);
    Serial.print(" ");
    byte dt = buf[i];
    for(int j=0; j<dt; j++) {
      Serial.print("#");
    }
    Serial.println();
  }

  while(true);
}

void check_data_cell_size(void) {
  noInterrupts();
  byte histogram[128];
  byte filtered[128];
  for(int i=0; i<128; i++) histogram[i] = filtered[i] = 0;

  TCNT1 = 0;
  TIFR1 |= 1<<ICF1;
  for(int i=0; i<512; i++) {
    while((TIFR1 & (1<<ICF1))==0) ;
    TCNT1 = 0;
    byte val = ICR1L;
    TIFR1 |= 1<<ICF1;
    histogram[val/2]++;
  }
  interrupts();
#if 0
  for(int i=0; i<128; i++) {
    Serial.print(i*2);
    Serial.print(" ");
    Serial.println(histogram[i]);
  }
#endif

  for(int i=0+2; i<128-2; i++) {
    filtered[i] = (
      histogram[i-2]*1 +
      histogram[i-1]*2 +
      histogram[i  ]*4 +
      histogram[i+1]*2 +
      histogram[i+2]*1 ) / (1+2+4+2+1);
  }
#if 0
  for(int i=0; i<128; i++) {
    Serial.print(i*2);
    Serial.print(" ");
    Serial.println(filtered[i]);
  }
#endif

  // find 3 peaks
  byte peaks[3];
  byte prev_peak_val = 255;
  for(int i=0; i<3; i++) {
    byte peak_val = 0;
    byte peak_pos = 0;  
    for(int j=0+1; j<128-1; j++) {
      byte curr_val = filtered[j];
      if(filtered[j-1]<=curr_val && filtered[j+1]<=curr_val) {
        if(curr_val>peak_val && curr_val<prev_peak_val) {
          peak_pos = j*2;
          peak_val = curr_val;
        }
      }
    }
    peaks[i] = peak_pos;
    prev_peak_val = peak_val;
  }
  Serial.print("@@Peaks = ");
  for(int i=0; i<3; i++) {
    Serial.print(peaks[i]);
    Serial.print(" ");
  }
  Serial.println();
  byte cell_size = ((peaks[1]-peaks[0]) + (peaks[2]-peaks[1]))/2;
  Serial.print("@@Estimated cell size = ");
  Serial.println(cell_size);
}



void setup() {
  init_io();

  fdd_restore();
  fdd_side(0);
}


void loop() {

  Serial.println("** FD-CAPTURE-LITE");
  Serial.println("++START");
  Serial.flush();
  delay(500);

#if 0
  rt_test();
  while(true) ;
#endif

  check_data_cell_size();

#if TEST_MODE==1
  for(byte trk=0; trk<4; trk++) {
#else
  for(byte trk=0; trk<MEDIA_TRACK_NUM; trk++) {
#endif
    fdd_side(0);
    digitalWrite(LED_BUILTIN, HIGH);
    read_track();
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println();

    fdd_side(1);
    digitalWrite(LED_BUILTIN, HIGH);
    read_track();
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println();

    fdd_step_in();
  }
  delay(500);
  Serial.println("++END");
  Serial.flush();
  while(true);
}
