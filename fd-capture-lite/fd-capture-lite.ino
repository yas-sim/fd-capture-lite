// Options: 0=Normal, 1=Test (read only 4 tracks)
#define TEST_MODE     (1)



byte g_media_max_track = 40;
byte g_num_step_pulse = 1;

// 16bit Timer1 (TC1) : FDD RD signal input capture
// ATmega328P pin assignment
//  ICP1 = 14pin (PB0 (ICP1/CLKO/PCINT0))  -> IO8 on Arduino expansion connector

// Arduino pin assignment
//
// The 'RD' signal from FDD must be connected to Arduino D8 port. This program
// requires input capture function on 16-bit timer 1. The input port for the 
// input capture on timer 1 is D8.
//
// Other signals can be assigned to any digital ports except D0 and D1. 
// (D0 and D1 are used for serial communication (USART))
//
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
#define FDD_RD        (8) /* PB1/ICP1 ** DO NOT MODIFY ** */
#define FDD_SIDE1     (5) /* PD5 */

#define FDD_HEAD_LOAD (3) /* PD3 */
#define FDD_M_ON      (4) /* PD4 */

bool fdd_is_trk00(void) {
  return digitalRead(FDD_TRK00)==LOW?true:false;
}

bool fdd_step(void) {
  for(byte i=0; i<g_num_step_pulse; i++) {
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
  digitalWrite(FDD_SIDE1, side_sel==0?HIGH:LOW);
}

inline bool fdd_index(void) {
  return digitalRead(FDD_INDEX)==0?true:false;
}

inline void fdd_wait_index(void) {
  while(fdd_index()==true);
  while(fdd_index()==false);
}

void init_io(void) {
  Serial.begin(2e6);              // USART 2Mbps

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
  // b7   = ICNC1 - Input capture 1 noise canceller (0=Disable)
  // b6   = ICES1 - Input capture 1 edge select (0=Falling, 1=Rising)
  // b2:0 = Clock select (001=CLKio/1 (no prescaler), 010=CLKio/8, 011=CLKio/64, 100=CLKio/256, 101=CLKio/1024)
  TCCR1B = 0b00000001; // | (1<<ICNC1) | (1<<ICES1);
  TCCR1C = 0b00000000;
  // Timer1 interrupt
  TIMSK1 = 0b00000000;
}

// data_rate = 0:500Kbps (2D/2DD), 1:1Mbps (2HD)
void read_track(byte cell_ofst=0, byte data_rate=0) {
  byte ic_val = 0;
  byte bit_buf = 0;
  byte bit_cnt = 0;
  byte quantized = 0;

  byte prev_idx = 0;
  byte index_passed = 0;

  // Memo:
  // 2D/2DD Plus period = 4us/6us/8us, 2HD=2us/3us/4us
  //
  // Arduino Timer 1 clock setting = 16MHz (1:1)
  // 2D/2DD Cell size = 32
  //   (2us=32count) 4us=64count, 6us=96count, 8us=128count
  //         1             2            3           4
  // 2HD Cell size = 16
  //   (2us=16count) 4us=32count, 6us=48count, 8us=64count
  //         1             2            3           4
  // 
  //
  // Actual TCNT1 (input capture) measurement (2D)
  // Peak1=54count, Peak2=84count, Peak3=118count
  //    64-54=10        96-86=10     128-118=10
  //
  // Actual TCNT1 (input capture) measurement (2D)
  // Peak1=22count, Peak2=40count, Peak3=56count
  //    32-22=10        40-32=8     64-56=8
  //
  //  2 => '01', 3 => '001', 4 => '0001'
  //
  // ; TCNT1 count might be smaller than expected ideal counts in 8 to 10 counts. (Due to TCNT1 clear delay in SW)
  if(cell_ofst==0) {
    if(data_rate==0)  cell_ofst = 2+32/2;     // 2D/2DD
    else              cell_ofst = 2+16/2;     // 2HD
  }

  Serial.print("==");
  Serial.flush();
  delay(50);

  if(data_rate==0) {
    // Data rate == 0 (2D/2DD)
    // INDEX=IO6 == PD6
    asm volatile(
      "clr %[v_bit_buf]"                "\n\t"
      "ldi %[v_bit_cnt],0xff"           "\n\t"
      "clr %[v_index_passed]"           "\n\t"

      "ldi r20,lo8(bitptn_%=)"          "\n\t"
      "ldi r21,hi8(bitptn_%=)"          "\n\t"

      "cli"                             "\n\t"  // Disable all interrupts

      // Wait for the index hole
      "L_IDX0_%=:"                      "\n\t"
      "sbis %[io_PIND],%[bit_index]"    "\n\t"
      "rjmp L_IDX0_%="                  "\n\t"
      "L_IDX1_%=:"                      "\n\t"
      "sbic %[io_PIND],%[bit_index]"    "\n\t"
      "rjmp L_IDX1_%="                  "\n\t"  // Index hole detected

      // Clear TCNT1
      "sts %[io_TCNT1H],__zero_reg__"   "\n\t"
      "sts %[io_TCNT1L],__zero_reg__"   "\n\t"
      "clr r18"                         "\n\t"  // prev_ICR1L = 0

      "sbi %[io_TIFR1],%[bit_ICF1]"     "\n\t"  // Clear ICF1

      "L_MAIN_LOOP_%=:"                 "\n\t"

      // Wait for input capture flag 1 on timer 1
      "L_WAIT_FOR_ICF1_%=:"             "\n\t"
      "sbis %[io_TIFR1],%[bit_ICF1]"    "\n\t"  // TIFR1.ICF1
      "rjmp L_WAIT_FOR_ICF1_%="         "\n\t"

      "lds %[v_ic_val],%[io_ICR1L]"     "\n\t"
      "sub %[v_ic_val],r18"             "\n\t"  // ic_val = curr_ICR1L - prev_ICR1L
      "lds r18,%[io_ICR1L]"             "\n\t"  // prev_ICR1L = ICR1L
      "sbi %[io_TIFR1],%[bit_ICF1]"     "\n\t"  // Clear ICF1

      // Quantize captured value (val+v_cell_ofst)/cell_size
      "add %[v_ic_val],%[v_cell_ofst]"  "\n\t"
      "lsr %[v_ic_val]"                 "\n\t"
      "swap %[v_ic_val]"                "\n\t"  // swap ~= right shift for 4bits
      "andi %[v_ic_val],0x0f"           "\n\t"
      "add %[v_bit_cnt],%[v_ic_val]"    "\n\t"  // bit_cnt += quantized_val

      "L_USART_loop_%=:"                "\n\t"
      "cpi %[v_bit_cnt],6"              "\n\t"
      "brcs L_skip_usart%="             "\n\t"  // if bitcnt >= 6, then output the bit buffer to USART

      // Encode to a printable charactor
      "ldi r17,0x20"                    "\n\t"
      "add r17,%[v_bit_buf]"            "\n\t"

      // Wait for UDRE0 (USART0 data register empty)
      "L_WAIT_UDRE0_%=:"                "\n\t"
      "lds r16,%[io_UCSR0A]"            "\n\t"  // x==UCSR0A
      "sbrs r16,%[bit_UDRE0]"           "\n\t"
      "rjmp L_WAIT_UDRE0_%="            "\n\t"

      "sts %[io_UDR0],r17"              "\n\t"  // y==UDR0, output encoded data to USART
      "subi %[v_bit_cnt],6"             "\n\t"  // bit_cnt -= 6
      "clr %[v_bit_buf]"                "\n\t"  // bit_buf = 0
      "rjmp L_USART_loop_%="            "\n\t"

      "L_skip_usart%=:"                 "\n\t"  // Make 1<<bit_cnt value
      "movw r30,r20"                    "\n\t"  // x(r31,r30) <= r21,r20 (bit pattern table)
      "add r30,%[v_bit_cnt]"            "\n\t"
      "adc r31,__zero_reg__"            "\n\t"
      "lpm r17,z"                       "\n\t"  // get bit pattern from a table
      "or %[v_bit_buf],r17"             "\n\t"  // bit_buf |= 1<<bit_cnt

      "sbic %[io_PIND],%[bit_index]"    "\n\t"  // #1/2
      "ori %[v_index_passed],0x01"      "\n\t"  // #1
      "andi %[v_index_passed],0x01"     "\n\t"  // #1
      "breq L_MAIN_LOOP_%="             "\n\t"  // #1/2
      "sbic %[io_PIND],%[bit_index]"    "\n\t"  // #1/2
      "rjmp L_MAIN_LOOP_%="             "\n\t"  // #2
      "rjmp L_EXIT_%="                  "\n\t"
      "bitptn_%=:"                      "\n\t"
      ".byte 0x01,0x02,0x04,0x08,0x10,0x20,0x20,0x20"  "\n\t"
      ".byte 0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20"  "\n\t"
      "L_EXIT_%=:"                      "\n\t"
      "sei"                             "\n\t"  // Enable all interrupts
      : [v_bit_buf]   "+r" (bit_buf),
        [v_bit_cnt]   "+r" (bit_cnt),
        [v_ic_val]    "+r" (ic_val),
        [v_index_passed]  "+r" (index_passed)
      : [io_PIND]   "I" (_SFR_IO_ADDR(PIND)), 
        [io_TIFR1]  "I" (_SFR_IO_ADDR(TIFR1)),
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
      : "r16", "r17", "r18",
        "r20", "r21",
        "r30", "r31"
      );
  } else {
    // Data rate == 1 (2HD)
    // INDEX=IO6 == PD6
    asm volatile(
      "clr %[v_bit_buf]"                "\n\t"
      "ldi %[v_bit_cnt],0xff"           "\n\t"
      "clr %[v_index_passed]"           "\n\t"

      "ldi r20,lo8(bitptn_%=)"          "\n\t"
      "ldi r21,hi8(bitptn_%=)"          "\n\t"

      "cli"                             "\n\t"  // Disable all interrupts

      // Wait for the index hole
      "L_IDX0_%=:"                      "\n\t"
      "sbis %[io_PIND],%[bit_index]"    "\n\t"
      "rjmp L_IDX0_%="                  "\n\t"
      "L_IDX1_%=:"                      "\n\t"
      "sbic %[io_PIND],%[bit_index]"    "\n\t"
      "rjmp L_IDX1_%="                  "\n\t"  // Index hole detected

      // Clear TCNT1
      "sts %[io_TCNT1H],__zero_reg__"   "\n\t"
      "sts %[io_TCNT1L],__zero_reg__"   "\n\t"
      "clr r18"                         "\n\t"  // prev_ICR1L = 0

      "sbi %[io_TIFR1],%[bit_ICF1]"     "\n\t"  // Clear ICF1

      "L_MAIN_LOOP_%=:"                 "\n\t"

      // Wait for input capture flag 1 on timer 1
      "L_WAIT_FOR_ICF1_%=:"             "\n\t"
      "sbis %[io_TIFR1],%[bit_ICF1]"    "\n\t"  // TIFR1.ICF1
      "rjmp L_WAIT_FOR_ICF1_%="         "\n\t"

      "lds %[v_ic_val],%[io_ICR1L]"     "\n\t"
      "sub %[v_ic_val],r18"             "\n\t"  // ic_val = curr_ICR1L - prev_ICR1L
      "lds r18,%[io_ICR1L]"             "\n\t"  // prev_ICR1L = ICR1L
      "sbi %[io_TIFR1],%[bit_ICF1]"     "\n\t"  // Clear ICF1

      // Quantize captured value (val+v_cell_ofst)/cell_size
      "add %[v_ic_val],%[v_cell_ofst]"  "\n\t"
      "swap %[v_ic_val]"                "\n\t"  // swap ~= right shift for 4bits
      "andi %[v_ic_val],0x07"           "\n\t"
      "add %[v_bit_cnt],%[v_ic_val]"    "\n\t"  // bit_cnt += quantized_val

      "L_USART_loop_%=:"                "\n\t"
      "cpi %[v_bit_cnt],6"              "\n\t"
      "brcs L_skip_usart%="             "\n\t"  // if bitcnt >= 6, then output the bit buffer to USART

      // Encode to a printable charactor
      "ldi r17,0x20"                    "\n\t"
      "add r17,%[v_bit_buf]"            "\n\t"

      // Wait for UDRE0 (USART0 data register empty)
      "L_WAIT_UDRE0_%=:"                "\n\t"
      "lds r16,%[io_UCSR0A]"            "\n\t"  // x==UCSR0A
      "sbrs r16,%[bit_UDRE0]"           "\n\t"
      "rjmp L_WAIT_UDRE0_%="            "\n\t"

      "sts %[io_UDR0],r17"              "\n\t"  // y==UDR0, output encoded data to USART
      "subi %[v_bit_cnt],6"             "\n\t"  // bit_cnt -= 6
      "clr %[v_bit_buf]"                "\n\t"  // bit_buf = 0
      "rjmp L_USART_loop_%="            "\n\t"

      "L_skip_usart%=:"                 "\n\t"  // Make 1<<bit_cnt value
      "movw r30,r20"                    "\n\t"  // x(r31,r30) <= r21,r20 (bit pattern table)
      "add r30,%[v_bit_cnt]"            "\n\t"
      "adc r31,__zero_reg__"            "\n\t"
      "lpm r17,z"                       "\n\t"  // get bit pattern from a table
      "or %[v_bit_buf],r17"             "\n\t"  // bit_buf |= 1<<bit_cnt

      "sbic %[io_PIND],%[bit_index]"    "\n\t"  // #1/2
      "ori %[v_index_passed],0x01"      "\n\t"  // #1
      "andi %[v_index_passed],0x01"     "\n\t"  // #1
      "breq L_MAIN_LOOP_%="             "\n\t"  // #1/2
      "sbic %[io_PIND],%[bit_index]"    "\n\t"  // #1/2
      "rjmp L_MAIN_LOOP_%="             "\n\t"  // #2
      "rjmp L_EXIT_%="                  "\n\t"
      "bitptn_%=:"                      "\n\t"
      ".byte 0x01,0x02,0x04,0x08,0x10,0x20,0x20,0x20"  "\n\t"
      ".byte 0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20"  "\n\t"
      ".byte 0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20"  "\n\t"
      "L_EXIT_%=:"                      "\n\t"
      "sei"                             "\n\t"  // Enable all interrupts
      : [v_bit_buf]   "+r" (bit_buf),
        [v_bit_cnt]   "+r" (bit_cnt),
        [v_ic_val]    "+r" (ic_val),
        [v_index_passed]  "+r" (index_passed)
      : [io_PIND]   "I" (_SFR_IO_ADDR(PIND)), 
        [io_TIFR1]  "I" (_SFR_IO_ADDR(TIFR1)),
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
      : "r16", "r17", "r18",
        "r20", "r21",
        "r30", "r31"
      );
  }
}


void rt_test(void) 
{
  byte ic_val = 0;
  byte quantized = 0;
  byte prev_ICR1L = 0;

  byte buf[128];

  for(int i=0; i<128; i++) buf[i]=0;

  noInterrupts();
  fdd_wait_index();
  TCNT1 = 0;
  prev_ICR1L = 0;
  TIFR1 = 1<<ICF1;                         // Clear ICF1
  while(true) {
    while((TIFR1 & (1<<ICF1))==0) ;        // Input Captuer interrupt request flag
    ic_val = ICR1L;                        // Read captured time (ignore high byte)
    ic_val -= prev_ICR1L;
    prev_ICR1L = ICR1L;
    TIFR1 = 1<<ICF1;                       // Clear ICF1
    if(++buf[ic_val>>1]==255) break;
  }
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


// Read floppy disk and make a histogram of pulse-period
// to check software latency and actual pulse period.
//
// This routine uses the identical inline-asm code as 
// the 'read_track' routine.
//
// "byte buffer[]" must have 128 elements.
void get_histogram(byte buffer[]) {
  for(int i=0; i<128; i++) buffer[i] = 0;

  byte ic_val;

  asm volatile(
    "movw r22,%[v_buffer]"            "\n\t"

    "cli"                             "\n\t"  // Disable all interrupts

    // Clear TCNT1
    "sts %[io_TCNT1H],__zero_reg__"   "\n\t"
    "sts %[io_TCNT1L],__zero_reg__"   "\n\t"
    "clr r18"                         "\n\t"

    "sbi %[io_TIFR1],%[bit_ICF1]"     "\n\t"  // Clear ICF1

    "L_MAIN_LOOP_%=:"                 "\n\t"

    // Wait for input capture flag 1 on timer 1
    "L_WAIT_FOR_ICF1_%=:"             "\n\t"
    "sbis %[io_TIFR1],%[bit_ICF1]"    "\n\t"  // TIFR1.ICF1
    "rjmp L_WAIT_FOR_ICF1_%="         "\n\t"

    "lds %[v_ic_val],%[io_ICR1L]"     "\n\t"
    "sub %[v_ic_val],r18"             "\n\t"  // ic_val = curr_ICR1L - prev_ICR1L
    "lds r18,%[io_ICR1L]"             "\n\t"  // prev_ICR1L = ICR1L
    "sbi %[io_TIFR1],%[bit_ICF1]"     "\n\t"  // Clear ICF1

    "movw r26,r22"                    "\n\t"  // r27:r26 = xreg
    "lsr %[v_ic_val]"                 "\n\t"
    "add r26,%[v_ic_val]"             "\n\t"
    "adc r27,__zero_reg__"            "\n\t"  // xreg += (v_ic_val>>1)
    "ld r16,x"                        "\n\t"  // r16 = buff[v_ic_val>>1]
    "cpi r16,255"                      "\n\t"  
    "breq L_EXIT_%="                  "\n\t"  // if one_of_the_element reaches to 255, exit
    "inc r16"                         "\n\t"  // buff[v_ic_val>>1]++
    "st x,r16"                        "\n\t"
    "rjmp L_MAIN_LOOP_%="             "\n\t"

    "L_EXIT_%=:"                      "\n\t"
    "sei"                             "\n\t"
    : [v_ic_val]  "+r" (ic_val)
    : [io_TIFR1]  "I" (_SFR_IO_ADDR(TIFR1)),
      [io_TCNT1H] "M" (_SFR_MEM_ADDR(TCNT1H)), 
      [io_TCNT1L] "M" (_SFR_MEM_ADDR(TCNT1L)),
      [io_ICR1H]  "M" (_SFR_MEM_ADDR(ICR1H)),
      [io_ICR1L]  "M" (_SFR_MEM_ADDR(ICR1L)),
      [bit_ICF1]  "I" (ICF1),
      [v_buffer] "e" ((uint16_t)buffer)
    : "r16", "r18",
      "r22", "r23", 
      "r26", "r27"    // regX
    );
}

// "byte buf[]" must have 128 elements.
void disp_histogram(byte buffer[]) {
    for(int i=0; i<128; i++) {
    Serial.print(i<<1);
    Serial.print(" ");
    byte val = buffer[i];
    for(int j=0; j<val; j++) {
      Serial.print("#");
    }
    Serial.println();
  }
}

void smooth_histogram(byte inbuf[], byte outbuf[]) {
  for(int i=0+2; i<128-2; i++) {
    outbuf[i] = (
      inbuf[i-2]*1 +
      inbuf[i-1]*2 +
      inbuf[i  ]*4 +
      inbuf[i+1]*2 +
      inbuf[i+2]*1 ) / (1+2+4+2+1);
  }
}

void find_3_peaks_from_histogram(byte histogram[], byte peaks[])
{
  byte prev_peak_val = 255;

  for(int i=0; i<3; i++) peaks[i] = 0;

  for(int i=0; i<3; i++) {
    byte peak_val = 0;
    byte peak_pos = 0;  
    for(int j=0+1; j<128-1; j++) {
      byte curr_val = histogram[j];
      if(histogram[j-1]<=curr_val && histogram[j+1]<=curr_val) {
        if(curr_val>peak_val && curr_val<prev_peak_val) {
          peak_pos = j*2;
          peak_val = curr_val;
        }
      }
    }
    peaks[i] = peak_pos;
    prev_peak_val = peak_val;
  }
}

// data_rate 0=500Kbps (2D/2DD), 1=1Mbps (2HD)
void check_data_cell_size_and_estimate_offset(byte *ofst, byte *data_rate) {
  byte histogram[128];
  byte filtered[128];
  byte peaks[3];
  byte cell_size;
  byte capture_offset;

  for(int i=0; i<128; i++) histogram[i] = filtered[i] = 0;

  Serial.println(F("@@Checking data pulse condition of the floppy disk."));
  int start_over_count = 0;
  while(true) {
    if(start_over_count == 1) {
      Serial.print(F("@@    Failed. Starting over."));
    }
    if(start_over_count > 0) {
      Serial.print(".");
      Serial.flush();
    }
    start_over_count++;
    get_histogram(histogram);
    smooth_histogram(histogram, filtered);

#if 0
    disp_histogram(histogram);
    while(true) ;
#endif
    find_3_peaks_from_histogram(filtered, peaks);
    if(peaks[0]==0 || peaks[1]==0 || peaks[2]==0) {
      continue; // start over
    }
    cell_size = ((peaks[1]-peaks[0]) + (peaks[2]-peaks[1]))/2;
    capture_offset = cell_size*2 - peaks[0];

    if(capture_offset>(32*1.5)) {
      continue; // start over
    }
    break;
  }
  Serial.println();

  Serial.print(F("@@    Peaks of pulse-to-pulse time = "));
  for(int i=0; i<3; i++) {
    Serial.print(peaks[i]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.print(F("@@    Estimated cell size = "));
  Serial.println(cell_size);

  Serial.print(F("@@    Estimated input capture offset = "));
  Serial.println(capture_offset);

  byte recommended_offset = capture_offset + cell_size/2;
  Serial.print(F("@@    Recommended total offset = "));
  Serial.println(recommended_offset);

  *ofst = recommended_offset; 
  *data_rate = (cell_size) <= (16+8) ? 1:0;   // 1=1Mbps (2HD), 0=500Kbps (2D/2DD)

  Serial.print(F("@@    Estimated media density = "));
  switch(*data_rate) {
    case 0:  Serial.println(F("2D/2DD"));      break;
    case 1:  Serial.println(F("2HD"));         break;
    default: Serial.println(F("**UNKNOWN**")); break;
  }
}

// Checks whether the drive type is 2D or 2DD/2HD.
//   Move head to track 44 and step-out until TRK00 
//   gets asserted. The step-out count implies the 
//   drive type.
byte check_fdd_type(void) {
  g_num_step_pulse = 1;

  Serial.println(F("@@Detecting FDD type."));

  fdd_restore();

  int trk=0;
  for(trk=0; trk<44; trk++) {
    fdd_step_in();
  }

  trk = 0;
  while(fdd_is_trk00()==false) {
    fdd_step_out();
    if(trk++ > 50) break;
  }
  //Serial.print("@@");
  //Serial.println(trk);
  byte fdd_type;
  Serial.print(F("@@    FDD type = "));
  if (trk>42) {
    fdd_type = 1;
    Serial.println(F("2DD/2HD (80 tracks)"));
    if(g_media_max_track == 40) {
      g_num_step_pulse = 2;       // 2D media in a 2DD/2HD drive
    } else {
      g_num_step_pulse = 1;       // 2DD/2HD media in a 2DD/2HD drive
    }
  } else {
    fdd_type = 0;
    Serial.println(F("2D (40 tracks)"));
    g_num_step_pulse = 1;         // 2D media in a 2D drive
  }

  return fdd_type;
}

byte g_offset = 0;
byte g_data_rate = 0;

void setup() {
  init_io();

  fdd_restore();
  fdd_side(0);

#if 0
  rt_test();
  while(true) ;
#endif

  while(Serial.find("+++")==false) ;    // wait for '+++'
  String param;
  param = Serial.readString();
  param.trim();
  if(param=="2DD" || param=="2HD") {
    g_media_max_track = 80;
  } else {
    g_media_max_track = 40;
  }

  Serial.println(F("@@** FD-CAPTURE-LITE"));

  byte fdd_type = check_fdd_type();

#if 0
  byte buf[128];
  get_histogram(buf);
  while(true) ;
#endif

#if 0
  rt_test();
  while(true) ;
#endif

  Serial.print(F("@@Number of step pulse(s) per track = "));
  Serial.println(g_num_step_pulse);

  check_data_cell_size_and_estimate_offset(&g_offset, &g_data_rate);
}

void loop() {
  int max_track = 0;

#if TEST_MODE==1
  max_track = 4;
#else
  max_track = g_media_max_track;
#endif

  Serial.println("++START");
  Serial.flush();

  delay(500);

  for(byte trk=0; trk<max_track; trk++) {
    fdd_side(0);
    read_track(g_offset, g_data_rate);
    Serial.println();

    fdd_side(1);
    read_track(g_offset, g_data_rate);
    Serial.println();

    fdd_step_in();
  }

  delay(500);

  Serial.println("++END");
  Serial.flush();

  while(true) ;
}
