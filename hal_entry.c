#include "hal_data.h"

#define BUFFER_SIZE 100
FSP_CPP_HEADER
void R_BSP_WarmStart(bsp_warm_start_event_t event);
FSP_CPP_FOOTER

#define ICU_SCI5_IRQn (IRQn_Type) 0


void printUART(const char *str)
{
    if (!str) return;
    while (*str)
    {
        uint8_t ch = (uint8_t)*str++;
        if (ch == '\n')
        {
            while (R_SCI5->SSR_b.TDRE == 0) { __NOP(); }
            R_SCI5->TDR_b.TDR = '\r';
        }
        while (R_SCI5->SSR_b.TDRE == 0) { __NOP(); }
        R_SCI5->TDR_b.TDR = ch;
    }
    while (R_SCI5->SSR_b.TEND == 0) { __NOP(); }
}

void UART5_Handler(void){
     /*
      *
      * */
}

bool uart_is_connected(void)
{
    uint32_t timeout = 50000; // n lần polling

    while (R_SCI5->SSR_b.TDRE == 0 && timeout--)
        __NOP();

    return (timeout > 0);
}

void uart_init (void){


    R_MSTP->MSTPCRB_b.MSTPB26 = 0;      // enable module SCI5
    R_SCI5->SCR = 0;                    // tắt Tx, Rx
    R_SCI5->SMR_b.CM = 0;               // chọn mode async
    R_SCI5->SCMR_b.CHR1 |= 1;           // CHR1 = 1, CHR = 0: 8 bit data
    R_SCI5->SMR_b.CHR = 0;
    R_SCI5->SMR_b.PE = 0;               // không dùng parity
    R_SCI5->SMR_b.STOP = 0;             // 1 stop bit
    R_SCI5->SMR_b.CKS = 0b01;           // n = 1
    R_SCI5->BRR_b.BRR = 80;             // N = 80
    R_SCI5->SCR_b.RE = 1;               // cho phép Rx
    R_SCI5->SCR_b.RIE = 1;              // Cho phép ngắt Rx
    R_SCI5->SCR_b.TE = 1;               // Cho phép Tx

    R_PMISC->PWPR_b.B0WI = 0; // Enable writable PFS
    R_PMISC->PWPR_b.PFSWE = 1;
    R_PFS->PORT[5].PIN[01].PmnPFS_b.PMR=1;              // Peripheral mode phải cấu hình thành chế độ ngoại vi để nó chạy được mode ngoại vi, như trong trường hợp này là cho SCI
    R_PFS->PORT[5].PIN[01].PmnPFS_b.PSEL=0b00101;
    R_PFS->PORT[5].PIN[02].PmnPFS_b.PMR=1;
    R_PFS->PORT[5].PIN[02].PmnPFS_b.PSEL=0b00101;

    R_SYSTEM->PRCR = (0xA5u<<8)|(1<<4);                     // mở khóa SAIELSRn
    R_CPSCU->ICUSARG_b.SAIELSRn &= ~((uint32_t)0b1);        // secure vùng ICU cho 3 handler
    NVIC->ITNS[0] &= ~((uint32_t)0b1);                  // secure vùng NVIC

    R_ICU->IELSR_b[0].IELS = 0x19E;
    NVIC_SetPriority(ICU_SCI5_IRQn, 3);
    NVIC_EnableIRQ(ICU_SCI5_IRQn);
    NVIC_SetVector(ICU_SCI5_IRQn, (uint32_t)&UART5_Handler);
}


double humidity_raw_data;
double temp_raw_data;
double temp_data;
double humidity_data;


int Value_1;
int Value_2;
int Value_3;
int Value_4;
int TRASH_VALUE;
int Temp_raw;
int Press_raw;
float Read_Temp;
float Read_Pressure;
float Height;

int TRASH_VALUE;
int data_read[10];

int Pressure_MMSB;
int Pressure_MLSB;
int Pressure_CRC_checksum;
int Pressure_LMSB;
int Pressure_LLSB;
int Temperature_MSB;
int Temperature_LSB;
int Temperature_CRC_checksum;

float Temp_ICP10101;
float Pressure_ICP10101;


int p_LSB; // p_LSB -- Raw pressure data from sensor
int T_LSB; //T_LSB -- Raw temperature data from sensor

short otp[4];
float sensor_constants[4]; // OTP values
float p_Pa_calib[3];
float LUT_lower;
float LUT_upper;
float quadr_factor;
float offst_factor;




void INIT_HS3001(){
    // KHOI DOI ZMOD4510
        R_MSTP->MSTPCRB_b.MSTPB9 = 0;

        //Cau hinh cho chan reset


        //Mo khoa PWPR- Write-Protect Register

        R_PMISC->PWPR_b.B0WI = 0; // Enable writable PFS
        R_PMISC->PWPR_b.PFSWE = 1;
        R_PFS->PORT[4].PIN[0].PmnPFS_b.PMR= 1; // P400 chuc nang ngoai vi I/O
        R_PFS->PORT[4].PIN[1].PmnPFS_b.PMR= 1; // P401 chuc nang ngoai vi I/O

        R_PFS->PORT[4].PIN[0].PmnPFS_b.PCR= 1; // P400 chuc nang ngoai vi I/O
        R_PFS->PORT[4].PIN[1].PmnPFS_b.PCR= 1; // P401 chuc nang ngoai vi I/O


        R_PFS->PORT[4].PIN[0].PmnPFS_b.PSEL= 0b00111; //P400 la SCL0_A
        R_PFS->PORT[4].PIN[1].PmnPFS_b.PSEL= 0b00111; //P401 la SDA0_A


        R_PMISC->PWPR_b.PFSWE = 0; // khoa writable PFS
        R_PMISC->PWPR_b.B0WI = 1;

        //Cau hinh I2C bang 31.5
        R_IIC0->ICCR1_b.ICE = 0; //SCLn và SDAn trạng thái không hoạt động
        R_IIC0->ICCR1_b.IICRST= 1; //Đặt bit ICCR1.IICRST = 1 để khởi động chế độ reset của IIC.
        R_IIC0->ICCR1_b.ICE= 1; //Đặt bit ICCR1.ICE = 1 để bắt đầu reset bên trong (internal reset).
        R_IIC0->ICSER_b.SAR0E= 1;// Cho phep cau hinh cac bit SARLy va SARUy
        R_IIC0->SAR[0].U_b.FS= 0;  //7-bit address
        R_IIC0->SAR[0].L_b.SVA = (0x44); //Dia chi HS3001 --> CO CAN DICH PHAI KO???????????????????**************************!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        R_IIC0->ICMR1_b.CKS= 0b011; // PCLKB = 50MHZ, 100 Kbps
        R_IIC0->ICBRL_b.BRL= 0xFE; // Transfer rate (kbps) =  100 --> BRL = 30, TABBLE 31.6
        R_IIC0->ICBRH_b.BRH= 0xF8; //Transfer rate (kbps) =  100 --> BRH = 24, TABBLE 31.6

        R_IIC0->ICFER_b.NACKE= 1; //Slave gửi NACK thì master dừng truyền (tránh lỗi).???? CAN KO Z !!!!!!!!!!!!!!!!!!!!!!!!
        R_IIC0->ICFER_b.SCLE= 1; //đồng bộ xung SCL với bus thực tế.
        R_IIC0->ICCR1_b.IICRST= 0; //Release from the internal reset state
        //XONGGGGGGGGGGGGGGGGGGGGGGGGG
}


int VALUE_HS3001(){

    while(R_IIC0->ICCR2_b.BBSY == 1) {
        ;//Đọc cờ BBSY trong thanh ghi ICCR2 để kiểm tra xem bus có đang rảnh không
    }
    R_IIC0->ICCR2_b.ST= 1;//yêu cầu phát Start Condition
    while(R_IIC0->ICSR2_b.TDRE == 0) {
           ;//kTRA XEM DU LIEU TRUYEN HET CHUA
    }

    // Gửi địa chỉ Write
    R_IIC0->ICDRT = (0x44 << 1) | 0;
    while(R_IIC0->ICSR2_b.TDRE == 0);

    R_IIC0->ICDRT = 0x00;     // Lệnh đo tem
    while(R_IIC0->ICSR2_b.TEND == 0);
    R_BSP_SoftwareDelay(28, BSP_DELAY_UNITS_MILLISECONDS);


    // STOP
    R_IIC0->ICSR2_b.STOP = 0;
    R_IIC0->ICCR2_b.SP = 1;
    while(R_IIC0->ICSR2_b.STOP == 0);


    //******************************************************

    while(R_IIC0->ICCR2_b.BBSY == 1);
    R_IIC0->ICCR2_b.ST = 1;
    while(R_IIC0->ICSR2_b.TDRE == 0);

    // Gửi địa chỉ READ

     R_IIC0->ICDRT = (0x44<<1)|1;
     while(R_IIC0->ICSR2_b.RDRF == 0) {
         ;
     }
     if(R_IIC0->ICSR2_b.NACKF == 0) {
         TRASH_VALUE = R_IIC0->ICDRR;

         for(int i=1; i<3; i++) {
             while(R_IIC0->ICSR2_b.RDRF == 0){
                         ;
             }

             if(i == 1) {
                 Value_1 = R_IIC0->ICDRR;
             }

             if(i == 2) {
                 R_IIC0->ICMR3_b.WAIT = 1;
                 Value_2 = R_IIC0->ICDRR;
             }
         }

         while(R_IIC0->ICSR2_b.RDRF == 0){
             ;
         }
         R_IIC0->ICMR3_b.ACKBT = 1;
         Value_3 = R_IIC0->ICDRR;
         while(R_IIC0->ICSR2_b.RDRF == 0){
             ;
         }
         R_IIC0->ICSR2_b.STOP = 0; // XOA CO
         R_IIC0->ICCR2_b.SP = 1; // PHAT BIT STOP
         Value_4 = R_IIC0->ICDRR;
         R_IIC0->ICMR3_b.WAIT = 0;
     }
     else{
         R_IIC0->ICSR2_b.STOP = 0; // XOA CO
         R_IIC0->ICCR2_b.SP = 1; // PHAT BIT STOP
         TRASH_VALUE = R_IIC0->ICDRR;

     }
     while(R_IIC0->ICSR2_b.STOP == 0) {
         ;
     }
     R_IIC0->ICSR2_b.NACKF = 0;
     R_IIC0->ICSR2_b.STOP = 0;
     return 1;
}

void I2C_ReleaseBus(void)
{

    R_PMISC->PWPR_b.B0WI = 0; // Enable writable PFS
    R_PMISC->PWPR_b.PFSWE = 1;
    // 1. Chuyển chân SCL/SDA sang GPIO output open-drain
    R_PFS->PORT[4].PIN[0].PmnPFS_b.PMR = 0; // SCL as GPIO
    R_PFS->PORT[4].PIN[1].PmnPFS_b.PMR = 0; // SDA as GPIO

    R_PMISC->PWPR_b.PFSWE = 0; // khoa writable PFS
    R_PMISC->PWPR_b.B0WI = 1;
    // Set SDA HIGH trước
    R_PORT4->PDR_b.PDR0 = 1;   // SCL output
    R_PORT4->PDR_b.PDR1 = 1;   // SDA output

    // 2. 9 xung SCL
    R_PORT4->PDR_b.PDR1 = 1; // output

    for(int i=0;i<9;i++)
    {
        R_PORT4->PODR_b.PODR0 = 1; // SCL HIGH
        R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MICROSECONDS);
        R_PORT4->PODR_b.PODR0 = 0; // SCL LOW
        R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MICROSECONDS);
    }

    // 3. START → STOP giả
    R_PORT4->PODR_b.PODR1 = 0; // SDA LOW  (START)
    R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MICROSECONDS);
    R_PORT4->PODR_b.PODR0 = 1; // SCL HIGH
    R_PORT4->PODR_b.PODR1 = 1; // SDA HIGH (STOP)

    // 4. Trả lại chức năng I2C
}

void INIT_ICP10101(){
    // KHOI DOI ZMOD4510
        R_MSTP->MSTPCRB_b.MSTPB9 = 0;

        //Mo khoa PWPR- Write-Protect Register

        R_PMISC->PWPR_b.B0WI = 0; // Enable writable PFS
        R_PMISC->PWPR_b.PFSWE = 1;
        R_PFS->PORT[4].PIN[0].PmnPFS_b.PMR= 1; // P400 chuc nang ngoai vi I/O
        R_PFS->PORT[4].PIN[1].PmnPFS_b.PMR= 1; // P401 chuc nang ngoai vi I/O

        R_PFS->PORT[4].PIN[0].PmnPFS_b.PCR= 1; // P400 chuc nang ngoai vi I/O
        R_PFS->PORT[4].PIN[1].PmnPFS_b.PCR= 1; // P401 chuc nang ngoai vi I/O


        R_PFS->PORT[4].PIN[0].PmnPFS_b.PSEL= 0b00111; //P400 la SCL0_A
        R_PFS->PORT[4].PIN[1].PmnPFS_b.PSEL= 0b00111; //P401 la SDA0_A


        R_PMISC->PWPR_b.PFSWE = 0; // khoa writable PFS
        R_PMISC->PWPR_b.B0WI = 1;

        //Cau hinh I2C bang 31.5
        R_IIC0->ICCR1_b.ICE = 0; //SCLn và SDAn trạng thái không hoạt động
        R_IIC0->ICCR1_b.IICRST= 1; //Đặt bit ICCR1.IICRST = 1 để khởi động chế độ reset của IIC.
        R_IIC0->ICCR1_b.ICE= 1; //Đặt bit ICCR1.ICE = 1 để bắt đầu reset bên trong (internal reset).
        R_IIC0->ICSER_b.SAR0E= 1;// Cho phep cau hinh cac bit SARLy va SARUy
        R_IIC0->SAR[0].U_b.FS= 0;  //7-bit address
        R_IIC0->SAR[0].L_b.SVA = (0x63); //Dia chi HS3001 --> CO CAN DICH PHAI KO???????????????????**************************!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        R_IIC0->ICMR1_b.CKS= 0b011; // PCLKB = 50MHZ, 100 Kbps
        R_IIC0->ICBRL_b.BRL= 0xFE; // Transfer rate (kbps) =  100 --> BRL = 30, TABBLE 31.6
        R_IIC0->ICBRH_b.BRH= 0xF8; //Transfer rate (kbps) =  100 --> BRH = 24, TABBLE 31.6

        R_IIC0->ICFER_b.NACKE= 1; //Slave gửi NACK thì master dừng truyền (tránh lỗi).???? CAN KO Z !!!!!!!!!!!!!!!!!!!!!!!!
        R_IIC0->ICFER_b.SCLE= 1; //đồng bộ xung SCL với bus thực tế.
        R_IIC0->ICCR1_b.IICRST= 0; //Release from the internal reset state
        //XONGGGGGGGGGGGGGGGGGGGGGGGGG
}


void I2C_RECEIVE_MULTI(uint8_t addr) {

    while(R_IIC0->ICCR2_b.BBSY == 1) {
        ;
    }
    R_IIC0->ICCR2_b.ST = 1;
    while(R_IIC0->ICSR2_b.TDRE == 0) {
        ;
    }

    // Gửi địa chỉ READ

     R_IIC0->ICDRT = (addr<<1)|1;
     while(R_IIC0->ICSR2_b.RDRF == 0) {
         ;
     }

     if(R_IIC0->ICSR2_b.NACKF == 0) {
        TRASH_VALUE = R_IIC0->ICDRR;
        while(R_IIC0->ICSR2_b.RDRF == 0){
                         ;
        }
        R_IIC0->ICMR3_b.WAIT = 1;
        data_read[0]  = R_IIC0->ICDRR;
        while(R_IIC0->ICSR2_b.RDRF == 0){
             ;
         }
         R_IIC0->ICMR3_b.ACKBT = 1;
         data_read[1] = R_IIC0->ICDRR;
         while(R_IIC0->ICSR2_b.RDRF == 0){
             ;
         }
         R_IIC0->ICSR2_b.STOP = 0; // XOA CO
         R_IIC0->ICCR2_b.SP = 1; // PHAT BIT STOP
         data_read[2] = R_IIC0->ICDRR;
         R_IIC0->ICMR3_b.WAIT = 0;
     }
     else{
         R_IIC0->ICSR2_b.STOP = 0; // XOA CO
         R_IIC0->ICCR2_b.SP = 1; // PHAT BIT STOP
         TRASH_VALUE = R_IIC0->ICDRR;

     }
     while(R_IIC0->ICSR2_b.STOP == 0) {
         ;
     }
     R_IIC0->ICSR2_b.NACKF = 0;
     R_IIC0->ICSR2_b.STOP = 0;

}



void read_otp_from_i2c() {
// OTP Read mode

    //I2C_TRANS_MULTI(0x63, data_write, 5); //********************************************************

    while(R_IIC0->ICCR2_b.BBSY == 1) {
            ;//Đọc cờ BBSY trong thanh ghi ICCR2 để kiểm tra xem bus có đang rảnh không
    }

    R_IIC0->ICCR2_b.ST= 1;//yêu cầu phát Start Condition
    while(R_IIC0->ICSR2_b.TDRE == 0) {
           ;//kTRA XEM DU LIEU TRUYEN HET CHUA
    }

    // Gửi địa chỉ Write
    R_IIC0->ICDRT = (0x63 << 1) | 0;
    while(R_IIC0->ICSR2_b.TDRE == 0);
    R_IIC0->ICDRT = 0xC5;     // TRANSMIT PRESSURE FIRST - NORMAL
    while(R_IIC0->ICSR2_b.TEND == 0);
    R_IIC0->ICDRT = 0x95;
    while(R_IIC0->ICSR2_b.TEND == 0);
    R_IIC0->ICDRT = 0x00;
    while(R_IIC0->ICSR2_b.TEND == 0);
    R_IIC0->ICDRT = 0x66;
    while(R_IIC0->ICSR2_b.TEND == 0);
    R_IIC0->ICDRT = 0x9C;
    while(R_IIC0->ICSR2_b.TEND == 0);

    // STOP
    R_IIC0->ICSR2_b.NACKF = 0;
    R_IIC0->ICSR2_b.STOP = 0;
    R_IIC0->ICCR2_b.SP = 1;
    while(R_IIC0->ICSR2_b.STOP == 0);

    R_BSP_SoftwareDelay(50, BSP_DELAY_UNITS_MILLISECONDS);

// Read OTP values
    for (int i = 0; i < 4; i++) {

        //I2C_TRANS_MULTI(0x63, data_write, 2);

        //***************************************************************
        while(R_IIC0->ICCR2_b.BBSY == 1) {
            ;//Đọc cờ BBSY trong thanh ghi ICCR2 để kiểm tra xem bus có đang rảnh không
        }

        R_IIC0->ICCR2_b.ST= 1;//yêu cầu phát Start Condition
        while(R_IIC0->ICSR2_b.TDRE == 0) {
               ;//kTRA XEM DU LIEU TRUYEN HET CHUA
        }

        // Gửi địa chỉ Write
        R_IIC0->ICDRT = (0x63 << 1) | 0;
        while(R_IIC0->ICSR2_b.TDRE == 0);
        R_IIC0->ICDRT = 0xC7;     // TRANSMIT PRESSURE FIRST - NORMAL
        while(R_IIC0->ICSR2_b.TEND == 0);
        R_IIC0->ICDRT = 0xF7;
        while(R_IIC0->ICSR2_b.TEND == 0);

        // STOP
        R_IIC0->ICSR2_b.NACKF = 0;
        R_IIC0->ICSR2_b.STOP = 0;
        R_IIC0->ICCR2_b.SP = 1;
        while(R_IIC0->ICSR2_b.STOP == 0);

        R_BSP_SoftwareDelay(50, BSP_DELAY_UNITS_MILLISECONDS);

        //***************************************************************
        I2C_RECEIVE_MULTI(0x63);
        otp[i] = data_read[0]<<8 | data_read[1];

    }

}


void init_base() // s = inv_invpres_t
{
    int i;
    for(i = 0; i < 4; i++) {
        sensor_constants[i] = (float)otp[i];
    }
    p_Pa_calib[0] = 45000.0;
    p_Pa_calib[1] = 80000.0;
    p_Pa_calib[2] = 105000.0;
    LUT_lower = 3.5 * (1<<20);
    LUT_upper = 11.5 * (1<<20);
    quadr_factor = 1 / 16777216.0;
    offst_factor = 2048.0;
}

void inv_invpres_init() {
    read_otp_from_i2c();
    init_base();

}


void calculate_conversion_constants(float *p_Pa, float *p_LUT, float *out)
{
    float A,B,C;
    C = (p_LUT[0] * p_LUT[1] * (p_Pa[0] - p_Pa[1]) +
            p_LUT[1] * p_LUT[2] * (p_Pa[1] - p_Pa[2]) +
            p_LUT[2] * p_LUT[0] * (p_Pa[2] - p_Pa[0])) /
            (p_LUT[2] * (p_Pa[0] - p_Pa[1]) +
            p_LUT[0] * (p_Pa[1] - p_Pa[2]) +
            p_LUT[1] * (p_Pa[2] - p_Pa[0]));
    A = (p_Pa[0] * p_LUT[0] - p_Pa[1] * p_LUT[1] - (p_Pa[1] - p_Pa[0]) * C) / (p_LUT[0] - p_LUT[1]);
    B = (p_Pa[0] - A) * (p_LUT[0] + C);
    out[0] = A;
    out[1] = B;
    out[2] = C;
}

void inv_invpres_process_data(int p_LSB, int T_LSB) {

    float t;
    float s1,s2,s3;
    float in[3];
    float out[3];
    float A,B,C;
    t = (float)(T_LSB - 32768);
    s1 = LUT_lower + (float)(sensor_constants[0] * t * t) * quadr_factor;
    s2 = offst_factor * sensor_constants[3] + (float)(sensor_constants[1] * t * t) * quadr_factor;
    s3 = LUT_upper + (float)(sensor_constants[2] * t * t) * quadr_factor;
    in[0] = s1;
    in[1] = s2;
    in[2] = s3;
    calculate_conversion_constants(p_Pa_calib, in, out);
    A = out[0];
    B = out[1];
    C = out[2];
    Pressure_ICP10101 = A + B / (C + p_LSB);
    Temp_ICP10101 = -45.f + 175.f/65536.f * T_LSB;

    }

    // p_Pa -- List of 3 values corresponding to applied pressure in Pa
    // p_LUT -- List of 3 values corresponding to the measured p_LUT values at the applied pressures.

int VALUE_ICP10101(){

    while(R_IIC0->ICCR2_b.BBSY == 1) {
        ;//Đọc cờ BBSY trong thanh ghi ICCR2 để kiểm tra xem bus có đang rảnh không
    }

    R_IIC0->ICCR2_b.ST= 1;//yêu cầu phát Start Condition
    while(R_IIC0->ICSR2_b.TDRE == 0) {
           ;//kTRA XEM DU LIEU TRUYEN HET CHUA
    }

    // Gửi địa chỉ Write
    R_IIC0->ICDRT = (0x63 << 1) | 0;
    while(R_IIC0->ICSR2_b.TDRE == 0);
    R_IIC0->ICDRT = 0x48;     // TRANSMIT PRESSURE FIRST - NORMAL
    while(R_IIC0->ICSR2_b.TEND == 0);
    R_IIC0->ICDRT = 0xA3;
    while(R_IIC0->ICSR2_b.TEND == 0);

    // STOP
    R_IIC0->ICSR2_b.STOP = 0;
    R_IIC0->ICCR2_b.SP = 1;
    while(R_IIC0->ICSR2_b.STOP == 0);

    R_BSP_SoftwareDelay(50, BSP_DELAY_UNITS_MILLISECONDS);

    //******************************************************
    R_BSP_SoftwareDelay(50, BSP_DELAY_UNITS_MILLISECONDS);

     //************************************** DOC DATA NE
     while(R_IIC0->ICCR2_b.BBSY == 1) {
            ;//Đọc cờ BBSY trong thanh ghi ICCR2 để kiểm tra xem bus có đang rảnh không
        }

     R_IIC0->ICCR2_b.ST= 1;//yêu cầu phát Start Condition
     while(R_IIC0->ICSR2_b.TDRE == 0) {
               ;//kTRA XEM DU LIEU TRUYEN HET CHUA
     }

     R_IIC0->ICDRT = (0x63<<1)|1;



     while(R_IIC0->ICSR2_b.RDRF == 0) {
         ;
     }

     if(R_IIC0->ICSR2_b.NACKF == 0) {
         TRASH_VALUE = R_IIC0->ICDRR;

         for(int i=1; i< 8; i++) {
             while(R_IIC0->ICSR2_b.RDRF == 0){
                         ;
             }

             if(i == 1) {
                 Pressure_MMSB = R_IIC0->ICDRR;
             }

             if(i == 2) {
                 Pressure_MLSB= R_IIC0->ICDRR;
             }
             if(i == 3) {
                 Pressure_CRC_checksum= R_IIC0->ICDRR;
             }
             if(i == 4) {
                 Pressure_LMSB= R_IIC0->ICDRR;
             }
             if(i == 5) {
                 Pressure_LLSB= R_IIC0->ICDRR;
             }
             if(i == 6) {
                 Pressure_CRC_checksum= R_IIC0->ICDRR;
             }
             if(i == 7) {
                 R_IIC0->ICMR3_b.WAIT = 1;
                 Temperature_MSB = R_IIC0->ICDRR;
             }
         }

         while(R_IIC0->ICSR2_b.RDRF == 0){
             ;
         }


         R_IIC0->ICMR3_b.ACKBT = 1;
         Temperature_LSB= R_IIC0->ICDRR;
         while(R_IIC0->ICSR2_b.RDRF == 0){
             ;
         }
         R_IIC0->ICSR2_b.STOP = 0; // XOA CO
         R_IIC0->ICCR2_b.SP = 1; // PHAT BIT STOP
         Temperature_CRC_checksum = R_IIC0->ICDRR;
         R_IIC0->ICMR3_b.WAIT = 0;
     }
     else{
         R_IIC0->ICSR2_b.STOP = 0; // XOA CO
         R_IIC0->ICCR2_b.SP = 1; // PHAT BIT STOP
         TRASH_VALUE = R_IIC0->ICDRR;

     }
     while(R_IIC0->ICSR2_b.STOP == 0) {
         ;
     }
     R_IIC0->ICSR2_b.NACKF = 0;
     R_IIC0->ICSR2_b.STOP = 0;
     return 1;
}

void READ_HS3001() {
    INIT_HS3001();
    VALUE_HS3001();

    humidity_raw_data = ((Value_1 & 0x3F)<<8) | Value_2;
    temp_raw_data = ((Value_3<<8) | Value_4)>>2;
    humidity_data = ((humidity_raw_data)/(16384-1))*100; //2^14 = 16384
    temp_data = ((temp_raw_data)/(16384-1))*165-40;
}

void READ_ICP10101() {
    I2C_ReleaseBus();
    INIT_ICP10101();
    VALUE_ICP10101();
    Press_raw = (Pressure_MMSB<<16) | (Pressure_MLSB<<8) | (Pressure_LMSB);
    Temp_raw = (Temperature_MSB<<8) | (Temperature_LSB);
    inv_invpres_process_data(Press_raw, Temp_raw);
    Read_Pressure = Pressure_ICP10101;
    Read_Temp = Temp_ICP10101;
    Height = 44330*(1-pow(Pressure_ICP10101/101325, 0.1903));
}


void Float_To_String(float value, char *out)
{
    Double_To_String((double)value, out); // tái sử dụng
}

void Double_To_String(double value, char *out )
{
    int int_part = (int)value;
    double frac = value - int_part;
    if (frac < 0) frac = -frac;

    /* convert phần nguyên */
    int len = 0;
    if (value < 0)
    {
        out[len++] = '-';
        int_part = -int_part;
    }

    /* chuyển số nguyên sang chuỗi */
    char tmp[16];
    int i = 0;
    do {
        tmp[i++] = (int_part % 10) + '0';
        int_part /= 10;
    } while (int_part > 0);

    /* đảo chuỗi */
    while (i > 0)
        out[len++] = tmp[--i];

    /* thêm dấu '.' */
    out[len++] = '.';

    /* chuyển phần thập phân */
    for (uint8_t d = 0; d < 2; d++)
    {
        frac *= 10;
        int digit = (int)frac;
        out[len++] = digit + '0';
        frac -= digit;
    }

    out[len] = '\0';
}

#define STRING_BUFFER_SIZE 16



#if (1 == BSP_MULTICORE_PROJECT) && BSP_TZ_SECURE_BUILD
bsp_ipc_semaphore_handle_t g_core_start_semaphore =
{
    .semaphore_num = 0
};
#endif

/*******************************************************************************************************************//**
 * main() is generated by the RA Configuration editor and is used to generate threads if an RTOS is used.  This function
 * is called by main() when no RTOS is used.
 **********************************************************************************************************************/
void hal_entry(void) {
    I2C_ReleaseBus();

    INIT_HS3001();
    uart_init();
    R_BSP_SoftwareDelay(500, BSP_DELAY_UNITS_MILLISECONDS);

    inv_invpres_init();

    char HS_3001_Temp[16];
    char HS_3001_Temp1[16];
    char HS_3001_Humi[16];
    char ICP_10101_Press[16];
    char ICP_10101_temp[16];
    float hs1;
    float hs2;
    float icp1;
    float icp2;

    while(1){
    	READ_HS3001();          // đọc temp_data, humidity_data
    	READ_ICP10101();        // đọc Read_Temp, Read_Pressure

    	Double_To_String(temp_data,HS_3001_Temp);
    	Double_To_String(humidity_data,HS_3001_Humi);
    	Float_To_String(Read_Pressure,ICP_10101_Press);
    	Float_To_String(Read_Temp,ICP_10101_temp);

    }

	/* Wake up 2nd core if this is first core and we are inside a multicore project. */
#if (0 == _RA_CORE) && (1 == BSP_MULTICORE_PROJECT) && !BSP_TZ_NONSECURE_BUILD

#if BSP_TZ_SECURE_BUILD
    /* Take semaphore so 2nd core can clear it */
    R_BSP_IpcSemaphoreTake(&g_core_start_semaphore);
#endif

    R_BSP_SecondaryCoreStart();

#if BSP_TZ_SECURE_BUILD
    /* Wait for 2nd core to start and clear semaphore */
    while(FSP_ERR_IN_USE == R_BSP_IpcSemaphoreTake(&g_core_start_semaphore))
    {
        ;
    }
#endif
#endif

#if (1 == _RA_CORE) && (1 == BSP_MULTICORE_PROJECT) && BSP_TZ_SECURE_BUILD
    /* Signal to 1st core that 2nd core has started */
    R_BSP_IpcSemaphoreGive(&g_core_start_semaphore);
#endif

#if BSP_TZ_SECURE_BUILD
    /* Enter non-secure code */
    R_BSP_NonSecureEnter();
#endif
}

#if BSP_TZ_SECURE_BUILD

FSP_CPP_HEADER
BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ();

/* Trustzone Secure Projects require at least one nonsecure callable function in order to build (Remove this if it is not required to build). */
BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ()
{

}
FSP_CPP_FOOTER

#endif
