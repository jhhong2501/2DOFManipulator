#include "mcu_init.h"
#include "dataType.h"

///////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Function  ////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
void InitLS7366();
void EncoderPulse();
void DC_Output (double p_voltage);
double Pos_Control (double p_tar);
void Speed_Control(double s_tar, volatile double v_limit);
void Current_Control(double t_tar, volatile double c_limit);

///////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Global Var////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
#define		dt					0.0005						// delta time
#define		GEAR_RATIO			81							// Gear Ratio
#define		Kt					0.0683						// Torque Constant
#define		ENCODER_RESOULTION	1024.0						// Encoder Resolution
#define		D2R					0.01745329251994329576		// Degree to Radian
#define		R2D					57.295779513082320876798	// Radian to Degree

//////// Position Control Gain ////////
#define		Kp_p				2
#define		Kd_p				0.085
//////// Speed Control Gain ////////
#define		Kp_s				7.5
#define		Ki_s				0.05
#define		Ka_s				1/Kp_s
//////// Current Control Gain ////////
#define		Kp_c				0.1316
#define		Ki_c				352
#define		Ka_c				1/Kp_c

//////// Packet Communication ////////
volatile int32_t g_Cnt, g_preCnt;
volatile int g_SendFlag = 0;
volatile double g_Pdes;
volatile double g_Vlimit = 1.;
volatile double g_Climit = 1.;
volatile Packet_t g_PacketBuffer;
volatile unsigned char g_PacketMode;
volatile unsigned char g_ID = 1;
volatile unsigned char checkSize;
volatile unsigned char g_buf[256], g_BufWriteCnt, g_BufReadCnt;

//////// Current Var ////////
double		m_degree				= 0.0;
double		m_velocity				= 0.0;
double		m_current				= 0.0;

//////// Control Frequency Var ////////
int c_cnt = 0; int s_cnt = 0; int p_cnt = 0;

//////// Position Control Var ////////
double P_tar = 0; double P_cur = 0;
double P_er = 0; double P_pre_er = 0; double P_er_sum = 0;
double P_Omega = 0;
//////// Speed Control Var ////////
double S_tar = 0; double S_cur = 0;
double S_er = 0; double S_er_sum = 0; double m_degree_pre = 0;
double S_anti = 0;
double S_Current = 0;
//////// Current Control Var ////////
double C_tar = 0; double C_cur = 0;
double C_er = 0; double C_er_sum = 0; double m_current_pre = 0;
double C_anti = 0;
double C_Voltage = 0;

///////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// Interrupt  ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
ISR(USART0_RX_vect){
	g_buf[g_BufWriteCnt++] = UDR0;
}
g_Pdes = g_PacketBuffer.data.pos / 1000;							// 각도 지령
g_Vlimit = g_PacketBuffer.data.velo / 1000;							// 속도 지령
g_Climit = g_PacketBuffer.data.cur / 1000;							// 전류 지령
ISR(TIMER3_OVF_vect){
	TCNT3 = 65536 - 125;	// 제어주기 0.0005 (20ms)
	
	EncoderPulse();			// 엔코더 펄스 받아옴

	m_degree		= (g_Cnt/(ENCODER_RESOULTION*GEAR_RATIO*4))*2*M_PI;		// 현재 각도
	m_velocity		= (m_degree - m_degree_pre)*2000;						// 현재 속도
	m_current		= 10.0*(GetADC(0)*5.0/1024.0-2.5);						// 현재 전류
						
	P_er = g_Pdes - m_degree;								// 오차 = 목표치-현재값

	if(p_cnt == 1000)										// 위치 제어기 0.5
	{										
		p_cnt = 0;
		P_Omega = Kp_p*P_er + Kd_p*(P_er-P_pre_er)*2;
	}	
	if(s_cnt == 100)										// 속도 제어기 0.05
	{
		s_cnt=0;
		Speed_Control(P_Omega, g_Vlimit);
	}	
	if(c_cnt == 10)											// 전류 제어기 0.005
	{										
		c_cnt = 0;
		Current_Control(S_Current, g_Climit);
	}
	if(g_Pdes == 0 && g_Vlimit == 0 && g_Climit == 0)		// 초기 정지
	{		
		DC_Output(0);
	}
	else
	{														// 지령값에 따른 모터 구동
		DC_Output(P_Omega);
	}
	
	P_pre_er = P_er;
	g_SendFlag++;	
	m_degree_pre = m_degree;								// 현재오차를 이전오차로

	c_cnt++; s_cnt++; p_cnt++;								// 통신 flag
}
int main(void){
	Packet_t packet;
	packet.data.header[0] = packet.data.header[1] = packet.data.header[2] = packet.data.header[3] = 0xFF;
	InitIO();		InitUart0();		InitSPI();
	InitTimer1();	
	InitTimer3();	InitADC();		InitLS7366();
	OCR1A = 0; OCR1B = 0;
	TCNT1 = 0; 
	DC_Output(0);
	TCNT3 = 65536-125;
	
	sei();
	unsigned char check = 0;
	
    while (1){
		for(;g_BufReadCnt != g_BufWriteCnt;g_BufReadCnt++){
			switch (g_PacketMode){
//////////////////////////////////////////////////////////////////////////
				case 0:																			// case 0 
				if(g_buf[g_BufReadCnt] == 0xFF){												// 일정 시간이 되면
					checkSize++;
					if (checkSize == 4){														// case 1로
						g_PacketMode = 1; 
					}
				}
				else{																			// 다시검사
					checkSize = 0;
				}
				break;
//////////////////////////////////////////////////////////////////////////
				case 1:																			// case 1
				g_PacketBuffer.buffer[checkSize++] = g_buf[g_BufReadCnt];
				if(checkSize == 8){
					if(g_PacketBuffer.data.id == g_ID){
						g_PacketMode = 2;
					}
					else{																		// 다시검사
						g_PacketMode = 0; checkSize = 0;
					}
				}
				break;
//////////////////////////////////////////////////////////////////////////
				case 2:																			// case 2
				g_PacketBuffer.buffer[checkSize++] = g_buf[g_BufReadCnt];
				check += g_buf[g_BufReadCnt];
				if(checkSize == g_PacketBuffer.data.size){
					if(check == g_PacketBuffer.data.check){
						switch(g_PacketBuffer.data.mode){
							case 2:
							g_Pdes = g_PacketBuffer.data.pos / 1000;							// 각도 지령
							g_Vlimit = g_PacketBuffer.data.velo / 1000;							// 속도 지령
							g_Climit = g_PacketBuffer.data.cur / 1000;							// 전류 지령
							break;
						}
					}
					check = 0; g_PacketMode = 0; checkSize = 0;
				}
				else if(checkSize > g_PacketBuffer.data.size || checkSize > sizeof(Packet_t)){	// 다시검사
					check = 0; g_PacketMode = 0; checkSize = 0;	
				}
			}	// switch end
		}		// for end
//////////////////////////////////////////////////////////////////////////
		if(g_SendFlag > 19){								// ISR 20번돌면 전송
			g_SendFlag = 0;
			
			packet.data.id = g_ID;							// 주소 확인
			packet.data.size = sizeof(Packet_data_t);		// 크기확인
			packet.data.mode = 3;							// 모드
			packet.data.check = 0;							// 에러 확인

			packet.data.pos = m_degree*1000;				// 위치 데이터
			packet.data.velo = m_velocity*1000;				// 속도 데이터
			packet.data.cur = m_current*1000;				// 전류 데이터
			
			for(int i = 8;i < sizeof(Packet_t);i++)
				packet.data.check += packet.buffer[i];
			for(int i=0;i<packet.data.size; i++)
				TransUart0(packet.buffer[i]);
		}
	}	// while end
}		// main end

//////////////////////////////////////////////////////////////////
// InitLS7366()
// Encoder setting
// Input : Non
// Output : Non
//////////////////////////////////////////////////////////////////
void InitLS7366(){
	PORTB = 0x00;
	SPI_MasterSend(SELECT_MDR0 | WR_REG);
	SPI_MasterSend(X4_QUAD | FREE_RUN | DISABLE_INDEX | SYNCHRONOUS_INDEX | FILTER_CDF_1);
	PORTB = 0x01;
	
	PORTB = 0x00;
	SPI_MasterSend(SELECT_MDR1 | WR_REG);
	SPI_MasterSend(FOUR_BYTE_COUNT_MODE | ENABLE_COUNTING);
	PORTB = 0x01;
	
	PORTB = 0x00;
	SPI_MasterSend(SELECT_CNTR | CLR_REG);
	PORTB = 0x01;
}

//////////////////////////////////////////////////////////////////
// EncoderPulse()
// Encoder pulse reading
// Input : Non
// Output : g_Cnt(Encoder Count)
//////////////////////////////////////////////////////////////////
void EncoderPulse()
{
	int32_t cnt;
	
	PORTB = 0x00;
	SPI_MasterSend(SELECT_OTR | LOAD_REG);           
	PORTB = 0x01;
			
	PORTB = 0x00;
	SPI_MasterSend(SELECT_OTR | RD_REG);
	cnt = SPI_MasterRecv();		cnt = cnt<< 8;
	cnt |= SPI_MasterRecv();	cnt = cnt<< 8;
	cnt |= SPI_MasterRecv();	cnt = cnt<< 8;
	cnt |= SPI_MasterRecv();
	PORTB = 0x01;
	g_Cnt = -cnt;
}

//////////////////////////////////////////////////////////////////
// DC_Output()
// PWM duty ratio
// Input : voltage
// Output : None(PWM)
//////////////////////////////////////////////////////////////////
void DC_Output(double p_voltage){
	while(TCNT1  == 0);
	int ocr = p_voltage * (400. / 24.) + 400;
	if(ocr > OCR_MAX)	ocr = OCR_MAX;
	else if(ocr < OCR_MIN)	ocr = OCR_MIN;
	OCR1A = OCR1B = ocr;
}

//////////////////////////////////////////////////////////////////
// Pos_Control()
// Position PD control
// Input : Target Angel	[rad]
// Output : Omega		[rad/s]
//////////////////////////////////////////////////////////////////
double Pos_Control(double p_tar)
{								
	return Kp_p*P_er + Kd_p*(P_er-P_pre_er)*2;					// 현재 각도
}

//////////////////////////////////////////////////////////////////
// Speed_Control()
// Speed PI control
// Input : target Speed [rad/s], target speed [rad/s]
// Output : current		[A]
//////////////////////////////////////////////////////////////////
void Speed_Control(double s_tar, volatile double v_limit)
{
	if(s_tar > v_limit)			s_tar = v_limit;				
	else if(s_tar < -v_limit)	s_tar = -v_limit;				
	
	S_tar = s_tar;												// 타겟 속도
	S_cur = m_velocity;											// 현재 속도
	S_er = S_tar - S_cur;										// 오차 = 목표치-현재값
	S_er_sum += S_er - Ka_s*S_anti;								// 오차적분 = 오차적분 + 오차*(dt*100)
	
	if(S_er_sum > 642)			S_er_sum = 642;					
	else if(S_er_sum < -642)	S_er_sum = -642;				
	
	S_Current = Kp_s*S_er + Ki_s*S_er_sum*0.05 + Kt*m_current;	// 0.05, 전향보상
	
	if(S_Current > 642){										// Antiwindup Saturation
		S_anti = S_Current - 642;
		S_Current = 642;
	}
	else if(S_Current < -642){									// Antiwindup Saturation
		S_anti = S_Current + 642;
		S_Current = -642;
	}
	
}

//////////////////////////////////////////////////////////////////
// Current_Control()
// Current PI control
// Input : target current	[A]
// Output : voltage			[V]
//////////////////////////////////////////////////////////////////
void Current_Control(double t_tar, volatile double c_limit)
{
	if(t_tar > c_limit)			t_tar = c_limit;						
	else if(t_tar < -c_limit)	t_tar = -c_limit;						
	
	C_tar = t_tar;														// 타겟 전류
	C_cur = m_current;													// 현재 전류
	C_er = C_tar - C_cur;												// 오차 = 목표치-현재값
	C_er_sum += C_er - Ka_c*C_anti;										// 오차적분 = 오차적분 + 오차*dt
	
	if(C_er_sum > 24.0) C_er_sum = 24.0;								
	else if(C_er_sum < -24.0) C_er_sum = -24.0;							
	
	C_Voltage = Kp_c*C_er + Ki_c*C_er_sum*0.005 + Kt*m_velocity;
	
	if(C_Voltage>24.0){													// Antiwindup Saturation
		C_anti = C_Voltage - 24.0;
		C_Voltage = 24.0;
	}
	else if(C_Voltage<-24.0){											// Antiwindup Saturation
		C_anti = C_Voltage + 24.0;
		C_Voltage = -24.0;
	}
}