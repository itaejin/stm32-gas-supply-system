#ifndef TS_COMMU_H
#define TS_COMMU_H

#include "stm32f1xx_hal.h"

// 🚀 최적화된 타입 정의들
typedef enum { GasGood=0, GasWait, GasInSupply, GasPrepReplace, GasAlarmReplace=4 } GasGaugeLamp_t;  
typedef enum { NoFlow = 0, Flow = 8 } GasFlowLampVel_t;
typedef enum { MorePrepReplace = 8, PrepReplace = 7, AlarmReplace = 6 } GasLamp_t;
typedef enum { Main = 11, Config = 41, Alarm = 45, PW_Input = 111, PW_Change, PW_Wrong, PW_Change_OK, AlertAlarm = 120 } Window_t;

typedef struct {
	uint16_t Value;
	uint16_t Low;			// GasAlarmReplace
	uint16_t LowLow;		// GasPrepReplace
	uint16_t Offset;
	uint16_t MinSValue;		// Min. Value of Sensor
	uint16_t MaxSValue;		// Max. Value of Sensor
	uint16_t MaxValue;		// Max. Value of Gas Chaeging
	GasLamp_t GasLamp;
	uint16_t Reserved[2];
} DataPT12_t;

typedef struct {
	uint16_t Value;
	uint16_t MinSValue;		// Min. Supply Value
	uint16_t Reserved;
	uint16_t Offset;
	uint16_t MaxSValue;		// Max. Value of Sensor
	uint16_t Reserved2;
} DataPT3_t;

typedef struct {
	uint16_t BSVelL;	// Before Solenoid Velocity Left (GasFlowLampVel_t)
	uint16_t BSLampL;	// Before Solenoid Lamp Left (GasLamp_t)
	uint16_t ASVelL;	// After Solenoid Velocity Left
	uint16_t ASLampL;	// After Solenoid Lamp Left
	uint16_t BSVelR;	// Before Solenoid Velocity Right
	uint16_t BSLampR;	// Before Solenoid Lamp Right
	uint16_t ASVelR;	// After Solenoid Velocity Right
	uint16_t ASLampR;	// After Solenoid Lamp Right
	uint16_t MVel;		// After Solenoid Velocity Right
	uint16_t MLamp;		// After Solenoid Lamp Right
} FlowLamp_t;

typedef struct {
	uint16_t Value;
	uint16_t LowLimit;		// GasAlarmReplace
	uint16_t UpperLimit;	// GasPrepReplace
	uint16_t TargetValue;	// GasPrepReplace
	uint16_t MinValue;		// Min. Value of Sensor
	uint16_t MaxValue;		// Max. Value of Gas Chaeging
} GasGraph_t;

typedef struct {
	uint16_t CurValue;		// 비밀번호 현재 
	uint16_t SavedValue;	// 비밀번호 저장
	uint16_t ChangedValue;	// 비밀번호 변경
} Password_t;

#pragma pack(push,1)
typedef struct{
	unsigned MainWinSW:1;		// 1 --> 0
	unsigned rsvd01:1;		
	unsigned ConfigWinSW:1;		// 3 --> 2
	unsigned rsvd04_08:5;
	unsigned AlarmWinSW:1;		// 9 --> 8
	unsigned SupplyGasLSW:1;	// 10 --> 9
	unsigned StopGasLSW:1;		// 11 --> 10
	unsigned rsvd12_19:8;
	unsigned SupplyGasRSW:1;	// 20 --> 19
	unsigned StopGasRSW:1;		// 21 --> 20
	unsigned rsvd022_29:8;
	unsigned AlarmResetSW:1;	// 30 --> 29
	unsigned rsvd031_46:16;
	unsigned rsvd047_62:16;
	unsigned rsvd063_78:17;
	unsigned rsvd079:1;
	unsigned SolNoNCSW:1;		// 80 --> 79
	unsigned UseBuzzerSW:1;		// 81 --> 80
	unsigned rsvd82_89:8;
	unsigned InputPWSW:1;		// 90 --> 89
	unsigned ConfirmPWSW:1;		// 91 --> 90
	unsigned OpenWinPWSW:1;		// 92 --> 91
	unsigned rsvd93_100:8;
	
	unsigned MainWinLamp:1;			// 101 --> 0
	unsigned rsvd101:1;		
	unsigned ConfigWinLamp:1;		// 103 --> 2
	unsigned rsvd104_108:5;
	unsigned AlarmWinLamp:1;		// 109 --> 8
	unsigned SupplyGasLLamp:1;		// 110 --> 9
	unsigned StopGasLLamp:1;		// 111 --> 10
	unsigned rsvd112_119:8;
	unsigned SupplyGasRLamp:1;		// 120 --> 19
	unsigned StopGasRLamp:1;		// 121 --> 20
	unsigned rsvd0122_129:8;
	unsigned AlarmReseLampW:1;		// 130 --> 29
	unsigned rsvd0131_146:16;
	unsigned rsvd0147_162:16;
	unsigned rsvd0163_178:16;
	unsigned rsvd0179:1;
	unsigned SolNoNCLamp:1;			// 180 --> 79
	unsigned UseBuzzerLamp:1;		// 181 --> 80
	unsigned rsvd182_189:8;
	unsigned InputPWLamp:1;			// 190 --> 89
	unsigned ConfirmPWLamp:1;		// 191 --> 90
	unsigned OpenWinPWLamp:1;		// 192 --> 91
	unsigned rsvd193_199:8;
	
	unsigned EmerSWOnLamp:1;		// 200 --> 199
	unsigned SubEmerSWOnLamp:1;		// 201 --> 200
	unsigned PressureLowLimit:1;	// 202 --> 201
	unsigned PrepChgPT1:1;			// 203 --> 202
	unsigned PrepChgPT1Alarm:1;		// 204 --> 203
	unsigned PrepChgPT2:1;			// 205 --> 204
	unsigned PrepChgPT2Alarm:1;		// 206 --> 205
	unsigned MinSuuplPressPT3:1;	// 207 --> 206
	unsigned rsvd208:1;
	unsigned rsvd209_224:16;
	unsigned rsvd225_240:16;
} IO_t;
#pragma pack(pop)

typedef struct {
	uint8_t GasName[12];
	uint8_t GasUnit[6];
	uint16_t Password;
	uint16_t PT1Data[6];	
	uint16_t PT2Data[6];
	uint16_t PT3Data[4];
	uint32_t dummy;
} FlashParam_t;

//==============================================================================
// 🚀 전역 변수 선언
//==============================================================================
extern DataPT12_t PT1, PT2;
extern DataPT3_t PT3;
extern FlowLamp_t FlowLamp;
extern GasGraph_t GraphPT1, GraphPT2;
extern Password_t Password;
extern IO_t IOdata;
extern Window_t NewWindow;
extern FlashParam_t fParam;
extern uint8_t TestFlash;
extern uint32_t FlashData;

// 🆕 최적화 관련 변수들
extern uint8_t flash_save_pending;
extern uint8_t update_flow_pending;
extern volatile uint8_t modbus_frame_ready;
extern volatile uint8_t modbus_processing;

// 통신 관련 변수들
extern UART_HandleTypeDef huart3;
extern uint8_t Data[8][20];
extern uint8_t Func[8], DataIdx, CmdIdx, PrevIdx;
extern uint16_t NewAddr, NewQnty, NewData;
extern uint8_t NewFunc, NewByteCount;
extern uint8_t GasName[12], GasUnit[6];
extern uint8_t tData[64];

//==============================================================================
// 🚀 함수 프로토타입 - 핵심 함수들
//==============================================================================

// 초기화 및 시작
void init_TS_commu(void);
void start_TS_commu(void);

// 🚀 최적화된 처리 함수들
void ProcessBackgroundTasks(void);      // 메인 루프에서 호출
void ProcessModbusFrame(void);          // Modbus 프레임 처리
void process_tim2_interrupt(void);       // 타이머 인터럽트

// Flash 관련
HAL_StatusTypeDef EraseFlash(void);
HAL_StatusTypeDef WriteFlash(uint32_t *pData32);
HAL_StatusTypeDef ReadFlash(uint32_t *pData32);
void ReadFlashParam(void);
void SaveFlashParam(void);
void SetDefaultFlashParam(void);

// 고급 기능들
void UpdateFlowLampColors(void);
void ProcessAutoSwitch(void);
void ProcessAlarmSystem(void);
void ProcessSimulationMode(void);

// 윈도우 제어
void SetMainLamp(void);
void SetConfigLamp(void);
void SetAlarmLamp(void);
void OpenWinInputPW(void);
void OpenWinChangePW(void);

// 가스 제어
void SetSupplyGasL(void);
void SetStopGasL(void);
void SetSupplyGasR(void);
void SetStopGasR(void);
void AlarmReset(void);

// 시스템 제어
void SetSolNONC(void);
void ToggleUseBuzzer(void);
void OnChangePW(void);
void VerifyPassword(void);

// 데이터 처리
void InputPassword(uint16_t PW);
void SetCurrentWindow(uint16_t NewData);
void SetChangeWindow(uint16_t NewData);
void SetAlarmWindow(uint16_t NewData);

// 유틸리티 함수들
uint16_t CRC16(const uint8_t *nData, uint16_t wLength);
void myMemCopy(uint8_t* pDest, uint8_t* pSrc, uint16_t Len);
void SetWord(uint8_t *pDest, uint8_t *pSrc, uint8_t wordlen);
void UpdateSingleWord(uint8_t *pDest, uint8_t *pSrc);
void AdjustUnalignedSAddr(uint8_t* pDest, uint8_t* pSrc, uint16_t Qnty, uint8_t offset);

// 콜백 함수들
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

//==============================================================================
// 🚀 매크로 정의들 (성능 최적화용)
//==============================================================================
#define RXLEN           80
#define MAX_DLEN        20
#define RESPONSE_BUF_SIZE  100

// 🔥 통신 우선순위 정의
#define PRIORITY_MODBUS     1    // 최고 우선순위
#define PRIORITY_SENSOR     2    // 중간 우선순위
#define PRIORITY_FLOW       3    // 낮은 우선순위
#define PRIORITY_FLASH      4    // 최저 우선순위

// 🔥 타이밍 정의 (성능 최적화)
#define MODBUS_TIMEOUT_MS   50   // Modbus 타임아웃
#define SENSOR_INTERVAL_MS  100  // 센서 읽기 간격
#define FLOW_UPDATE_MS      200  // Flow 업데이트 간격
#define FLASH_DELAY_MS      1000 // Flash 저장 지연

#endif // TS_COMMU_H
