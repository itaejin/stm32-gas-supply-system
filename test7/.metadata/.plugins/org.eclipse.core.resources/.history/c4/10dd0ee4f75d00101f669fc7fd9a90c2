#include "TS_commu.h"

DataPT12_t PT1, PT2;
DataPT3_t PT3;
FlowLamp_t FlowLamp;
GasGraph_t GraphPT1, GraphPT2;
Password_t Password;

IO_t IOdata;

uint8_t t_point = 0;
uint8_t rsdata[] = {0x01, 0x01, 0x00, 0x01, 0x00, 0x01, 0xFF, 0xFF};
char redata[8];
uint16_t tim2_cnt;
#define RXLEN		80	// 32
#define MAX_DLEN	20
uint8_t RxBuf3[RXLEN], RxPtr3, PrcdPtr3;
uint8_t Func[8], Data[8][MAX_DLEN], DataIdx, CmdIdx, PrevIdx, rcntDataIdx, NewFunc;
uint8_t NewFunc0, NewByteCount, PrevNewFunc;
uint16_t NewAddr0, NewQnty0, NewData;
Window_t NewWindow;
uint16_t RxCRC[8], Addr[8], Qnty[8], RxCRC1[8], RxCRC2[8], TxCRC, NewAddr, NewQnty, Datalen;
uint8_t *sData;
uint8_t tData[64];			// =512/8
uint8_t TxBuf3[100];		// Modbus Response 

uint8_t GasName[2*6] = "CO2";
uint8_t GasUnit[2*3] = "mmHg";

uint16_t NoRespCnt = 0, IO_SolNONC_cnt = 0, IO_UseBuzzer = 0, RxDataOVCnt = 0;
uint16_t Data99, Data101, Data102;
uint16_t UnalignedCnt = 0, UnalignedSAddr, UnalignedQnty, UnalignedLen;

//------------------------------------------------------------------------------------------------------
// Prototype of Functions for Request == 05 (Write Single Coil)
//------------------------------------------------------------------------------------------------------
void SetMainLamp();
void SetConfigLamp();
void SetAlarmLamp();
void SetSupplyGasL();
void SetStopGasL();
void SetSupplyGasR();
void SetStopGasR();
void AlarmReset();
void SetSolNONC();
void ToggleUseBuzzer();
void OpenWinInputPW();
void OpenWinChangePW();
void OnChangePW();
//------------------------------------------------------------------------------------------------------
// Prototype of Functions for Request == 16 (Write Multiple Registers) with Quantity = 1
//------------------------------------------------------------------------------------------------------
void InputPassword(uint16_t PW);
void ChangeNewPassword(uint16_t PW);
void SetCurrentWindow(uint16_t NewData); 
void SetChangeWindow(uint16_t NewData);
void SetAlarmWindow(uint16_t NewData);
//------------------------------------------------------------------------------------------------------
void UpdateSingleWord(uint8_t *pDest, uint8_t *pSrc);
void SetWord(uint8_t *pDest, uint8_t *pSrc, uint8_t wordlen);
void myMemCopy(uint8_t* pDest, uint8_t* pSrc, uint16_t Len);
uint16_t CRC16 (const uint8_t *nData, uint16_t wLength);
void AdjustUnalignedSAddr(uint8_t* pDest, uint8_t* pSrc, uint16_t Qnty, uint8_t offset);
//------------------------------------------------------------------------------------------------------

// stm32f1xx_hal
//// #include "stm32f1xx_hal_flash.h"
void    FLASH_PageErase(uint32_t PageAddress);
// #include "stm32f1xx_hal_flash_ex.h"

#define FLASH_USER_START_ADDR  0x0803F800  	// Start address of user flash area
#define DATA_32                0x12345678  	// Example data to write
uint32_t FLASH_USER_END_ADDR;  				// End address of user flash area
// FLASH_PAGE_SIZE 0x800

uint8_t TestFlash = 0;
uint32_t FlashData = 0x1234;
__IO uint32_t data32 = 0, MemoryProgramStatus = 0;
uint32_t Address = FLASH_USER_START_ADDR;

#define GetPage(a) ((a >= 0x8000000) && (a < 0x8040000)) ? ((a>>11)&0x7F) : 0x80
uint32_t UserPage;
uint32_t LastPage;
uint32_t NbOfPages;

uint32_t errorcode;

HAL_StatusTypeDef EraseFlash()
{
  uint32_t PageError = 0;

  /* Unlock to control */         // 1) Control register unlock
  HAL_FLASH_Unlock();

  /* Calculate sector index */    // 2) 吏?곌퀬???섎뒗 ?뱁꽣???ш린 怨꾩궛
  UserPage = GetPage(FLASH_USER_START_ADDR);
  LastPage = GetPage(FLASH_USER_END_ADDR);
  NbOfPages = LastPage - UserPage + 1;

  /* Erase sectors */             // 3) ?뱁꽣 ??젣 紐낅졊
  FLASH_EraseInitTypeDef EraseInitStruct;
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks = FLASH_BANK_1;
  EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
  EraseInitStruct.NbPages = NbOfPages;

  if(HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
  {
    errorcode = HAL_FLASH_GetError();
    return HAL_ERROR;
  }

  /* Lock flash control register */    // 5) Control register lock
  HAL_FLASH_Lock();

  return HAL_OK;
}


/*
void EraseFlash0()
{
  // Unlock Flash access
  HAL_FLASH_Unlock();
  FLASH_PageErase(FLASH_USER_START_ADDR);
  // Lock Flash access
  HAL_FLASH_Lock();
}
*/

// Function to write data to flash
HAL_StatusTypeDef WriteFlash(uint32_t *pData32) {
  Address = FLASH_USER_START_ADDR;
  // Unlock Flash access
  HAL_FLASH_Unlock();

  // FLASH_PageErase(FLASH_USER_START_ADDR);

  while (Address < FLASH_USER_END_ADDR) {
  	FlashData = *pData32;
    // Program a word (32-bit) to the specified address
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, FlashData) == HAL_OK) {
    // if (FLASH_Program_HalfWord(Address, FlashData) == HAL_OK) {
    	pData32++;
      	Address += 4; // Move to the next word address
    } else {
      	// Handle error
      	HAL_FLASH_GetError();
      	return HAL_ERROR;
    }
  }

  // Lock Flash access
  HAL_FLASH_Lock();
  return HAL_OK;
}

// Function to read data from flash
HAL_StatusTypeDef ReadFlash(uint32_t *pData32) {
  Address = FLASH_USER_START_ADDR;

  while (Address < FLASH_USER_END_ADDR) {
    *pData32 = *(__IO uint32_t*)Address; // Read data from flash

	pData32++;
    Address += 4; // Move to the next word address
  }

  if (MemoryProgramStatus > 0) {
    return HAL_ERROR; // Data mismatch
  }
  return HAL_OK;
}

FlashParam_t fParam;

void SetDefaultFlashParam()
{
	myMemCopy(fParam.GasName, (uint8_t*)"CO2", 4);
	myMemCopy(fParam.GasUnit, (uint8_t*)"mmHg", 5);
	fParam.Password = 0;
	fParam.PT1Data[0] = 40;		// GasAlarmReplace
	fParam.PT1Data[1] = 20;		// GasPrepReplace
	fParam.PT1Data[2] = 0;		// Offset
	fParam.PT1Data[3] = 10;		// Min. Value of Sensor
	fParam.PT1Data[4] = 120;	// Max. Value of Sensor
	fParam.PT1Data[5] = 110;	// Max. Value of Gas Chaeging
	fParam.PT2Data[0] = 42;		// GasAlarmReplace
	fParam.PT2Data[1] = 22;		// GasPrepReplace
	fParam.PT2Data[2] = 0;		// Offset
	fParam.PT2Data[3] = 12;		// Min. Value of Sensor
	fParam.PT2Data[4] = 122;	// Max. Value of Sensor
	fParam.PT2Data[5] = 112;	// Max. Value of Gas Chaeging
	fParam.PT3Data[0] = 24;		// Min. Supply Value.
	fParam.PT3Data[2] = 0;		// Offset
	fParam.PT3Data[3] = 124;	// Max. Value of Sensor

	EraseFlash();		
	WriteFlash((uint32_t*)&fParam);
	
}

void ReadFlashParam()
{
	ReadFlash((uint32_t*)&fParam);
	if(fParam.Password == 0xFFFF) {
		SetDefaultFlashParam();
	} else {
		myMemCopy(GasName, fParam.GasName, 12);
		myMemCopy(GasUnit, fParam.GasUnit, 6);
		Password.CurValue = fParam.Password;
		PT1.Low = fParam.PT1Data[0];		// PT1 : low value
		PT1.LowLow = fParam.PT1Data[1];		// PT1 : low low value
		PT1.Offset = fParam.PT1Data[2];		// PT1 : Offset
		PT1.MinSValue = fParam.PT1Data[3];	// PT1 : Min. Value of Sensor
		PT1.MaxSValue = fParam.PT1Data[4];	// PT1 : Max. Value of Sensor
		PT1.MaxValue = fParam.PT1Data[5];	// PT1 : Max. Value of Gas Chaeging
		
		PT2.Low = fParam.PT2Data[0];		// PT2 : low value
		PT2.LowLow = fParam.PT2Data[1];		// PT2 : low low value 
		PT2.Offset = fParam.PT2Data[2];		// PT2 : Offset
		PT2.MinSValue = fParam.PT2Data[3];	// PT2 : Min. Value of Sensor
		PT2.MaxSValue = fParam.PT2Data[4];	// PT2 : Max. Value of Sensor
		PT2.MaxValue = fParam.PT2Data[5];	// PT2 : Max. Value of Gas Chaeging
		
		PT3.MinSValue = fParam.PT3Data[0];	// PT3 : min value
		PT3.Offset = fParam.PT3Data[2];		// PT3 : min value
		PT3.MaxSValue = fParam.PT3Data[3];	// PT3 : Max. Value of Sensor
	}
}

void init_TS_commu()
{
	TxBuf3[0] = 1;		/// Slave Address
	sData = &TxBuf3[3];
	  
	NewWindow = Main;
	  
	RxPtr3 = PrcdPtr3 = 0;
	DataIdx = CmdIdx = 0;
	
	FLASH_USER_END_ADDR =FLASH_USER_START_ADDR+sizeof(FlashParam_t);  
//	FLASH_USER_END_ADDR =0x0803FFFF;  
	ReadFlashParam();

	  
	PT1.Value = 80;		// PT1 : value
	
	PT2.Value = 82;		// PT2 : value
	
	PT3.Value = 80;		// PT3 : value

	GraphPT1.Value = PT1.Value;
	GraphPT1.MinValue = 20;
	GraphPT1.MaxValue = 120;
	GraphPT1.LowLimit = 20;
	GraphPT1.UpperLimit = 50;
	GraphPT1.TargetValue = 70;
	GraphPT1.MaxValue = 120;
	
	GraphPT2.Value = PT2.Value;
	GraphPT2.MinValue = 50;
	GraphPT2.MaxValue = 150;
	
	Password.CurValue = 1234;
	Password.SavedValue = 4321;
}

//------------------------------------------------------------------------------------------------------
void start_TS_commu()
{
	HAL_UART_Receive_IT(&huart3, RxBuf3+RxPtr3, 1);
}

//------------------------------------------------------------------------------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart -> Instance == USART3) {
		RxPtr3 = (RxPtr3+1) & (RXLEN-1);
		HAL_UART_Receive_IT(&huart3, RxBuf3+RxPtr3, 1);
	}
}

//------------------------------------------------------------------------------------------------------
void process_tim2_interrupt()
{
	if(PrcdPtr3 != RxPtr3) {
		do {
			if(DataIdx < MAX_DLEN) 	Data[CmdIdx][DataIdx++] = RxBuf3[PrcdPtr3];
			else					RxDataOVCnt++;
			PrcdPtr3 = (PrcdPtr3+1) & (RXLEN-1);
		} while(PrcdPtr3 != RxPtr3);
	} else if(DataIdx != 0) {
		if(DataIdx >= 8) {
			rcntDataIdx = DataIdx;
			if(Data[CmdIdx][0] == 1) {
				NewFunc = Func[CmdIdx] = Data[CmdIdx][1];
				NewAddr = Addr[CmdIdx] = (Data[CmdIdx][2] << 8) + Data[CmdIdx][3];
				if(NewFunc == 5) {
					NewData = Qnty[CmdIdx] = (Data[CmdIdx][4] << 8) + Data[CmdIdx][5];
				} else if(NewFunc == 16) {
					NewQnty = Qnty[CmdIdx] = (Data[CmdIdx][4] << 8) + Data[CmdIdx][5];
					NewByteCount = Data[CmdIdx][6];
					if(NewByteCount == 2)
						NewData = (Data[CmdIdx][7] << 8) + Data[CmdIdx][8];
				} else {
					NewQnty = Qnty[CmdIdx] = (Data[CmdIdx][4] << 8) + Data[CmdIdx][5];
				}
				RxCRC[CmdIdx] = *(uint16_t*)&Data[CmdIdx][DataIdx-2];
////				RxCRC1[CmdIdx] = calculateCRC(&Data[CmdIdx][0], DataIdx-2);
				RxCRC2[CmdIdx] = CRC16(&Data[CmdIdx][0], DataIdx-2);
				PrevIdx = CmdIdx;
				CmdIdx = (CmdIdx+1) & 7;
			}
		}
		DataIdx = 0;
	} else {
		if(NewFunc) {
//------------------------------------------------------- // Read Holding Registers : WORD
			if(NewFunc == 3) {	
				Datalen = (uint8_t)NewQnty*2;	
				TxBuf3[1] = NewFunc;
				TxBuf3[2] = Datalen;		// Byte Count		

				if((NewAddr == 0) && ((NewQnty == 31) || ((NewQnty == 34)))) {
					// 가스:PT1/2/3
					SetWord(sData, GasName, 6);
					SetWord(sData+12, GasUnit, 3);
					SetWord(sData+18, (uint8_t*)&PT1, 10);
					SetWord(sData+38, (uint8_t*)&PT2, 10);
					SetWord(sData+58, (uint8_t*)&PT3, 6);

Modbus_Reply_CalcCRC:
					TxCRC =  CRC16(TxBuf3, Datalen+3);
					TxBuf3[Datalen+3] = (uint8_t)(TxCRC & 0xFF);
					TxBuf3[Datalen+4] = (uint8_t)(TxCRC >> 8);
Modbus_Send_Reply:					
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
					HAL_UART_Transmit(&huart3, TxBuf3, Datalen+5, HAL_MAX_DELAY);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
				} else if((NewAddr == 101) && (NewQnty == 1)) {
					// 변경화면[102]
					SetWord(sData, (uint8_t*)&NewWindow, 1);
					goto Modbus_Reply_CalcCRC;
				} else if((NewAddr == 103) && (NewQnty == 1)) {
					// 알람윈도우[104]
//					if(NewWindow == Alarm)  *(uint16_t*)sData = 0;
//					else					*(uint16_t*)sData = 1;
					SetWord(sData, (uint8_t*)&NewWindow, 1);	// ??????
					goto Modbus_Reply_CalcCRC;
				} else if((NewAddr == 79) && (NewQnty == 10)) {
					// 흐름램프
					SetWord(sData, (uint8_t*)&FlowLamp, 10);
					goto Modbus_Reply_CalcCRC;
				} else if((NewAddr == 89) && (NewQnty == 1)) {
					// 흐름램프
					SetWord(sData, (uint8_t*)&Password, 1);
					goto Modbus_Reply_CalcCRC;
				} else if((NewAddr == 90) && (NewQnty == 2)) {
					// 흐름램프
					SetWord(sData, (uint8_t*)&Password, 2);
					goto Modbus_Reply_CalcCRC;
				} else if((NewAddr == 109) && (NewQnty == 16)) {
					// graph of PT1 and PT2
					SetWord(sData, (uint8_t*)&GraphPT1, 6);
					SetWord(sData+20, (uint8_t*)&GraphPT2, 6);
					goto Modbus_Reply_CalcCRC;
				} else {
					goto Cmd_Not_Supported;
				}

			}
//------------------------------------------------------- // Read Discrete Inputs : IO(BIT)			 
			else if(NewFunc == 2) {
				Datalen = (uint8_t)NewQnty/8;
				TxBuf3[1] = NewFunc;
				TxBuf3[2] = Datalen;		// Byte Count		

				if((NewAddr + NewQnty) / 8 < sizeof(IOdata)) {
					if((NewAddr % 8) == 0) {	// Aligned Start Addess
						uint8_t *pIOBuf = (uint8_t*)&IOdata;
						myMemCopy(sData, pIOBuf + (NewAddr/8), (uint16_t)(Datalen));
					} else {					// Unaligned Start Addesss
						uint8_t *pIOBuf = (uint8_t*)&IOdata;
						uint8_t Templen = (NewAddr + NewQnty)/8 - (NewAddr/8) + 1;
						UnalignedCnt++, UnalignedSAddr = NewAddr, UnalignedQnty = NewQnty, UnalignedLen = Templen;
						myMemCopy(tData, pIOBuf + (NewAddr/8), (uint16_t)(Templen));
						AdjustUnalignedSAddr(sData, tData, NewQnty, (uint8_t)(NewAddr % 8) );
					}				
					goto Modbus_Reply_CalcCRC;
				} else {
					goto Cmd_Not_Supported;
				}
			} 
//------------------------------------------------------- // Write Single Coil : IO(BIT)	
			else if(NewFunc == 5) {		
				if(NewAddr == 0) 		SetMainLamp(); 		// 메인화면으로
				else if(NewAddr == 2) 	SetConfigLamp();	// 설정화면으로
				else if(NewAddr == 8)	SetAlarmLamp();		// 알람화면으로 
				else if(NewAddr == 9)	SetSupplyGasL();	// 가스공급 (좌)
				else if(NewAddr == 10) 	SetStopGasL();		// 가스공급정지 (좌)
				else if(NewAddr == 19) 	SetSupplyGasR();	// 가스공급 (우)
				else if(NewAddr == 20)	SetStopGasR();		// 가스공급정지 (우)
				else if(NewAddr == 29) 	AlarmReset();		// 알람 리셋 
				else if(NewAddr == 79) 	SetSolNONC();		// SOL NO/NC
				else if(NewAddr == 80) 	ToggleUseBuzzer();	// 부저사용/미사용
				else if(NewAddr == 89) 	InputPassword(NewData);	// 비밀번호 입력 
				// else if(NewAddr == 90) 	ConfirmChangePW();	// 비밀번호 변경 확인 ?
				else if(NewAddr == 91) 	OpenWinChangePW();	// 비밀번호 변경창 열기 ?
				else if(NewAddr == 92) 	OnChangePW();		// 비밀번호 변경
				// else if(NewAddr == 109) 					// 가스공급(좌) 램프
				else 					goto Cmd_Not_Supported;

Modbus_Reply_Echo:
				Datalen = 4;
				TxBuf3[1] = NewFunc;
				myMemCopy(&TxBuf3[2], &Data[PrevIdx][0], (uint16_t)(Datalen));
				TxCRC =  CRC16(TxBuf3, Datalen+2);
				TxBuf3[Datalen+2] = (uint8_t)(TxCRC & 0xFF);
				TxBuf3[Datalen+3] = (uint8_t)(TxCRC >> 8);
				Datalen--;
				goto Modbus_Send_Reply;	
					
			} 
//------------------------------------------------------- // Write Multiple Registers : WORD
			else if(NewFunc == 16) {		
				if(NewByteCount==2) {
					if(NewWindow == Config) {
						if(NewAddr < 79) {
							UpdateSingleWord((uint8_t*)&PT1, (uint8_t*)&NewData);		
/*
					SetWord(sData, GasName, 6);
					SetWord(sData+12, GasUnit, 3);
					SetWord(sData+18, (uint8_t*)&PT1, 10);
					SetWord(sData+38, (uint8_t*)&PT2, 10);
					SetWord(sData+58, (uint8_t*)&PT3, 6);
*/
						} else if(NewAddr == 99) 	SetCurrentWindow(NewData); 
						else if(NewAddr == 101) 	SetChangeWindow(NewData);
						else if(NewAddr == 102) 	SetAlarmWindow(NewData);
						else						goto Cmd_Not_Supported;
						
					} else if(NewAddr == 91)			InputPassword(NewData); 
					else if(NewAddr == 99) 		SetCurrentWindow(NewData); 
					else if(NewAddr == 101) 	SetChangeWindow(NewData);
					else if(NewAddr == 102) 	SetAlarmWindow(NewData);
					else 						goto Cmd_Not_Supported;
				} else {
					goto Cmd_Not_Supported;
				}
				goto Modbus_Reply_Echo;
			} 
//------------------------------------------------------- // Record Not Yet Supported Request
			else {
Cmd_Not_Supported:
				NoRespCnt++;
				NewFunc0 = NewFunc;
				NewAddr0 = NewAddr;
				NewQnty0 = NewQnty;
				NewData = NewData;
			}
//------------------------------------------------------- // End of Modbus Send Reply:			
			PrevNewFunc = NewFunc;
			NewFunc = 0;
		}	// end of if(NewFunc)
	}
}

//------------------------------------------------------------------------------------------------------
// Functions for Request == 16 (Write Multiple Registers) with Quantity = 1
//------------------------------------------------------------------------------------------------------
void InputPassword(uint16_t PW)
{
	if(Password.CurValue == PW) IOdata.ConfirmPWSW = 1;
	else 						IOdata.ConfirmPWSW = 0;
}

void SetCurrentWindow(uint16_t NewData)
{
	Data99 = NewData;
	if(NewData == Main)	 	SetMainLamp();;
	if(NewData == Config) 	SetConfigLamp();
	if(NewData == Alarm) 	SetAlarmLamp();
	if(NewData == PW_Input) 	OpenWinInputPW();
	if(NewData == PW_Change) 	OpenWinChangePW();
	if(NewData == PW_Change_OK) 	SetMainLamp();
}

void SetChangeWindow(uint16_t NewData)
{
	Data101 = NewData;
}

void SetAlarmWindow(uint16_t NewData)
{
	Data102 = NewData;
	if(NewData == Main)	 	SetMainLamp();;
	if(NewData == Config) 	SetConfigLamp();;
	if(NewData == Alarm) 	SetAlarmLamp();
}

//------------------------------------------------------------------------------------------------------
// Functions for Request == 05 (Write Single Coil)
//------------------------------------------------------------------------------------------------------
void SetMainLamp()
{	
	NewWindow = Main;	// 11;
	IOdata.MainWinSW = 1;
	IOdata.ConfigWinSW = 0;
	IOdata.AlarmWinSW = 0;
	IOdata.MainWinLamp = 1;
	IOdata.ConfigWinLamp = 0;
	IOdata.AlarmWinLamp = 0;
}

void SetConfigLamp()
{	
	NewWindow = Config;	// 41;
	IOdata.MainWinSW = 0;
	IOdata.ConfigWinSW = 1;
	IOdata.AlarmWinSW = 0;
	IOdata.MainWinLamp = 0;
	IOdata.ConfigWinLamp = 1;
	IOdata.AlarmWinLamp = 0;
}

void SetAlarmLamp()
{	
	NewWindow = Alarm;	// 45;
	IOdata.MainWinSW = 0;
	IOdata.ConfigWinSW = 0;
	IOdata.AlarmWinSW = 1;
	IOdata.MainWinLamp = 0;
	IOdata.ConfigWinLamp = 0;
	IOdata.AlarmWinLamp = 1;	
}

void SetSupplyGasL()
{	
	IOdata.SupplyGasLSW = 1;
	IOdata.SupplyGasRSW = 0;
	IOdata.StopGasLLamp = 0;
	IOdata.StopGasRLamp = 0;
}

void SetStopGasL()
{	
	IOdata.SupplyGasLSW = 0;
	IOdata.SupplyGasRSW = 0;
	IOdata.StopGasLLamp = 1;
	IOdata.StopGasRLamp = 0;
}

void SetSupplyGasR()
{	
	IOdata.SupplyGasLSW = 0;
	IOdata.SupplyGasRSW = 1;
	IOdata.StopGasLLamp = 0;
	IOdata.StopGasRLamp = 0;
}

void SetStopGasR()
{	
	IOdata.SupplyGasLSW = 0;
	IOdata.SupplyGasRSW = 0;
	IOdata.StopGasLLamp = 0;
	IOdata.StopGasRLamp = 1;
}

void AlarmReset()
{	
	
}

void SetSolNONC()
{
	IO_SolNONC_cnt++; 

}

void ToggleUseBuzzer()
{
	IO_UseBuzzer++; 

}

void OpenWinInputPW()
{
	NewWindow = PW_Input;	// 111;	
}

void OpenWinChangePW()
{
	NewWindow = PW_Change;	// 112;	
}

void OnChangePW()
{
	NewWindow = PW_Change_OK;	// 114;		
}

//------------------------------------------------------------------------------------------------------
void SetWord(uint8_t *pDest, uint8_t *pSrc, uint8_t wordlen) 
{
	for(int i=0; i < wordlen; i++) {
		*(pDest+1) = *pSrc, *pDest = *(pSrc+1);
		pDest+=2, pSrc+=2;
	}
}

void UpdateSingleWord(uint8_t *pDest, uint8_t *pSrc) 
{
	*(pDest+1) = *pSrc, *pDest = *(pSrc+1);
}

void AdjustUnalignedSAddr(uint8_t* pDest, uint8_t* pSrc, uint16_t Qnty, uint8_t offset)
{
	uint16_t dLen = Qnty / 8, sLen = (Qnty + offset) / 8;
	for(int i = 0; i < dLen-1; i++) {
		*pDest = (uint8_t)( ((*pSrc) >> offset) + (*(pSrc+1) << (8-offset)));
		pDest++, pSrc++;
	}
	if(dLen == sLen) 	*pDest = (uint8_t)((*pSrc) >> offset);
	else 			 	*pDest = (uint8_t)( ((*pSrc) >> offset) + (*(pSrc+1) << (8-offset)));
}
//------------------------------------------------------------------------------------------------------
#if 0
unsigned short calculateCRC(unsigned char *crcdata, unsigned int len)
{

	unsigned int i, j;
	unsigned short crc = 0;

		for(i=0; i<len; i++) {
			crc = crc ^ ((*crcdata) << 8);
			crcdata++;
			for(j=0; j<8; j++) {
				if(crc & 0x8000) {
					crc = (crc << 1) ^ 0x1021;
				} else {
					crc = (crc << 1);
				}
			}
		}
		return crc;
}
#endif

uint16_t CRC16 (const uint8_t *nData, uint16_t wLength)
{
static const uint16_t wCRCTable[] = {
   0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
   0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
   0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
   0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
   0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
   0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
   0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
   0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
   0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
   0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
   0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
   0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
   0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
   0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
   0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
   0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
   0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
   0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
   0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
   0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
   0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
   0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
   0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
   0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
   0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
   0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
   0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
   0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
   0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
   0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
   0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
   0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 };

uint8_t nTemp;
uint16_t wCRCWord = 0xFFFF;

   while (wLength--)
   {
      nTemp = *nData++ ^ wCRCWord;
      wCRCWord >>= 8;
      wCRCWord  ^= wCRCTable[nTemp];
   }
   return wCRCWord;
} // End: CRC16

void myMemCopy(uint8_t* pDest, uint8_t* pSrc, uint16_t Len) {
	for(int i=0; i<Len; i++) *pDest++ = *pSrc++;
}
//------------------------------------------------------------------------------------------------------
