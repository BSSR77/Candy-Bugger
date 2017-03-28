/*
 * errors.h
 *
 *  Created on: Jan 4, 2017
 *      Author: jamesliu
 */

#ifndef ERRORS_H_
#define ERRORS_H_

typedef struct{
	uint8_t *str;
	uint16_t length;
}ERR_Msg_t;

/*
 * HAL CAN errors are defined as such
 * #define   HAL_CAN_ERROR_NONE        ((uint32_t)0x00000000)    !< No error
 * #define   HAL_CAN_ERROR_EWG         ((uint32_t)0x00000001)    !< EWG error
#define   HAL_CAN_ERROR_EPV         ((uint32_t)0x00000002)    !< EPV error
#define   HAL_CAN_ERROR_BOF         ((uint32_t)0x00000004)    !< BOF error
#define   HAL_CAN_ERROR_STF         ((uint32_t)0x00000008)    !< Stuff error
#define   HAL_CAN_ERROR_FOR         ((uint32_t)0x00000010)    !< Form error
#define   HAL_CAN_ERROR_ACK         ((uint32_t)0x00000020)    !< Acknowledgment error
#define   HAL_CAN_ERROR_BR          ((uint32_t)0x00000040)    !< Bit recessive        *
#define   HAL_CAN_ERROR_BD          ((uint32_t)0x00000080)    !< LEC dominant         *
#define   HAL_CAN_ERROR_CRC         ((uint32_t)0x00000100)   !< LEC transfer error   *
 */

uint8_t ERR_invalidHexCharMsg[] = "Invalid hexadecimal character received!";
uint8_t ERR_invalidCommandMsg[] = "Invalid UART command received!";
uint8_t ERR_invalidRtrBitsMsg[] = "Invalid RTR bits! Use 0b00[RTR][RTRm].";
uint8_t ERR_idIsOutOfRangeMsg[] = "ID is out of range for your configuration!";
uint8_t ERR_CanErrorEWGMsg[] = "CAN Error Warning error!";
uint8_t ERR_CanErrorEPVMsg[] = "CAN Error Passive error!";
uint8_t ERR_CanErrorBOFMsg[] = "CAN Bus Off error!";
uint8_t ERR_CanErrorSTFMsg[] = "What's a CAN STF error?";
uint8_t ERR_CanErrorFORMsg[] = "What's a CAN FOR error?";
uint8_t ERR_CanErrorACKMsg[] = "CAN ACK error!";
uint8_t ERR_CanErrorBRMsg[] = "What's a CAN BR error?";
uint8_t ERR_CanErrorBDMsg[] = "What's a CAN BD error?";
uint8_t ERR_CanErrorCRCMsg[] = "CAN CRC error!";
uint8_t ERR_invalidDecCharMsg[] = "Invalid decimal character received!";
uint8_t ERR_invalidFilterNumMsg[] = "Filter Number out of range!";

typedef enum{
	ERR_invalidHexChar,
	ERR_invalidCommand,
	ERR_invalidRtrBits,
	ERR_idIsOutOfRange,
	ERR_CanErrorEWG,
	ERR_CanErrorEPV,
	ERR_CanErrorBOF,
	ERR_CanErrorSTF,
	ERR_CanErrorFOR,
	ERR_CanErrorACK,
	ERR_CanErrorBR,
	ERR_CanErrorBD,
	ERR_CanErrorCRC,
	ERR_invalidDecChar,
	ERR_invalidFilterNum,
}LogErrorCode_t;

typedef enum{
	ERR_Fatal,
	ERR_Warn,
	ERR_Abort,
	ERR_CanRst
}LogErrorLevel_t;

ERR_Msg_t Err_Messages[] = { //MUST follow order of LogErrorCode_t
		{ERR_invalidHexCharMsg, sizeof(ERR_invalidHexCharMsg)-1},
		{ERR_invalidCommandMsg, sizeof(ERR_invalidCommandMsg)-1},
		{ERR_invalidRtrBitsMsg, sizeof(ERR_invalidRtrBitsMsg)-1},
		{ERR_idIsOutOfRangeMsg, sizeof(ERR_idIsOutOfRangeMsg)-1},
		{ERR_CanErrorEWGMsg, sizeof(ERR_CanErrorEWGMsg)-1},
		{ERR_CanErrorEPVMsg, sizeof(ERR_CanErrorEPVMsg)-1},
		{ERR_CanErrorBOFMsg, sizeof(ERR_CanErrorBOFMsg)-1},
		{ERR_CanErrorSTFMsg, sizeof(ERR_CanErrorSTFMsg)-1},
		{ERR_CanErrorFORMsg, sizeof(ERR_CanErrorFORMsg)-1},
		{ERR_CanErrorACKMsg, sizeof(ERR_CanErrorACKMsg)-1},
		{ERR_CanErrorBRMsg, sizeof(ERR_CanErrorBRMsg)-1},
		{ERR_CanErrorBDMsg, sizeof(ERR_CanErrorBDMsg)-1},
		{ERR_CanErrorCRCMsg, sizeof(ERR_CanErrorCRCMsg)-1},
		{ERR_invalidDecCharMsg, sizeof(ERR_invalidDecCharMsg)-1},
		{ERR_invalidFilterNumMsg, sizeof(ERR_invalidFilterNumMsg)-1}
};

#endif /* ERRORS_H_ */
