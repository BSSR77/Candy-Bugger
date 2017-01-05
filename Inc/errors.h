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
 * Psst.
 * Try to keep all the error names the same length.
 * It looks so tidy.
 */

uint8_t ERR_invalidHexCharMsg[] = "Invalid hexadecimal character received!";
uint8_t ERR_invalidCommandMsg[] = "Invalid UART command received!";
uint8_t ERR_invalidRtrBitsMsg[] = "Invalid RTR bits! Use 0b00[RTR][RTRm].";
uint8_t ERR_idIsOutOfRangeMsg[] = "ID is out of range for your configuration!";

typedef enum{
	ERR_invalidHexChar,
	ERR_invalidCommand,
	ERR_invalidRtrBits,
	ERR_idIsOutOfRange
}LogErrorCode_t;

typedef enum{
	ERR_Fatal,
	ERR_Warn,
	ERR_Abort
}LogErrorLevel_t;

ERR_Msg_t Err_Messages[] = { //MUST follow order of LogErrorCode_t
		{ERR_invalidHexCharMsg, sizeof(ERR_invalidHexCharMsg)-1},
		{ERR_invalidCommandMsg, sizeof(ERR_invalidCommandMsg)-1},
		{ERR_invalidRtrBitsMsg, sizeof(ERR_invalidRtrBitsMsg)-1},
		{ERR_idIsOutOfRangeMsg, sizeof(ERR_idIsOutOfRangeMsg)-1}
};

#endif /* ERRORS_H_ */
