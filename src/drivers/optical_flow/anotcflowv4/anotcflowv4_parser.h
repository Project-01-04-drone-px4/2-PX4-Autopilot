#ifndef ANOTCFLOWV4_PARSER_H
#define ANOTCFLOWV4_PARSER_H

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define ANOTCFLOWV4_PROTOCOL_HEAD 0xAA
#define ANOTCFLOWV4_PROTOCOL_D_ADDR 0xFF
#define ANOTCFLOWV4_PROTOCOL_FUNCTION_ID_OF 0x51
#define ANOTCFLOWV4_PROTOCOL_LEN_OF_ORIGIN 0x05
#define ANOTCFLOWV4_PROTOCOL_MODE 0x00
#define ANOTCFLOWV4_PROTOCOL_VALID_STATE 0x01

enum class ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN {
	STATE1_HEAD = 0,
	STATE2_D_ADDR,
	STATE3_FUNCTION_ID,
	STATE4_LEN,
	STATE5_MODE_OF_ORIGIN,
	STATE6_VALID_STATE_OF_ORIGIN,
	STATE7_DX_0_OF_ORIGIN,
	STATE8_DY_0_OF_ORIGIN,
	STATE9_QUALITY_OF_ORIGIN,
	STATE10_SC_VALUE_OF_ORIGIN,
	STATE11_AC_VALUE_OF_ORIGIN
};

typedef struct {
	int dx_0;
	int dy_0;
	uint8_t quality;
} ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN_STRUCT;

int anotcflowv4_parse_of_origin(char c, char* parserbuf, unsigned* parserbuf_index,
    ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN* state, ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN_STRUCT* dist);

#endif // !ANOTCFLOWV4_PARSER_H