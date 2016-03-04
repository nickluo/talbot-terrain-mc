#ifndef __COMMAND_PARSER_H
#define __COMMAND_PARSER_H

#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif

struct MotorConfig
{
	int LW;
	int RW;
	int Speed;
};

enum CommandEnum
{
	SetMotorCommand,
	GetMotorCommand,
	TestModeCommand,
	InvalidCommand = 0xff,
};

typedef void (*CommandHandler)(enum CommandEnum command, const void *params);

int CommandParse(uint8_t *data, int size, CommandHandler handler);	


#ifdef __cplusplus
 }
#endif
 
#endif
