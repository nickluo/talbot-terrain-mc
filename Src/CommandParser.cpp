#include "CommandParser.h"

#include <string>
#include <cctype>
#include <cstdlib>

using namespace std;

static int command_index = -1;
static MotorConfig mc;
static uint8_t testMode = 0;

const std::string Commands[] = {"setmotor", "getmotor", "testmode", };

const std::string Parameters[] = {"lwheeldist", "rwheeldist", "speed"};

const std::string TestModeParameters[] = {"off", "on", "debug"};

enum ParseState
{
	ParseState_Command,
	ParseState_Parameter,
	ParseState_Value,
};

static ParseState state = ParseState_Command;

void Parse(const std::string &s)
{
	static int pindex = -1;
	if (s.length()==0)
		return;
	int i;
	switch (state)
	{
		case ParseState_Command:	
			command_index = -1;
			for(i=0;i<3;++i)
			{
				if (s==Commands[i])
				{
					command_index = i;
					if (i==0)
					{
						pindex = -1;
						state = ParseState_Value;
					}
					else
						state = ParseState_Parameter;
					break;
				}
			}
			break;
		case ParseState_Parameter:
			if (command_index==0)		//SetMotor
			{
//				for(i=0;i<3;++i)
//				{
//					if (s==Parameters[i])
//					{
//						pindex = i;
//						state = ParseState_Value;
//						break;
//					}
//				}
			}
			else if (command_index==2)	//TestMode
			{
				for(i=0;i<3;++i)
				{
					if (s==TestModeParameters[i])
					{
						testMode = i;
						break;
					}
				}
			}
			break;
		case ParseState_Value:
			if (command_index!=0)
				break;
			int val = strtol(s.c_str(),NULL,0);
			++pindex;
			if (pindex==0)
				mc.LW = val;
			else if (pindex==1)
				mc.RW = val;
			else if (pindex==2)
				mc.Speed = val;
			//state = ParseState_Parameter;
			state = ParseState_Value;
			break;
	}
}

int CommandParse(uint8_t *data, int size, CommandHandler handler)
{
	if (size==0)
	{
		handler(InvalidCommand, NULL);
		return 1;
	}
	command_index = -1;
	string str(reinterpret_cast<char *>(data), size);
  do
  {
		size_t found = str.find_first_of(' ');
		if (found==string::npos)
			found = str.length();
    string word = str.substr(0, found);
		for (string::size_type i=0; i<word.length(); ++i)
			word[i] = tolower(word[i]);
		
		Parse(word);

		str.erase(0, found+1);
  } while (str.length()>0);
	
	state = ParseState_Command;
	
	if (handler!=NULL)
	{
		if (command_index == 0)
			handler((CommandEnum)command_index, &mc);
		else if (command_index == 1)
			handler((CommandEnum)command_index, NULL);
		else if (command_index == 2)
			handler((CommandEnum)command_index, &testMode);
		else
			handler(InvalidCommand, NULL);
	}
	return 0;
}
