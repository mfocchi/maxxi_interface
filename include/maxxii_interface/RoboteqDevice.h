#ifndef __RoboteqDevice_H_
#define __RoboteqDevice_H_

using namespace std;
#include <vector>
#include <string>
#include "Constants.h"
#include "ErrorCodes.h"

using namespace std;

#define RESET   "\033[0m"
#define RED     "\033[31m"      /* Red */
#define BLUE     "\033[34m"      /* Blue */

string ReplaceString(string source, string find, string replacement);
void sleepms(int milliseconds);

class RoboteqDevice
{
private:
	int device_fd;
	int fd0;
	int handle;

protected:
	void InitPort();

	int Write(string str);
	int ReadAll(string &str);

	int IssueCommand(string commandType, string command, string args, int waitms, string &response, bool isplusminus = false);
	int IssueCommand(string commandType, string command, int waitms, string &response, bool isplusminus = false);

public:
	bool IsConnected();
	int Connect(string port);
	void Disconnect();

	int SetConfig(int configItem, int index, int value);
	int SetConfig(int configItem, int value);

	int SetCommand(int commandItem, int index, int value);
	int SetCommand(int commandItem, int value);
	int SetCommand(int commandItem);

	int GetConfig(int configItem, int index, int &result);
	int GetConfig(int configItem, int &result);

	int GetValue(int operatingItem, int index, std::string &result);
	int GetValue(int operatingItem, std::string &result);

	int MixedModeMotorMove(float throttle, float steering, string &response);

	RoboteqDevice();
	~RoboteqDevice();
};

#endif
