#include "error_codes.h"

void printErrorCode(int code)
{
    switch (code)
    {
    case FAILED_SET_MOTOR_CMD:
        std::cout << "Failed to send motor command CODE:["<< code << "]" << std::endl;
        break;
    case FAILED_ENCODER_RESET:
        std::cout << "Failed to reset encoder CODE:["<< code << "]" << std::endl;
        break;
    case FAILED_D1_OUT:
        std::cout << "Failed to send D1 out CODE:["<< code << "]" << std::endl;
        break;
    case FAILED_D2_OUT:
        std::cout << "Failed to send D2 out CODE:["<< code << "]" << std::endl;
        break;
    case FAILED_CONNECTION:
        std::cout << "Failed to setup connection CODE:["<< code << "]" << std::endl;
        break;
    case FAILED_READ_ENCODER_COUNT:
        std::cout << "Failed to read encoder count CODE:["<< code << "]" << std::endl;
        break;
    case FAILED_READ_ENCODER_SPEED:
        std::cout << "Failed to read encoder speed CODE:["<< code << "]" << std::endl;
        break;
    case FAILED_READ_FEEDBACK:
        std::cout << "Failed to read feedback from motor CODE:["<< code << "]" << std::endl;
        break;
    case FAILED_READ_FIRMWARE:
        std::cout << "Failed to read firmware CODE:["<< code << "]" << std::endl;
        break;
    case WORNG_INDEXING:
        std::cout << "Index is not allowed, choose 0 or 1 CODE:["<< code << "]" << std::endl;
        break;
    case FAILED_READ_MOTOR_CURRENT:
        std::cout << "Failed to read motor current CODE:["<< code << "]" << std::endl;
        break;
    case FAILED_READ_MOTOR_VOLTAGE:
        std::cout << "Failed to read motor voltage CODE:["<< code << "]" << std::endl;
        break;
    case FAILED_READ_TEMPERATURE:
        std::cout << "Failed to read temperature CODE:["<< code << "]" << std::endl;
        break;
    case NEED_VELOCITY_MODE:
        std::cout << "ERROR: trying to use velocity command in torque mode CODE:["<< code << "]" << std::endl;
        break;
    case NEED_TORQUE_MODE:
        std::cout << "ERROR: trying to use torque command in velocity mode CODE:["<< code << "]" << std::endl;
        break;
    case WRONG_SIZE_JOINTSTATE_CMD:
        std::cout << "ERROR: JointState message does not have exactly 2 elements in the 'position' array. CODE:["<< code << "]"<< std::endl;
        break;
    case DIFFERENT_MOTOR_MODES:
        std::cout << "ERROR: trying to control the motors with different mode. CODE:["<< code << "]"<< std::endl;
        break;
    default:
        std::cout << "Code not found CODE:["<< code << "]" << std::endl;
        break;
    }
}