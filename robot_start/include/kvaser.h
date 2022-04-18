//
// Created by lab306 on 2022/1/11.
//

#ifndef SRC_KVASER_H
#define SRC_KVASER_H

#include "canlib.h"
//#include "delay.h"

#include <string>
#define PI 3.1415

class Kvaser final
{
public:
    enum Mode
    {
        TORQUE_MODE = 1, // 力矩/电流模式
        SPEED_MODE = 2, // 速度模式
        MICRO_STEPPING_MODE = 3,
        DUAL_FEEDBACK_POSITION_MODE = 4,
        SINGLE_FEEDBACK_POSITION_MODE = 5, // 位置模式
    };

    enum ErrorType
    {
        INIT_ERROR = 1,
        CONNECT_ERROR = 2,
        DISCONNECT_ERROR = 3,
        SEND_MESSAGE_ERROR = 4,
        RELEASE_ERROR = 5,
    };

    int canInit(int channel_number = DEFAULT_CHANNEL_NUMBER);

    int canRelease();

    int canSend(uint8_t *msg, long can_id, unsigned int dlc);

    int connectMotor(long can_id);

    int motorEnable(long can_id);

    int motorDisable(long can_id);

    int modeChoose(long can_id, Mode mode);

    int positionMode(long can_id, int32_t position, int32_t speed);

    int speedMode(long can_id, int32_t speed);

    int torqueMode(long can_id, float torque);

    float protect(float torque);

    int beginMovement(long can_id);

    float getPosition(long can_id, uint16_t encoder_count, uint16_t reduction_ratio);

    float getVelocity(long can_id, uint16_t encoder_count, uint16_t reduction_ratio);

    int getInfAndRead(uint8_t *send_msg, uint8_t *read_msg, long can_id, unsigned int send_dlc, unsigned int *read_dlc);

private:
    canHandle handle;
    canStatus status;
    static const int DEFAULT_CHANNEL_NUMBER = 0;

    int checkStatus(const std::string &id);

    void dataIntegrationInt(uint8_t *temp, int32_t temp_value);

    void dataIntegrationFloat(uint8_t *temp, float temp_value);
};



#endif //SRC_KVASER_H
