#ifndef COMMANDS
#define COMMANDS

enum class Command {
    POSE_RECEIVED,
    TWIST_ACK,

    TRANSFORM_ROVER, //ignore
    TRANSFORM_DRONE, //ignore
    TRANSFORM_SEGWAY,//ignore
    DRIVE=102,
    FLY=103,
    BALANCE=104,
    STOP=105,

    KILL_ALL,
    NOT_IMPLEMENTED,

    WHEEL_IN=100,
    WHEEL_OUT=101
};

#endif//COMMMANDS