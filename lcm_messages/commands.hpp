/*
Defines enums for the LCM commands.
*/

#ifndef LCM_COMMANDS_H
#define LCM_COMMANDS_H

namespace Commands
{
    enum RunId
    {
        RUN_CONTROLLER,
        STOP_CONTROLLER
    };

    enum RunResult
    {
        SUCCESS,
        FAIL
    };

    enum CameraCommandId
    {
        START_CAMERA,
        STOP_AND_SAVE
    };
}

#endif
