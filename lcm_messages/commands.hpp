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
        RUN_SUCCESS,
        RUN_FAIL
    };

    enum CameraCommandId
    {
        START_CAMERA,
        STOP_AND_SAVE
    };

    enum SaveResult
    {
        SAVE_SUCCESS,
        SAVE_FAIL
    };
}

#endif
