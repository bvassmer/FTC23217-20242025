package org.firstinspires.ftc.teamcode;

public class Enum {
    public enum Strike {
        NOT_FOUND,
        RIGHT,
        CENTER,
        LEFT,
    };

    public enum TeamColor {
        BLUE,
        RED,
    }

    public enum AutonomousState {
        WAITING,
        ABOUT_TO_MOVE,
        LOWER_CAR_WASH,
        RAISE_LIFT,
        RAISING_LIFT,
        RAISE_HOOK,
        LOWER_LIFT,
        LOWERING_LIFT,
        LOWER_HOOK,
        RESET_HOOK,
        WAIT_FOR_LOWERED_CAR_WASH,
        WAIT_FOR_CAMERA_SETTLING,
        MOVE_TO_LEFT_STRIKE,
        MOVE_TO_CENTER_STRIKE,
        MOVE_TO_RIGHT_STRIKE,
        MOVE_TO_RIGHT_STRIKE2,
        MOVE_TO_PIXEL_BYPASS,
        MOVE_TO_BACKOFF,
        MOVE_TO_BRIDGE_STOP,
        MOVE_TO_BACKDROP_RIGHT,
        MOVE_TO_BACKDROP,
        FIND_PIXEL,
        FOUND_PIXEL,
        EJECT_PIXEL,
        WAIT_FOR_EJECTED_PIXEL,
        FORCE_EJECTING,
        MOVE,
        MOVING,
        FIND_BACKDROP,
        FIND_APRIL_TAG,
        APRIL_TAG_SHIFT_LEFT,
        APRIL_TAG_SHIFT_RIGHT,
        APRIL_TAG_SHIFT_FORWARD,
        APRIL_TAG_SHIFT_BACKWARD,
        LIFT_EXTENSION_ARM,
        LIFTING_EXTENSION_ARM,
        POWER_FORWARD,
        DROP_PIXEL,
        DONE,
    }
}
