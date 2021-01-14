from nrmkindy.script import *

if __name__ == '__main__':
    config_script(NAME_INDY_7)
    start_script()

    tool_on = DIOSet(0, HIGH, 1, HIGH)
    tool_off = DIOSet(0, LOW, 1, LOW)

    home_pos = JointPos(0, 0, -90, 0, -90, 0)
    pick_ready = JointPos(-15, -12, -90, 0, -83, -15)
    place_ready = JointPos(-40, -21, -55, 0, -103, 40)

    movej(home_pos)

    mwait_done()
    for i in range(3):
        movej(pick_ready)
        rmovel(TaskPos(z=-0.15))
        mwait_done()
        do(tool_on)
        rmovel(TaskPos(z=0.15))

        mwait_done()
        if di()[0] == HIGH:
            movej(place_ready)
            rmovel(TaskPos(z=-0.2))
            mwait_done()
            do(tool_off)
            rmovel(TaskPos(z=0.2))
        else:
            do(tool_off)
            movej(home_pos)
            
        mwait_done()

    mwait_done()
    end_script()
