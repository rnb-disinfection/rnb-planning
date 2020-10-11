from nrmkindy.script import *

if __name__ == '__main__':
    config_script(NAME_INDY_7)
    start_script()

    target_pos = JointPos(0, 0, -90, 0, -90, 45)

    movej(target_pos)

    rmovej(JointPos(q5=-45))
    rmovel(TaskPos(y=0.15))

    mwait_done()
    end_script()
