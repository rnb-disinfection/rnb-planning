from nrmkindy.script import *

if __name__ == '__main__':
    config_script(NAME_INDY_7)
    start_script()

    home()
    movel(TaskPos(x=0.2), mode=MoveBaseMode.RELATIVE)

    mwait_done()
    end_script()
