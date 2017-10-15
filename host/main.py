from joystick import Controller
from comm import SerialCom
import time

global left_axis
global right_axis
def main():
    # Initialise
    joy = Controller(0)
    com = SerialCom()
    moved = False
    n_reps = 0
    out_prev = [10, 10, 10]
    global left_axis
    global right_axis
    try:
        while True:
            # Pump function is needed to receive input
            joy.pump()
            # Get joystick controls
            # z = up and down
            # x = forward and back
            # y = left and right
            y_axis, z_axis = joy.get_left_stick()
            useless, x_axis = joy.get_right_stick()

            out = [int(-10*x_axis)+10, int(10*y_axis)+10, int(-10*z_axis)+10]

            if out[0] == out_prev[0] and  out[1] == out_prev[1] and  out[2] == out_prev[2]:
                n_reps += 1
            else:
                n_reps = 0

            if n_reps <= 10:
                com.send(out)

            out_prev = out
            time.sleep(0.01)



    except KeyboardInterrupt:
        print('interrupted!')

if __name__ == '__main__':
    main()
