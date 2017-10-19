from joystick import Controller
from comm import SerialCom
import time


def main():
    # Initialise
    joy = Controller(0)
    com = SerialCom("COM7")
    moved = False
    n_reps = 0
    out_prev = []
    a_p, b_p, x_p, y_p, l1_p, r1_p, back_p, start_p, l3_p, r3_p = (0, 0, 0, 0 ,0 ,0 ,0 ,0 ,0 ,0)
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
            a, b, x, y, l1, r1, back, start, l3, r3 = joy.get_buttons()
            # a, b, x, y, l1, r1, back, start, l3, r3 = ((a^a_p)&a, (b^b_p)&b, (x^x_p)&x, (y^y_p)&y, (l1^l1_p)&l1, (r1^r1_p)&r1, (back^back_p)&back, (start^start_p)&start, (l3^l3_p)&l3, (r3^r3_p)&r3)
            # 0b**bsABXY
            buttons_a =  y | x << 1 | b << 2 | a << 3 | start << 4 | back << 5
            # 0b***L3R3L1R1
            buttons_b = r1 | l1 << 1 | r3 << 2 | l3 << 3

            out = [int(-10*x_axis)+10, int(10*y_axis)+10, int(-10*z_axis)+10, buttons_a, buttons_b]

            if out == out_prev:
                n_reps += 1
            else:
                com.send(out)

            # if n_reps <= 5:
            #     pass
            #     com.send(out)

            out_prev = out
            time.sleep(0.01)

            # a, b, x, y, l1, r1, back, start, l3, r3 = (0, 0, 0, 0 ,0 ,0 ,0 ,0 ,0 ,0)


    except KeyboardInterrupt:
        print('interrupted!')

if __name__ == '__main__':
    main()
