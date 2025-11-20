# main.py
### well here we go again


from arm_class import Arm
import time

if __name__ == "__main__":

    # Create a list of three Arm objects
    arms = [Arm(L=1.0) for _ in range(3)]

    # Give each arm a different jerk (example)
    arms[0].set_jerk(0.01)
    arms[1].set_jerk(0.01)
    arms[2].set_jerk(0.01)

    prev = time.perf_counter()

    while True:
        now_ns = time.perf_counter_ns()
        dt_us = int((now_ns - prev) /1_000)  # microseconds
        prev = now_ns

        # Update all arms
        for arm in arms:
            arm.update(dt_us)

        # Example: print one of them
        print(f"theta0={arms[0].theta:.4f} rad  vel={arms[0].theta_d:.4f}")

        # cap at 10 kHz
        time.sleep(0.0001)
# RRR.py
