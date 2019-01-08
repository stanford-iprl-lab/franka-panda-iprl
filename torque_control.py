#!/usr/bin/env python

import redis
import numpy as np
import time
import signal
import sys

import frankapanda

redis_db = None

def signal_handler(sig, frame):
    global redis_db
    if redis_db is not None:
        redis_db.set("franka_panda::control::mode", "floating")
        redis_db.set("franka_panda::control::tau", " ".join(["0."] * 7))
        print("Cleared torques and set control mode to 'floating'.")
    sys.exit(0)

def sign(x, epsilon=1e-2):
    if x > epsilon:
        return 1
    elif x < -epsilon:
        return -1
    else:
        return 0

def main():
    global redis_db
    redis_db = redis.Redis()

    signal.signal(signal.SIGINT, signal_handler)

    robot = frankapanda.Model()

    kp_joint = 10
    kv_joint = 0#13
    redis_db.set("franka_panda::control::kp_joint", kp_joint)
    redis_db.set("franka_panda::control::kv_joint", kv_joint)

    robot.q = np.array(list(map(float, redis_db.get("franka_panda::sensor::q").decode("utf-8").strip().split(" "))))
    robot.dq = np.array(list(map(float, redis_db.get("franka_panda::sensor::dq").decode("utf-8").strip().split(" "))))

    q_des = np.array(robot.q)
    q_des[6] = 0
    test_joint = True
    idx_joint = 5
    sign_joint = np.sign(robot.q[idx_joint])

    i = 0
    while redis_db.get("franka_panda::driver::status").decode("utf8") == "running":
        time.sleep(0.001)

        kp_joint = float(redis_db.get("franka_panda::control::kp_joint"))
        kv_joint = float(redis_db.get("franka_panda::control::kv_joint"))

        robot.q = np.array(list(map(float, redis_db.get("franka_panda::sensor::q").decode("utf-8").strip().split(" "))))
        dq = np.array(list(map(float, redis_db.get("franka_panda::sensor::dq").decode("utf-8").strip().split(" "))))
        robot.dq = dq

        A = frankapanda.inertia(robot)
        V = frankapanda.centrifugal_coriolis(robot)
        G = frankapanda.gravity(robot)

        ddq = -kp_joint * (robot.q - q_des) - kv_joint * robot.dq

        tau_cmd = A.dot(ddq)
        if (abs(tau_cmd[6]) < 0.54):
            tau_cmd[6] = sign(tau_cmd[6], epsilon=0.1) * 0.54

        if test_joint:
            tau_cmd[idx_joint] = -sign_joint * min(1., 0.1 * (time.time() - t_start))
            if abs(robot.dq[idx_joint]) > 1e-1:
               print(robot.dq[idx_joint], tau_cmd[idx_joint])
               break

        t = time.time()
        redis_db.set("franka_panda::control::tau", " ".join(map(str, tau_cmd.tolist())))
        redis_db.set("franka_panda::control::mode", "torque")

        if i % 100 == 0:
            print("dq: ", robot.dq)
        #     print("q: ", q)
        #     print("tau: ", tau)
        #     print("dtau: ", dtau)
        #     print("A: ", A)
        #     print("V: ", V)
        #     print("G: ", G)
        #     print("tau_cmd: ", tau_cmd)
        #     print("ddq", ddq)
        #     print("kp_joint", kp_joint)
        i += 1

    redis_db.set("franka_panda::control::mode", "floating")
    redis_db.set("franka_panda::control::tau", " ".join(["0."] * 7))
    print("Cleared torques and set control mode to 'floating'.")

if __name__ == "__main__":
    main()
