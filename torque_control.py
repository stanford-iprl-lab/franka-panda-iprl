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
    stage = 0

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

        #A[-3,-3] += 0.07
        #A[-2,-2] += 0.07
        #A[-1,-1] += 0.07
        tau_cmd = A.dot(ddq)
        if (abs(robot.q[6] - q_des[6]) > 1e-2 and abs(tau_cmd[6]) < 0.54):
            tau_cmd[6] = np.sign(tau_cmd[6]) * 0.54
            #print(tau_cmd[6])

        if stage == 1:
            #tau_cmd[6] = -0.53
            #tau_cmd[6] = 0.53#min(1., 0.1 * (time.time() - t_start))
            #if robot.dq[6] > 1e-1:
            #    print(robot.dq[6], tau_cmd[6])
            #if -robot.dq[6] > 1e-1:
            #    print(robot.dq[6], tau_cmd[6])
            #    break
            pass

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
