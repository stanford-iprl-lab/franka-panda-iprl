#!/usr/bin/env python

import redis
import numpy as np
import time

import frankapanda

def main():
    robot = frankapanda.Model()

    redis_db = redis.Redis()

    kp_joint = 10
    kv_joint = 0#13
    redis_db.set("franka_panda::control::kp_joint", kp_joint)
    redis_db.set("franka_panda::control::kv_joint", kv_joint)

    robot.q = np.array(list(map(float, redis_db.get("franka_panda::sensor::q").decode("utf-8").strip().split(" "))))
    robot.dq = np.array(list(map(float, redis_db.get("franka_panda::sensor::dq").decode("utf-8").strip().split(" "))))

    q_des = np.array(franka.q)

    i = 0
    t = time.time()
    while True:
        time.sleep(0.001)

        kp_joint = float(redis_db.get("franka_panda::control::kp_joint"))
        kv_joint = float(redis_db.get("franka_panda::control::kv_joint"))

        franka.q = np.array(list(map(float, redis_db.get("franka_panda::sensor::q").decode("utf-8").strip().split(" "))))
        franka.dq = np.array(list(map(float, redis_db.get("franka_panda::sensor::dq").decode("utf-8").strip().split(" "))))

        A = frankapanda.inertia(robot)
        V = frankapanda.centrifugal_coriolis(robot)
        G = frankapanda.gravity(robot)

        ddq = -kp_joint * (frankapanda.q - q_des) - kv_joint * frankapanda.dq

        A[-3,-3] += 0.07
        A[-2,-2] += 0.07
        A[-1,-1] += 0.07
        tau_cmd = A.dot(ddq)

        t = time.time()
        redis_db.set("franka_panda::control::tau", " ".join(map(str, tau_cmd.tolist())))
        redis_db.set("franka_panda::control::mode", "torque")

        # if i % 1000 == 0:
        #     print("q: ", q)
        #     print("dq: ", dq)
        #     print("tau: ", tau)
        #     print("dtau: ", dtau)
        #     print("A: ", A)
        #     print("V: ", V)
        #     print("G: ", G)
        #     print("tau_cmd: ", tau_cmd)
        #     print("ddq", ddq)
        #     print("kp_joint", kp_joint)
        i += 1


if __name__ == "__main__":
    main()
