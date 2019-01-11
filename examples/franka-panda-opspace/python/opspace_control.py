#!/usr/bin/env python

import redis
import numpy as np
import json
import time
import signal
import sys

import spatialdyn
import frankapanda
from spatialdyn_frankapanda import *

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

def filter(x, threshold, epsilon):
    if abs(x) < threshold:
        if abs(x) < epsilon:
            dx = x / epsilon - np.sign(x)
            return -np.sign(x) * threshold * (dx*dx - 1)
        return np.sign(x) * threshold
    return x

def main():
    global redis_db
    redis_db = redis.Redis()
    timer = spatialdyn.Timer(1000)

    signal.signal(signal.SIGINT, signal_handler)

    ab = ArticulatedBody(spatialdyn.urdf.load_model("resources/franka_panda.urdf"))

    ab.inertia_compensation = np.array([0.2, 0.1, 0.1])

    kp_pos = 40
    kv_pos = 5
    kp_ori = 40
    kv_ori = 5
    kp_joint = 5
    kv_joint = 0
    redis_db.set("franka_panda::control::kp_pos", kp_pos)
    redis_db.set("franka_panda::control::kv_pos", kv_pos)
    redis_db.set("franka_panda::control::kp_ori", kp_ori)
    redis_db.set("franka_panda::control::kv_ori", kv_ori)

    ab.q = np.array(list(map(float, redis_db.get("franka_panda::sensor::q").decode("utf-8").strip().split(" "))))
    ab.dq = np.array(list(map(float, redis_db.get("franka_panda::sensor::dq").decode("utf-8").strip().split(" "))))

    ee_offset = np.array([0., 0., 0.107])
    q_des = np.array([0., np.pi/6., 0., -5./6. * np.pi, 0., 2./3. * np.pi, 0.])
    x_0 = spatialdyn.position(ab, offset=ee_offset)
    quat_des = spatialdyn.orientation(ab)

    json_ee = json.loads(redis_db.get("franka_panda::model::inertia_ee").decode("utf8"))
    m_ee = float(json_ee["m"])
    com_ee = np.array(list(map(float, json_ee["com"])))
    I_com_ee = np.array(list(map(float, json_ee["I_com"])))
    I_load = spatialdyn.SpatialInertiad(m_ee, com_ee, I_com_ee)
    ab.replace_load(I_load)

    try:
        while redis_db.get("franka_panda::driver::status").decode("utf8") == "running":
            timer.sleep()

            kp_pos = float(redis_db.get("franka_panda::control::kp_pos"))
            kv_pos = float(redis_db.get("franka_panda::control::kv_pos"))
            kp_ori = float(redis_db.get("franka_panda::control::kp_ori"))
            kv_ori = float(redis_db.get("franka_panda::control::kv_ori"))

            ab.q = np.array(list(map(float, redis_db.get("franka_panda::sensor::q").decode("utf-8").strip().split(" "))))
            ab.dq = np.array(list(map(float, redis_db.get("franka_panda::sensor::dq").decode("utf-8").strip().split(" "))))

            x_des = np.array(x_0)
            x_des[1] += 0.2 * np.sin(timer.time_sim())
            x_des[0] += 0.2 * (np.cos(timer.time_sim()) - 1)
            x = spatialdyn.position(ab, offset=ee_offset)
            x_err = x - x_des
            dx_err = spatialdyn.linear_jacobian(ab, offset=ee_offset).dot(ab.dq)
            ddx = -kp_pos * x_err - kv_pos * dx_err

            quat = spatialdyn.orientation(ab)
            ori_err = spatialdyn.opspace.orientation_error(quat, quat_des)
            w_err = spatialdyn.angular_jacobian(ab).dot(ab.dq)
            dw = -kp_ori * ori_err - kv_ori * w_err

            ddx_dw = np.hstack((ddx, dw))
            J = spatialdyn.jacobian(ab, offset=ee_offset)
            N = np.eye(ab.dof)
            tau_cmd = spatialdyn.opspace.inverse_dynamics(ab, J, ddx_dw, N)

            I = np.eye(ab.dof)
            q_err = ab.q - q_des
            dq_err = ab.dq
            ddq = -kp_joint * q_err - kv_joint * dq_err
            tau_cmd += spatialdyn.opspace.inverse_dynamics(ab, I, ddq, N)

            #tau_cmd[6] = filter(tau_cmd[6], 0.7, 0.01)
            tau_cmd[6] = filter(tau_cmd[6], 0.6, 0.1)
            tau_cmd[5] = filter(tau_cmd[5], 0.8, 0.1)
            tau_cmd[4] = filter(tau_cmd[4], 0.8, 0.1)

            tau_cmd += spatialdyn.gravity(ab)

            redis_db.set("franka_panda::control::tau", " ".join(map(str, tau_cmd.tolist())))
            redis_db.set("franka_panda::control::mode", "torque")
            redis_db.set("franka_panda::trajectory::pos", " ".join(map(str, x.tolist())))
            redis_db.set("franka_panda::trajectory::ori", " ".join(map(str, quat.coeffs.tolist())))
            redis_db.set("franka_panda::trajectory::pos_err", " ".join(map(str, x_err.tolist())))
            redis_db.set("franka_panda::trajectory::ori_err", " ".join(map(str, ori_err.tolist())))
    except Exception as e:
        print(e)

    redis_db.set("franka_panda::control::mode", "floating")
    redis_db.set("franka_panda::control::tau", " ".join(["0."] * 7))
    print("Cleared torques and set control mode to 'floating'.")

    print("Simulated {}s in {}s.".format(timer.time_sim(), timer.time_elapsed()))

if __name__ == "__main__":
    main()
