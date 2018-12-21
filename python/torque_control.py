import redis
import numpy as np
import time

def main():
    redis_db = redis.Redis()

    kp_joint = 40
    kv_joint = 13
    redis_db.set("franka_panda::control::kp_joint", kp_joint)
    redis_db.set("franka_panda::control::kv_joint", kv_joint)

    print(redis_db.get("franka_panda::sensor::q"))
    q_des = np.array(list(map(float, redis_db.get("franka_panda::sensor::q").decode("utf-8").strip().split(" "))))

    while True:
        time.sleep(0.001)

        kp_joint = float(redis_db.get("franka_panda::control::kp_joint"))
        kv_joint = float(redis_db.get("franka_panda::control::kv_joint"))

        q = np.array(list(map(float, redis_db.get("franka_panda::sensor::q").decode("utf-8").strip().split(" "))))
        dq = np.array(list(map(float, redis_db.get("franka_panda::sensor::dq").decode("utf-8").strip().split(" "))))
        tau = np.array(list(map(float, redis_db.get("franka_panda::sensor::tau").decode("utf-8").strip().split(" "))))
        dtau = np.array(list(map(float, redis_db.get("franka_panda::sensor::dtau").decode("utf-8").strip().split(" "))))

        A = np.array([list(map(float, row.strip().split(" "))) for row in redis_db.get("franka_panda::model::mass_matrix").decode("utf-8").strip().split(";")])
        V = np.array(list(map(float, redis_db.get("franka_panda::model::coriolis").decode("utf-8").strip().split(" "))))
        G = np.array(list(map(float, redis_db.get("franka_panda::model::gravity").decode("utf-8").strip().split(" "))))

        ddq = -kp_joint * (q - q_des) - kv_joint * dq

        tau = A * ddq

        print("q: ", q)
        print("dq: ", dq)
        print("tau: ", tau)
        print("dtau: ", dtau)
        print("A: ", A)
        print("V: ", V)
        print("G: ", G)

        redis_db.set("franka_panda::control::tau", " ".join(map(str, ab.q.tolist())))
        redis_db.set("franka_panda::control::mode", "torque")

if __name__ == "__main__":
    main()
