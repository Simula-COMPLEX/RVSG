import csv
import os
import signal
import subprocess
import time
from test_pal_robotics.test_hunav import output_folder, route_start_end


def popen_command(command):
    return subprocess.Popen(command, shell=True, preexec_fn=os.setsid)


def is_worker_wp_uniform(file_path):
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            if not reader.fieldnames:
                return False, False

            if "worker1_wp" not in reader.fieldnames or "worker2_wp" not in reader.fieldnames:
                return False, False

            w1_list = []
            w2_list = []
            for row in reader:
                w1_list.append(row["worker1_wp"])
                w2_list.append(row["worker2_wp"])

            w1_uniform = len(set(w1_list)) <= 1
            w2_uniform = len(set(w2_list)) <= 1
            if w1_uniform and w2_uniform:
                return True, True
            else:
                return True, False
    except Exception:
        return False, False


def main():
    best_folder = f'best_scenario' # best_scenario
    # alg_list = ['jerk', 'distance', 'reach', 'no_req']
    alg_list = ['reach']
    req_list = ['jerk', 'distance', 'reach']
    # req_list = ['jerk']
    # route_list = ['route_1', 'route_2', 'route_3', 'route_4', 'route_5']
    route_list = ['route_5']
    number = 16
    start = 15

    for alg in alg_list:
        if alg == 'no_req':
            r_list = req_list
        else:
            r_list = [alg]
        for req in r_list:
            if alg == 'no_req':
                b_folder = f'{best_folder}/{alg}/{req}'
            else:
                b_folder = f'{best_folder}/{req}'
            for route in route_list:
                x = route_start_end[route]["start"]["x"]
                y = route_start_end[route]["start"]["y"]
                yaw = route_start_end[route]["start"]["yaw"]

                print("\n*************  human_file_generate  ******************\n")
                human_file_process = popen_command(
                    f"python src/test_pal_robotics/test_pal_robotics/human_file_generate.py --req_folder {b_folder} --req {alg} --route {route} --number 0 --episode 0")
                human_file_process.wait()

                for i in range(start, number):

                    while True:
                        print("\n*************  navigate_to_pose_requirement  ******************\n")
                        test_hunav_launch = popen_command(
                            f"ros2 launch test_hunav test_hunav.launch.py is_public_sim:=True navigation:=True world_name:=no_roof_small_warehouse x:={x} y:={y} yaw:={yaw}")
                        time.sleep(15)

                        navigate_process = popen_command(
                            f"ros2 run test_pal_robotics navigate_to_pose_requirement --ros-args -p number:={i} -p episode:=0 -p route:={route} -p req:={b_folder}")
                        navigate_process.wait()

                        time.sleep(3)
                        os.killpg(os.getpgid(test_hunav_launch.pid), signal.SIGTERM)
                        test_hunav_launch.wait(timeout=5)
                        if navigate_process.poll() is None:
                            os.killpg(os.getpgid(navigate_process.pid), signal.SIGTERM)
                            navigate_process.wait(timeout=5)
                        subprocess.run("pkill -f 'rviz2' || true", shell=True)
                        subprocess.run("pkill -f 'gzclient' || true", shell=True)
                        subprocess.run("pkill -f 'gzserver' || true", shell=True)

                        log_path = f"{output_folder}/{b_folder}/{route}/0/log_{i}.csv"
                        if os.path.isfile(log_path):
                            pass
                        else:
                            continue
                        has_wp, is_uniform = is_worker_wp_uniform(log_path)
                        if has_wp and is_uniform:
                            os.remove(log_path)
                            with open(f"{output_folder}/{b_folder}/{route}/0/log.csv", 'r', encoding='utf-8') as f:
                                lines = f.readlines()
                            with open(f"{output_folder}/{b_folder}/{route}/0/log.csv", 'w', encoding='utf-8') as f:
                                f.writelines(lines[:-1])
                        else:
                            break


if __name__ == '__main__':
    main()