import csv
import os
import subprocess
import time
import signal


def run_command(command):
    return subprocess.run(command, shell=True, capture_output=True, text=True)

def popen_command(command):
    return subprocess.Popen(command, shell=True, preexec_fn=os.setsid)

model = "gpt-4.1-2025-04-14"
output_folder = "output"
route_start_end = {
    "route_1": {
        "start": {"x": -0.1, "y": -0.35, "yaw": 0.0},
        "end": {"x": 1.75, "y": -8.6, "yaw": 4.5},
        "timeout": 25,
    },
    "route_2": {
        "start": {"x": -4.4, "y": -8.6, "yaw": 1.2},
        "end": {"x": 1.75, "y": -2.15, "yaw": 0.0},
        "timeout": 25,
    },
    "route_3": {
        "start": {"x": 5.8, "y": 2.1, "yaw": 3.15},
        "end": {"x": -0.1, "y": 6.8, "yaw": 2.5},
        "timeout": 25,
    },
    "route_4": {
        "start": {"x": -4.4, "y": 5.85, "yaw": -1.0},
        "end": {"x": -0.9, "y": -4.85, "yaw": -0.75},
        "timeout": 28,
    },
    "route_5": {
        "start": {"x": -0.1, "y": -6.75, "yaw": 1.5},
        "end": {"x": -0.1, "y": 7.8, "yaw": 1.5},
        "timeout": 35,
    },
}

def main():

    if not os.path.exists(f'{output_folder}'):
        os.makedirs(f'{output_folder}')

    # print("\n*************  api image analysis  ******************\n")
    # api_process = popen_command("python src/test_pal_robotics/test_pal_robotics/api.py image")
    # api_process.wait()

    # algorithm_list = ['random', 'no_req', 'req']
    algorithm_list = ['req']
    # route_list = ['route_1', 'route_2', 'route_3', 'route_4', 'route_5']
    route_list = ['route_1']
    # requirement_list = ['jerk', 'distance', 'reach']
    requirement_list = ['distance']
    episode = 10

    out_folder = f'{output_folder}/execution_time.csv'
    header = ['algorithm_requirement', 'route', 'episode', 'number', 'total_time_seconds', 'total_time_minutes']
    if not os.path.exists(out_folder):
        with open(out_folder, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(header)

    for alg in algorithm_list:
        if alg == 'random' or alg == 'no_req':
            number = 1
            req_list = [alg]
        else:
            number = 5
            req_list = requirement_list
        for req in req_list:
            for route in route_list:
                x = route_start_end[route]["start"]["x"]
                y = route_start_end[route]["start"]["y"]
                yaw = route_start_end[route]["start"]["yaw"]
                start_time = time.time()

                for i in range(episode):
                    for j in range(number):
                        if alg == "random":
                            print("\n*************  random  ******************\n")
                            random_process = popen_command(
                                f"python src/test_pal_robotics/test_pal_robotics/random_algorithm.py --req {req} --route {route} --number {j} --episode {i}")
                            random_process.wait()
                        elif alg == "no_req":
                            print(f"\n*************  api requirement {req}  ******************\n")
                            api_process = popen_command(
                                f"python src/test_pal_robotics/test_pal_robotics/api.py req --req {req} --route {route} --episode {i}")
                            api_process.wait()
                        else:
                            if j == 0:
                                print(f"\n*************  api requirement {req}  ******************\n")
                                m = False if i == 0 else True
                                api_process = popen_command(
                                    f"python src/test_pal_robotics/test_pal_robotics/api.py req --req {req} --route {route} --episode {i} --memory {m}")
                                api_process.wait()
                            else:
                                print(f"\n*************  api requirement {req} multi  ******************\n")
                                api_process = popen_command(
                                    f"python src/test_pal_robotics/test_pal_robotics/api.py req_m --req {req} --route {route} --number {j} --episode {i}")
                                api_process.wait()

                        print("\n*************  human_file_generate  ******************\n")
                        human_file_process = popen_command(
                            f"python src/test_pal_robotics/test_pal_robotics/human_file_generate.py --req_folder {req} --req {req} --route {route} --number {j} --episode {i}")
                        human_file_process.wait()

                        print("\n*************  navigate_to_pose_requirement  ******************\n")
                        test_hunav_launch = popen_command(
                            f"ros2 launch test_hunav test_hunav.launch.py is_public_sim:=True navigation:=True world_name:=no_roof_small_warehouse x:={x} y:={y} yaw:={yaw} number:={j} episode:={i}")
                        time.sleep(15)

                        navigate_process = popen_command(
                            f"ros2 run test_pal_robotics navigate_to_pose_requirement --ros-args -p number:={j} -p episode:={i} -p route:={route} -p req:={req}")
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

                        if req != "random" and req != "no_req":
                            print("\n*************  api requirement json_scenario_req  ******************\n")
                            api_process = popen_command(
                                f"python src/test_pal_robotics/test_pal_robotics/api.py json_req --req {req} --route {route} --number {j} --episode {i}")
                            api_process.wait()

                        print(f"\n*************  number {j} finished  ******************\n\n")

                    if req != "random" and req != "no_req":
                        print("\n*************  api requirement json_memory_push  ******************\n")
                        api_process = popen_command(
                            f"python src/test_pal_robotics/test_pal_robotics/api.py json_memory --req {req} --route {route} --episode {i}")
                        api_process.wait()

                    print(f"\n*************  episode {i} finished  ******************\n\n")

                end_time = time.time()
                total_seconds = end_time - start_time
                minutes = int(total_seconds // 60)
                seconds = total_seconds % 60
                print(f"total time {total_seconds:.2f}s ({minutes} min {seconds:.2f}s)\n")
                with open(out_folder, 'a', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow([req, route, episode, number, f"{total_seconds:.2f}s", f"{minutes} min {seconds:.2f}s"])


if __name__ == "__main__":
    main()