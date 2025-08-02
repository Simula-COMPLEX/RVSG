import os
import csv

def is_effectively_empty(file_path):
    if os.path.getsize(file_path) == 0:
        return True
    if file_path.endswith('.csv'):
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                reader = csv.reader(f)
                rows = list(reader)
                if len(rows) <= 1:
                    return True
        except Exception:
            return True
    return False

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

def check_folder_structure(req_list, route_list, base_path="."):
    missing = {"req": [], "route": [], "json": [], "number": [], "file_count": [], "empty": [], "csv_uniform": []}

    for req in req_list:
        req_path = os.path.join(base_path, req)
        if not os.path.isdir(req_path):
            missing["req"].append(req)
            continue

        for route in route_list:
            route_path = os.path.join(req_path, route)
            if not os.path.isdir(route_path):
                missing["route"].append(route_path)
                continue

            if req not in ['no_req', 'random']:
                json_files = [
                    f for f in os.listdir(route_path)
                    if f.endswith(".json") and os.path.isfile(os.path.join(route_path, f))
                ]
                if not json_files:
                    missing["json"].append(route_path)
                else:
                    for json_file in json_files:
                        json_path = os.path.join(route_path, json_file)
                        if os.path.getsize(json_path) == 0:
                            missing["empty"].append(json_path)

            if req == 'no_req':
                expect_num_files = 6
            elif req == 'random':
                expect_num_files = 4
            else:
                expect_num_files = 22

            for i in range(10):
                num_folder = os.path.join(route_path, str(i))
                if not os.path.isdir(num_folder):
                    missing["number"].append(num_folder)
                    continue

                files = [
                    f for f in os.listdir(num_folder)
                    if os.path.isfile(os.path.join(num_folder, f))
                ]
                if len(files) != expect_num_files:
                    missing["file_count"].append(f"{num_folder} (found {len(files)}, expected {expect_num_files})")
                for file in files:
                    file_path = os.path.join(num_folder, file)
                    if is_effectively_empty(file_path):
                        missing["empty"].append(file_path)
                    if file_path.endswith('.csv'):
                        has_wp, is_uniform = is_worker_wp_uniform(file_path)
                        if has_wp and is_uniform:
                            missing["csv_uniform"].append(file_path)

    for k, v in missing.items():
        if v:
            print(f"{k} miss/error:\n")
            for name in v:
                print(name, "\n")
        else:
            print(f"{k}: OK\n")

# req_list = ['jerk', 'distance', 'reach', 'no_req', 'random']
# route_list = ['route_1', 'route_2', 'route_3', 'route_4', 'route_5']
# check_folder_structure(req_list, route_list)

def check_best_scenario():
    best_folder = f'best_scenario'
    # alg_list = ['jerk', 'distance', 'reach', 'no_req']
    alg_list = ['jerk', 'distance', 'reach']
    req_list = ['jerk', 'distance', 'reach']
    # req_list = ['jerk']
    route_list = ['route_1', 'route_2', 'route_3', 'route_4', 'route_5']
    # route_list = ['route_1']
    number = 30

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
                folder = f"./{b_folder}/{route}/0"
                files = [
                    f for f in os.listdir(folder)
                    if os.path.isfile(os.path.join(folder, f))
                ]
                if len(files) != 32:
                    print(folder)

                for i in range(number):
                    log_path = f"./{b_folder}/{route}/0/log_{i}.csv"
                    has_wp, is_uniform = is_worker_wp_uniform(log_path)
                    if has_wp and is_uniform:
                        print(f"{log_path}")

check_best_scenario()
