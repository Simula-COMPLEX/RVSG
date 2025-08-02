import json
import os
import shutil
import numpy as np
import pandas as pd
import seaborn as sns
import textdistance
import torch
from scipy.stats import mannwhitneyu
from sentence_transformers import SentenceTransformer
from tslearn.metrics import dtw


def calculate_avg(file_path, req):
    df = pd.read_csv(file_path)
    if req == "jerk":
        col = df["robot_jerk"]
        mean_value = col.iloc[1:].mean()
    elif req == "distance":
        col = df["distance_to_closest_person"]
        mean_value = col.mean()
    elif req == "reach":
        mean_value = df["timestamp"].iloc[-1]
    return mean_value


def avg_req():
    min_reqs = ['distance']
    for alg in alg_list:
        if alg == 'random' or alg == 'no_req':
            number = 1
        else:
            number = 5
        for route in route_list:
            file_path = f'{output_folder}/{alg}/{route}'
            for req in req_list:

                best_req_list = []
                best_index_list = []
                for i in range(episode):
                    f_path = f'{file_path}/{i}'
                    if req in min_reqs:
                        best_mean_value = float('inf')
                        best_index = None
                        compare = lambda a, b: a < b
                    else:
                        best_mean_value = float('-inf')
                        best_index = None
                        compare = lambda a, b: a > b
                    for j in range(number):
                        mean_value = calculate_avg(f'{f_path}/log_{j}.csv', req)
                        if compare(mean_value, best_mean_value):
                            best_mean_value = mean_value
                            best_index = j
                    best_req_list.append(best_mean_value)
                    best_index_list.append(best_index)

                with open(f'{file_path}/result_{req}.txt', 'w', encoding='utf-8') as f:
                    f.write(f"{req} values: {best_req_list}\n")
                    f.write(f"scenario index: {best_index_list}\n\n")
                    f.write(f"{req} average: {sum(best_req_list) / len(best_req_list)}\n\n")


def sentence_similarity():
    model = SentenceTransformer("all-MiniLM-L6-v2")
    for alg in alg_list:
        if alg == 'random':
            continue
        for route in route_list:
            file_path = f'{output_folder}/{alg}/{route}'
            sentences = []
            for i in range(episode):
                with open(f'{file_path}/{i}/{alg}_scenario_gpt-4.1-2025-04-14.json', 'r', encoding='utf-8') as f:
                    data = json.load(f)
                sentences.append(data['scenario'])

            embeddings = model.encode(sentences)
            similarities = model.similarity(embeddings, embeddings)

            indices = torch.triu_indices(similarities.size(0), similarities.size(1), offset=1)
            vals = similarities[indices[0], indices[1]]
            avg_sim = vals.mean().item()
            diversity = 1 - avg_sim

            with open(f'{file_path}/result_sentence_similarity.txt', 'w', encoding='utf-8') as f:
                f.write(f"sentence_similarity values: {vals}\n\n")
                f.write(f"sentence_similarity average: {avg_sim}\n\n")
                f.write(f"diversity: {diversity}\n\n")


def levenshtein_distance():
    min_reqs = ['distance']
    for alg in alg_list:
        if alg == 'random' or alg == 'no_req':
            number = 1
            r_list = req_list
        else:
            number = 5
            r_list = [alg]
        for route in route_list:
            file_path = f'{output_folder}/{alg}/{route}'
            for req in r_list:
                worker1_seqs = []
                worker2_seqs = []
                scenario_index = []
                for i in range(episode):
                    f_path = f'{file_path}/{i}'
                    if req in min_reqs:
                        best_mean_value = float('inf')
                        best_index = None
                        compare = lambda a, b: a < b
                    else:
                        best_mean_value = float('-inf')
                        best_index = None
                        compare = lambda a, b: a > b
                    for j in range(number):
                        mean_value = calculate_avg(f'{f_path}/log_{j}.csv', req)
                        if compare(mean_value, best_mean_value):
                            best_mean_value = mean_value
                            best_index = j
                    with open(f'{f_path}/{alg}_scenario_gpt-4.1-2025-04-14.json', 'r', encoding='utf-8') as f:
                        scenario_data = json.load(f)
                    worker1_seqs.append(scenario_data[f'scenario_{best_index}']['Worker 1'])
                    worker2_seqs.append(scenario_data[f'scenario_{best_index}']['Worker 2'])
                    scenario_index.append(best_index)

                distances = []
                for i in range(episode - 1):
                    for j in range(i + 1, episode):
                        dist = textdistance.levenshtein(worker1_seqs[i], worker1_seqs[j])
                        dist += textdistance.levenshtein(worker1_seqs[i], worker2_seqs[j])
                        dist += textdistance.levenshtein(worker2_seqs[i], worker1_seqs[j])
                        dist += textdistance.levenshtein(worker2_seqs[i], worker2_seqs[j])
                        dist /= 4
                        distances.append(dist)
                avg_dist = sum(distances) / len(distances)

                with open(f'{file_path}/result_{req}_high_diversity.txt', 'w', encoding='utf-8') as f:
                    f.write(f"scenario index: {scenario_index}\n\n")
                    f.write(f"levenshtein_distance values: {distances}\n\n")
                    f.write(f"levenshtein_distance average: {avg_dist}\n\n")


def dynamic_time_warping():
    min_reqs = ['distance']
    for alg in alg_list:
        if alg == 'random' or alg == 'no_req':
            number = 1
            r_list = req_list
        else:
            number = 5
            r_list = [alg]
        for route in route_list:
            file_path = f'{output_folder}/{alg}/{route}'
            for req in r_list:
                simulation_data = []
                scenario_index = []
                cols = ['robot_x', 'robot_y', 'worker1_x', 'worker1_y', 'worker2_x', 'worker2_y']
                for i in range(episode):
                    f_path = f'{file_path}/{i}'
                    if req in min_reqs:
                        best_mean_value = float('inf')
                        best_index = None
                        compare = lambda a, b: a < b
                    else:
                        best_mean_value = float('-inf')
                        best_index = None
                        compare = lambda a, b: a > b
                    for j in range(number):
                        mean_value = calculate_avg(f'{f_path}/log_{j}.csv', req)
                        if compare(mean_value, best_mean_value):
                            best_mean_value = mean_value
                            best_index = j
                    df = pd.read_csv(f'{f_path}/log_{best_index}.csv')
                    data = df[cols].to_numpy()
                    simulation_data.append(data)
                    scenario_index.append(best_index)

                distances = []
                for i in range(episode - 1):
                    for j in range(i + 1, episode):
                        dist = dtw(simulation_data[i], simulation_data[j])
                        distances.append(dist)
                avg_dist = np.mean(distances)

                with open(f'{file_path}/result_{req}_simulation_diversity.txt', 'w', encoding='utf-8') as f:
                    f.write(f"scenario index: {scenario_index}\n\n")
                    f.write(f"dynamic_time_warping values: {distances}\n\n")
                    f.write(f"dynamic_time_warping average: {avg_dist}\n\n")


def avg_metrics():
    min_reqs = ['distance']
    req_map_col = {"jerk": "avg_robot_jerk", "distance": "avg_distance_to_closest_person", "reach": "time_to_reach_goal"}
    for alg in alg_list:
        for route in route_list:
            file_path = f'{output_folder}/{alg}/{route}'
            dfs = []
            for i in range(episode):
                f_path = f'{file_path}/{i}/log.csv'
                df = pd.read_csv(f_path)
                if alg in req_list:
                    if alg in min_reqs:
                        best_row = df.loc[df[req_map_col[alg]].idxmin()]
                    else:
                        best_row = df.loc[df[req_map_col[alg]].idxmax()]
                    dfs.append(best_row.to_frame().T)
                else:
                    dfs.append(df)
            all_data = pd.concat(dfs, ignore_index=True)
            cols_to_avg = all_data.columns[1:]

            mean_dict = {}
            for col in cols_to_avg:
                if col == 'completed':
                    completed_col = all_data['completed']
                    completed_bool = completed_col.map(lambda x: str(x).strip().lower() == 'true')
                    num_completed_true = int(completed_bool.sum())
                    mean_dict['completed'] = num_completed_true
                else:
                    mean_dict[col] = all_data[col].mean()

            mean_df = pd.DataFrame([mean_dict])
            mean_df.to_csv(f'{file_path}/avg_result.csv', index=False)


def avg_metrics_format():
    header = ["route", "algorithm",
              "time_to_reach_goal", "path_length", "cumulative_heading_changes",
              "avg_distance_to_closest_person", "minimum_distance_to_people", "completed",
              "minimum_distance_to_target", "final_distance_to_target",
              "robot_on_person_collision", "person_on_robot_collision", "time_not_moving",
              "avg_robot_linear_speed", "avg_robot_angular_speed",
              "avg_robot_acceleration", "avg_robot_jerk"]
    all_dfs = []
    for route in route_list:
        for alg in ['random', 'no_req', 'distance', 'jerk', 'reach']:
            file_path = f'{output_folder}/{alg}/{route}'
            df = pd.read_csv(f'{file_path}/avg_result.csv')
            float_cols = df.select_dtypes(include=['float']).columns
            df[float_cols] = df[float_cols].round(3)
            df['route'] = route
            df['algorithm'] = alg
            df = df[header]
            all_dfs.append(df)
    df_all = pd.concat(all_dfs, ignore_index=True)
    df_all.to_csv(f'./{output_folder}/merged_avg_result.csv', index=False)


def avg_metrics_requirement_format():
    requirement_to_obj = {
        'distance': 'avg_distance_to_closest_person',
        'jerk': 'avg_robot_jerk',
        'reach': 'time_to_reach_goal',
    }
    other_cols = [
        'robot_on_person_collision',
        'person_on_robot_collision',
        'cumulative_heading_changes',
        'path_length',
        'time_not_moving'
    ]

    rows = []
    for requirement, obj_col in requirement_to_obj.items():
        for route in route_list:
            file_no_req = f'{output_folder}/no_req/{route}/avg_result.csv'
            df_no_req = pd.read_csv(file_no_req)
            file_req = f'{output_folder}/{requirement}/{route}/avg_result.csv'
            df_req = pd.read_csv(file_req)

            row_no_req = df_no_req.iloc[0]
            row_req = df_req.iloc[0]

            obj_no_req = round(row_no_req[obj_col], 3)
            obj_req = round(row_req[obj_col], 3)
            objective_str = f'{obj_no_req} / {obj_req}'

            merged_others = []
            for col in other_cols:
                val_no_req = round(row_no_req[col], 3)
                val_req = round(row_req[col], 3)
                merged_others.append(f'{val_no_req} / {val_req}')

            row = [requirement, route, objective_str] + merged_others
            rows.append(row)

    final_header = (
            ['requirement', 'route', 'objective'] + other_cols
    )

    df_final = pd.DataFrame(rows, columns=final_header)
    df_final.to_csv(f'./{output_folder}/merged_avg_requirement_result.csv', index=False)


def obtain_best_scenario():
    best_folder = f'{output_folder}/best_scenario'
    req_map_col = {"jerk": "avg_robot_jerk", "distance": "avg_distance_to_closest_person", "reach": "time_to_reach_goal"}
    req_map_metric = {"jerk": "average_robot_jerk", "distance": "average_distance_to_closest_person", "reach": "time_to_reach_goal"}
    for alg in alg_list:
        if alg == 'random':
            continue
        elif alg == 'no_req':
            for route in route_list:
                file_path = f'{output_folder}/{alg}/{route}'
                for req in req_list:
                    if req == 'distance':
                        maximize = False
                    else:
                        maximize = True
                    best_value = float('-inf') if maximize else float('inf')
                    best_index = None
                    for i in range(episode):
                        f_path = f'{file_path}/{i}/log.csv'
                        df = pd.read_csv(f_path)
                        mean_value = df[req_map_col[req]].iloc[-1]
                        if maximize:
                            if mean_value > best_value:
                                best_value = mean_value
                                best_index = i
                        else:
                            if mean_value < best_value:
                                best_value = mean_value
                                best_index = i
                    f_path = f'{file_path}/{best_index}'
                    b_folder = f'{best_folder}/{alg}/{req}/{route}/0'
                    if not os.path.exists(b_folder):
                        os.makedirs(b_folder)
                    src_file = f'{f_path}/{alg}_gpt-4.1-2025-04-14_0.json'
                    dst_file = os.path.join(b_folder, f'{alg}_gpt-4.1-2025-04-14_0.json')
                    shutil.copy(src_file, dst_file)
                    src_file = f'{f_path}/log_0.csv'
                    dst_file = os.path.join(b_folder, f'log_0.csv')
                    shutil.copy(src_file, dst_file)
                    df = pd.read_csv(f'{f_path}/log.csv')
                    best_row = df.iloc[-1].copy()
                    best_row['experiment_tag'] = '0_0'
                    dst_file = os.path.join(b_folder, 'log.csv')
                    best_row_df = best_row.to_frame().T
                    best_row_df.to_csv(dst_file, index=False)
            continue
        elif alg == 'distance':
            maximize = False
        else:
            maximize = True
        for route in route_list:
            file_path = f'{output_folder}/{alg}/{route}'
            memory_file = f'{file_path}/{alg}_memory_gpt-4.1-2025-04-14.json'
            with open(memory_file, 'r', encoding='utf-8') as f:
                memory_data = json.load(f)
            best_value = float('-inf') if maximize else float('inf')
            best_index = None
            for key, value in memory_data.items():
                avg_value = value[req_map_metric[alg]]
                if maximize:
                    if avg_value > best_value:
                        best_value = avg_value
                        best_index = int(key.split('_')[1])
                else:
                    if avg_value < best_value:
                        best_value = avg_value
                        best_index = int(key.split('_')[1])
            f_path = f'{file_path}/{best_index}'
            scenario_file = f'{f_path}/{alg}_scenario_gpt-4.1-2025-04-14.json'
            with open(scenario_file, 'r', encoding='utf-8') as f:
                scenario_data = json.load(f)
            best_index = None
            for key, value in scenario_data.items():
                if key == "scenario":
                    continue
                avg_value = value[req_map_metric[alg]]
                if avg_value == best_value:
                    best_index = int(key.split('_')[1])
                    break
            b_folder = f'{best_folder}/{alg}/{route}/0'
            if not os.path.exists(b_folder):
                os.makedirs(b_folder)
            src_file = f'{f_path}/{alg}_gpt-4.1-2025-04-14_{best_index}.json'
            dst_file = os.path.join(b_folder, f'{alg}_gpt-4.1-2025-04-14_0.json')
            shutil.copy(src_file, dst_file)
            src_file = f'{f_path}/log_{best_index}.csv'
            dst_file = os.path.join(b_folder, f'log_0.csv')
            shutil.copy(src_file, dst_file)
            df = pd.read_csv(f'{f_path}/log.csv')
            mask = df['experiment_tag'].astype(str).apply(lambda x: x.split('_')[-1] == str(best_index))
            best_row = df[mask].copy()
            best_row['experiment_tag'] = '0_0'
            dst_file = os.path.join(b_folder, f'log.csv')
            best_row.to_csv(dst_file, index=False)


def obtain_worst_scenario():
    worst_folder = f'{output_folder}/worst_scenario'
    req_map_col = {"jerk": "avg_robot_jerk", "distance": "avg_distance_to_closest_person", "reach": "time_to_reach_goal"}
    req_map_metric = {"jerk": "average_robot_jerk", "distance": "average_distance_to_closest_person", "reach": "time_to_reach_goal"}
    for alg in alg_list:
        if alg == 'random' or alg == 'no_req':
            continue
        elif alg == 'distance':
            maximize = True
        else:
            maximize = False
        for route in route_list:
            file_path = f'{output_folder}/{alg}/{route}'
            memory_file = f'{file_path}/{alg}_memory_gpt-4.1-2025-04-14.json'
            with open(memory_file, 'r', encoding='utf-8') as f:
                memory_data = json.load(f)
            best_value = float('-inf') if maximize else float('inf')
            best_index = None
            for key, value in memory_data.items():
                avg_value = value[req_map_metric[alg]]
                if maximize:
                    if avg_value > best_value:
                        best_value = avg_value
                        best_index = int(key.split('_')[1])
                else:
                    if avg_value < best_value:
                        best_value = avg_value
                        best_index = int(key.split('_')[1])
            f_path = f'{file_path}/{best_index}'
            scenario_file = f'{f_path}/{alg}_scenario_gpt-4.1-2025-04-14.json'
            with open(scenario_file, 'r', encoding='utf-8') as f:
                scenario_data = json.load(f)
            best_index = None
            for key, value in scenario_data.items():
                if key == "scenario":
                    continue
                avg_value = value[req_map_metric[alg]]
                if avg_value == best_value:
                    best_index = int(key.split('_')[1])
                    break
            b_folder = f'{worst_folder}/{alg}/{route}/0'
            if not os.path.exists(b_folder):
                os.makedirs(b_folder)
            src_file = f'{f_path}/{alg}_gpt-4.1-2025-04-14_{best_index}.json'
            dst_file = os.path.join(b_folder, f'{alg}_gpt-4.1-2025-04-14_0.json')
            shutil.copy(src_file, dst_file)
            src_file = f'{f_path}/log_{best_index}.csv'
            dst_file = os.path.join(b_folder, f'log_0.csv')
            shutil.copy(src_file, dst_file)
            df = pd.read_csv(f'{f_path}/log.csv')
            mask = df['experiment_tag'].astype(str).apply(lambda x: x.split('_')[-1] == str(best_index))
            best_row = df[mask].copy()
            best_row['experiment_tag'] = '0_0'
            dst_file = os.path.join(b_folder, f'log.csv')
            best_row.to_csv(dst_file, index=False)


def std_all_route():
    header = ["requirement", "route", "algorithm",
              "time_to_reach_goal", "path_length", "cumulative_heading_changes",
              "avg_distance_to_closest_person", "minimum_distance_to_people",
              "minimum_distance_to_target", "final_distance_to_target",
              "robot_on_person_collision", "person_on_robot_collision", "time_not_moving",
              "avg_robot_linear_speed", "avg_robot_angular_speed",
              "avg_robot_acceleration", "avg_robot_jerk",
              "avg_pedestrian_velocity", "avg_closest_pedestrian_velocity"]
    std_rows = []
    for req in req_list:
        for route in route_list:
            for scenario in ['best_scenario', 'worst_scenario']:
                folder_path = f'./{output_folder}/{scenario}/{req}/{route}/0'
                df = pd.read_csv(f'{folder_path}/log.csv')

                cols_to_keep = [col for col in df.columns[1:] if col != 'completed']
                df_data = df[cols_to_keep]
                df_data = df_data.apply(pd.to_numeric, errors='coerce')

                std_vals = df_data.std(axis=0)
                std_row = [req, route, scenario] + std_vals.tolist()
                std_rows.append(std_row)

    df_std = pd.DataFrame(std_rows, columns=header)
    df_std.to_csv(f'./{output_folder}/best_worst_std.csv', index=False)

    metrics = header[3:]
    df_std_mean = df_std.groupby(['requirement', 'algorithm'])[metrics].mean().reset_index()
    df_std_mean.to_csv(f'./{output_folder}/best_worst_std_all_route.csv', index=False)


def dtw_all_route():
    number = 30
    cols = ['robot_x', 'robot_y', 'worker1_x', 'worker1_y', 'worker2_x', 'worker2_y']
    dtw_rows = []
    for req in req_list:
        for scenario in ['best_scenario', 'worst_scenario']:
            for route in route_list:
                file_path = f'{output_folder}/{scenario}/{req}/{route}/0'
                simulation_data = []
                for i in range(number):
                    df = pd.read_csv(f'{file_path}/log_{i}.csv')
                    data = df[cols].to_numpy()
                    simulation_data.append(data)

                distances = []
                for i in range(number - 1):
                    for j in range(i + 1, number):
                        dist = dtw(simulation_data[i], simulation_data[j])
                        distances.append(dist)
                avg_dist = np.mean(distances)

                dtw_rows.append([req, scenario, route, avg_dist])

    df_dtw = pd.DataFrame(dtw_rows, columns=["requirement", "algorithm", "route", "dtw"])
    df_dtw.to_csv(f'./{output_folder}/best_worst_dtw.csv', index=False)
    df_mean = df_dtw.groupby(['requirement', 'algorithm'], as_index=False)['dtw'].mean()
    df_mean.columns = ['requirement', 'algorithm', 'dtw']
    df_mean.to_csv(f'./{output_folder}/best_worst_dtw_all_route.csv', index=False)


def p_a12_std(p_value, a12):
    if p_value < 0.01:
        p_sign = '<0.01'
    elif p_value < 0.05:
        p_sign = '<0.05'
    else:
        p_sign = '>=0.05'
    if 0.638 <= a12 < 0.714 or 1 - 0.638 >= a12 > 1 - 0.714:
        a12_sign = 'medium'
    elif 0.714 <= a12 <= 1.0 or 1 - 0.714 >= a12 >= 0:
        a12_sign = 'large'
    elif 0.556 <= a12 < 0.638 or 1 - 0.556 >= a12 > 1 - 0.638:
        a12_sign = 'small'
    else:
        a12_sign = 'no'
    return p_sign, a12_sign


def best_worst_statistic_test():
    header = [
        "requirement", "route",
        "time_to_reach_goal_p", "time_to_reach_goal_p_sign",
        "time_to_reach_goal_a12", "time_to_reach_goal_a12_sign",
        "path_length_p", "path_length_p_sign",
        "path_length_a12", "path_length_a12_sign",
        "cumulative_heading_changes_p", "cumulative_heading_changes_p_sign",
        "cumulative_heading_changes_a12", "cumulative_heading_changes_a12_sign",
        "avg_distance_to_closest_person_p", "avg_distance_to_closest_person_p_sign",
        "avg_distance_to_closest_person_a12", "avg_distance_to_closest_person_a12_sign",
        "minimum_distance_to_people_p", "minimum_distance_to_people_p_sign",
        "minimum_distance_to_people_a12", "minimum_distance_to_people_a12_sign",
        "minimum_distance_to_target_p", "minimum_distance_to_target_p_sign",
        "minimum_distance_to_target_a12", "minimum_distance_to_target_a12_sign",
        "final_distance_to_target_p", "final_distance_to_target_p_sign",
        "final_distance_to_target_a12", "final_distance_to_target_a12_sign",
        "robot_on_person_collision_p", "robot_on_person_collision_p_sign",
        "robot_on_person_collision_a12", "robot_on_person_collision_a12_sign",
        "person_on_robot_collision_p", "person_on_robot_collision_p_sign",
        "person_on_robot_collision_a12", "person_on_robot_collision_a12_sign",
        "time_not_moving_p", "time_not_moving_p_sign",
        "time_not_moving_a12", "time_not_moving_a12_sign",
        "avg_robot_linear_speed_p", "avg_robot_linear_speed_p_sign",
        "avg_robot_linear_speed_a12", "avg_robot_linear_speed_a12_sign",
        "avg_robot_angular_speed_p", "avg_robot_angular_speed_p_sign",
        "avg_robot_angular_speed_a12", "avg_robot_angular_speed_a12_sign",
        "avg_robot_acceleration_p", "avg_robot_acceleration_p_sign",
        "avg_robot_acceleration_a12", "avg_robot_acceleration_a12_sign",
        "avg_robot_jerk_p", "avg_robot_jerk_p_sign",
        "avg_robot_jerk_a12", "avg_robot_jerk_a12_sign",
        "avg_pedestrian_velocity_p", "avg_pedestrian_velocity_p_sign",
        "avg_pedestrian_velocity_a12", "avg_pedestrian_velocity_a12_sign",
        "avg_closest_pedestrian_velocity_p", "avg_closest_pedestrian_velocity_p_sign",
        "avg_closest_pedestrian_velocity_a12", "avg_closest_pedestrian_velocity_a12_sign"
    ]
    rows = []
    for req in req_list:
        for route in route_list:
            best_path = f'./{output_folder}/best_scenario/{req}/{route}/0'
            worst_path = f'./{output_folder}/worst_scenario/{req}/{route}/0'

            best_df = pd.read_csv(f'{best_path}/log.csv')
            worst_df = pd.read_csv(f'{worst_path}/log.csv')
            common_cols = [col for col in best_df.columns[1:] if col in worst_df.columns[1:]]

            results = []
            for col in common_cols:
                if col == 'completed':
                    continue
                else:
                    best_data = pd.to_numeric(best_df[col], errors='coerce').dropna()
                    worst_data = pd.to_numeric(worst_df[col], errors='coerce').dropna()
                    u_statistic, p_value = mannwhitneyu(best_data, worst_data, alternative='two-sided')
                    n1 = len(best_data)
                    n2 = len(worst_data)
                    a12 = (u_statistic / (n1 * n2))
                    p_sign, a12_sign = p_a12_std(p_value, a12)
                    results += [p_value, p_sign, a12, a12_sign]

            rows.append([req, route] + results)

    pd.DataFrame(rows, columns=header).to_csv(f'./{output_folder}/best_worst_statistic.csv', index=False)


def best_worst_statistic_test_all_route():
    header = [
        "requirement",
        "time_to_reach_goal_p", "time_to_reach_goal_p_sign",
        "time_to_reach_goal_a12", "time_to_reach_goal_a12_sign",
        "path_length_p", "path_length_p_sign",
        "path_length_a12", "path_length_a12_sign",
        "cumulative_heading_changes_p", "cumulative_heading_changes_p_sign",
        "cumulative_heading_changes_a12", "cumulative_heading_changes_a12_sign",
        "avg_distance_to_closest_person_p", "avg_distance_to_closest_person_p_sign",
        "avg_distance_to_closest_person_a12", "avg_distance_to_closest_person_a12_sign",
        "minimum_distance_to_people_p", "minimum_distance_to_people_p_sign",
        "minimum_distance_to_people_a12", "minimum_distance_to_people_a12_sign",
        "minimum_distance_to_target_p", "minimum_distance_to_target_p_sign",
        "minimum_distance_to_target_a12", "minimum_distance_to_target_a12_sign",
        "final_distance_to_target_p", "final_distance_to_target_p_sign",
        "final_distance_to_target_a12", "final_distance_to_target_a12_sign",
        "robot_on_person_collision_p", "robot_on_person_collision_p_sign",
        "robot_on_person_collision_a12", "robot_on_person_collision_a12_sign",
        "person_on_robot_collision_p", "person_on_robot_collision_p_sign",
        "person_on_robot_collision_a12", "person_on_robot_collision_a12_sign",
        "time_not_moving_p", "time_not_moving_p_sign",
        "time_not_moving_a12", "time_not_moving_a12_sign",
        "avg_robot_linear_speed_p", "avg_robot_linear_speed_p_sign",
        "avg_robot_linear_speed_a12", "avg_robot_linear_speed_a12_sign",
        "avg_robot_angular_speed_p", "avg_robot_angular_speed_p_sign",
        "avg_robot_angular_speed_a12", "avg_robot_angular_speed_a12_sign",
        "avg_robot_acceleration_p", "avg_robot_acceleration_p_sign",
        "avg_robot_acceleration_a12", "avg_robot_acceleration_a12_sign",
        "avg_robot_jerk_p", "avg_robot_jerk_p_sign",
        "avg_robot_jerk_a12", "avg_robot_jerk_a12_sign",
        "avg_pedestrian_velocity_p", "avg_pedestrian_velocity_p_sign",
        "avg_pedestrian_velocity_a12", "avg_pedestrian_velocity_a12_sign",
        "avg_closest_pedestrian_velocity_p", "avg_closest_pedestrian_velocity_p_sign",
        "avg_closest_pedestrian_velocity_a12", "avg_closest_pedestrian_velocity_a12_sign"
    ]
    rows = []
    for req in req_list:
        all_best_df = []
        all_worst_df = []
        for route in route_list:
            best_path = f'./{output_folder}/best_scenario/{req}/{route}/0'
            worst_path = f'./{output_folder}/worst_scenario/{req}/{route}/0'
            best_df = pd.read_csv(f'{best_path}/log.csv')
            all_best_df.append(best_df)
            worst_df = pd.read_csv(f'{worst_path}/log.csv')
            all_worst_df.append(worst_df)

        best_df = pd.concat(all_best_df, ignore_index=True)
        worst_df = pd.concat(all_worst_df, ignore_index=True)
        common_cols = [col for col in best_df.columns[1:] if col in worst_df.columns[1:]]

        results = []
        for col in common_cols:
            if col == 'completed':
                continue
            else:
                best_data = pd.to_numeric(best_df[col], errors='coerce').dropna()
                worst_data = pd.to_numeric(worst_df[col], errors='coerce').dropna()
                u_statistic, p_value = mannwhitneyu(best_data, worst_data, alternative='two-sided')
                n1 = len(best_data)
                n2 = len(worst_data)
                a12 = (u_statistic / (n1 * n2))
                p_sign, a12_sign = p_a12_std(p_value, a12)
                results += [p_value, p_sign, a12, a12_sign]

        rows.append([req] + results)

    pd.DataFrame(rows, columns=header).to_csv(f'./{output_folder}/best_worst_statistic_all_route.csv', index=False)


def std_statistic_format():
    metric_list = ["avg_distance_to_closest_person",
                   "avg_robot_jerk",
                   "time_to_reach_goal",
                   "robot_on_person_collision",
                   "cumulative_heading_changes",
                   "path_length",
                   "time_not_moving"]
    folder_path = f'./{output_folder}'
    all_results = {
        "route": [route for route in route_list for _ in range(len(metric_list))],
        "metric": metric_list * len(route_list),
    }
    for req in ['distance', 'jerk', 'reach']:
        for fm in ['std', 'statistic']:
            df = pd.read_csv(f'{folder_path}/best_worst_{fm}.csv')
            results_best = []
            results_worst = []
            results = []
            rs = []
            for route in route_list:
                for metric in metric_list:
                    if fm == 'statistic':
                        filtered = df[(df['requirement'] == req) & (df['route'] == route)]
                        value_p = filtered.iloc[0][f'{metric}_p_sign'] if not filtered.empty else None
                        value_s = filtered.iloc[0][f'{metric}_a12'] if not filtered.empty else None
                        results.append(value_p)
                        rs.append(round(value_s, 3))
                    else:
                        filtered_best = df[
                            (df['requirement'] == req) & (df['route'] == route) & (df['algorithm'] == 'best_scenario')]
                        value_best = filtered_best.iloc[0][metric] if not filtered_best.empty else None
                        if isinstance(value_best, float):
                            value_best = round(value_best, 3)
                        filtered_worst = df[
                            (df['requirement'] == req) & (df['route'] == route) & (df['algorithm'] == 'worst_scenario')]
                        value_worst = filtered_worst.iloc[0][metric] if not filtered_worst.empty else None
                        if isinstance(value_worst, float):
                            value_worst = round(value_worst, 3)
                        results_best.append(value_best)
                        results_worst.append(value_worst)
            if fm == 'statistic':
                all_results[f'{req}_{fm}_p'] = results
                all_results[f'{req}_{fm}_s'] = rs
            else:
                all_results[f'{req}_{fm}_best'] = results_best
                all_results[f'{req}_{fm}_worst'] = results_worst

    df = pd.DataFrame(all_results)
    df.to_csv(f'{folder_path}/merged_std_statistic.csv', index=False)


def std_statistic_format_all_route():
    metric_list = ["avg_distance_to_closest_person",
                   "avg_robot_jerk",
                   "time_to_reach_goal",
                   "robot_on_person_collision",
                   "cumulative_heading_changes",
                   "path_length",
                   "time_not_moving"]
    folder_path = f'./{output_folder}'
    all_results = {"metric": metric_list}
    for req in ['distance', 'jerk', 'reach']:
        for fm in ['std', 'statistic']:
            df = pd.read_csv(f'{folder_path}/best_worst_{fm}_all_route.csv')
            results_best = []
            results_worst = []
            results = []
            rs = []
            for metric in metric_list:
                if fm == 'statistic':
                    filtered = df[(df['requirement'] == req)]
                    value_p = filtered.iloc[0][f'{metric}_p_sign'] if not filtered.empty else None
                    value_s = filtered.iloc[0][f'{metric}_a12'] if not filtered.empty else None
                    results.append(value_p)
                    rs.append(round(value_s, 3))
                else:
                    filtered_best = df[(df['requirement'] == req) & (df['algorithm'] == 'best_scenario')]
                    filtered_worst = df[(df['requirement'] == req) & (df['algorithm'] == 'worst_scenario')]
                    value_best = filtered_best.iloc[0][f'{metric}'] if not filtered_best.empty else None
                    value_worst = filtered_worst.iloc[0][f'{metric}'] if not filtered_worst.empty else None
                    if isinstance(value_best, float):
                        value_best = round(value_best, 3)
                    if isinstance(value_worst, float):
                        value_worst = round(value_worst, 3)
                    results_best.append(value_best)
                    results_worst.append(value_worst)
            if fm == 'statistic':
                all_results[f'{req}_{fm}_p'] = results
                all_results[f'{req}_{fm}_s'] = rs
            else:
                all_results[f'{req}_{fm}_best'] = results_best
                all_results[f'{req}_{fm}_worst'] = results_worst

    df = pd.DataFrame(all_results)
    df.to_csv(f'{folder_path}/merged_all_route_std_statistic.csv', index=False)


def best_worst_plot_data():
    header = ["requirement", "route", "algorithm", "experiment_tag",
              "time_to_reach_goal", "path_length", "cumulative_heading_changes",
              "avg_distance_to_closest_person", "minimum_distance_to_people", "completed",
              "minimum_distance_to_target", "final_distance_to_target",
              "robot_on_person_collision", "person_on_robot_collision", "time_not_moving",
              "avg_robot_linear_speed", "avg_robot_angular_speed",
              "avg_robot_acceleration", "avg_robot_jerk",
              "avg_pedestrian_velocity", "avg_closest_pedestrian_velocity"]
    all_dfs = []
    for req in ['distance', 'jerk', 'reach']:
        for route in route_list:
            for scenario in ['best_scenario', 'worst_scenario']:
                folder_path = f'./{output_folder}/{scenario}/{req}/{route}/0'
                df = pd.read_csv(f'{folder_path}/log.csv')
                df['requirement'] = req
                df['route'] = route
                df['algorithm'] = scenario
                df = df[header]
                all_dfs.append(df)
    df_all = pd.concat(all_dfs, ignore_index=True)
    df_all.to_csv(f'./{output_folder}/merged_log.csv', index=False)


def best_worst_plot_figure():
    df = pd.read_csv(f'./{output_folder}/merged_log.csv')
    metrics = ['avg_distance_to_closest_person', 'avg_robot_jerk', 'time_to_reach_goal',
               'robot_on_person_collision', 'cumulative_heading_changes', 'path_length', 'time_not_moving']
    requirement_rename = {'distance': 'R1', 'jerk': 'R2', 'reach': 'R3'}
    metrics_rename = {
        'time_to_reach_goal': 'TRG',
        'path_length': 'PL',
        'cumulative_heading_changes': 'CHC',
        'avg_distance_to_closest_person': 'DTO',
        'robot_on_person_collision': 'RPC',
        'time_not_moving': 'TNM',
        'avg_robot_jerk': 'Jerk'
    }
    route_rename = {'route_1': 'Route_1', 'route_2': 'Route_2', 'route_3': 'Route_3', 'route_4': 'Route_4', 'route_5': 'Route_5'}

    df_long = df.melt(id_vars=["requirement", "route", "algorithm"],
                      value_vars=metrics,
                      var_name='metric', value_name='value')
    df_long['requirement'] = df_long['requirement'].map(requirement_rename)
    df_long['metric'] = df_long['metric'].map(metrics_rename)
    df_long['route'] = df_long['route'].map(route_rename)

    g = sns.catplot(
        data=df_long,
        x="requirement",
        y="value",
        hue="algorithm",
        kind="violin",
        split=True,
        col="route",
        row="metric",
        height=2.5,
        aspect=1.2,
        density_norm='width',
        inner=None,
        sharey=False
    )
    g.set_titles("{col_name}")
    n_rows = len(g.row_names)
    n_cols = len(g.col_names)
    for row in range(n_rows):
        for col in range(n_cols):
            ax = g.axes[row, col]
            if row == 0:
                ax.set_title(ax.get_title(), fontsize=30, fontweight='bold')
            else:
                ax.set_title("")
    # g.figure.subplots_adjust(wspace=0.1)
    g.figure.subplots_adjust(hspace=0.07)

    for row_idx, metric in enumerate(g.row_names):
        ax = g.axes[row_idx, -1]
        ax.text(
            1.05, 0.5,
            metric,
            fontsize=30, fontweight='bold',
            rotation=-90,
            va='center', ha='left',
            transform=ax.transAxes
        )

    for row_idx, metric in enumerate(g.row_names):
        for col_idx, route in enumerate(g.col_names):
            ax = g.axes[row_idx, col_idx]
            df_sub = df_long[(df_long['route'] == route) & (df_long['metric'] == metric)]
            sns.boxplot(
                data=df_sub,
                x="requirement",
                y="value",
                hue="algorithm",
                ax=ax,
                showcaps=True,
                capprops=dict(color='white', linewidth=2.5),
                boxprops=dict(edgecolor='white', linewidth=2.5, facecolor='black'),
                whiskerprops=dict(color='white', linewidth=2.5, linestyle='-'),
                showfliers=False,
                medianprops=dict(color='none', linewidth=0, visible=False),
                showmeans=True,
                meanline=True,
                meanprops=dict(color='white', linewidth=2.5, linestyle='-'),
                zorder=10,
                dodge=True,
                width=0.3
            )
            ax.get_legend().remove() if ax.get_legend() else None

    if g._legend is not None:
        g._legend.remove()
    handles, labels = None, None
    for ax in g.axes.flatten():
        h, l = ax.get_legend_handles_labels()
        if len(l) > 0:
            handles, labels = h, l
            break
    if handles and labels:
        new_labels = [r'$\mathbf{RVSG_{B}}$', r'$\mathbf{RVSG_{W}}$']
        leg = g.figure.legend(
            handles, new_labels,
            loc='lower center',
            bbox_to_anchor=(0.8, -0.02),
            ncol=len(labels),
            frameon=True
        )
        leg.get_frame().set_edgecolor('black')
        leg.get_frame().set_linewidth(1.2)
        leg.get_frame().set_linestyle('solid')
        for text in leg.get_texts():
            text.set_fontsize(22)
            text.set_fontweight('bold')

    for row in range(n_rows):
        for col in range(n_cols):
            ax = g.axes[row, col]
            if row == n_rows - 1 and col == n_cols // 2:
                ax.set_xlabel("Requirement", fontsize=30, fontweight='bold', labelpad=8)
            else:
                ax.set_xlabel("")
            if row == n_rows // 2 and col == 0:
                ax.set_ylabel("Value", fontsize=30, fontweight='bold')
            else:
                ax.set_ylabel("")

            ax.tick_params(axis='x', labelsize=18)
            for label in ax.get_xticklabels():
                label.set_fontweight('bold')
            ax.tick_params(axis='y', labelsize=18)
            for label in ax.get_yticklabels():
                label.set_fontweight('bold')

            yticks = ax.get_yticks()
            if len(yticks) > 2:
                yticks = yticks[1:-1]
            for y in yticks:
                if ax.get_ylim()[0] < y < ax.get_ylim()[1]:
                    ax.axhline(
                        y,
                        color='lightgray',
                        linestyle='-',
                        linewidth=0.7,
                        zorder=-10
                    )

    g.savefig(f'./{output_folder}/plot.pdf', dpi=300, bbox_inches='tight')
    g.savefig(f'./{output_folder}/plot.png', dpi=300, bbox_inches='tight')


def best_worst_plot_figure_all_route():
    df = pd.read_csv(f'./{output_folder}/merged_log.csv')
    metrics = ['avg_distance_to_closest_person', 'avg_robot_jerk', 'time_to_reach_goal',
               'robot_on_person_collision', 'cumulative_heading_changes', 'path_length', 'time_not_moving']
    requirement_rename = {'distance': 'R1', 'jerk': 'R2', 'reach': 'R3'}
    metrics_rename = {
        'time_to_reach_goal': 'TRG',
        'path_length': 'PL',
        'cumulative_heading_changes': 'CHC',
        'avg_distance_to_closest_person': 'DTO',
        'robot_on_person_collision': 'RPC',
        'time_not_moving': 'TNM',
        'avg_robot_jerk': 'Jerk'
    }

    df_long = df.melt(id_vars=["requirement", "algorithm"],
                      value_vars=metrics,
                      var_name='metric', value_name='value')
    df_long['requirement'] = df_long['requirement'].map(requirement_rename)
    df_long['metric'] = df_long['metric'].map(metrics_rename)

    g = sns.catplot(
        data=df_long,
        x="requirement",
        y="value",
        hue="algorithm",
        kind="violin",
        split=True,
        col="metric",
        col_wrap=4,
        height=3,
        aspect=1.0,
        density_norm='width',
        inner=None,
        sharey=False
    )
    g.set_titles("{col_name}")
    for ax in g.axes.flatten():
        ax.set_title(ax.get_title(), fontsize=20, fontweight='bold')
    # g.figure.subplots_adjust(wspace=0.5)

    for ax, metric in zip(g.axes.flatten(), g.col_names):
        df_sub = df_long[df_long['metric'] == metric]
        sns.boxplot(
            data=df_sub,
            x="requirement",
            y="value",
            hue="algorithm",
            ax=ax,
            showcaps=True,
            capprops=dict(color='white', linewidth=2.5),
            boxprops=dict(edgecolor='white', linewidth=2.5, facecolor='black'),
            whiskerprops=dict(color='white', linewidth=2.5, linestyle='-'),
            showfliers=False,
            medianprops=dict(color='none', linewidth=0, visible=False),
            showmeans=True,
            meanline=True,
            meanprops=dict(color='white', linewidth=2.5, linestyle='-'),
            zorder=10,
            dodge=True,
            width=0.3
        )
        ax.get_legend().remove() if ax.get_legend() else None

    if g._legend is not None:
        g._legend.remove()
    handles, labels = None, None
    for ax in g.axes.flatten():
        h, l = ax.get_legend_handles_labels()
        if len(l) > 0:
            handles, labels = h, l
            break
    if handles and labels:
        new_labels = [r'$\mathbf{RVSG_{B}}$', r'$\mathbf{RVSG_{W}}$']
        leg = g.figure.legend(
            handles, new_labels,
            loc='center left',
            bbox_to_anchor=(0.72, 0.28),
            ncol=1,
            frameon=True,
            title='Scenario'
        )
        leg.get_title().set_fontsize(18)
        leg.get_title().set_fontweight('bold')
        leg.get_frame().set_edgecolor('black')
        leg.get_frame().set_linewidth(1.2)
        leg.get_frame().set_linestyle('solid')
        for text in leg.get_texts():
            text.set_fontsize(16)
            text.set_fontweight('bold')

    g.figure.text(
        0.46,
        0.0,
        "Requirement",
        ha='center', va='center',
        fontsize=22, fontweight='bold'
    )

    g.figure.text(
        0.025,
        0.52,
        "Value",
        ha='center', va='center', rotation='vertical',
        fontsize=22, fontweight='bold'
    )

    n_cols = 4
    n_subplots = len(g.axes.flatten())
    n_rows = (n_subplots + n_cols - 1) // n_cols
    for i, ax in enumerate(g.axes.flatten()):
        # col_idx = i % n_cols
        # row_idx = i // n_cols
        # if col_idx == n_cols // 2 and row_idx == n_rows - 1:
        #     ax.set_xlabel("Requirement", fontsize=22, fontweight='bold', labelpad=8)
        # else:
        #     ax.set_xlabel("")
        # if row_idx == n_rows // 2 and col_idx == 0:
        #     ax.set_ylabel("Value", fontsize=22, fontweight='bold')
        # else:
        #     ax.set_ylabel("")

        ax.set_xlabel("")
        ax.set_ylabel("")

        ax.tick_params(axis='x', labelsize=16)
        for label in ax.get_xticklabels():
            label.set_fontweight('bold')
        ax.tick_params(axis='y', labelsize=14)
        for label in ax.get_yticklabels():
            label.set_fontweight('bold')

        yticks = ax.get_yticks()
        if len(yticks) > 2:
            yticks = yticks[1:-1]
        for y in yticks:
            if ax.get_ylim()[0] < y < ax.get_ylim()[1]:
                ax.axhline(
                    y,
                    color='lightgray',
                    linestyle='-',
                    linewidth=0.7,
                    zorder=-10
                )

    g.savefig(f'./{output_folder}/plot_all_route.pdf', dpi=300, bbox_inches='tight')
    g.savefig(f'./{output_folder}/plot_all_route.png', dpi=300, bbox_inches='tight')


if __name__ == "__main__":
    output_folder = "output"
    alg_list = ['jerk', 'distance', 'reach', 'no_req', 'random']
    req_list = ['jerk', 'distance', 'reach']
    route_list = ['route_1', 'route_2', 'route_3', 'route_4', 'route_5']
    episode = 10

    # python ./src/test_pal_robotics/test_pal_robotics/analyze.py
    avg_req()
    avg_metrics()
    avg_metrics_format()
    avg_metrics_requirement_format()
    obtain_best_scenario()
    obtain_worst_scenario()

    std_all_route()
    dtw_all_route()
    best_worst_statistic_test()
    best_worst_statistic_test_all_route()

    std_statistic_format()
    std_statistic_format_all_route()

    sentence_similarity()
    levenshtein_distance()
    dynamic_time_warping()

    best_worst_plot_data()
    best_worst_plot_figure()
    best_worst_plot_figure_all_route()
