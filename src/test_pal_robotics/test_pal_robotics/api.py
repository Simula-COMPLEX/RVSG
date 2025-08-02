import argparse
import json
import os
import re
import sys
import time
import pandas as pd
from openai import OpenAI
from prompt_image import prompt_image_1, prompt_image_2, prompt_image_3
from prompt_route import (prompt_route_1, prompt_route_1_start_end, prompt_route_2, prompt_route_2_start_end,
                          prompt_route_3, prompt_route_3_start_end, prompt_route_4, prompt_route_4_start_end,
                          prompt_route_5, prompt_route_5_start_end)
from prompt_random import prompt_random_1, prompt_random_2, prompt_random_3, prompt_random_4, prompt_random_5
from prompt_jerk import prompt_jerk_1, prompt_jerk_2, prompt_jerk_3, prompt_jerk_4, prompt_jerk_5, prompt_jerk_6, prompt_jerk_memory_1
from prompt_distance import prompt_distance_1, prompt_distance_2, prompt_distance_3, prompt_distance_4, prompt_distance_5, prompt_distance_6, prompt_distance_memory_1
from prompt_reach import prompt_reach_1, prompt_reach_2, prompt_reach_3, prompt_reach_4, prompt_reach_5, prompt_reach_6, prompt_reach_memory_1
from test_hunav import model, output_folder


image_1_url = "https://raw.githubusercontent.com/wjh-test/image/refs/heads/main/000.png"
image_2_url = "https://raw.githubusercontent.com/wjh-test/image/refs/heads/main/003.png"
route_prompt = {
    "route_1": [prompt_route_1, prompt_route_1_start_end],
    "route_2": [prompt_route_2, prompt_route_2_start_end],
    "route_3": [prompt_route_3, prompt_route_3_start_end],
    "route_4": [prompt_route_4, prompt_route_4_start_end],
    "route_5": [prompt_route_5, prompt_route_5_start_end],
}


def chat_completions(client, input_messages, top_logprobs=0):
    response = client.chat.completions.create(
        model=model,
        messages=input_messages,
        logprobs=True,
        top_logprobs=top_logprobs
    )
    return response


def write_to_file(output_file, prompt, output_text):
    with open(output_file, "a", encoding="utf-8") as f:
        f.write(f"\n{prompt}\n\n")
        f.write("=" * 50 + "\n\n\n")
        f.write(f"{output_text}\n\n\n")
        f.write("=" * 200 + "\n\n")


def api_image(client):

    output_file = f"{output_folder}/image_analysis_{model}.txt"
    with open(output_file, "w") as f:
        f.write("=" * 200 + "\n\n")

    start_time = time.time()

    input_messages = [{
        "role": "user",
        "content": [
            {
                "type": "image_url",
                "image_url": {
                    "url": image_1_url,
                }
            },
            {
                "type": "image_url",
                "image_url": {
                    "url": image_2_url,
                }
            },
            {"type": "text", "text": prompt_image_1},
        ],
    }]
    response = chat_completions(client, input_messages)
    write_to_file(output_file, prompt_image_1, response.choices[0].message.content)
    input_messages.append({"role": response.choices[0].message.role, "content": response.choices[0].message.content})

    input_messages.append({
        "role": "user",
        "content": [
            {
                "type": "image_url",
                "image_url": {
                    "url": image_1_url,
                }
            },
            {
                "type": "image_url",
                "image_url": {
                    "url": image_2_url,
                }
            },
            {"type": "text", "text": prompt_image_2},
        ],
    })
    response = chat_completions(client, input_messages)
    write_to_file(output_file, prompt_image_2, response.choices[0].message.content)
    input_messages.append({"role": response.choices[0].message.role, "content": response.choices[0].message.content})

    input_messages.append({
        "role": "user",
        "content": [
            {"type": "text", "text": prompt_image_3},
        ],
    })
    response = chat_completions(client, input_messages, 20)
    write_to_file(output_file, prompt_image_3, response.choices[0].message.content)
    json_save(response.to_dict(), output_file.split(".txt")[0] + '_logprobs.json')

    end_time = time.time()
    json_extract_check(response.choices[0].message.content, output_file.replace(".txt", ".json"))
    print(f"api image analysis time {end_time - start_time:.2f}s\n")


def api_req(client, req, route, episode, memory):

    if req == "jerk":
        if memory:
            prompt_list = [prompt_jerk_memory_1.format(route_prompt[route][0], json_memory_sample(req, route), route_prompt[route][1], route_prompt[route][1]),
                           prompt_jerk_2, prompt_jerk_3, prompt_jerk_4, prompt_jerk_5]
        else:
            prompt_list = [prompt_jerk_1.format(route_prompt[route][0], route_prompt[route][1], route_prompt[route][1]),
                           prompt_jerk_2, prompt_jerk_3, prompt_jerk_4, prompt_jerk_5]
    elif req == "distance":
        if memory:
            prompt_list = [prompt_distance_memory_1.format(route_prompt[route][0], json_memory_sample(req, route), route_prompt[route][1], route_prompt[route][1]),
                           prompt_distance_2, prompt_distance_3, prompt_distance_4, prompt_distance_5]
        else:
            prompt_list = [prompt_distance_1.format(route_prompt[route][0], route_prompt[route][1], route_prompt[route][1]),
                           prompt_distance_2, prompt_distance_3, prompt_distance_4, prompt_distance_5]
    elif req == "reach":
        if memory:
            prompt_list = [prompt_reach_memory_1.format(route_prompt[route][0], json_memory_sample(req, route), route_prompt[route][1], route_prompt[route][1]),
                           prompt_reach_2, prompt_reach_3, prompt_reach_4, prompt_reach_5]
        else:
            prompt_list = [prompt_reach_1.format(route_prompt[route][0], route_prompt[route][1], route_prompt[route][1]),
                           prompt_reach_2, prompt_reach_3, prompt_reach_4, prompt_reach_5]
    elif req == "no_req":
        prompt_list = [prompt_random_1.format(route_prompt[route][0]), prompt_random_2, prompt_random_3, prompt_random_4, prompt_random_5]

    print(f"{model}...")
    out_folder = f"{output_folder}/{req}/{route}/{episode}"
    if not os.path.exists(out_folder):
        os.makedirs(out_folder)
    output_file = f"{out_folder}/{req}_{model}.txt"
    with open(output_file, "w") as f:
        f.write("=" * 200 + "\n\n")

    while True:
        try:
            start_time = time.time()
            input_messages = []
            for index, prompt in enumerate(prompt_list):
                if index == 0:
                    input_messages += [{
                        "role": "user",
                        "content": [
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": image_1_url,
                                }
                            },
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": image_2_url,
                                }
                            },
                            {"type": "text", "text": prompt},
                        ],
                    }]
                    response = chat_completions(client, input_messages)
                    write_to_file(output_file, prompt, response.choices[0].message.content)
                    json_extract_check(response.choices[0].message.content, f"{output_file.split(model)[0]}scenario_{model}.json")
                    input_messages.append({"role": response.choices[0].message.role, "content": response.choices[0].message.content})
                else:
                    input_messages.append({
                        "role": "user",
                        "content": [
                            {"type": "text", "text": prompt},
                        ],
                    })
                    if index == len(prompt_list) - 1:
                        response = chat_completions(client, input_messages, 20)
                        json_save(response.to_dict(), output_file.split(".txt")[0] + '_0_logprobs.json')
                    else:
                        response = chat_completions(client, input_messages)
                    write_to_file(output_file, prompt, response.choices[0].message.content)
                    input_messages.append({"role": response.choices[0].message.role, "content": response.choices[0].message.content})
            end_time = time.time()
            break
        except Exception as e:
            print('Exception Error: ', e)

    print(f"api requirement {req} time {end_time - start_time:.2f}s\n")
    json_extract_check(response.choices[0].message.content, f"{output_file.split('.txt')[0]}_0.json")
    json_scenario_summary(req, route, 0, episode)
    json_save(input_messages, f"{output_folder}/input_messages.json")


def api_req_multi(client, req, route, number, episode):

    output_file = f"{output_folder}/{req}/{route}/{episode}/{req}_{model}.txt"
    num = number - 1
    req_avg = calculate_avg(req, route, num, episode)
    feedback_log = log_to_json(req, route, num, episode).replace('{', '{{').replace('}', '}}')

    if req == "jerk":
        prompt_req_6 = prompt_jerk_6.format(req_avg, feedback_log)
        prompt_req_4 = prompt_jerk_4
        prompt_req_5 = prompt_jerk_5
    elif req == "distance":
        prompt_req_6 = prompt_distance_6.format(req_avg, feedback_log)
        prompt_req_4 = prompt_distance_4
        prompt_req_5 = prompt_distance_5
    elif req == "reach":
        prompt_req_6 = prompt_reach_6.format(req_avg, feedback_log)
        prompt_req_4 = prompt_reach_4
        prompt_req_5 = prompt_reach_5

    while True:
        try:
            input_messages = json_load(f"{output_folder}/input_messages.json")
            start_time = time.time()

            input_messages.append({
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt_req_6},
                ],
            })
            response = chat_completions(client, input_messages)
            write_to_file(output_file, prompt_req_6, response.choices[0].message.content)
            input_messages.append({"role": response.choices[0].message.role, "content": response.choices[0].message.content})

            input_messages.append({
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt_req_4},
                ],
            })
            response = chat_completions(client, input_messages)
            write_to_file(output_file, prompt_req_4, response.choices[0].message.content)
            input_messages.append({"role": response.choices[0].message.role, "content": response.choices[0].message.content})

            input_messages.append({
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt_req_5},
                ],
            })
            response = chat_completions(client, input_messages, 20)
            write_to_file(output_file, prompt_req_5, response.choices[0].message.content)
            input_messages.append({"role": response.choices[0].message.role, "content": response.choices[0].message.content})
            json_save(response.to_dict(), f'{output_file.split(".txt")[0]}_{number}_logprobs.json')

            end_time = time.time()
            break
        except Exception as e:
            print('Exception Error: ', e)

    print(f"api requirement {req} multi time {end_time - start_time:.2f}s\n")
    json_extract_check(response.choices[0].message.content, f"{output_file.split('.txt')[0]}_{number}.json")
    json_scenario_summary(req, route, number, episode)
    json_save(input_messages, f"{output_folder}/input_messages.json")


def json_save(data, output_file):
    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

def json_load(input_file):
    with open(input_file, 'r', encoding='utf-8') as f:
        data = json.load(f)
    return data


def json_extract_check(text, output_file):
    match = re.search(r"```json\s*(\{.*?\})\s*```", text, re.DOTALL)
    if not match:
        matches = re.findall(r"(\{.*?\})", text, re.DOTALL)
        if matches:
            json_str = matches[-1]
        else:
            print("No valid JSON block found.")
            sys.exit(1)
    else:
        json_str = match.group(1)
    try:
        data = json.loads(json_str)
    except json.JSONDecodeError as e:
        print("Invalid JSON format:", e)
        sys.exit(1)

    with open(output_file, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def json_scenario_summary(req, route, number, episode):
    scenario_filename = f"{output_folder}/{req}/{route}/{episode}/{req}_scenario_{model}.json"
    with open(scenario_filename, 'r', encoding='utf-8') as f:
        data = json.load(f)

    filename = f"{output_folder}/{req}/{route}/{episode}/{req}_{model}_{number}.json"
    with open(filename, 'r', encoding='utf-8') as f:
        task_data = json.load(f)
    key = f"scenario_{number}"
    tasks_1 = [item["Task"] for item in task_data.get("Worker 1 Tasks", []) if "Task" in item]
    tasks_2 = [item["Task"] for item in task_data.get("Worker 2 Tasks", []) if "Task" in item]
    data[key] = {
        "Worker 1": tasks_1,
        "Worker 2": tasks_2
    }

    with open(scenario_filename, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def json_scenario_req(req, route, number, episode):
    scenario_filename = f"{output_folder}/{req}/{route}/{episode}/{req}_scenario_{model}.json"
    with open(scenario_filename, 'r', encoding='utf-8') as f:
        data = json.load(f)
    if req == "jerk":
        metric_name = f"average_robot_{req}"
    elif req == "distance":
        metric_name = f"average_distance_to_closest_person"
    elif req == "reach":
        metric_name = f"time_to_reach_goal"
    data[f"scenario_{number}"][metric_name] = calculate_avg(req, route, number, episode)
    with open(scenario_filename, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def obtain_max_scenario(scenario_data, metric_name):
    max_req = float('-inf')
    max_scenario = None
    for scenario, data in scenario_data.items():
        if isinstance(data, dict) and metric_name in data:
            avg_req = data[metric_name]
            if avg_req > max_req:
                max_req = avg_req
                max_scenario = scenario
    return scenario_data[max_scenario]

def obtain_min_scenario(scenario_data, metric_name):
    min_req = float('inf')
    min_scenario = None
    for scenario, data in scenario_data.items():
        if isinstance(data, dict) and metric_name in data:
            avg_req = data[metric_name]
            if avg_req < min_req:
                min_req = avg_req
                min_scenario = scenario
    return scenario_data[min_scenario]

def json_memory_push(req, route, episode):
    scenario_filename = f"{output_folder}/{req}/{route}/{episode}/{req}_scenario_{model}.json"
    with open(scenario_filename, 'r', encoding='utf-8') as f:
        scenario_data = json.load(f)

    scenario_dict = {}
    if 'scenario' in scenario_data:
        scenario_dict['summary'] = scenario_data['scenario']

    if req == "no_req":
        scenario_dict.update(scenario_data['scenario_0'])
    else:
        if req == "jerk":
            metric_name = f"average_robot_{req}"
            scenario_dict.update(obtain_max_scenario(scenario_data, metric_name))
        elif req == "distance":
            metric_name = f"average_distance_to_closest_person"
            scenario_dict.update(obtain_min_scenario(scenario_data, metric_name))
        elif req == "reach":
            metric_name = f"time_to_reach_goal"
            scenario_dict.update(obtain_max_scenario(scenario_data, metric_name))

    memory_filename = f"{output_folder}/{req}/{route}/{req}_memory_{model}.json"
    if os.path.exists(memory_filename):
        with open(memory_filename, 'r', encoding='utf-8') as f:
            data = json.load(f)
    else:
        data = {}
    data[f'scenario_{episode}'] = scenario_dict
    with open(memory_filename, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def json_memory_sample(req, route):
    memory_filename = f"{output_folder}/{req}/{route}/{req}_memory_{model}.json"
    with open(memory_filename, 'r', encoding='utf-8') as f:
        data = json.load(f)

    str_data = ['{']
    indent = 2
    space = ' ' * indent
    for scen, content in data.items():
        str_data.append(f'{space}"{scen}": {{')
        for key, value in content.items():
            line = space * 2 + f'"{key}": ' + json.dumps(value, ensure_ascii=False) + ','
            str_data.append(line)
        str_data.append(f'{space}' + '},')
    str_data.append('}')
    return '\n'.join(str_data)


def calculate_avg(req, route, number, episode):
    df = pd.read_csv(f'{output_folder}/{req}/{route}/{episode}/log_{number}.csv')
    if req == "jerk":
        col = df["robot_jerk"]
        mean_value = col.iloc[1:].mean()
    elif req == "distance":
        col = df["distance_to_closest_person"]
        mean_value = col.mean()
    elif req == "reach":
        mean_value = df["timestamp"].iloc[-1]
    return mean_value


def log_to_json(req, route, number, episode):
    df = pd.read_csv(f'{output_folder}/{req}/{route}/{episode}/log_{number}.csv')
    if req == "jerk":
        cols = ['timestamp', 'robot_wp', 'worker1_wp', 'worker2_wp', "robot_jerk"]
    elif req == "distance":
        cols = ['timestamp', 'robot_wp', 'worker1_wp', 'worker2_wp', "distance_to_closest_person"]
    elif req == "reach":
        cols = ['timestamp', 'robot_wp', 'worker1_wp', 'worker2_wp']
    df_sub = df[cols]
    data = df_sub.to_dict(orient='records')
    json_lines = "[\n  " + ",\n  ".join(json.dumps(row, ensure_ascii=False) for row in data) + "\n]"
    with open(f'{output_folder}/{req}/{route}/{episode}/log_{number}.json', 'w', encoding='utf-8') as f:
        f.write(json_lines)
    return json_lines


def str_to_bool(value):
    if value.lower() in ('true', '1', 'yes'):
        return True
    elif value.lower() in ('false', '0', 'no'):
        return False


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Select function to run")
    parser.add_argument("func", choices=["image", "req", "req_m", "json_req", "json_memory", "avg", "log"], help="Function to execute")
    parser.add_argument("--number", type=int, default=0, help="number")
    parser.add_argument("--episode", type=int, default=0, help="episode")
    parser.add_argument("--route", type=str, default="route_1", help="route")
    parser.add_argument("--req", type=str, default="jerk", help="requirement")
    parser.add_argument("--memory", type=str_to_bool, default="False", help="memory")
    args = parser.parse_args()

    api_key = "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"
    client = OpenAI(api_key=api_key)

    if args.func == "image":
        api_image(client)
    elif args.func == "req":
        api_req(client, args.req, args.route, args.episode, args.memory)
    elif args.func == "req_m":
        api_req_multi(client, args.req, args.route, args.number, args.episode)
    elif args.func == "json_req":
        json_scenario_req(args.req, args.route, args.number, args.episode)
    elif args.func == "json_memory":
        json_memory_push(args.req, args.route, args.episode)
    elif args.func == "avg":
        calculate_avg(args.req, args.route, args.number, args.episode)
    elif args.func == "log":
        log_to_json(args.req, args.route, args.number, args.episode)
