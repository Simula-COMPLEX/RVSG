import argparse
import json
import os
import random
from test_hunav import model, output_folder
from waypoints import areas, area_map_waypoints, waypoints


def point_index(p):
    return rows.index(p[0]), int(p[1:])-1


def is_neighbor(p1, p2):
    r1, c1 = point_index(p1)
    r2, c2 = point_index(p2)
    return max(abs(r1 - r2), abs(c1 - c2)) == 1


def point_str(row_idx, col_idx):
    return rows[row_idx] + str(col_idx + 1)


def all_points_in_grid(p1, p2):
    r1, c1 = point_index(p1)
    r2, c2 = point_index(p2)
    row_min, row_max = min(r1, r2), max(r1, r2)
    col_min, col_max = min(c1, c2), max(c1, c2)
    points = [point_str(r, c)
              for r in range(row_min, row_max+1)
              for c in range(col_min, col_max+1)]
    points = [point for point in points if point in waypoints]
    return points


def divide_path(start, end, path):
    if is_neighbor(start, end):
        return
    grid_points = all_points_in_grid(start, end)
    grid_points = [pt for pt in grid_points if pt not in [start, end]]
    if not grid_points:
        return
    mid = random.choice(grid_points)
    divide_path(start, mid, path)
    path.append(mid)
    divide_path(mid, end, path)


def generate_worker_tasks():
    area_num = random.randint(2, len(areas))
    area_list = random.choices(areas, k=area_num)
    area_map_waypoint = []
    for area in area_list:
        waypoint = random.choice(area_map_waypoints[area])
        area_map_waypoint.append(waypoint)

    worker_tasks = []
    for index in range(len(area_map_waypoint)):
        area1 = area_list[index]
        area2 = area_list[(index + 1) % len(area_map_waypoint)]
        waypoint1 = area_map_waypoint[index]
        waypoint2 = area_map_waypoint[(index + 1) % len(area_map_waypoint)]

        full_path = []
        divide_path(waypoint1, waypoint2, full_path)
        full_path = [waypoint1] + full_path + [waypoint2]

        task_dict = {
            "Task": f"{area1} → {area2}",
            "Start": waypoint1,
            "End": waypoint2,
            "Path": full_path
        }
        worker_tasks.append(task_dict)

    # print(worker_tasks)
    return worker_tasks


def json_scenario_summary(req, route, number, episode):
    scenario_filename = f"{output_folder}/{req}/{route}/{episode}/{req}_scenario_{model}.json"
    filename = f"{output_folder}/{req}/{route}/{episode}/{req}_{model}_{number}.json"
    with open(filename, 'r', encoding='utf-8') as f:
        task_data = json.load(f)
    key = f"scenario_{number}"
    tasks_1 = [item["Task"] for item in task_data.get("Worker 1 Tasks", []) if "Task" in item]
    tasks_2 = [item["Task"] for item in task_data.get("Worker 2 Tasks", []) if "Task" in item]
    data = {
        key: {
            "Worker 1": tasks_1,
            "Worker 2": tasks_2
        }
    }
    with open(scenario_filename, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def main(args=None):
    worker1_tasks = generate_worker_tasks()
    worker2_tasks = generate_worker_tasks()
    data = {
        "Worker 1 Tasks": worker1_tasks,
        "Worker 2 Tasks": worker2_tasks
    }

    out_folder = f"{output_folder}/{args.req}/{args.route}/{args.episode}"
    if not os.path.exists(out_folder):
        os.makedirs(out_folder)
    output_file = f"{out_folder}/{args.req}_{model}_{args.number}.json"
    with open(output_file, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    json_scenario_summary(req=args.req, route=args.route, number=args.number, episode=args.episode)


if __name__ == '__main__':
    rows = [chr(ord('A') + i) for i in range(13)]  # A-M
    cols = [str(i + 1) for i in range(22)]  # 1-22
    all_points = [r + c for r in rows for c in cols]
    all_points = [point for point in all_points if point in waypoints]

    parser = argparse.ArgumentParser()
    parser.add_argument("--req", type=str, default="random", help="requirement")
    parser.add_argument("--route", type=str, default="route_1", help="route")
    parser.add_argument("--number", type=int, default=0, help="number")
    parser.add_argument("--episode", type=int, default=0, help="episode")
    args = parser.parse_args()
    main(args)