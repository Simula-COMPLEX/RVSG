import argparse
import json
import yaml
from waypoints import waypoints
from test_hunav import model, output_folder


def main(args=None):

    json_file = f"{output_folder}/{args.req_folder}/{args.route}/{args.episode}/{args.req}_{model}_{args.number}.json"
    with open(json_file, "r", encoding="utf-8") as f:
        data = json.load(f)

    worker1 = data.get("Worker 1 Tasks", [])
    worker2 = data.get("Worker 2 Tasks", [])

    worker1_goals = []
    for task in worker1:
        for point in task["Path"]:
            if point in waypoints:
                worker1_goals.append(waypoints[point])
    worker2_goals = []
    for task in worker2:
        for point in task["Path"]:
            if point in waypoints:
                worker2_goals.append(waypoints[point])

    yaml_data = {
        "hunav_loader": {
            "ros__parameters": {
                "map": "map",
                "publish_people": True,
                "agents": ["worker1"],
                "worker1": {
                    "id": 1,
                    "skin": 0,
                    "group_id": -1,
                    "max_vel": 1.5,
                    "radius": 0.4,
                    "behavior": {
                        "type": 1,
                        "configuration": 0,
                        "duration": 40.0,
                        "once": True,
                        "vel": 0.6,
                        "dist": 0.0,
                        "goal_force_factor": 2.0,
                        "obstacle_force_factor": 10.0,
                        "social_force_factor": 5.0,
                        "other_force_factor": 20.0,
                    },
                    "init_pose": {
                        "x": worker1_goals[0]["x"],
                        "y": worker1_goals[0]["y"],
                        "z": 1.250000,
                        "h": 0.0,
                    },
                    "goal_radius": 0.3,
                    "cyclic_goals": True,
                    "goals": [f"g{i}" for i in range(len(worker1_goals))]
                }
            }
        }
    }

    for i, g in enumerate(worker1_goals):
        yaml_data["hunav_loader"]["ros__parameters"]["worker1"][f"g{i}"] = {
            "x": g["x"],
            "y": g["y"],
            "h": 1.250000
        }

    yaml_data["hunav_loader"]["ros__parameters"]["agents"].append("worker2")

    worker2_data = {
        "id": 2,
        "skin": 4,
        "group_id": -1,
        "max_vel": 1.5,
        "radius": 0.4,
        "behavior": {
            "type": 1,
            "configuration": 2,
            "duration": 40.0,
            "once": True,
            "vel": 0.6,
            "dist": 0.0,
            "goal_force_factor": 2.0,
            "obstacle_force_factor": 10.0,
            "social_force_factor": 5.0,
            "other_force_factor": 20.0,
        },
        "init_pose": {
            "x": worker2_goals[0]["x"],
            "y": worker2_goals[0]["y"],
            "z": 1.250000,
            "h": 0.0,
        },
        "goal_radius": 0.3,
        "cyclic_goals": True,
        "goals": [f"g{i}" for i in range(len(worker2_goals))]
    }

    yaml_data["hunav_loader"]["ros__parameters"]["worker2"] = worker2_data

    for i, g in enumerate(worker2_goals):
        yaml_data["hunav_loader"]["ros__parameters"]["worker2"][f"g{i}"] = {
            "x": g["x"],
            "y": g["y"],
            "h": 1.250000
        }

    class HunavDumper(yaml.SafeDumper):
        def represent_bool(self, data):
            return self.represent_scalar('tag:yaml.org,2002:bool', 'true' if data else 'false')

    HunavDumper.add_representer(bool, HunavDumper.represent_bool)

    with open("./src/hunav_sim/hunav_agent_manager/config/test.yaml", "w", encoding="utf-8") as f:
        yaml.dump(yaml_data, f, sort_keys=False, Dumper=HunavDumper)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--req_folder", type=str, default="jerk", help="requirement_folder")
    parser.add_argument("--req", type=str, default="jerk", help="requirement")
    parser.add_argument("--route", type=str, default="route_1", help="route")
    parser.add_argument("--number", type=int, default=0, help="number")
    parser.add_argument("--episode", type=int, default=0, help="episode")
    args = parser.parse_args()
    main(args)