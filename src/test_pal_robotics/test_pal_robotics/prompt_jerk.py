prompt_jerk_1 = """You are a warehouse operations scenario planner. Your role is to create a realistic and practical operational use case based on the provided warehouse layout and the specific requirements described below. Do not include any direct communication or command exchange between humans and the robot. All robot reactions should be triggered by environmental changes only.


Provided Known Information:

1. Warehouse Layout Description (from Image 1):
The first image depicts a warehouse layout divided into four main regions: Region 1 contains Boxes 1–3, Region 2 holds Shelf 1–6, Region 3 includes operational zones Area 1–5 and a Waste Area, and Region 4 contains Large Shelf 1–4. Storage units are categorized as boxes, shelves, and large shelves, with boxes placed on the left for bulk storage, shelves centrally aligned for frequently accessed items, and large shelves along the bottom for bulky or less-used stock. Operational zones in Region 3 are used for sorting, inspection, staging, and dispatch, with Areas 1 and 2 holding items and Areas 3 and 4 empty. The Waste Area is isolated for non-usable materials. A pallet jack is positioned near Area 5 for handling tasks such as receiving, internal transport, and dispatch. Movement pathways run horizontally and vertically, with central corridors providing access across regions. Safety features include hazard markings, while some shelving arrangements and equipment placement present possible movement constraints.

2. Waypoint Navigation Grid (from Image 2):
The second image overlays the same warehouse with a coordinate grid (Rows A–M, Columns 1–22) and a waypoint network where only solid blue dots denote valid navigation points. Region 1 spans A–E, Columns 1–9; Region 2 spans A–E, Columns 10–22; Region 3 occupies F–J, Columns 1–21 and includes Waste Area (F–J, 1–4) and Areas 1–5; Region 4 spans L–M, Columns 1–22. Pathways are defined along Rows E, F, K, and L for horizontal movement and Columns 1, 4, 9, and 22 for vertical transitions, enabling access between zones. Storage zones are bordered by waypoints that support side or front access for picking and placing. Operational zones are surrounded by valid waypoints to allow entry and exit from multiple sides, with each area aligned for specific workflow stages. The pallet jack is located at G–H, Column 22, directly accessible via adjacent waypoints and connected to dispatch and storage areas through primary grid corridors.

3. Robot Navigation Route:
{}


Special Requirements:

The use case you generate must be designed so that the coordination, task sequence, and workflow of the two human workers are likely to cause the robot, which follows a fixed path, to experience a higher average jerk (rate of change of acceleration) during execution.

When designing the operational scenario, you are encouraged to include complex cases in which both human workers may perform similar or overlapping tasks, operate within the same functional area(s), or have frequent task, spatial, and timing crossovers. Consider not only traditional division-of-labor situations, but also real-world high-activity scenarios where workers collaborate, alternate, or independently repeat similar processes in close proximity or shared zones. These scenarios may feature frequent mutual interference, shared resource use, and dynamic spatial congestion.

Note:
- There is no direct communication or command exchange between the human workers and the robot.
- The robot operates completely autonomously and only reacts to changes in the physical environment (e.g., detecting new obstacles), not to any explicit instructions or signals from humans.
- Any influence of the workers on the robot is indirect and arises solely from their activities in the shared workspace, such as moving objects, occupying certain locations, or causing temporary obstructions.


You must:
- Define a realistic operational use case involving activities such as receiving, sorting, inspection, storage, or dispatch.
- Involve only two human workers.
- Clearly assign either distinct or collaborative operational roles and responsibilities to each worker. Workers may perform complementary, alternating, or similar/cooperative tasks if operationally realistic.
- Ensure that worker tasks involve physical movement across multiple or shared functional areas of the warehouse, referencing the provided warehouse zones and layout description.
- Most importantly, design the roles, workflows, and task sequences so that the natural activities of the two workers are likely to cause indirect disruption to the robot’s motion along its fixed route ({}), in a way that the scenario execution leads to a higher average jerk for the robot.
- Explicitly describe how and why worker behaviors or process flows may lead to increased jerk values in the robot's operation, focusing on realistic environmental disruptions such as temporary obstructions, adjacent activity, or object placement that affect the robot's motion along its fixed route ({}).
- Ensure that all worker tasks emerge from legitimate workflow requirements, not intentional interference or contrived positioning.
- Do not include any direct interaction between humans and the robot. All robot responses must be based solely on environmental factors.
- Provide a detailed explanation of why the scenario is operationally realistic and how each worker’s role fits within the warehouse layout and overall process flow.
- Provide a concise natural language scenario summary as a JSON object with the key "scenario" only.
- Strictly following all requirements above, provide a detailed, step-by-step scenario analysis in natural language. Output only the scenario summary in JSON format as the last part of your answer. Do not output the JSON alone, and do not merge the analysis and JSON. Both sections are required and must be clearly separated. If the analysis section is missing, the answer is incomplete.
"""

prompt_jerk_2 = """You are a warehouse operations process designer. Using the warehouse layout and the previously defined operational scenario, your task is to design a high-level cyclic task sequence for each of the two human workers.


Special Requirements:

Your task sequence design should intentionally structure the workers’ movements and workflows in a way that increases the likelihood of the robot experiencing a higher average jerk (rate of change of acceleration) during execution.

There is no direct communication or command exchange between the workers and the robot; the robot operates autonomously and only reacts to environmental changes caused by the workers’ actions in the warehouse.

The goal is to make the workers' task flow indirectly create environmental conditions, such as unexpected obstructions, urgent handoffs, or dynamic space usage, that can lead to more frequent or intense changes in the robot's movement (higher jerk values).


Your instructions must:
- Base the design strictly on the previously defined operational scenario.
- Use only functional area names (e.g., "Boxes 1 → Area 1").
- Ensure that the end location of each task is the start of the next.
- Ensure the final task returns to the worker’s starting location, forming a complete loop.
- Intentionally select and order task steps so that the sequence creates more situations where the robot, as it operates along its fixed path, will encounter environmental changes likely to increase its average jerk.
- Explain your reasoning for the order and selection of each functional area in the task sequence, including how it supports the operational goal and each worker's role, and how it may contribute to a higher jerk scenario for the robot.
"""

prompt_jerk_3 = """You are a warehouse navigation route planner. Based on the high-level task sequences for each worker, your job is to generate detailed waypoint-based movement paths for each step. Follow the special requirements, area valid waypoints, warehouse invalid waypoints, and navigation rules to ensure all routes are valid and efficient.


Special Requirements:

Your path planning should intentionally structure worker movement so that their navigation through the warehouse increases the likelihood that the robot, following its fixed path, will experience a higher average jerk (rate of change of acceleration).

The workers’ movement paths should create more dynamic and unpredictable environmental changes (such as temporary obstructions, path overlaps, or urgent cross-throughs) along the robot's route, but without any direct communication or interaction with the robot itself.

All such influences on the robot must arise solely from the way the workers navigate and use the space.

When designing task sequences or navigation routes, do not restrict valid waypoints to only the central or mid-grid areas. Always consider edge waypoints, like those on rows A and B (e.g., A12, B16, A22), as potential and legitimate start, end, or transit points, especially for any operation that involves shelves or storage zones located at or near the warehouse boundaries.


Area Valid Waypoints:
Define the following valid waypoints for each warehouse area:
- Boxes 1: A1, B1, C1, D1, E1, E2, E3
- Boxes 2: E3, E4, E5, E6
- Boxes 3: E6, E7, E8, D9, C9, B9, A9
- Shelf 1: A9, B9, C9, D9, A12, B12, C12, D12
- Shelf 2: A12, B12, C12, D12, A14, B14, C14, D14
- Shelf 3: A14, B14, C14, D14, A16, B16, C16, D16
- Shelf 4: A16, B16, C16, D16, A18, B18, C18, D18
- Shelf 5: A18, B18, C18, D18, A20, B20, C20, D20
- Shelf 6: A20, B20, C20, D20, A22, B22, C22, D22
- Waste Area: F3, G3
- Area 1: G4, G5, G6, G7, H4, I4, J4
- Area 2: G7, G8, G9, G10
- Area 3: G11, G12, G13, G14, H11, H12, H13, H14, I11, I12, I13, I14, J11, J12, J13, J14
- Area 4: G15, G16, G17, H15, H16, H17, I15, I16, I17, J15, J16, J17
- Area 5: G18, G19, G20, G21, H18, H19, H20, H21
- Pallet Jack: F22, G21, H21, I22
- Large Shelf 1: M2, M3, M4, M5, M6, M7
- Large Shelf 2: M8, M9, M10, M11, M12
- Large Shelf 3: M13, M14, M15, M16, M17
- Large Shelf 4: M18, M19, M20, M21, M22
Use these exact waypoints as possible start, end, or intermediate points for navigation tasks within each respective area.


Warehouse Invalid Waypoints:
Some waypoints are explicitly unavailable due to obstructions, physical layout, or operational constraints. These waypoints must never be used under any circumstances. Ensure that all planned routes avoid the following invalid waypoints:
- Row A: A2, A3, A4, A5, A6, A7, A8, A10, A11, A13, A15, A17, A19, A21
- Row B: B2, B3, B4, B5, B6, B7, B8, B10, B11, B13, B15, B17, B19, B21
- Row C: C2, C3, C4, C5, C6, C7, C8, C10, C11, C13, C15, C17, C19, C21
- Row D: D2, D3, D4, D5, D6, D7, D8, D10, D11, D13, D15, D17, D19, D21
- Row F: F1, F2
- Row G: G1, G2, G22
- Row H: H1, H2, H3, H5, H6, H7, H8, H9, H10, H22
- Row I: I1, I2, I3, I5, I6, I7, I8, I9, I10, I18, I19, I20, I21
- Row J: J1, J2, J3, J5, J6, J7, J8, J9, J10, J18, J19, J20, J21
Always cross-check your route against this list to ensure validity.


Navigation Rules:
- Use only valid navigation points (solid blue dots).
- Each path must start exactly where the previous one ended.
- Include both the start and end points in the path.
- Always avoid the specified invalid waypoints.
- For any task involving a specific area, always use only the waypoints listed in that area's Area Valid Waypoints as possible start, end, or intermediate points.
- Always consider and use edge points as start, end, or intermediate waypoints when they are valid for the area.


For each step in each worker’s task sequence, you must provide:
- Start coordinate (e.g., C9).
- End coordinate (e.g., G6).
- Full waypoint path as an ordered list, including both the start and end points (e.g., [C9, D9, E9, E8, E7, E6, F6, G6]).
- Explanation of why this route was chosen, (if applicable) how edge waypoints are used, how it avoids obstacles and invalid waypoints, and why it is efficient given the warehouse layout.
- Analysis of how this specific route and movement may contribute to the robot experiencing higher average jerk, such as by creating temporary obstacles, requiring the robot to change speed or direction, or increasing unpredictability in the shared workspace.
"""

prompt_jerk_4 = """You are a warehouse route validation specialist. Your task is to carefully review all previously generated waypoint paths for each worker, according to the following requirements:

Check that the start point, end point, and every waypoint in each path are all valid and not listed as blocked or unavailable.

Ensure that all task transitions are continuous: the end point of each step must exactly match the start point of the next step.

Confirm that each worker’s full path forms a closed loop, returning to their original starting point.

Verify that all routes respect the physical layout and functional roles: for example, shelves are accessed from designated access points, and blocked or restricted zones are avoided.

When a task step involves a specific area, any waypoints used within that area must be selected only from that area's Area Valid Waypoints list:
- Boxes 1: A1, B1, C1, D1, E1, E2, E3
- Boxes 2: E3, E4, E5, E6
- Boxes 3: E6, E7, E8, D9, C9, B9, A9
- Shelf 1: A9, B9, C9, D9, A12, B12, C12, D12
- Shelf 2: A12, B12, C12, D12, A14, B14, C14, D14
- Shelf 3: A14, B14, C14, D14, A16, B16, C16, D16
- Shelf 4: A16, B16, C16, D16, A18, B18, C18, D18
- Shelf 5: A18, B18, C18, D18, A20, B20, C20, D20
- Shelf 6: A20, B20, C20, D20, A22, B22, C22, D22
- Waste Area: F3, G3
- Area 1: G4, G5, G6, G7, H4, I4, J4
- Area 2: G7, G8, G9, G10
- Area 3: G11, G12, G13, G14, H11, H12, H13, H14, I11, I12, I13, I14, J11, J12, J13, J14
- Area 4: G15, G16, G17, H15, H16, H17, I15, I16, I17, J15, J16, J17
- Area 5: G18, G19, G20, G21, H18, H19, H20, H21
- Pallet Jack: F22, G21, H21, I22
- Large Shelf 1: M2, M3, M4, M5, M6, M7
- Large Shelf 2: M8, M9, M10, M11, M12
- Large Shelf 3: M13, M14, M15, M16, M17
- Large Shelf 4: M18, M19, M20, M21, M22

All planned routes must avoid the following invalid waypoints at all times:
- Row A: A2, A3, A4, A5, A6, A7, A8, A10, A11, A13, A15, A17, A19, A21
- Row B: B2, B3, B4, B5, B6, B7, B8, B10, B11, B13, B15, B17, B19, B21
- Row C: C2, C3, C4, C5, C6, C7, C8, C10, C11, C13, C15, C17, C19, C21
- Row D: D2, D3, D4, D5, D6, D7, D8, D10, D11, D13, D15, D17, D19, D21
- Row F: F1, F2
- Row G: G1, G2, G22
- Row H: H1, H2, H3, H5, H6, H7, H8, H9, H10, H22
- Row I: I1, I2, I3, I5, I6, I7, I8, I9, I10, I18, I19, I20, I21
- Row J: J1, J2, J3, J5, J6, J7, J8, J9, J10, J18, J19, J20, J21


For every path segment, validate the following in order:
- The start and end coordinates are valid and permitted.
- Every waypoint in the path is valid and not listed as invalid.
- When a task step involves a specific area, all in-area waypoints must be selected only from that area's Area Valid Waypoints list.
- The path is continuous, with no gaps or jumps between steps.
- The full sequence forms a closed loop, returning to the starting point.

For each path segment, list all validation errors you find (such as invalid waypoints, discontinuities, or area-valid-waypoint issues), and then propose a revised, valid route that preserves the intended task flow logic.

Do not stop at the first error. Check the entire segment thoroughly before suggesting corrections. After completing the validation and correction for the current segment, move on to the next segment and repeat the process.
"""

prompt_jerk_5 = """You are a warehouse worker route and task reporter. Based on the defined task and navigation sequences for both workers, your job is to output the entire process in the following strict JSON structure:

```json
{{
  "Worker 1 Tasks": [
    {{
      "Task": "<From Zone> → <To Zone>",
      "Start": "<Start Coordinate>",
      "End": "<End Coordinate>",
      "Path": ["<Waypoint List>"]
    }}
  ],
  "Worker 2 Tasks": [
    {{
      "Task": "<From Zone> → <To Zone>",
      "Start": "<Start Coordinate>",
      "End": "<End Coordinate>",
      "Path": ["<Waypoint List>"]
    }}
  ]
}}
```


You must:

Output only valid JSON, which must be syntactically correct and fully parseable.

Enclose all string values in double quotes.

Format every list using square brackets, with correct comma placement.

For each task, include:
- The zone transition in the "Task" field (e.g., "Boxes 1 → Area 1").
- "Start" and "End" coordinates (e.g., "C9").
- The full list of waypoints in the "Path" array (each as a string, e.g., ["C9", "D9", "E9", "F9"]).

Do not include any explanatory text or comments outside the JSON structure.
"""

prompt_jerk_6 = """You are a warehouse operations optimization analyst. Your primary goal is to redesign or adjust the operational scenario so that the average robot_jerk value is as high as possible. All your analysis and proposals should focus on maximizing the mean robot_jerk, based on feedback data from a simulator run.

You are provided with feedback data in JSON format. Each entry contains the following fields:
- timestamp: Simulation time
- robot_wp: Robot position (waypoint)
- worker1_wp: Worker 1 position (waypoint)
- worker2_wp: Worker 2 position (waypoint)
- robot_jerk: The rate of change of the robot’s acceleration between the current and previous timestamps

The current scenario produces an average robot_jerk value of **{}**. Your target is to design operational changes that will raise this average as much as possible, while keeping the overall scenario logic and operational goals unchanged.

Feedback Data:
```json
{}
```


Your tasks:

Analyze the feedback data to identify time intervals, spatial arrangements, or task moments when the robot’s jerk is highest.

Based on these insights, propose adjustments to the operational scenario (if possible) that would likely further increase the robot’s average jerk, such as:
- Modifying the sequence, timing, or overlap of worker tasks
- Adjusting worker routes to create more dynamic or unpredictable environments for the robot
- Introducing more situations where workers indirectly create temporary obstacles or space conflicts along the robot's fixed path

If possible, provide both a revised high-level and low-level route for each worker:
High-level route: For each worker, list the full, step-by-step sequence of functional areas visited (e.g., "Boxes 1 → Area 1 → Shelf 2 → Boxes 1"), ensuring the sequence forms a complete loop.
Low-level route: For each step in each worker’s task sequence, provide all of the following:
- Start coordinate (e.g., C9)
- End coordinate (e.g., G6)
- Full waypoint path as an ordered list, including both the start and end points (e.g., [C9, D9, E9, E8, E7, E6, F6, G6])
Thoroughly include every step in both the high-level area sequence and the corresponding detailed waypoint path for each worker, ensuring that the route forms a closed loop.
Additional notes for low-level route: All waypoints in each path must be valid navigation points. For any step involving a specific area, only use the waypoints listed in that area's Area Valid Waypoints, including any listed edge waypoints, as start, end, or intermediate points. Edge waypoints are valid and should be used when appropriate.

If the scenario is already near-optimal for provoking high robot jerk, explain why no significant further improvement can be made without fundamentally changing the scenario.

Note:
When optimizing, your primary goal is to maximize the mean robot_jerk value, even if this causes some moments where the jerk increases sharply.
Do not propose changes that sacrifice the scenario’s operational validity, but any minor adjustment is acceptable if it increases the mean jerk.


You must provide:

Analysis of Robot Jerk Patterns: Identification of when and where the robot jerk is highest, along with the underlying causes of the observed peaks.

Proposed Optimizations: Recommendations for changes in worker task assignment, timing, or routing that could further increase robot jerk.

Revised High-Level Worker Task Sequences: A complete area-by-area loop for each worker, reflecting the revised task sequence.

Revised Low-Level Worker Routes: Detailed start and end coordinates, as well as full waypoint paths as ordered lists for each step in each worker’s high-level sequence, ensuring all paths form closed loops. Only use valid waypoints for each area, including edge waypoints when they are available.

Scenario Justification: Explanation of how the proposed changes are expected to increase robot jerk, along with justification for maintaining scenario integrity.
"""

prompt_jerk_memory_1 = """You are a warehouse operations scenario planner. Your role is to create a realistic and practical operational use case based on the provided warehouse layout and the specific requirements described below. Do not include any direct communication or command exchange between humans and the robot. All robot reactions should be triggered by environmental changes only.


Provided Known Information:

1. Warehouse Layout Description (from Image 1):
The first image depicts a warehouse layout divided into four main regions: Region 1 contains Boxes 1–3, Region 2 holds Shelf 1–6, Region 3 includes operational zones Area 1–5 and a Waste Area, and Region 4 contains Large Shelf 1–4. Storage units are categorized as boxes, shelves, and large shelves, with boxes placed on the left for bulk storage, shelves centrally aligned for frequently accessed items, and large shelves along the bottom for bulky or less-used stock. Operational zones in Region 3 are used for sorting, inspection, staging, and dispatch, with Areas 1 and 2 holding items and Areas 3 and 4 empty. The Waste Area is isolated for non-usable materials. A pallet jack is positioned near Area 5 for handling tasks such as receiving, internal transport, and dispatch. Movement pathways run horizontally and vertically, with central corridors providing access across regions. Safety features include hazard markings, while some shelving arrangements and equipment placement present possible movement constraints.

2. Waypoint Navigation Grid (from Image 2):
The second image overlays the same warehouse with a coordinate grid (Rows A–M, Columns 1–22) and a waypoint network where only solid blue dots denote valid navigation points. Region 1 spans A–E, Columns 1–9; Region 2 spans A–E, Columns 10–22; Region 3 occupies F–J, Columns 1–21 and includes Waste Area (F–J, 1–4) and Areas 1–5; Region 4 spans L–M, Columns 1–22. Pathways are defined along Rows E, F, K, and L for horizontal movement and Columns 1, 4, 9, and 22 for vertical transitions, enabling access between zones. Storage zones are bordered by waypoints that support side or front access for picking and placing. Operational zones are surrounded by valid waypoints to allow entry and exit from multiple sides, with each area aligned for specific workflow stages. The pallet jack is located at G–H, Column 22, directly accessible via adjacent waypoints and connected to dispatch and storage areas through primary grid corridors.

3. Robot Navigation Route:
{}


Special Requirements:

The use case you generate must be designed so that the coordination, task sequence, and workflow of the two human workers are likely to cause the robot, which follows a fixed path, to experience a higher average jerk (rate of change of acceleration) during execution.

When designing the operational scenario, you are encouraged to include complex cases in which both human workers may perform similar or overlapping tasks, operate within the same functional area(s), or have frequent task, spatial, and timing crossovers. Consider not only traditional division-of-labor situations, but also real-world high-activity scenarios where workers collaborate, alternate, or independently repeat similar processes in close proximity or shared zones. These scenarios may feature frequent mutual interference, shared resource use, and dynamic spatial congestion.

Note:
- There is no direct communication or command exchange between the human workers and the robot.
- The robot operates completely autonomously and only reacts to changes in the physical environment (e.g., detecting new obstacles), not to any explicit instructions or signals from humans.
- Any influence of the workers on the robot is indirect and arises solely from their activities in the shared workspace, such as moving objects, occupying certain locations, or causing temporary obstructions.


Important Instruction:
One or more previous scenarios will be provided below, formatted in JSON. These are provided for reference only to help ensure that your new scenario is fundamentally different from any previous ones. You do not need to use this format for your new scenario.

```json
{}
```

Carefully analyze all provided previous scenarios. The new scenario you generate must be fundamentally different from all previous ones, not only in surface description but also in the operational flow, workers’ activity sequences, workspace overlap patterns, functional area usage, item movement paths, and the indirect mechanisms by which workers impact the robot's jerk.

Do not reuse or create variants of any prior scenario’s worker task allocation, main routes, or typical process flow. Ensure the scenario you produce is novel, distinct, and substantially different in both operational details and robot-environment interaction patterns.


You must:
- Define a realistic operational use case involving activities such as receiving, sorting, inspection, storage, or dispatch.
- Ensure your scenario is fundamentally different from all previous cases, not reusing or varying any prior example.
- Involve only two human workers.
- Clearly assign either distinct or collaborative operational roles and responsibilities to each worker. Workers may perform complementary, alternating, or similar/cooperative tasks if operationally realistic.
- Ensure that worker tasks involve physical movement across multiple or shared functional areas of the warehouse, referencing the provided warehouse zones and layout description.
- Most importantly, design the roles, workflows, and task sequences so that the natural activities of the two workers are likely to cause indirect disruption to the robot’s motion along its fixed route ({}), in a way that the scenario execution leads to a higher average jerk for the robot.
- Explicitly describe how and why worker behaviors or process flows may lead to increased jerk values in the robot's operation, focusing on realistic environmental disruptions such as temporary obstructions, adjacent activity, or object placement that affect the robot's motion along its fixed route ({}).
- Ensure that all worker tasks emerge from legitimate workflow requirements, not intentional interference or contrived positioning.
- Do not include any direct interaction between humans and the robot. All robot responses must be based solely on environmental factors.
- Provide a detailed explanation of why the scenario is operationally realistic and how each worker’s role fits within the warehouse layout and overall process flow.
- Provide a concise natural language scenario summary as a JSON object with the key "scenario" only.
- Strictly following all requirements above, provide a detailed, step-by-step scenario analysis in natural language. Output only the scenario summary in JSON format as the last part of your answer. Do not output the JSON alone, and do not merge the analysis and JSON. Both sections are required and must be clearly separated. If the analysis section is missing, the answer is incomplete.
"""
