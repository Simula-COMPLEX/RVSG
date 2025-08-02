prompt_route_1 = """The robot starts near grid point G12 in Area 3, and travels rightward from row G to row E, moving from column 12 to column 21, and ends around E21 near Shelf 6. The approximate waypoints it passes through are: G12 → F12 → F13 → F14 → F15 → F16 → F17 → F18 → E18 → E19 → E20 → E21. This route suggests the robot is likely performing a material transport task, picking up an item from Area 3 and delivering it to Shelf 6 for storage or replenishment."""

prompt_route_1_start_end = "from G12 to E21"

prompt_route_2 = """The robot starts near grid point M21, close to Large Shelf 4, and travels leftward from row M to row E, moving from column 21 to column 14, and ends around E14 near Shelf 2 and Shelf 3. The approximate waypoints it passes through are: M21 → M20 → L20 → L19 → K18 → K17 → J17 → I17 → I16 → H16 → H15 → G15 → F15 → F14 → E14. This route suggests the robot is likely performing a material transport task, picking up an item from Large Shelf 4 and delivering it to Shelf 2 or Shelf 3 for storage or replenishment."""

prompt_route_2_start_end = "from M21 to E14"

prompt_route_3 = """The robot starts near grid point A9, close to Shelf 1, and travels leftward from row A to row G, moving from column 9 to column 4, and ends around G4 in Area 1. The approximate waypoints it passes through are: A9 → B9 → C9 → D9 → E9 → E8 → E7 → F7 → F6 → F5 → G4. This route suggests the robot is likely performing a material relocation task, picking up an item from Shelf 1 and delivering it to Area 1 for sorting, inspection, or further processing."""

prompt_route_3_start_end = "from A9 to G4"

prompt_route_4 = """The robot starts near grid point M5, close to Large Shelf 1, and travels rightward from row M to row H, moving from column 5 to column 17, and ends around H17 in Area 4. The approximate waypoints it passes through are: M5 → M6 → L6 → L7 → L8 → L9 → K9 → K10 → K11 → K12 → J13 → J14 → I14 → I15 → I16 → H16 → H17. This route suggests the robot is likely performing a material relocation task, picking up an item from Large Shelf 1 and delivering it to Area 4 for sorting, inspection, or further processing."""

prompt_route_4_start_end = "from M5 to H17"

prompt_route_5 = """The robot starts near grid point G19 in Area 5, and travels leftward along row G, moving from column 19 to column 3, and ends around G3 in Waste Area. The approximate waypoints it passes through are: G19 → G18 → G17 → G16 → G15 → G14 → G13 → G12 → G11 → G10 → G9 → G8 → G7 → G6 → G5 → G4 → G3. This route suggests the robot is likely performing a disposal task, picking up an item from Area 5 and delivering it to the Waste Area for discarding or recycling."""

prompt_route_5_start_end = "from G19 to G3"
