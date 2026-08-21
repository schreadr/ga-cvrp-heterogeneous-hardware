import math
import sys
import os

def get_scaled_coordinates(coords):
    """
    Finds the smallest power of 10 scaling factor such that all nodes have unique
    integer positions after scaling, then returns the scaled coordinate list.
    """
    for p in range(6):
        scale = 10**p
        scaled = []
        for node_id, x, y in coords:
            sx = int(round(x * scale))
            sy = int(round(y * scale))
            scaled.append((node_id, sx, sy))
        locations = set((sx, sy) for _, sx, sy in scaled)
        if len(locations) == len(coords):
            return scale, scaled
    return 1, [(node_id, int(round(x)), int(round(y))) for node_id, x, y in coords]

def parse_vrp_file(filepath):
    """
    Parses CVRP .vrp or .txt benchmark files, extracting dimension, capacity,
    node coordinates, and city demands.
    """
    if not os.path.exists(filepath):
        print(f"Error: File '{filepath}' does not exist.")
        sys.exit(1)
        
    with open(filepath, 'r') as f:
        lines = f.readlines()
        
    num_cities = 0
    vehicle_capacity = 0
    depot_x, depot_y = 0.0, 0.0
    coords = []
    demands = []
    
    state = None
    for line in lines:
        line = line.strip()
        if not line:
            continue
        # Parameter headers
        if line.startswith("DIMENSION"):
            num_cities = int(line.split(":")[-1].strip())
        elif line.startswith("CAPACITY"):
            vehicle_capacity = int(line.split(":")[-1].strip())
        elif line.startswith("NODE_COORD_SECTION"):
            state = "COORDS"
            continue
        elif line.startswith("DEMAND_SECTION"):
            state = "DEMANDS"
            continue
        elif line.startswith("DEPOT_SECTION") or line.startswith("EOF") or line == "-1":
            state = None
            continue
            
        if state == "COORDS":
            parts = line.split()
            if len(parts) >= 3:
                node_id = int(parts[0])
                x = float(parts[1])
                y = float(parts[2])
                coords.append((node_id, x, y))
                if node_id == 1:
                    depot_x, depot_y = x, y
        elif state == "DEMANDS":
            parts = line.split()
            if len(parts) >= 2:
                node_id = int(parts[0])
                demand = int(parts[1])
                demands.append((node_id, demand))
                
    return num_cities, vehicle_capacity, depot_x, depot_y, coords, demands

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 calc-params.py <path_to_problem_file>")
        sys.exit(1)
        
    filepath = sys.argv[1]
    num_cities, vehicle_capacity, depot_x, depot_y, coords, demands = parse_vrp_file(filepath)
    
    # 1. Coordinate Shifting (moves negative coordinates to the first quadrant)
    min_x = min(x for _, x, y in coords)
    min_y = min(y for _, x, y in coords)
    
    x_offset = abs(min_x) if min_x < 0 else 0.0
    y_offset = abs(min_y) if min_y < 0 else 0.0
    
    shifted_coords = [(node_id, x + x_offset, y + y_offset) for node_id, x, y in coords]
    
    # 2. Coordinate Scaling (converts decimal coordinates to integers uniquely)
    scale, scaled_coords = get_scaled_coordinates(shifted_coords)
    
    # 3. Maximum coordinate values and bit lengths
    max_x = max(sx for _, sx, _ in scaled_coords)
    max_y = max(sy for _, _, sy in scaled_coords)
    max_coord = max(max_x, max_y)
    
    lpCoordBitWidth = max_coord.bit_length()
    lpIDBitWidth = 2 * lpCoordBitWidth
    
    # 4. Depot coordinate values after shifting and scaling
    depot_node = [sc for sc in scaled_coords if sc[0] == 1][0]
    lpDepotXCoord = depot_node[1]
    lpDepotYCoord = depot_node[2]
    
    # 5. Capacity bit width (bits needed to represent maximum customer demand)
    lpCapBitWidth = vehicle_capacity.bit_length()
    
    # 6. Maximum possible distance bit width (lpDistMaxBitWidth)
    # Calculated as the upper bound of 2 * NumCities * max_coordinate_diagonal
    max_edge = math.sqrt(max_x**2 + max_y**2)
    max_possible_fitness = int(2 * num_cities * max_edge)
    lpDistMaxBitWidth = max_possible_fitness.bit_length()
    
    # 7. Genetic Algorithm hyperparameter heuristics
    lpRandomSubtourOffset = num_cities // 3
    lpMutationDistance = max(4, num_cities // 3)
    lpInitMutationProcesses = 25  # Standard thesis default for initiation mutation loops
    lpCityNumBitWidth = num_cities.bit_length()
    
    # 8. Memory Layout and Word Count (lpParentWordCount)
    # Evaluates how to pack city nodes (ID + Capacity + Depot_Bit) into 72-bit BRAM lines
    bits_per_node = lpIDBitWidth + lpCapBitWidth + 1
    lpMemLineEntryCount = 72 // bits_per_node
    
    num_customers = num_cities - 1
    base_lines = math.ceil(num_customers / lpMemLineEntryCount)
    rem = num_customers % lpMemLineEntryCount
    unused_bits = 72 - (rem * bits_per_node) if rem != 0 else 0
    
    # If the unused bits in the last line can accommodate the fitness value, 
    # we pack it there. Otherwise, we allocate 1 extra line (word) for the individual.
    if unused_bits >= lpDistMaxBitWidth:
        lpParentWordCount = base_lines
    else:
        lpParentWordCount = base_lines + 1
        
    # Output generated Verilog parameters
    print(f"// ==========================================================================")
    print(f"// Verilog Localparameters generated for CVRP instance: {os.path.basename(filepath)}")
    print(f"// Generated using calc-params.py")
    print(f"// ==========================================================================")
    print(f"localparam int lpNumCities = {num_cities};")
    print(f"localparam int lpCapBitWidth = {lpCapBitWidth};")
    print(f"localparam int lpIDBitWidth = {lpIDBitWidth};")
    print(f"localparam int lpCoordBitWidth = {lpCoordBitWidth};")
    print(f"localparam int lpDistMaxBitWidth = {lpDistMaxBitWidth};")
    print(f"localparam int lpVehicleCapacity = {vehicle_capacity};")
    print(f"localparam int lpRandomSubtourOffset = {lpRandomSubtourOffset};")
    print(f"localparam int lpMutationDistance = {lpMutationDistance};")
    print(f"localparam int lpInitMutationProcesses = {lpInitMutationProcesses};")
    print(f"localparam int lpDepotXCoord = {lpDepotXCoord};")
    print(f"localparam int lpDepotYCoord = {lpDepotYCoord};")
    print(f"localparam int lpCityNumBitWidth = {lpCityNumBitWidth};")
    print(f"localparam int lpMemLineEntryCount = 72 / ({lpIDBitWidth} + {lpCapBitWidth} + 1); // Evaluated: {lpMemLineEntryCount}")
    print(f"localparam int lpParentWordCount = {lpParentWordCount};")

if __name__ == "__main__":
    main()
