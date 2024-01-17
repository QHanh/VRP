from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import getdata

def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data['distance_matrix'] = getdata.distance_matrix
    data['time_matrix'] = getdata.time_matrix
    data['time_windows'] = getdata.time_windows
    data["demands"] = getdata.demands
    data["num_vehicles"] = getdata.num_vehicles
    data["vehicle_capacities"] = getdata.vehicle_capacities
    data["starts"] = getdata.starts
    data["ends"] = getdata.ends
    assert data['num_vehicles'] == len(data['vehicle_capacities'])
    assert len(data['time_matrix']) == len(data['distance_matrix'])
    assert len(data['time_matrix']) == len(data['time_windows'])
    assert len(data['time_matrix']) == len(data['demands'])
    return data


def print_solution(manager, routing, solution):
    """Prints solution on console."""
    print(f'Objective: {solution.ObjectiveValue()}')
    # Display dropped nodes.
    dropped_nodes = 'Dropped nodes:'
    for node in range(routing.Size()):
        if routing.IsStart(node) or routing.IsEnd(node):
            continue
        if solution.Value(routing.NextVar(node)) == node:
            dropped_nodes += ' {}'.format(manager.IndexToNode(node))
    print(dropped_nodes)
    # Print routes
    time_dimension = routing.GetDimensionOrDie('Time')
    distance_dimension = routing.GetDimensionOrDie('Distance')
    capacity_dimension = routing.GetDimensionOrDie('Capacity')
    total_time = 0
    total_distance = 0
    total_load=0
    for vehicle_id in range(manager.GetNumberOfVehicles()):
        index = routing.Start(vehicle_id)
        plan_output = f'Route for vehicle {vehicle_id}:\n'
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            time_var = time_dimension.CumulVar(index)
            distance_var = distance_dimension.CumulVar(index)
            capacity_var = capacity_dimension.CumulVar(index)
            plan_output += '{0} Time({1},{2}) Distance:{3} Load:{4}  -> '.format(
                node_index,
                solution.Min(time_var), solution.Max(time_var),
                solution.Value(distance_var),
                solution.Value(capacity_var))
            index = solution.Value(routing.NextVar(index))
        node_index = manager.IndexToNode(index)
        time_var = time_dimension.CumulVar(index)
        distance_var = distance_dimension.CumulVar(index)
        capacity_var = capacity_dimension.CumulVar(index)
        plan_output += '{0} Time({1},{2}) Distance:{3} Load:{4})\n'.format(
            manager.IndexToNode(index),
            solution.Min(time_var), solution.Max(time_var),
            solution.Value(distance_var),
            solution.Value(capacity_var))
        plan_output += 'Time of the route: {}sec\n'.format(solution.Min(time_var))
        plan_output += 'Distance of the route: {}m\n'.format(solution.Value(distance_var))
        plan_output += 'Load of the route: {}\n'.format(solution.Value(capacity_var))
        print(plan_output)
        total_time += solution.Min(time_var)
        total_distance += solution.Value(distance_var)
        total_load += solution.Value(capacity_var)
    print('Total time of all routes: {}sec'.format(total_time))
    print('Total distance of all routes: {}m'.format(total_distance))
    print('Total load of all routes: {}'.format(total_load))


def main():
    """Solve the VRP with time windows."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
            len(data['distance_matrix']),
            data['num_vehicles'],
            data["starts"], 
            data["ends"])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Allow to drop nodes.
    penalty = 1000000
    for node in range(1, len(data['distance_matrix'])):
        routing.AddDisjunction([manager.NodeToIndex(node)], penalty)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    distance_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(distance_callback_index)

    # Add Distance constraint.
    dimension_name = 'Distance'
    routing.AddDimension(
        distance_callback_index,
        0,  # no slack
        1000000,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')

    # Add Time Windows constraint.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]

    time_callback_index = routing.RegisterTransitCallback(time_callback)

    time = 'Time'
    routing.AddDimension(
        time_callback_index,
        1000,  # allow waiting time
        1000000,  # maximum time per vehicle
        False,  # Don't force start cumul to zero.
        time)
    time_dimension = routing.GetDimensionOrDie(time)

    # Add time window constraints for each location except hub.
    for location_idx, time_window in enumerate(data['time_windows']):
        if location_idx == 0:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
    
    # Add time window constraints for each vehicle start node.
    depot_idx = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(
            data['time_windows'][depot_idx][0],
            data['time_windows'][depot_idx][1])

    # Instantiate route start and end times to produce feasible times.
    for i in range(data['num_vehicles']):
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.Start(i)))
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.End(i)))

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    #search_parameters.log_search = True
    search_parameters.time_limit.FromSeconds(5)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(manager, routing, solution)
    else:
        print('no solution found')

if __name__ == '__main__':
    main()