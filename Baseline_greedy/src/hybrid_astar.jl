#Load Required Packages
#=
Added exp_details Parameter: The hybrid A* search now takes the experiment details containing target belief information

MAP Goal Integration:

Retrieve current MAP goal using get_map_target(exp_details)

Updated all goal references to use dynamic MAP estimate instead of fixed vehicle_params.goal

Modified heuristic calculation and goal checking to use current belief state
=#
using DataStructures
using Random
using LinearAlgebra
include("utils.jl")

struct NodeBin
    discrete_x::Float64
    discrete_y::Float64
    discrete_theta::Float64
end

struct GraphNode
    x::Float64
    y::Float64
    theta::Float64
    actual_cost::Float64
    heuristic_cost::Float64
    action_taken_to_reach_here::Float64
    discrete_x::Float64
    discrete_y::Float64
    discrete_theta::Float64
    parent::Union{GraphNode,Nothing}
    time_stamp::Float64
end

function calculate_heuristic_cost(node_x, node_y, node_theta, goal_x, goal_y, world)
    euclidean_distance =  ( (node_x - goal_x)^2 + (node_y - goal_y)^2 )^ 0.5
    direct_line_to_goal_slope = wrap_between_0_and_2Pi(atan(goal_y-node_y,goal_x-node_x))
    orientation_cost = 10* dot( (cos(direct_line_to_goal_slope), sin(direct_line_to_goal_slope)) ,
                                (cos(node_theta), sin(node_theta)) )
    return euclidean_distance - orientation_cost
end

function get_path(current_node::GraphNode)
    controls_sequence = Float64[]
    # @show(current_node.actual_cost)
    while(current_node.parent!=nothing)
        # println(current_node.x," ",current_node.y," ",current_node.theta," ",current_node.action_taken_to_reach_here)
        push!(controls_sequence, current_node.action_taken_to_reach_here)
        current_node = current_node.parent
    end
    # println(current_node.x," ",current_node.y," ",current_node.theta," ",current_node.action_taken_to_reach_here)
    return reverse(controls_sequence)
end

function get_vehicle_actions(max_delta_angle, min_delta_angle_difference)
    set_of_delta_angles = Float64[0.0]
    half_num_actions = Int( floor(max_delta_angle/min_delta_angle_difference) )
    for i in 1:half_num_actions
        push!(set_of_delta_angles, float(-min_delta_angle_difference*i*pi/180))
        push!(set_of_delta_angles, float(min_delta_angle_difference*i*pi/180))
    end
    return set_of_delta_angles
end

function get_vehicle_steer_actions(max_steering_angle, num_actions) 
    half_num_actions = Int((num_actions-1)/2)
    set_of_steer_angles = collect(-max_steering_angle:max_steering_angle/half_num_actions:max_steering_angle)
    return set_of_steer_angles
end

function get_new_vehicle_position(vehicle_x,vehicle_y,vehicle_theta,vehicle_wheelbase,
                                steering_angle,vehicle_speed,time_duration)
    new_x,new_y,new_theta = move_vehicle(vehicle_x,vehicle_y,vehicle_theta,vehicle_wheelbase,
                            steering_angle,vehicle_speed,time_duration)
    return (new_x,new_y,new_theta)
end

function get_discrete_state(x,y,theta,world,bin_width)
    max_num_bins_x = ceil(world.length/bin_width)
    discrete_x = clamp(ceil(x/bin_width),1.0,max_num_bins_x)
    max_num_bins_y = ceil(world.breadth/bin_width)
    discrete_y = clamp(ceil(y/bin_width),1.0,max_num_bins_y)
    discrete_theta = ceil(theta*180/pi)
    return discrete_x,discrete_y,discrete_theta
end

function predict_human_state(human,world,time_step)
    noise = 0.0
    new_human_state = update_human_position(human,world.length,world.breadth,time_step,noise)
    return new_human_state
end

function get_action_cost(world,nearby_humans,vehicle_x,vehicle_y,vehicle_L,action,time_stamp,planning_details)

    total_cost = 0.0

    #Cost from going out of bounds
    if(vehicle_x>world.length-vehicle_L || vehicle_y>world.breadth-vehicle_L || 
                        vehicle_x<0.0+vehicle_L || vehicle_y<0.0+vehicle_L)
        return Inf
    end

    #Cost from collision with obstacles
    for obstacle in world.obstacles
        # println(obstacle, " ", vehicle_x," ",vehicle_y," ",in_obstacle(vehicle_x,vehicle_y,obstacle,vehicle_L))
        if(in_obstacle(vehicle_x,vehicle_y,obstacle,vehicle_L))
            # println("OBS")
            return Inf
        else
            continue
        end
    end

    #Cost from potential collision with nearby humans
    num_nearby_humans = length(nearby_humans.position_data)
    for human_index in 1:num_nearby_humans
        # human_dist_threshold = 2.0
        human = nearby_humans.position_data[human_index]
        belief = nearby_humans.belief[human_index].pdf
        maximum_element, maximum_element_index = find_maximum_element(nearby_humans.belief[human_index].pdf,planning_details.num_human_goals)
        if(maximum_element < 0.5)
            euclidean_distance = sqrt( (vehicle_x - human.x)^2 + (vehicle_y - human.y)^2)
            if(euclidean_distance >= planning_details.radius_around_uncertain_human)
                continue
            elseif(euclidean_distance <= planning_details.min_safe_distance_from_human)
                return Inf
            else
                total_cost += planning_details.human_collision_cost * (1/euclidean_distance)
                #continue
            end
        else
            inferred_human_goal = planning_details.human_goals[maximum_element_index]
            inferred_human_state = HumanState(human.x,human.y,human.v,inferred_human_goal)
            predicted_human_position = predict_human_state(inferred_human_state,world,time_stamp)
            euclidean_distance::Float64 = sqrt( (vehicle_x - predicted_human_position.x)^2 + (vehicle_y - predicted_human_position.y)^2 )
            if(euclidean_distance >= planning_details.radius_around_uncertain_human)
                continue
            elseif(euclidean_distance <= planning_details.min_safe_distance_from_human)
                return Inf
            else
                #total_cost += pedestrian_cost * (1/distance_of_cart_from_expected_human_position)
                continue
            end
        end
    end

    #Cost from no change in steering angle
    if(action == 0.0)
       total_cost += -1.0
    end

    #Cost from Long Paths
    total_cost += 1

    return total_cost
end

function hybrid_astar_search(world, vehicle, vehicle_params, vehicle_actions, nearby_humans, planning_details, exp_details)
    # Get current MAP goal from belief
    map_goal = get_map_target(exp_details)

    # Create modified parameters with MAP goal
    modified_params = VehicleParametersLSPlanner(
        vehicle_params.wheelbase,
        vehicle_params.length,
        vehicle_params.breadth,
        vehicle_params.dist_origin_to_center,
        vehicle_params.radius,
        vehicle_params.max_speed,
        vehicle_params.max_steering_angle,
        map_goal,  # Use MAP goal instead of fixed goal
        vehicle_params.controls_sequence
    )
    
    # Print for debugging
    println("Planning path to MAP goal: ($(map_goal.x), $(map_goal.y))")
    println("Current belief: $(exp_details.target_belief.pdf)")

    # Action Set comprises of delta_heading_angles
    set_of_delta_angles = vehicle_actions
    # Variable to keep track of path length
    time_stamp = 0.0
    discretization_bin_width = planning_details.veh_path_planning_v*planning_details.one_time_step
    (;dist_origin_to_center) = vehicle_params

    path = Array{Float64,1}()
    open = PriorityQueue{NodeBin,Float64}(Base.Order.Forward)
    closed = Dict{NodeBin,GraphNode}()
    dict_of_nodes = Dict{NodeBin,GraphNode}()
    start_theta = wrap_between_0_and_2Pi(vehicle.theta)
    start_discrete_x,start_discrete_y,start_discrete_theta = get_discrete_state(vehicle.x,vehicle.y,vehicle.theta,world,discretization_bin_width)

    # Calculate initial heuristic using MAP goal
    start_heuristic = calculate_heuristic_cost(vehicle.x, vehicle.y, vehicle.theta, 
    modified_params.goal.x, modified_params.goal.y, world)

    # FIXED: Use map_goal instead of vehicle_params.goal
    start_node = GraphNode(vehicle.x,vehicle.y,vehicle.theta,0.0,
                start_heuristic,  # Use the heuristic calculated with map_goal
                -100.0,start_discrete_x,start_discrete_y,start_discrete_theta,nothing,time_stamp)

    node_key = NodeBin(start_node.discrete_x,start_node.discrete_y,start_node.discrete_theta)
    open[node_key] = (planning_details.lambda^start_node.time_stamp)*start_node.actual_cost + start_node.heuristic_cost
    dict_of_nodes[node_key] = start_node
    start_time = time()

    while(length(open) != 0)
        current_node = dict_of_nodes[dequeue!(open)]
        vehicle_center_x = current_node.x  + dist_origin_to_center*cos(current_node.theta)
        vehicle_center_y = current_node.y + dist_origin_to_center*sin(current_node.theta)
        
        # Check against current MAP goal
        if(is_within_range(vehicle_center_x,vehicle_center_y,modified_params.goal.x,modified_params.goal.y,
                            planning_details.radius_around_vehicle_goal))
            println("Time taken to find the Hybrid A* path: ", time()- start_time)
            return get_path(current_node)
        end

        node_key = NodeBin(current_node.discrete_x,current_node.discrete_y,current_node.discrete_theta)
        closed[node_key] = current_node

        for steering_angle in vehicle_actions
            new_x,new_y,new_theta = get_new_vehicle_position(current_node.x,current_node.y,current_node.theta,
                            vehicle_params.wheelbase,steering_angle,planning_details.veh_path_planning_v,planning_details.one_time_step)
            discrete_x,discrete_y,discrete_theta = get_discrete_state(new_x,new_y,new_theta,world,discretization_bin_width)
            node_key = NodeBin(discrete_x,discrete_y,discrete_theta)
            
            if(haskey(closed,node_key))
                continue
            end
            
            # Calculate actual cost of the new node
            future_cost = get_action_cost(world,nearby_humans,new_x,new_y,vehicle_params.wheelbase,steering_angle,
                                current_node.time_stamp+planning_details.one_time_step,planning_details)
            g = current_node.actual_cost + (planning_details.lambda^(current_node.time_stamp+planning_details.one_time_step))*future_cost 
            
            # Calculate heuristic with current MAP goal
            h = calculate_heuristic_cost(new_x,new_y,new_theta,modified_params.goal.x,modified_params.goal.y,world)
            
            # Define new graph node
            new_node = GraphNode(new_x,new_y,new_theta,g,h,steering_angle,discrete_x,discrete_y,discrete_theta,
                                        current_node,current_node.time_stamp+planning_details.one_time_step)
            
            if(new_node.actual_cost == Inf)
                closed[node_key] = new_node
                continue
            end
            
            if(haskey(open,node_key))
                if(dict_of_nodes[node_key].actual_cost > new_node.actual_cost)
                    dict_of_nodes[node_key] = new_node
                    open[node_key] = new_node.heuristic_cost + new_node.actual_cost
                end
            else
                dict_of_nodes[node_key] = new_node
                open[node_key] = new_node.heuristic_cost + new_node.actual_cost
            end
        end
        
        if(time()- start_time >= planning_details.planning_time)
            @show("Time exceeded")
            return path
        end
    end
    return path
end

