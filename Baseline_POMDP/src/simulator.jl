#=
Updated stop_simulation! to check against true target from exp_details

Modified propogate_vehicle to use MAP goal from belief

Added target belief initialization in experiment runner

Added update_target_belief! call before planning cycles

Modified Hybrid A* call to include exp_details for dynamic goal


Added transition_target_state! in simulation step

Maintained original safety checks while adding goal uncertainty
=#

struct NavigationSimulator{P}
    env::ExperimentEnvironment
    vehicle::Vehicle
    vehicle_params::P
    vehicle_sensor_data::VehicleSensor
    humans::Array{HumanState,1}
    humans_params::Array{HumanParameters,1}
    one_time_step::Float64
end


#=
Function for moving vehicle in the environment
Returns a vehicle struct object
=#

#Propogte vehicle for Extended Space Planner
function propogate_vehicle(vehicle::Vehicle, vehicle_params::VehicleParametersESPlanner, steering_angle::Float64, speed::Float64, time_duration::Float64)
    new_x,new_y,new_theta =  move_vehicle(vehicle.x,vehicle.y,vehicle.theta,vehicle_params.wheelbase,steering_angle,speed,time_duration)
    return Vehicle(new_x,new_y,new_theta,speed)
end

#Propogte vehicle for Limited Space Planner
function propogate_vehicle(vehicle::Vehicle, vehicle_params::VehicleParametersLSPlanner, vehicle_speed::Float64, current_time::Float64,
                                    time_duration::Float64, path_planning_details, exp_details)


      ############# Get current MAP goal from belief
    current_goal = get_map_target(exp_details)
    
    # Create modified parameters with dynamic goal
    modified_params = VehicleParametersLSPlanner(
        vehicle_params.wheelbase,
        vehicle_params.length,
        vehicle_params.breadth,
        vehicle_params.dist_origin_to_center,
        vehicle_params.radius,
        vehicle_params.max_speed,
        vehicle_params.max_steering_angle,
        current_goal,  # Dynamic goal
        vehicle_params.controls_sequence
    )

    num_steps_on_vehicle_path = Int64((vehicle_speed * time_duration)/(path_planning_details.veh_path_planning_v*path_planning_details.one_time_step))
    time_between_steering_angle_changes = time_duration/num_steps_on_vehicle_path
    num_sim_steps = Int64(time_duration/exp_details.simulator_time_step)
    simulator_time_duration = time_between_steering_angle_changes/num_sim_steps
    CURRENT_TIME_VALUE = current_time
    complete_vehicle_path = Vehicle[]
    final_vehicle_path = OrderedDict(CURRENT_TIME_VALUE => vehicle)
    current_x,current_y,current_theta = vehicle.x,vehicle.y,vehicle.theta

    num_loop_cycles = min(num_steps_on_vehicle_path, length(vehicle_params.controls_sequence))
    for i in 1:num_loop_cycles
        steering_angle = vehicle_params.controls_sequence[i]
        for j in 1:num_sim_steps
            new_x,new_y,new_theta = move_vehicle(current_x,current_y,current_theta,vehicle_params.wheelbase,steering_angle,vehicle_speed,simulator_time_duration)
            push!(complete_vehicle_path, Vehicle(new_x,new_y,new_theta,vehicle_speed))
            current_x,current_y,current_theta = new_x,new_y,new_theta
        end
    end

    if(vehicle_speed == 0.0)
        for i in 1:num_sim_steps
            CURRENT_TIME_VALUE = round(current_time + (i*exp_details.simulator_time_step), digits=1)
            final_vehicle_path[CURRENT_TIME_VALUE] = Vehicle(vehicle.x,vehicle.y,vehicle.theta,vehicle_speed)
        end
    else
        for i in 1:num_sim_steps
            CURRENT_TIME_VALUE = round(current_time + (i*exp_details.simulator_time_step), digits=1)
            if(i*num_steps_on_vehicle_path > length(complete_vehicle_path) )
                final_vehicle_path[CURRENT_TIME_VALUE] = complete_vehicle_path[end]
            else
                final_vehicle_path[CURRENT_TIME_VALUE] = complete_vehicle_path[i*num_steps_on_vehicle_path]
            end
        end
    end
    return final_vehicle_path,num_loop_cycles
end


#=
Function for moving human in the environment
Returns a human_state struct object
=#
function propogate_human(human::HumanState,human_params::HumanParameters)
    # modified_human_state = get_next_human_state()
    modified_human_state = human_params.path[human_params.path_index+1]
    return modified_human_state
end

#=
Functions to modify vehicle_params
Returns a vehicle_parameters struct object
=#
function modify_vehicle_params(params::VehicleParametersESPlanner)
    return VehicleParametersESPlanner(params.wheelbase,params.length,params.breadth,params.dist_origin_to_center,
                params.radius,params.max_speed,params.max_steering_angle,params.goal)
end

function modify_vehicle_params(params::VehicleParametersLSPlanner, vehicle_speed, path_planning_v)
    path_new_starting_index = Int64(vehicle_speed/path_planning_v) + 1
    return VehicleParametersLSPlanner(params.wheelbase,params.length,params.breadth,params.dist_origin_to_center,params.radius,params.max_speed,
                params.max_steering_angle,params.goal,params.controls_sequence[path_new_starting_index:end])
end

function modify_vehicle_params(params::VehicleParametersLSPlanner, new_controls_sequence)
    return VehicleParametersLSPlanner(params.wheelbase,params.length,params.breadth,params.dist_origin_to_center,params.radius,params.max_speed,
                params.max_steering_angle,params.goal,new_controls_sequence)
end

function get_future_vehicle_trajectory(current_action_trajectory, vehicle_params, new_start_index, path_planning_details,exp_details)
    new_vehicle_position = collect(values(current_action_trajectory))[end]
    trajectory_after_current_action = get_hybrid_astar_trajectory(new_vehicle_position,vehicle_params,new_start_index,path_planning_details,exp_details)

    time_values = collect(keys(current_action_trajectory))
    future_vehicle_trajectory_dict = OrderedDict()

    for i in 1:length(time_values)
        curr_vehicle = current_action_trajectory[time_values[i]]
        vehicle_future_trajectory_x = [curr_vehicle.x]
        vehicle_future_trajectory_y = [curr_vehicle.y]
        vehicle_future_trajectory_theta = [curr_vehicle.theta]
        for j in i+1:length(time_values)
            veh = current_action_trajectory[time_values[j]]
            push!(vehicle_future_trajectory_x,veh.x)
            push!(vehicle_future_trajectory_y,veh.y)
            push!(vehicle_future_trajectory_theta,veh.theta)
        end
        vehicle_future_trajectory_x = [vehicle_future_trajectory_x; trajectory_after_current_action[1]]
        vehicle_future_trajectory_y = [vehicle_future_trajectory_y; trajectory_after_current_action[2]]
        vehicle_future_trajectory_theta = [vehicle_future_trajectory_theta; trajectory_after_current_action[3]]
        future_vehicle_trajectory_dict[time_values[i]] = (vehicle_future_trajectory_x,vehicle_future_trajectory_y,vehicle_future_trajectory_theta)
    end
    return future_vehicle_trajectory_dict
end

#=
Function to check if it is time to stop the simulation
Returns true or false
=#
function stop_simulation!(sim_obj,curr_time,exp_details,output)
    #Check if the vehicle has reached the goal
    vehicle_center_x = sim_obj.vehicle.x + sim_obj.vehicle_params.dist_origin_to_center*cos(sim_obj.vehicle.theta)
    vehicle_center_y = sim_obj.vehicle.y + sim_obj.vehicle_params.dist_origin_to_center*sin(sim_obj.vehicle.theta)

    #########
    true_goal = exp_details.true_target
    if(is_within_range(vehicle_center_x,vehicle_center_y,true_goal.x,true_goal.y,exp_details.radius_around_vehicle_goal))
        output.vehicle_reached_goal = true
        println("Vehicle has reached the goal")
        return true
    end
    #Check if the vehicle is colliding with obstacles in the environment
    for obstacle in sim_obj.env.obstacles
        if(in_obstacle(vehicle_center_x,vehicle_center_y,obstacle,sim_obj.vehicle_params.radius))
            output.vehicle_ran_into_obstacle = true
            println("Vehicle collided with a static obstacle")
            return true
        end
    end
    #Check if the vehicle is colliding with environment's boundary
    if(vehicle_center_x<0.0+sim_obj.vehicle_params.radius || vehicle_center_y<0.0+sim_obj.vehicle_params.radius ||
        vehicle_center_x>sim_obj.env.length-sim_obj.vehicle_params.radius || vehicle_center_y>sim_obj.env.breadth-sim_obj.vehicle_params.radius)
        output.vehicle_ran_into_boundary_wall = true
        println("Vehicle collided with environment boundary")
        return true
    end
    if(curr_time >= exp_details.MAX_TIME_LIMIT)
        println("Vehicle didn't reach the goal in given time limit")
        return true
    end
    return false
end

#=
For Extended Space Planner
Function to simulate vehicle and humans in the environment for given time duration.
Propogate humans for simulator's one time step.
Propogate vehicle for simulator's one time step.
Get new vehicle sensor data.
Create new struct object for vehicle params.
Create new struct object for the vehicle.
Create new struct object for the humans.
=#
function simulate_vehicle_and_humans!(sim::NavigationSimulator, vehicle_steering_angle::Float64, vehicle_speed::Float64, current_time::Float64, time_duration::Float64,
                            exp_details::ExperimentDetails, output::Output)

    current_sim_obj = sim
    number_steps_in_sim = Int64(time_duration/current_sim_obj.one_time_step)

    for i in 1:number_steps_in_sim
        CURRENT_TIME_VALUE = round(current_time + (i*current_sim_obj.one_time_step),digits=1)
        propogated_humans = Array{HumanState,1}(undef,exp_details.num_humans_env)
        for i in 1:exp_details.num_humans_env
            # propogated_humans[i] = propogate_human(current_sim_obj.humans[i], current_sim_obj.env, current_sim_obj.one_time_step, exp_details.user_defined_rng)
            propogated_humans[i] = propogate_human(current_sim_obj.humans[i], current_sim_obj.humans_params[i])
            # current_sim_obj.humans_params[i].path_index = clamp(current_sim_obj.humans_params[i].path_index+1,1,length(current_sim_obj.humans_params[i].path))
            current_sim_obj.humans_params[i].path_index += 1
        end
        #Get new vehicle object and new vehicle_params object
        new_veh_obj = propogate_vehicle(current_sim_obj.vehicle, current_sim_obj.vehicle_params, vehicle_steering_angle, vehicle_speed, current_sim_obj.one_time_step)
        new_vehicle_parameters = modify_vehicle_params(current_sim_obj.vehicle_params)
        # new_humans,new_humans_params = propogated_humans,copy(current_sim_obj.humans_params)
        new_humans, new_humans_params = respawn_humans(propogated_humans,current_sim_obj.humans_params,new_veh_obj,new_vehicle_parameters,exp_details)

        #Get new vehicle sensor data
        new_sensor_data_obj = get_vehicle_sensor_data(new_veh_obj,new_humans,new_humans_params,current_sim_obj.vehicle_sensor_data,exp_details,CURRENT_TIME_VALUE)

        #Create a new simulator object
        new_sim_object = NavigationSimulator(current_sim_obj.env, new_veh_obj, new_vehicle_parameters, new_sensor_data_obj, new_humans, new_humans_params, current_sim_obj.one_time_step)

        #Check if this is a risky scenario
        num_risks_this_time_step = get_num_risks(new_veh_obj, new_vehicle_parameters, new_sensor_data_obj.lidar_data, exp_details.max_risk_distance)
        if(num_risks_this_time_step!=0)
            output.number_risky_scenarios += num_risks_this_time_step
            output.risky_scenarios[CURRENT_TIME_VALUE] = new_sim_object
        end

        output.sim_objects[CURRENT_TIME_VALUE] = new_sim_object
        output.nearby_humans[CURRENT_TIME_VALUE] = copy(output.nearby_humans[current_time])
        current_sim_obj = new_sim_object
    end
    return current_sim_obj
end

#=
For Limited Space Planner
Function to simulate vehicle and humans in the environment for given time duration.
Propogate humans for simulator's one time step.
Propogate vehicle for simulator's one time step.
Get new vehicle sensor data.
Create new struct object for vehicle params.
Create new struct object for the vehicle.
Create new struct object for the humans.
=#
function simulate_vehicle_and_humans!(sim::NavigationSimulator, vehicle_trajectory::OrderedDict, goal_paths::OrderedDict,
                                    current_time::Float64, time_duration::Float64, exp_details::ExperimentDetails, output::Output)


    ########## Update true target position with minimal noise
    transition_target_state!(exp_details)
    current_sim_obj = sim
    number_steps_in_sim = Int64(time_duration/current_sim_obj.one_time_step)

    for i in 1:number_steps_in_sim
        CURRENT_TIME_VALUE = round(current_time + (i*current_sim_obj.one_time_step), digits=1)
        propogated_humans = Array{HumanState,1}(undef,exp_details.num_humans_env)
        for i in 1:exp_details.num_humans_env
            # propogated_humans[i] = propogate_human(current_sim_obj.humans[i], current_sim_obj.env, current_sim_obj.one_time_step, exp_details.user_defined_rng)
            propogated_humans[i] = propogate_human(current_sim_obj.humans[i], current_sim_obj.humans_params[i])
            # current_sim_obj.humans_params[i].path_index = clamp(current_sim_obj.humans_params[i].path_index+1,1,length(current_sim_obj.humans_params[i].path))
            current_sim_obj.humans_params[i].path_index += 1
        end
        #Get new vehicle object and new vehicle_params object
        new_veh_obj = vehicle_trajectory[CURRENT_TIME_VALUE]
        new_vehicle_parameters = copy(current_sim_obj.vehicle_params)
        # new_humans,new_humans_params = propogated_humans,copy(current_sim_obj.humans_params)
        new_humans, new_humans_params = respawn_humans(propogated_humans,current_sim_obj.humans_params,new_veh_obj,new_vehicle_parameters,exp_details)

        #Get new vehicle sensor data
        new_sensor_data_obj = get_vehicle_sensor_data(new_veh_obj,new_humans,new_humans_params,current_sim_obj.vehicle_sensor_data,exp_details,CURRENT_TIME_VALUE)

        #Create a new simulator object
        new_sim_object = NavigationSimulator(current_sim_obj.env, new_veh_obj, new_vehicle_parameters, new_sensor_data_obj, new_humans, new_humans_params, current_sim_obj.one_time_step)

        #Check if this is a risky scenario
        num_risks_this_time_step = get_num_risks(new_veh_obj, new_vehicle_parameters, new_sensor_data_obj.lidar_data, exp_details.max_risk_distance)
        if(num_risks_this_time_step!=0)
            output.number_risky_scenarios += num_risks_this_time_step
            output.risky_scenarios[CURRENT_TIME_VALUE] = new_sim_object
        end

        #Get the vehicle trajectory from current position to its goal
        expected_vehicle_trajectory_to_goal = goal_paths[CURRENT_TIME_VALUE]
        output.vehicle_expected_trajectory[CURRENT_TIME_VALUE] = expected_vehicle_trajectory_to_goal
        output.sim_objects[CURRENT_TIME_VALUE] = new_sim_object
        output.nearby_humans[CURRENT_TIME_VALUE] = copy(output.nearby_humans[current_time])
        current_sim_obj = new_sim_object
    end
    return current_sim_obj
end

#=
Run the experiment for Extended Space Planner
=#
function run_experiment!(current_sim_obj::NavigationSimulator{VehicleParametersESPlanner},
                            planner, lower_bound_func, upper_bound_func,
                            pomdp_details::POMPDPlanningDetails, exp_details::ExperimentDetails, 
                            shield_utils, output::Output,
                            run_shield = false,
                            run_with_trials=false,
                            print_logs=true)

    # initialize variables
    current_time_value = 0.0
    current_vehicle_speed = 0.0
    current_vehicle_steering_angle = 0.0
    current_action = ActionExtendedSpacePOMDP(0.0,0.0)
    output.vehicle_actions[current_time_value] = current_action
    output.sim_objects[current_time_value] = current_sim_obj
    nbh = NearbyHumans(HumanState[], Int64[], HumanGoalsBelief[])
    output.nearby_humans[current_time_value] = nbh
    next_action = nothing
    info = nothing
    predicted_vehicle_pos_in_goal = false
    debug = false
    store_tree=true
    print_despot_time = true
    print_shielding_time = true
    despot_start_time = 0.0
    start_time = 0.0
    ES_pomdp = planner.pomdp

    # Run simulation
    # try
    while(!stop_simulation!(current_sim_obj,current_time_value,exp_details,output))
        pomdp_solver = DESPOTSolver(
                        bounds=IndependentBounds(lower_bound_func,upper_bound_func,check_terminal=true,consistency_fix_thresh=1e-5),
                        K=pomdp_details.num_scenarios,D=pomdp_details.tree_search_max_depth,
                        tree_in_info=true,
                        T_max = run_with_trials ? Inf : pomdp_details.planning_time,
                        max_trials = run_with_trials ? pomdp_details.max_num_trials : 100,
                        default_action=get_default_action,
                        rng = MersenneTwister(19)
                        )
        planner = POMDPs.solve(pomdp_solver, ES_pomdp);

        state_array = (current_sim_obj.vehicle.x, current_sim_obj.vehicle.y, current_sim_obj.vehicle.theta*180/pi, current_sim_obj.vehicle.v)
        action_array = (current_action.steering_angle*180/pi, current_action.delta_speed)
        if(print_logs)
            println("\nTime_k = ", current_time_value,
                  "\t State_k = ", round.(state_array, digits=3),
                  "\t Action_k = ", round.(action_array, digits=3))
        end
        # Simulate for (one_time_step - pomdp_planning_time - buffer_time) seconds
        if(debug)
            println("Simulating for one_time_step - pomdp_planning_time - buffer_time seconds : " , exp_details.one_time_step - (pomdp_details.planning_time + exp_details.buffer_time))
            start_time = time()
        end

        time_duration_until_planning_begins = round(exp_details.one_time_step - (pomdp_details.planning_time + exp_details.buffer_time), digits=1)
        current_sim_obj = simulate_vehicle_and_humans!(current_sim_obj, current_vehicle_steering_angle, current_vehicle_speed,
                                                current_time_value, time_duration_until_planning_begins, exp_details, output)
        current_time_value = round(current_time_value + time_duration_until_planning_begins,digits=1)

        if(debug)
            println("Finished simulation")
            time_taken = time() - start_time
            println("Time Taken : ", time_taken)
        end

        #=
        Take the current status of the environment.
        Predict the vehicle's future position using current_action after (pomdp_planning_time + buffer_time) seconds.
        Initialize scenarios with sampled human goals and predicted human positions after (pomdp_planning_time + buffer_time) seconds.
        Solve the POMDP and find the action.
        =#
        time_duration_until_pomdp_action_determined = round(pomdp_details.planning_time + exp_details.buffer_time, digits=1)
        predicted_vehicle_state = propogate_vehicle(current_sim_obj.vehicle, current_sim_obj.vehicle_params,
                                            current_vehicle_steering_angle, current_vehicle_speed, time_duration_until_pomdp_action_determined)
        predicted_vehicle_center_x = predicted_vehicle_state.x + current_sim_obj.vehicle_params.dist_origin_to_center*cos(predicted_vehicle_state.theta)
        predicted_vehicle_center_y = predicted_vehicle_state.y + current_sim_obj.vehicle_params.dist_origin_to_center*sin(predicted_vehicle_state.theta)
        modified_vehicle_params = modify_vehicle_params(current_sim_obj.vehicle_params)

        # Check if the predicted vehicle position is in the goal region
        if(is_within_range(predicted_vehicle_center_x,predicted_vehicle_center_y,current_sim_obj.vehicle_params.goal.x,current_sim_obj.vehicle_params.goal.y,exp_details.radius_around_vehicle_goal))
            println("Predicted vehicle state is in goal. Experiment Finished")
            next_pomdp_action = ActionExtendedSpacePOMDP(0.0,0.0)
            output.nearby_humans[current_time_value] = nbh
            output.b_root[current_time_value] = nothing
            output.despot_trees[current_time_value] = nothing
            predicted_vehicle_pos_in_goal = true
        # If it is not, then take observation from the environment and plan for the next action
        else
            if(debug)
                println("Starting POMDP planning")
                start_time = time()
            end
            # nbh = get_nearby_humans(current_sim_obj,pomdp_details.num_nearby_humans,pomdp_details.min_safe_distance_from_human,
            #                                         pomdp_details.cone_half_angle)
            nbh = get_nearby_humans(current_sim_obj,pomdp_details.num_nearby_humans,2.0,pomdp_details.cone_half_angle)
            b = TreeSearchScenarioParameters(predicted_vehicle_state.x,predicted_vehicle_state.y,predicted_vehicle_state.theta,predicted_vehicle_state.v,
                                modified_vehicle_params, exp_details.human_goal_locations, length(nbh.position_data), nbh.position_data, nbh.belief,
                                current_sim_obj.env.length,current_sim_obj.env.breadth,time_duration_until_pomdp_action_determined,
                                exp_details.true_target,           # NEW: true_target
                                exp_details.target_locations,      # NEW: target_locations
                                exp_details.radius_around_vehicle_goal)  # NEW: radius_around_vehicle_goal)
            output.nearby_humans[current_time_value] = nbh
            output.b_root[current_time_value] = b

            # Run DESPOT to calculate next action
            if(print_despot_time)
                println("Started Tree Construction")
                despot_start_time = time()
            end

            next_pomdp_action, info = action_info(planner, b)
            # next_pomdp_action, info = @profiler action_info(planner, b)
            # next_pomdp_action, info =
            # println(JET.@report_opt action_info(planner, b))

            if(print_despot_time)
                println("Finished Tree Construction. Action selected")
                time_taken = time() - despot_start_time
                println("Time Taken : ", time_taken)
            end

            # next_pomdp_action, info = @profiler action_info(planner, b)
            # next_pomdp_action, info =
            # println(JET.@report_opt action_info(planner, b))

            if(debug)
                println("Finished POMDP planning. Action selected")
                time_taken = time() - start_time
                println("Time Taken : ", time_taken)
            end

            if(store_tree)
                output.despot_trees[current_time_value] = info
            else
                output.despot_trees[current_time_value] = nothing
            end
        end

        if(debug)
            println("Simulating for pomdp_planning_time + buffer_time seconds : " , pomdp_details.planning_time + exp_details.buffer_time)
            start_time = time()
        end
        # Simulate for (pomdp_planning_time + buffer_time) seconds
        time_duration_until_next_action_is_applied = round(pomdp_details.planning_time + exp_details.buffer_time,digits=1)
        current_sim_obj = simulate_vehicle_and_humans!(current_sim_obj, current_vehicle_steering_angle, current_vehicle_speed,
                                                    current_time_value, time_duration_until_next_action_is_applied, exp_details, output)

        # Shielding
        if(run_shield)
            if(print_logs)
                println("POMDP requested action = ", (next_pomdp_action.steering_angle, next_pomdp_action.delta_speed))
            end
            if(print_shielding_time)
                println("Started Shielding")
                shielding_start_time = time()
            end

            shield_radius_humans = find_shield_radius_humans(shield_utils, ES_pomdp, current_sim_obj)
            # Check if the predicted vehicle position is in the goal region
            if(predicted_vehicle_pos_in_goal)
                next_action = next_pomdp_action
            # Else, check if the pomdp tree is empty (root node took default action)
            elseif length(info[:tree].children[1]) == 0
                next_action = next_pomdp_action
            # Else, run shield and return the best safe action
            else
                next_action = get_best_safe_action(ES_pomdp, shield_utils, shield_radius_humans, current_sim_obj.vehicle, info[:tree])
            end
            if(print_shielding_time)
                println("Finished Shielding. Action selected")
                time_taken = time() - shielding_start_time
                println("Time Taken : ", time_taken)
            end
            if(print_logs)
                println("Shield selected action = ", (next_action.steering_angle, next_action.delta_speed))
            end
        else
            next_action = next_pomdp_action
        end

        # Set parameters for the next cycle
        current_vehicle_speed = clamp((current_vehicle_speed + next_action.delta_speed), 0.0, pomdp_details.max_vehicle_speed)
        current_vehicle_steering_angle = next_action.steering_angle
        current_action = next_action
        current_time_value = round(current_time_value + time_duration_until_next_action_is_applied,digits=1)

        if(debug)
            println("Finished simulation")
            time_taken = time() - start_time
            println("Time Taken : ", time_taken)
        end

        # Store relevant values in exp_details
        output.vehicle_actions[current_time_value] = current_action
        if(current_action.delta_speed == -10.0)
            output.number_sudden_stops += 1
        end
    end

    # catch e
    #     println("\n Things failed during the simulation. \n The error message is : \n ")
    #     println(e)
    # end
    output.time_taken = current_time_value
end

#=
Run the experiment for Limited Space Planner
=#
function run_experiment!(current_sim_obj::NavigationSimulator{VehicleParametersLSPlanner}, path_planning_details::PathPlanningDetails,
                                                pomdp_details::POMPDPlanningDetails, exp_details::ExperimentDetails, 
                                                shield_utils, output::Output,
                                                sudden_break_flag = true,
                                                run_shield = false,
                                                run_with_trials=false,
                                                print_logs = true)

    # initialize variables
    current_time_value = 0.0
    current_vehicle_speed = 0.0
    vehicle_steer_actions = get_vehicle_steer_actions(pomdp_details.max_vehicle_steering_angle,21)
    current_action = ActionLimitedSpacePOMDP(0.0)
    output.vehicle_actions[current_time_value] = current_action
    output.sim_objects[current_time_value] = current_sim_obj
    nbh = NearbyHumans(HumanState[], Int64[], HumanGoalsBelief[])
    output.nearby_humans[current_time_value] = nbh
    expected_vehicle_path = get_future_vehicle_trajectory(OrderedDict(current_time_value=>current_sim_obj.vehicle),
                                                    current_sim_obj.vehicle_params,1,path_planning_details,exp_details)
    output.vehicle_expected_trajectory[current_time_value] = expected_vehicle_path[current_time_value]
    modified_vehicle_params = copy(current_sim_obj.vehicle_params)
    next_action = nothing
    info = nothing
    predicted_vehicle_pos_in_goal = false
    debug = false
    print_despot_time = true
    print_shielding_time = true
    despot_start_time = 0.0
    store_tree=true
    start_time = time()
    LS_pomdp = nothing

    #=
    Psuedocode
        while(!stop_condition)
            1) Print vehicle state, action and current time step
            2) Find vehicle trajectory using this action for one_time_step seconds
            3) Simulate humans for (one_time_step - pomdp_planning_time - buffer_time - path_planning_time) seconds
            4) a) Observe current human positions and their belief
               b) Predict future vehicle position
               c) Generate hybrid A* path
               d) Initialize scenarios
               e) Feed scenarios and hybrid A* path to the LS planner to get vehicle action
            5) Simulate humans for (pomdp_planning_time + buffer_time + path_planning_time) seconds
            6) Run shielding
            7) Set parameters for next cycle and store/modify required variables
        end
    =#

    # Run simulation
    # try
    ############ Initialize target belief if empty
    if isempty(exp_details.target_belief.pdf)
        initialize_target_belief!(exp_details)
    end
    while(!stop_simulation!(current_sim_obj,current_time_value,exp_details,output))
        ######## Before planning, update target belief
        println("Target belief before planning: ", exp_details.target_belief.pdf)
        update_target_belief!(exp_details, current_sim_obj.vehicle)
        println("Target belief after planning: ", exp_details.target_belief.pdf)

        # Store target belief in output for visualization
        output.target_belief[current_time_value] = deepcopy(exp_details.target_belief)
        # Add debug output
        println("Time: $current_time_value")
        println("Current position: ($(current_sim_obj.vehicle.x), $(current_sim_obj.vehicle.y))")
        println("Belief: $(exp_details.target_belief.pdf)")
        println("MAP goal: $(get_map_target(exp_details))")
        state_array = (current_sim_obj.vehicle.x, current_sim_obj.vehicle.y, current_sim_obj.vehicle.theta*180/pi, current_sim_obj.vehicle.v)
        action_array = (current_sim_obj.vehicle_params.controls_sequence[1]*180/pi, current_action.delta_speed)
        if(print_logs)
            println("\nTime_k = ", current_time_value,
                  "\t State_k = ", round.(state_array, digits=3),
                  "\t Action_k = ", round.(action_array, digits=3))
        end
        #=
        Generate vehicle trajectory using current action
        =#
        vehicle_trajectory, latest_controls_sequence_index = propogate_vehicle(current_sim_obj.vehicle, current_sim_obj.vehicle_params,current_vehicle_speed,
                                                            current_time_value, exp_details.one_time_step, path_planning_details, exp_details)
        vehicle_expected_goal_paths = get_future_vehicle_trajectory(vehicle_trajectory, current_sim_obj.vehicle_params,latest_controls_sequence_index+1,
                                                            path_planning_details,exp_details)

        #=
        Simulate for (one_time_step - path_planning_time - pomdp_planning_time - buffer_time) seconds
        =#
        if(debug)
            println("Simulating for one_time_step - path_planning_time - pomdp_planning_time - buffer_time seconds : " ,
                                exp_details.one_time_step - (path_planning_details.planning_time + pomdp_details.planning_time + exp_details.buffer_time))
            start_time = time()
        end
        time_duration_until_planning_begins = round(exp_details.one_time_step - (path_planning_details.planning_time +
                                                        pomdp_details.planning_time + exp_details.buffer_time), digits=1)
        current_sim_obj = simulate_vehicle_and_humans!(current_sim_obj, vehicle_trajectory, vehicle_expected_goal_paths,
                                                current_time_value, time_duration_until_planning_begins, exp_details, output)
        current_time_value = round(current_time_value+time_duration_until_planning_begins, digits=1)
        if(debug)
            println("Finished simulation")
            time_taken = time() - start_time
            println("Time Taken : ", time_taken)
        end

        #=
        Planning for the next cycle
        =#
        time_duration_until_pomdp_action_determined = round(pomdp_details.planning_time + exp_details.buffer_time, digits=1)
        predicted_vehicle_state = collect(values(vehicle_trajectory))[end]
        predicted_vehicle_center_x = predicted_vehicle_state.x + current_sim_obj.vehicle_params.dist_origin_to_center*cos(predicted_vehicle_state.theta)
        predicted_vehicle_center_y = predicted_vehicle_state.y + current_sim_obj.vehicle_params.dist_origin_to_center*sin(predicted_vehicle_state.theta)
        # println("Predicted Vehicle State : ", predicted_vehicle_state)
        # println("Predicted Vehicle Center : ", predicted_vehicle_center_x, " , ", predicted_vehicle_center_y)
        # Check if the predicted vehicle position is in the goal region
        # true_goal = exp_details.true_target
        if(is_within_range(predicted_vehicle_center_x,predicted_vehicle_center_y,current_sim_obj.vehicle_params.goal.x,current_sim_obj.vehicle_params.goal.y,exp_details.radius_around_vehicle_goal))
            println("Predicted vehicle state is in goal. Experiment Finished")
            next_pomdp_action = ActionLimitedSpacePOMDP(0.0)
            output.nearby_humans[current_time_value] = nbh
            output.b_root[current_time_value] = nothing
            output.despot_trees[current_time_value] = nothing
            predicted_vehicle_pos_in_goal = true
        # If it is not, then take observation from the environment and plan for the next action
        else
            # nbh = get_nearby_humans(current_sim_obj,pomdp_details.num_nearby_humans,pomdp_details.min_safe_distance_from_human,
            #                                         pomdp_details.cone_half_angle)
            nbh = get_nearby_humans(current_sim_obj,pomdp_details.num_nearby_humans,2.0,pomdp_details.cone_half_angle)
            if(debug)
                println("Starting Hybrid A* path planning")
                start_time = time()
            end
            vehicle_steering_controls = hybrid_astar_search(current_sim_obj.env, predicted_vehicle_state, modified_vehicle_params,
                                                    vehicle_steer_actions, nbh, path_planning_details, exp_details)
            if(debug)
                println("Finished Hybrid A* path planning")
                time_taken = time() - start_time
                println("Time Taken : ", time_taken)
            end

            if(length(vehicle_steering_controls) == 0)
                if(print_logs)
                    println("Hybrid A* path not found within the given time limit. Reusing old path")
                end
                #Modify the steering_controls_sequence to reuse the old path
                modified_vehicle_params = modify_vehicle_params(current_sim_obj.vehicle_params, current_vehicle_speed, path_planning_details.veh_path_planning_v)
            else
                #Modify vehicle params
                modified_vehicle_params = modify_vehicle_params(current_sim_obj.vehicle_params, vehicle_steering_controls)
            end

            if(debug)
                println("Starting POMDP planning")
                start_time = time()
            end
            b = TreeSearchScenarioParameters(predicted_vehicle_state.x,predicted_vehicle_state.y,predicted_vehicle_state.theta,predicted_vehicle_state.v,
                                modified_vehicle_params, exp_details.human_goal_locations, length(nbh.position_data), nbh.position_data, nbh.belief,
                                current_sim_obj.env.length,current_sim_obj.env.breadth,time_duration_until_pomdp_action_determined,
                                exp_details.true_target,           # NEW: true_target
                                exp_details.target_locations,      # NEW: target_locations
                                exp_details.radius_around_vehicle_goal)  # NEW: radius_around_vehicle_goal)
            output.nearby_humans[current_time_value] = nbh
            output.b_root[current_time_value] = b

            #=
            Define POMDP, POMDP Solver and POMDP Planner
            =#
            rollout_guide = HybridAStarPolicy(modified_vehicle_params.controls_sequence,length(modified_vehicle_params.controls_sequence))
            LS_pomdp = LimitedSpacePOMDP(pomdp_details,current_sim_obj.env,modified_vehicle_params,rollout_guide,sudden_break_flag,exp_details)
            lower_bound_func = DefaultPolicyLB(
                                    FunctionPolicy(b->calculate_lower_bound(LS_pomdp, b)),
                                    max_depth=pomdp_details.tree_search_max_depth
                                    )
            upper_bound_func = calculate_upper_bound
            get_default_action = make_default_action(LS_pomdp)
            pomdp_solver = DESPOTSolver(
                                bounds=IndependentBounds(lower_bound_func,upper_bound_func,check_terminal=true,consistency_fix_thresh=1e-5),
                                K=pomdp_details.num_scenarios,D=pomdp_details.tree_search_max_depth,
                                T_max = run_with_trials ? Inf : pomdp_details.planning_time,
                                max_trials = run_with_trials ? pomdp_details.max_num_trials : 1000,
                                tree_in_info=true,
                                default_action= get_default_action,
                                rng = MersenneTwister(19)
                                )
            planner = POMDPs.solve(pomdp_solver, LS_pomdp);

            # Run DESPOT to calculate next action
            # next_pomdp_action = ActionLimitedSpacePOMDP(0.0)
            if(print_despot_time)
                println("Started Tree Construction")
                despot_start_time = time()
            end

            next_pomdp_action, info = action_info(planner, b)

            if(print_despot_time)
                println("Finished Tree Construction. Action selected")
                time_taken = time() - despot_start_time
                println("Time Taken : ", time_taken)
            end

            if(debug)
                println("Finished POMDP planning. Action selected")
                time_taken = time() - start_time
                println("Time Taken : ", time_taken)
            end
            if(store_tree)
                output.despot_trees[current_time_value] = info
            else
                output.despot_trees[current_time_value] = nothing
            end
        end

        #=
        Simulate for (path_planning_time + pomdp_planning_time + buffer_time) seconds
        =#
        if(debug)
            println("Simulating for path_planning_time + pomdp_planning_time + buffer_time seconds : " , 
                        path_planning_details.planning_time + pomdp_details.planning_time + exp_details.buffer_time)
            start_time = time()
        end
        time_duration_until_next_action_is_applied = round(path_planning_details.planning_time + 
                                                    pomdp_details.planning_time + exp_details.buffer_time, digits=1)
        current_sim_obj = simulate_vehicle_and_humans!(current_sim_obj, vehicle_trajectory, vehicle_expected_goal_paths,
                                                current_time_value, time_duration_until_next_action_is_applied, exp_details, output)
        if(debug)
            println("Finished simulation")
            time_taken = time() - start_time
            println("Time Taken : ", time_taken)
        end

        #=
        Shielding
        =#
        if(run_shield)
            if(print_logs)
                println("POMDP requested action = ", (next_pomdp_action.delta_speed))
            end

            if(print_shielding_time)
                println("Started Shielding")
                shielding_start_time = time()
            end

            shield_radius_humans = find_shield_radius_humans(shield_utils, LS_pomdp, current_sim_obj)
            # Check if the predicted vehicle position is in the goal region
            if(predicted_vehicle_pos_in_goal)
                next_action = next_pomdp_action
            # Else, check if the pomdp tree is empty (root node took default action)
            elseif length(info[:tree].children[1]) == 0
                next_action = next_pomdp_action
            # Else, run shield and return the best safe action
            else
                veh_body = output.vehicle_body
                new_path_flag, next_action = get_best_safe_action(LS_pomdp, shield_utils, shield_radius_humans, 
                                                                current_sim_obj.vehicle, info[:tree])
                if(new_path_flag == false)
                    modified_vehicle_params = modify_vehicle_params(current_sim_obj.vehicle_params, current_vehicle_speed, 
                                                    path_planning_details.veh_path_planning_v)
                end
            end

            if(print_shielding_time)
                println("Finished Shielding. Action selected")
                time_taken = time() - shielding_start_time
                println("Time Taken : ", time_taken)
            end

            if(print_logs)
                println("Shield selected action = ", (next_action.delta_speed))
            end
        else
            next_action = next_pomdp_action
        end

        #=
        Set parameters for the next cycle
        =#
        current_action = next_action
        current_vehicle_speed = clamp((current_vehicle_speed + current_action.delta_speed), 0.0, pomdp_details.max_vehicle_speed)
        current_time_value = round(current_time_value+time_duration_until_next_action_is_applied,digits=1)
        current_sim_obj = copy(current_sim_obj,modified_vehicle_params)
        output.sim_objects[current_time_value] = current_sim_obj

        #=
        Store relevant values in exp_details
        =#
        output.vehicle_actions[current_time_value] = current_action
        if(current_action.delta_speed == -10.0)
            output.number_sudden_stops += 1
        end
    end

    # catch e
    #     println("\n Things failed during the simulation. \n The error message is : \n ")
    #     println(e)
    # end
    output.time_taken = current_time_value
end
