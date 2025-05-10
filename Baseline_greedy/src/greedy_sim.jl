
function run_experiment_greedy!(current_sim_obj::NavigationSimulator{VehicleParametersLSPlanner}, 
                               path_planning_details::PathPlanningDetails,
                               pomdp_details::POMPDPlanningDetails,  # Keep for compatibility
                               exp_details::ExperimentDetails, 
                               shield_utils,  # Keep for compatibility
                               output::Output, 
                               sudden_break_flag = true,  # Keep for compatibility
                               run_shield = false,  # Keep for compatibility
                               run_with_trials = false)  # Keep for compatibility
    
    # Initialize variables
    current_time_value = 0.0
    fixed_speed = 2.0  # Fixed speed (2 m/s)
    output.sim_objects[current_time_value] = current_sim_obj
    nbh = NearbyHumans(HumanState[], Int64[], HumanGoalsBelief[])
    output.nearby_humans[current_time_value] = nbh
    vehicle_steer_actions = get_vehicle_steer_actions(pomdp_details.max_vehicle_steering_angle, 21)
    
    # Initialize target belief if empty
    if isempty(exp_details.target_belief.pdf)
        initialize_target_belief!(exp_details)
    end
    
    # Store initial target belief
    output.target_belief[current_time_value] = deepcopy(exp_details.target_belief)
    
    # Run simulation
    while(!stop_simulation!(current_sim_obj, current_time_value, exp_details, output))
        # Update target belief based on current position
        prev_belief = copy(exp_details.target_belief.pdf)
        update_target_belief!(exp_details, current_sim_obj.vehicle)
        
        # Store updated belief
        output.target_belief[current_time_value] = deepcopy(exp_details.target_belief)
        
        # Check if belief has changed significantly (e.g., eliminated a target)
        belief_changed = false
        for i in 1:length(prev_belief)
            if prev_belief[i] > 0.1 && exp_details.target_belief.pdf[i] == 0.0
                belief_changed = true
                break
            end
        end
        
        # Check for nearby humans and adjust speed
        nbh = get_nearby_humans(current_sim_obj, 10, 1.0, 2*pi)  # Get all humans within 2m
        current_speed = fixed_speed
        if length(nbh.position_data) > 0
            # Stop if any human is nearby
            current_speed = 0.0
        end
        
        # Debug output
        println("Time: $current_time_value")
        println("Current position: ($(current_sim_obj.vehicle.x), $(current_sim_obj.vehicle.y))")
        println("Belief: $(exp_details.target_belief.pdf)")
        println("MAP goal: $(get_map_target(exp_details).x), $(get_map_target(exp_details).y)")
        println("Speed: $current_speed")
        
        # If belief has changed significantly, recompute path
        if belief_changed
            println("Belief changed significantly. Recomputing path.")
            
            # Get new MAP goal
            new_map_goal = get_map_target(exp_details)
            
            # Update goal in vehicle parameters
            modified_vehicle_params = VehicleParametersLSPlanner(
                current_sim_obj.vehicle_params.wheelbase,
                current_sim_obj.vehicle_params.length,
                current_sim_obj.vehicle_params.breadth,
                current_sim_obj.vehicle_params.dist_origin_to_center,
                current_sim_obj.vehicle_params.radius,
                current_sim_obj.vehicle_params.max_speed,
                current_sim_obj.vehicle_params.max_steering_angle,
                new_map_goal,  # Update goal
                Float64[]  # Empty controls sequence
            )
            
            # Recompute path
            vehicle_controls_sequence = hybrid_astar_search(
                current_sim_obj.env,
                current_sim_obj.vehicle,
                modified_vehicle_params,
                vehicle_steer_actions,
                nbh,
                path_planning_details,
                exp_details
            )
            
            # Update vehicle parameters with new path
            current_sim_obj = NavigationSimulator(
                current_sim_obj.env,
                current_sim_obj.vehicle,
                VehicleParametersLSPlanner(
                    current_sim_obj.vehicle_params.wheelbase,
                    current_sim_obj.vehicle_params.length,
                    current_sim_obj.vehicle_params.breadth,
                    current_sim_obj.vehicle_params.dist_origin_to_center,
                    current_sim_obj.vehicle_params.radius,
                    current_sim_obj.vehicle_params.max_speed,
                    current_sim_obj.vehicle_params.max_steering_angle,
                    new_map_goal,
                    vehicle_controls_sequence
                ),
                current_sim_obj.vehicle_sensor_data,
                current_sim_obj.humans,
                current_sim_obj.humans_params,
                current_sim_obj.one_time_step
            )
        end
        
        # Generate vehicle trajectory using current controls and speed
        vehicle_trajectory, latest_controls_index = propogate_vehicle(
            current_sim_obj.vehicle, 
            current_sim_obj.vehicle_params,
            current_speed,
            current_time_value, 
            exp_details.one_time_step, 
            path_planning_details, 
            exp_details
        )
        
        # Get expected path to goal
        vehicle_expected_goal_paths = get_future_vehicle_trajectory(
            vehicle_trajectory, 
            current_sim_obj.vehicle_params,
            latest_controls_index+1,
            path_planning_details,
            exp_details
        )
        
        # Simulate for one time step
        current_sim_obj = simulate_vehicle_and_humans!(
            current_sim_obj, 
            vehicle_trajectory, 
            vehicle_expected_goal_paths,
            current_time_value, 
            exp_details.one_time_step, 
            exp_details, 
            output
        )
        
        # Update time
        current_time_value = round(current_time_value + exp_details.one_time_step, digits=1)
        
        # Store actions for visualization
        output.vehicle_actions[current_time_value] = ActionLimitedSpacePOMDP(0.0)
        output.nearby_humans[current_time_value] = nbh
    end
    
    output.time_taken = current_time_value
end
