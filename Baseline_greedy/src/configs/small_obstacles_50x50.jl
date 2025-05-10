input_config =  InputParameters(
    MersenneTwister(0x32c57438), #=rng::AbstractRNG=#
    50.0, #=env_length::Float64=#
    50.0, #=env_breadth::Float64=#
    ObstacleLocation[ObstacleLocation(9,14,2.5), ObstacleLocation(10,29,2.5),
                    ObstacleLocation(15,40,2.5), ObstacleLocation(24,20,3.5),
                    ObstacleLocation(30,10,3), ObstacleLocation(36,35,3.5),
                    ObstacleLocation(46,45,2.5), ObstacleLocation(44,22,3) ], #=obstacles::Array{obstacle_location,1}=#
    200, #=num_humans_env::Int64=#
    1.0, #=human_start_v::Float64=#
    15.0, #=veh_start_x::Float64=#
    3.0, #=veh_start_y::Float64=#
    1/2*pi, #=veh_start_theta::Float64=#
    0.0, #=veh_start_v::Float64=#
    35.5, #=veh_goal_x::Float64=#
    45.0, #=veh_goal_y::Float64=#
    0.75, #=veh_wheelbase::Float64=#
    1.0, #=veh_length::Float64=#
    0.5, #=veh_breadth::Float64=#
    0.375, #=veh_dist_origin_to_center::Float64=#
    2.0, #=veh_max_speed::Float64=#
    0.475, #=veh_max_steering_angle=#
    0.5, #=veh_path_planning_v::Float64=#
    20.0, #=lidar_range::Float64=#
    6, #=num_nearby_humans::Int64=#
    2/3*pi, #=cone_half_angle::Float64=#
    0.98, #=pomdp_discount_factor::Float64=#
    1.0, #=min_safe_distance_from_human::Float64=#
    0.1, #=min_safe_distance_from_obstacle::Float64=#
    1.0, #=radius_around_vehicle_goal::Float64=#
    1.0, #=max_risk_distance::Float64=#
    -100.0, #=human_collision_penalty::Float64=#
    -500.0, #=obstacle_collision_penalty::Float64=#
    1000.0, #=goal_reached_reward::Float64=#
    5, #=num_segments_in_one_time_step::Int64=#
    1.0, #=observation_discretization_length::Float64=#
    100, #=tree_search_max_depth::Int64=#
    50, #=num_scenarios::Int64=#
    2.5, #=d_near::Float64 = 0.5=#
    4.0, #=d_far::Float64 = 0.5=#
    0.4, #=es_pomdp_planning_time::Float64=#
    11, #=es_max_num_trials::Float64=#
    0.3, #=ls_pomdp_planning_time::Float64=#
    101, #=ls_max_num_trials::Int64=#
    pi/4, #=pomdp_action_max_delta_heading_angle::Float64=#
    0.5, #=pomdp_action_delta_speed::Float64=#
    0.5, #=one_time_step::Float64=#
    0.1, #=simulator_time_step::Float64=#
    0.1, #=update_sensor_data_time_interval::Float64=#
    0.1, #=buffer_time::Float64=#
    500.0, #=MAX_TIME_LIMIT::Float64=#
    [Location(35.5, 45.0), Location(15.0, 25.0), Location(40.0, 15.0), Location(5.0, 35.0)],  # target_locations
    [0.20, 0.15, 0.35, 0.30],  # uniform initial_target_belief
    0.01 # target_transition_noise
)

#=
Weird scenarios
0xddb067f6
0x6d5e
never reached goal, causes local minima bellman problems - 0xc63596b3
never reached goal - 0x5b336665, 0x7961389e

collision - 0x595a3fa9, 0x7a24d142

Didn't reach goal - 0xf60674f2
=#
