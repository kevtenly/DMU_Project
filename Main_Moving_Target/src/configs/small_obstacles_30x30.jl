input_config =  InputParameters(
    MersenneTwister(0x62014574), #=rng::AbstractRNG=#
    30.0, #=env_length::Float64=#
    30.0, #=env_breadth::Float64=#
    # ObstacleLocation[],
    ObstacleLocation[ObstacleLocation(6,9,2), ObstacleLocation(7,19,1.5),
                    ObstacleLocation(12,25,1.5), ObstacleLocation(16,15,2.5),
                    ObstacleLocation(26,7,1.75) ], #=obstacles::Array{obstacle_location,1}=#
    20, #=num_humans_env::Int64=#
    1.0, #=human_start_v::Float64=#
    15.0, #=veh_start_x::Float64=#
    3.0, #=veh_start_y::Float64=#
    2/2*pi, #=veh_start_theta::Float64=#
    0.0, #=veh_start_v::Float64=#
    19.5, #=veh_goal_x::Float64=#
    27.0, #=veh_goal_y::Float64=#
    0.75, #=veh_wheelbase::Float64=#
    1.0, #=veh_length::Float64=#
    0.5, #=veh_breadth::Float64=#
    0.375, #=veh_dist_origin_to_center::Float64=#
    2.0, #=veh_max_speed::Float64=#
    0.475, #=veh_max_steering_angle=#
    0.5, #=veh_path_planning_v::Float64=#
    30.0, #=lidar_range::Float64=#
    6, #=num_nearby_humans::Int64=#
    3/3*pi, #=cone_half_angle::Float64=#
    0.98, #1.0, #=pomdp_discount_factor::Float64=#
    1.0, #=min_safe_distance_from_human::Float64=#
    0.0, #=min_safe_distance_from_obstacle::Float64=#
    1.0, #=radius_around_vehicle_goal::Float64=#
    1.0, #=max_risk_distance::Float64=#
    -100.0, #=human_collision_penalty::Float64=#
    -100.0, #=obstacle_collision_penalty::Float64=#
    1000.0, #=goal_reached_reward::Float64=#
    5, #=num_segments_in_one_time_step::Int64=#
    5.0, #=observation_discretization_length::Float64=#
    100, #=tree_search_max_depth::Int64=#
    50, #=num_scenarios::Int64=#
    2.0, #=d_near::Float64 = 0.5=#
    4.0, #=d_far::Float64 = 0.5=#
    0.4, #=es_pomdp_planning_time::Float64=#
    0.3, #=ls_pomdp_planning_time::Float64=#
    pi/4, #=pomdp_action_max_delta_heading_angle::Float64=#
    0.5, #=pomdp_action_delta_speed::Float64=#
    0.5, #=one_time_step::Float64=#
    0.1, #=simulator_time_step::Float64=#
    0.1, #=update_sensor_data_time_interval::Float64=#
    0.1, #=buffer_time::Float64=#
    10.0 #=MAX_TIME_LIMIT::Float64=#
)


#=
Weird collision seeds
0x5cb57f4a
0x62014574
27 => 0x62014574
52 => 0x031437d5
108 => 0xb2fb7736
121 => 0x93e372e2
=#
