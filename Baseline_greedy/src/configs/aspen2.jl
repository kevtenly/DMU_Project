input_config =  InputParameters(
    MersenneTwister(), #=rng::AbstractRNG=#
    15.518, #=env_length::Float64=#
    21.036, #=env_breadth::Float64=#
    ObstacleLocation[ObstacleLocation(11.8,6.4,2.5),ObstacleLocation(8.7,13.5,3.5)], #=obstacles::Array{obstacle_location,1}=#
    20, #=num_humans::Int64=#
    1.0, #=human_start_v::Float64=#
    2.75, #=veh_start_x::Float64=#
    1.5, #=veh_start_y::Float64=#
    0.5*pi, #=veh_start_theta::Float64=#
    0.0, #=veh_start_v::Float64=#
    12.759, #=veh_goal_x::Float64=#
    20.5, #=veh_goal_y::Float64=#
    0.324, #=veh_wheelbase::Float64=#
    0.5207, #=veh_length::Float64=#
    0.2762, #=veh_breadth::Float64=#
    0.1715, #=veh_dist_origin_to_center::Float64=#
    2.0, #=veh_max_speed::Float64=#
    0.475, #=veh_max_steering_angle=#
    0.5, #=veh_path_planning_v::Float64=#
    30.0, #=lidar_range::Float64=#
    6, #=num_nearby_humans::Int64=#
    2*pi/3, #=cone_half_angle::Float64=#
    0.98, #=pomdp_discount_factor::Float64=#
    0.5, #=min_safe_distance_from_human::Float64=#
    0.1, #=min_safe_distance_from_obstacle::Float64=#
    1.0, #=radius_around_vehicle_goal::Float64=#
    0.5, #=max_risk_distance::Float64=#
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
    50.0 #=MAX_TIME_LIMIT::Float64=#
)
