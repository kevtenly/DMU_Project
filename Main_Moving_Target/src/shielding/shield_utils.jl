include("../utils.jl")
include("../polygon_intersection.jl")
include("../ES_POMDP_Planner.jl")
include("../LS_POMDP_Planner.jl")

struct ShieldUtils{P,Q}
    max_human_speed::Float64
    one_time_step::Float64
    time_until_vehicle_action::Float64
    num_human_goals::Int
    num_steering_angles::Int
    shield_radius_humans::Array{HumanState,1}
    FRS_points_shield_radius_humans::Q
    human_goal_locations::Array{Location,1}
    ϕ_sequence_set::P
    safe_actions::Array{Bool,1}
    best_safe_action_indices::Array{Int,1}
    shield_rng::AbstractRNG
end 

function ShieldUtils(exp_details, pomdp_details, wheelbase)

    max_num_humans = exp_details.num_humans_env
    max_human_speed = 1.25 #Make this generic instead of hard coding
    num_human_goals = length(exp_details.human_goal_locations)
    max_steps_to_stop = Int(pomdp_details.max_vehicle_speed/pomdp_details.action_delta_speed)
    max_num_human_points = 1 + num_human_goals^(max_steps_to_stop+1)   
    FRS_points_shield_radius_humans = Array{SVector{2,Float64},3}(undef,max_num_humans,max_steps_to_stop+1,
                                                    max_num_human_points)
    shield_radius_humans = Array{HumanState,1}(undef,max_num_humans)

    #Generate all possible action sequences

    possible_steering_angles = compute_steering_angle_set(1.0, 1.0, 1.0, 1.0, 1.0) 
    num_steering_angles = length(possible_steering_angles)
    max_sequence_length = max_steps_to_stop
    num_different_speeds = Int(pomdp_details.max_vehicle_speed/pomdp_details.action_delta_speed)
    max_num_sequences = num_steering_angles^max_sequence_length
    ϕ_sequence_set = Array{SVector{max_sequence_length,Float64},2}(undef,num_different_speeds,max_num_sequences)
    for i in 1:size(ϕ_sequence_set,1)
        for j in 1:size(ϕ_sequence_set,2)
            ϕ_sequence_set[i,j] = SVector(repeat(SVector(Inf),max_sequence_length)...)
        end
    end
    (;action_delta_speed, action_max_delta_heading_angle, max_vehicle_steering_angle, one_time_step) = pomdp_details 
    # (;wheelbase) = vehicle_params
    populate_steering_angle_sequences!(ϕ_sequence_set,max_sequence_length,action_delta_speed,wheelbase,
                    action_max_delta_heading_angle, max_vehicle_steering_angle, one_time_step)

    
    max_num_actions = 12 #Make this generic instead of hard coding
    safe_actions = Array{Bool,1}( repeat(SVector(false),max_num_actions) )
    best_safe_action_indices = zeros(Int,max_num_actions)
    
    time_until_vehicle_action = 0.1 #Make this generic instead of hard coding

    return ShieldUtils(
                max_human_speed,
                one_time_step,
                time_until_vehicle_action,
                num_human_goals,
                num_steering_angles,
                shield_radius_humans,
                FRS_points_shield_radius_humans,
                exp_details.human_goal_locations,
                ϕ_sequence_set,
                safe_actions,
                best_safe_action_indices,
                MersenneTwister(111)
                )
end

function compute_steering_angle_set(vehicle_speed, vehicle_wheelbase, max_delta_angle, max_steering_angle, Dt)
    ϕ = clamp(get_steering_angle(vehicle_wheelbase, max_delta_angle, vehicle_speed, Dt), 0.0, max_steering_angle)
    # set = (-ϕ, -2/3*ϕ, -1/3*ϕ, 0.0, 1/3*ϕ, 2/3*ϕ, ϕ)
    set = (-ϕ, 0.0, ϕ)
    return set
end

function populate_steering_angle_sequences!(ϕ_sequence_set, max_sequence_length, delta_speed, vehicle_wheelbase,
    max_delta_angle, max_steering_angle, time_step)

    #=
    First, add values for lowest vehicle_speed, i.e. delta_speed. 
    This is a sequence with just one steering angle, with rest of them as Inf.
    =#
    ϕ_set = compute_steering_angle_set(delta_speed, vehicle_wheelbase, max_delta_angle, max_steering_angle, time_step)
    for i in 1:length(ϕ_set)
        new_sequence = SVector(ϕ_set[i], repeat(SVector(Inf),max_sequence_length-1)...)
        ϕ_sequence_set[1,i] = new_sequence
    end

    #=
    Now create steering angle sequences for other vehicle speeds by using slower vehicle speeds.
    =# 
    for i in 2:max_sequence_length
        vehicle_speed = i*delta_speed
        ϕ_set = compute_steering_angle_set(vehicle_speed, vehicle_wheelbase, max_delta_angle, max_steering_angle, time_step)
        previous_ϕ_set = ϕ_sequence_set[i-1,:]
        num_sequences_previous_ϕ_set = length(ϕ_set)^(i-1)
        for j in 1:length(ϕ_set)
            for k in 1:num_sequences_previous_ϕ_set
                ϕ_from_previous_set = previous_ϕ_set[k][1:i-1]
                new_sequence = (ϕ_set[j],ϕ_from_previous_set...,repeat(SVector(Inf),max_sequence_length-i)...)
                # println(i," ",j," ",k," ",new_sequence)
                # println(" ", ϕ_sequence_set[i,(j-1)*num_sequences_previous_ϕ_set+k])
                ϕ_sequence_set[i,(j-1)*num_sequences_previous_ϕ_set+k] = SVector(new_sequence)
            end
        end      
    end

end
#=
populate_steering_angle_sequences!(ϕ_sequence_set, 4, 0.5, 1.0, pi/4, pi, 0.5)
=#


function generate_FRS_points_shield_radius_humans(shield_utils, shield_radius_humans, num_steps)
    #= TBD
    Parallize this FRS generation for each human
    =#
    # (;position_data) = nearby_humans
    for human_index in 1:length(shield_radius_humans)
        human = shield_radius_humans[human_index]
        # human_index = ids[i]
        generate_FRS_points_human(shield_utils,(human.x,human.y),human_index,num_steps)
    end
end

function generate_FRS_points_human(shield_utils,human,human_index,num_time_steps)
    
    (;max_human_speed,one_time_step,time_until_vehicle_action,
    FRS_points_shield_radius_humans,human_goal_locations) = shield_utils

    #=
    Code to generate FRS points for the human for an initial time value of
    the difference between when the lidar data was collected and the time when the
    vehicle action will be executed.It is a stored parameter in shield_utils.
    =#
    t,i = 1,1
    for g in human_goal_locations
        new_human_point = move_human(human, max_human_speed, g, time_until_vehicle_action, 0.0)
        FRS_points_shield_radius_humans[human_index,t,i] = new_human_point
        i+=1
    end
    FRS_points_shield_radius_humans[human_index,t,i] = (Inf,Inf)

    #=
    Now compute the FRS of humans for the number of time steps that the vehicle
    needs to come to a stop.
    Note that:
    input num_time_steps = number of time steps the vehicle needs to come to a stop after applying an action + 1
    + 1 in the expression above is from the fact that we also store the FRS for the initial
    predicted position of the human after time_until_vehicle_action seconds from the moment when the
    human position was recorded.
    =#
    for t in 2:num_time_steps
        i = 1
        current_human_points = view(FRS_points_shield_radius_humans,human_index,t-1,:)
        for point in current_human_points
            if(point==SVector(Inf,Inf))
                break
            end
            for g in human_goal_locations
                # println(t, " ", i, " ", point, " ", g)
                new_human_point = move_human(point, max_human_speed, g, one_time_step, 0.0)
                FRS_points_shield_radius_humans[human_index,t,i] = new_human_point
                i+=1
            end
        end
        FRS_points_shield_radius_humans[human_index,t,i] = (Inf,Inf)
    end

end
#=
su = ShieldUtils(exp_details, pomdp_details, veh_params)
generate_FRS_points_human(su, (10.0,10.0), 1, 5)
@benchmark generate_FRS_points_human($su, (10.0,10.0), 1, 5)

=#

check_collision(x,y,r,z) = circle_polygon_intersection((x,y),r,z)


function find_shield_radius_humans(shield_utils, m, sim_obj)

    (;shield_radius_humans, max_human_speed) = shield_utils
    (;vehicle,humans) = sim_obj

    curr_max_vehicle_speed = clamp(vehicle.v+m.vehicle_action_delta_speed, 0.0, m.max_vehicle_speed)
    num_steps_to_stop = Int(curr_max_vehicle_speed/m.vehicle_action_delta_speed)
    #The value 1.0 added at the end in shield_radius is just a buffer value
    shield_radius = num_steps_to_stop*max_human_speed + m.min_safe_distance_from_human +
                sum(curr_max_vehicle_speed:-m.vehicle_action_delta_speed:0.0) + m.vehicle_R + 1.0
    println("Shield Radius : $shield_radius")
    srh_index = 1
    for i in 1:length(humans)
        human = humans[i]
        if(is_within_range(vehicle.x,vehicle.y,human.x,human.y,shield_radius))
            if(human.x!= human.goal.x || human.y!= human.goal.y)
                shield_radius_humans[srh_index] = human
                srh_index += 1
            end
        end
    end
   
    return @view shield_radius_humans[1:srh_index-1]
end
#=
su = ShieldUtils(exp_details, pomdp_details, veh_params)
t = 32.0
so = output.sim_objects[t]
find_shield_radius_humans(su, extended_space_pomdp, so)
@benchmark find_shield_radius_humans($su, $extended_space_pomdp, $so)

=#

function vehicle_can_reach_goal(m, vehicle)

    #=
    NOTE: Whenever this function is called, the starting vehicle speed has to be 0.0
    =#

    (;vehicle_wheelbase, max_vehicle_speed, one_time_step, vehicle_goal, world,
        vehicle_R, vehicle_D, vehicle_goal, radius_around_vehicle_goal,
        min_safe_distance_from_obstacle) = m
    max_num_steps = 100
    num_steps = 0
    curr_vehicle_state = vehicle
    curr_vehicle_speed = 0.0

    while(num_steps <= max_num_steps)
        vehicle_center_x = curr_vehicle_state[1] + vehicle_D*cos(curr_vehicle_state[3])
        vehicle_center_y = curr_vehicle_state[2] + vehicle_D*sin(curr_vehicle_state[3])
        #Check if vehicle center is inside the goal region
        if(is_within_range(vehicle_center_x,vehicle_center_y, vehicle_goal.x, vehicle_goal.y, 
                radius_around_vehicle_goal))
            return true
        end
        #Check if vehicle body is intersecting with the environment boundaries
        if( vehicle_center_x<0.0+vehicle_R || vehicle_center_y<0.0+vehicle_R || 
            vehicle_center_x>world.length-vehicle_R || vehicle_center_y>world.breadth-vehicle_R )
            return false
        end
        #Check if vehicle body is intersecting with any static obstacle
        for obstacle in world.obstacles
            if( is_within_range(vehicle_center_x,vehicle_center_y,obstacle.x,obstacle.y,
                obstacle.r+min_safe_distance_from_obstacle+vehicle_R) )
                # println("Collision with this obstacle " ,obstacle)
                # println(vehicle_center_x, " ", vehicle_center_y)
                return false
            end
        end
        state = SVector(curr_vehicle_state[1],curr_vehicle_state[2],wrap_between_negative_pi_to_pi(curr_vehicle_state[3]),
                    curr_vehicle_speed)
        ϕ,δv = BPDE.optimal_HJB_policy(m.rollout_guide, state)
        # println("Step Number : $num_steps; ϕ : $ϕ; δv: $δv")
        new_vehicle_speed = clamp(curr_vehicle_speed+δv, 0.0, m.max_vehicle_speed)
        new_vehicle_state = move_vehicle(curr_vehicle_state...,vehicle_wheelbase,ϕ,
                                        new_vehicle_speed,one_time_step)
        curr_vehicle_state = new_vehicle_state
        curr_vehicle_speed = new_vehicle_speed
        num_steps += 1
    end 
    return false
end
#=
vehicle_can_reach_goal(extended_space_pomdp, (10.0,10.0,0.0) )
vehicle_can_reach_goal(extended_space_pomdp, (35.0,15.0,0.0) )
@benchmark vehicle_can_reach_goal($extended_space_pomdp, (10.0,10.0,0.0) )

=#


function generate_safe_action_set(m::LimitedSpacePOMDP{P,N}, shield_utils, shield_radius_humans, vehicle, 
                action_set) where {P,N}

    (;vehicle_wheelbase, max_vehicle_speed, one_time_step, vehicle_R, vehicle_D, 
        min_safe_distance_from_human, vehicle_action_delta_speed, rollout_guide) = m
    (;controls_sequence) = rollout_guide

    max_next_step_vehicle_speed = clamp(vehicle.v+vehicle_action_delta_speed, 0.0, max_vehicle_speed)
    num_steps = 1 + Int(max_next_step_vehicle_speed/vehicle_action_delta_speed)
    generate_FRS_points_shield_radius_humans(shield_utils, shield_radius_humans, num_steps)
    (;ϕ_sequence_set,FRS_points_shield_radius_humans,safe_actions,
        num_human_goals) = shield_utils

    for a_index in 1:length(action_set)

        a = action_set[a_index]
        # println("Action: ",a)
        curr_vehicle_speed = clamp(vehicle.v+a.delta_speed, 0.0, max_vehicle_speed)

        if(curr_vehicle_speed == 0.0)
            safe_actions[a_index] = true
            continue
        end

        #=
        1) Propogate vehicle with current action.
        2) Check for collision between the new vehicle position and corresponding FRS set at that time.
            a) If there is collision, mark that action as unsafe.
        3) If there is no collision, time to start deceleration and see if it is possible to stop without
        colliding with any human's FRS. 
            a) If there is no collision for all time steps until the vehicle stops, mark that action 
                as safe, else unsafe
        =#

        path_start_index = 1
        num_segments_in_one_time_step = Int(curr_vehicle_speed/vehicle_action_delta_speed)
        num_segments = min(num_segments_in_one_time_step, length(controls_sequence)-path_start_index+1)
        time_duration_per_segment = one_time_step/num_segments_in_one_time_step
        curr_vehicle_state =  move_vehicle_over_astar_path((vehicle.x,vehicle.y,vehicle.theta),curr_vehicle_speed,
                vehicle_wheelbase,controls_sequence,path_start_index,num_segments,time_duration_per_segment)
        path_start_index += num_segments

        action_safe = true
        t = 2  
        #=
        t is going to refer to the time index of the FRS set for the human after it has moved for one time step.
        It is initialized to 2 because the FRS set at time index 1 is the initial FRS set of the human.
        =#

        while true
            vehicle_center_x = curr_vehicle_state[1] + vehicle_D*cos(curr_vehicle_state[3])
            vehicle_center_y = curr_vehicle_state[2] + vehicle_D*sin(curr_vehicle_state[3])
            vehicle_radius = vehicle_R + min_safe_distance_from_human
            num_human_points = num_human_goals^t
            for human_index in 1:length(shield_radius_humans)
                #These human points correspond to the verices of the polygon of the FRS of that human
                human_points = view(FRS_points_shield_radius_humans,human_index,t,1:num_human_points)
                #Check collision between this FRS polygon and the circular vehicle body
                collision = check_collision(vehicle_center_x,vehicle_center_y,vehicle_radius,human_points)
                if(collision)
                    action_safe = false
                    break
                end
            end
            if(action_safe==false)
                break
            end
            curr_vehicle_speed = clamp(curr_vehicle_speed-vehicle_action_delta_speed, 0.0, max_vehicle_speed)
            if(curr_vehicle_speed==0.0)
                break
            end
            num_segments_in_one_time_step = Int(curr_vehicle_speed/vehicle_action_delta_speed)
            num_segments = min(num_segments_in_one_time_step, length(controls_sequence)-path_start_index+1)
            time_duration_per_segment = one_time_step/num_segments_in_one_time_step
            curr_vehicle_state =  move_vehicle_over_astar_path(curr_vehicle_state,curr_vehicle_speed,
                vehicle_wheelbase,controls_sequence,path_start_index,num_segments,time_duration_per_segment)
            path_start_index += num_segments
            t += 1
        end

        safe_actions[a_index] = action_safe
    end
end
#=
su = ShieldUtils(exp_details, pomdp_details, veh_params)
t = 32.0
# nh = output.nearby_humans[t]
# nh = NearbyHumans(vsd.lidar_data,vsd.ids,vsd.belief)
vsd = output.sim_objects[t].vehicle_sensor_data
nh = vsd.lidar_data
v = output.sim_objects[t].vehicle
as = view(output.despot_trees[t][:tree].ba_action,output.despot_trees[t][:tree].children[1]) 
generate_safe_action_set(extended_space_pomdp, su, nh, v, as)
@benchmark generate_safe_action_set($extended_space_pomdp, $su, $nh, $v, $as)

=#


function move_vehicle_over_astar_path(vehicle,vehicle_speed,vehicle_wheelbase,controls_sequence, 
                    path_start_index,num_segments,time_duration_per_segment)

    curr_vehicle_state = vehicle
    for i in path_start_index:path_start_index+num_segments-1
        steering_angle = controls_sequence[i]
        # println(" SA: ", steering_angle)
        curr_vehicle_state = move_vehicle(curr_vehicle_state...,vehicle_wheelbase,
                                        steering_angle,vehicle_speed,time_duration_per_segment)
    end
    return curr_vehicle_state
end


function static_obstacle_collision(m,vehicle_center_x,vehicle_center_y)
    #=
    Check if the vehicle body is intersecting with any static obstacle or boundary wall
    =#
    (;vehicle_R,world,min_safe_distance_from_obstacle) = m

    if( vehicle_center_x<0.0+vehicle_R || vehicle_center_y<0.0+vehicle_R || 
        vehicle_center_x>world.length-vehicle_R || vehicle_center_y>world.breadth-vehicle_R )
        return true
    end

    for obstacle in world.obstacles
        if( is_within_range(vehicle_center_x,vehicle_center_y,obstacle.x,obstacle.y,
            obstacle.r+min_safe_distance_from_obstacle+vehicle_R) )
            return true
        end
    end

    return false 
end


function generate_safe_action_set(m::ExtendedSpacePOMDP{P,N}, shield_utils, shield_radius_humans, 
                vehicle, action_set) where {P,N}

    (;vehicle_wheelbase, max_vehicle_speed, one_time_step, vehicle_R, vehicle_D, 
        min_safe_distance_from_human, vehicle_action_delta_speed) = m

    max_next_step_vehicle_speed = clamp(vehicle.v+vehicle_action_delta_speed, 0.0, max_vehicle_speed)
    num_steps = 1 + Int(max_next_step_vehicle_speed/vehicle_action_delta_speed)
    generate_FRS_points_shield_radius_humans(shield_utils, shield_radius_humans, num_steps)
    (;ϕ_sequence_set,FRS_points_shield_radius_humans,safe_actions,
        num_human_goals,num_steering_angles) = shield_utils

    for a_index in 1:length(action_set)

        a = action_set[a_index]
        # println("Action: ",a)
        ϕ = a.steering_angle
        new_vehicle_speed = clamp(vehicle.v+a.delta_speed, 0.0, max_vehicle_speed)
        if(ϕ == -10.0 || new_vehicle_speed == 0.0)
            safe_actions[a_index] = true
            continue
        end

        #=
        1) Propogate vehicle with current action.
        2) Check for collision between the new vehicle position and corresponding FRS set at that time.
            a) If there is collision, mark that action as unsafe.
        3) If there is no collision, time to start deceleration and see if it is possible to stop without
        colliding with any human's FRS. 
            a) If there is no collision for all time steps until the vehicle stops, mark that action 
                as safe, else unsafe
        =#

        new_vehicle_state =  move_vehicle(vehicle.x,vehicle.y,vehicle.theta,vehicle_wheelbase,ϕ,
                                        new_vehicle_speed,one_time_step)
        #Check if the vehicle is in the FRS of any human after first time step
        action_safe = true
        t = 2
        # println("T: ",t)
        vehicle_center_x = new_vehicle_state[1] + vehicle_D*cos(new_vehicle_state[3])
        vehicle_center_y = new_vehicle_state[2] + vehicle_D*sin(new_vehicle_state[3])
        vehicle_radius = vehicle_R + min_safe_distance_from_human
        num_human_points = num_human_goals^t
        for human_index in 1:length(shield_radius_humans)
            #These human points correspond to the verices of the polygon of the FRS of that human
            human_points = view(FRS_points_shield_radius_humans,human_index,t,1:num_human_points)
            #Check collision between this FRS polygon and the circular vehicle body
            collision = check_collision(vehicle_center_x,vehicle_center_y,vehicle_radius,human_points)
            # println("Collision: ",collision)
            if(collision)
                action_safe = false
                break
            end
        end
        if(action_safe==false || static_obstacle_collision(m,vehicle_center_x,vehicle_center_y))
            safe_actions[a_index] = false
            continue
        end

        #Start decelerating and check if the vehicle can stop without colliding with any human's FRS
        num_steps_until_stop = Int(new_vehicle_speed/vehicle_action_delta_speed) - 1
        if(num_steps_until_stop==0)
            safe_actions[a_index] = true
            continue
        end
        num_sequences = num_steering_angles^num_steps_until_stop
        deceleration_action_sequence_set = view(ϕ_sequence_set,num_steps_until_stop,1:num_sequences)
        for action_sequence in deceleration_action_sequence_set
            # println("Action sequence set: ",action_sequence)  
            curr_vehicle_state = new_vehicle_state
            curr_vehicle_speed = new_vehicle_speed
            curr_action_sequence_safe = true
            i,t = 1,3
            while true
                # println("T: ",t)
                curr_vehicle_speed = clamp(curr_vehicle_speed-vehicle_action_delta_speed, 0.0, max_vehicle_speed)
                if(curr_vehicle_speed==0.0)
                    break
                end
                curr_ϕ = action_sequence[i]
                @assert curr_ϕ!=Inf "Tried to access wrong index in the ϕ sequence. Find out why."
                curr_vehicle_state = move_vehicle(curr_vehicle_state...,vehicle_wheelbase,curr_ϕ,
                                        curr_vehicle_speed,one_time_step)
                vehicle_center_x = curr_vehicle_state[1] + vehicle_D*cos(curr_vehicle_state[3])
                vehicle_center_y = curr_vehicle_state[2] + vehicle_D*sin(curr_vehicle_state[3])
                vehicle_radius = vehicle_R + min_safe_distance_from_human
                num_human_points = num_human_goals^t
                for human_index in 1:length(shield_radius_humans)
                    #These human points correspond to the verices of the polygon of the FRS of that human
                    human_points = view(FRS_points_shield_radius_humans,human_index,t,1:num_human_points)
                    #Check collision between this FRS polygon and the circular vehicle body
                    collision = check_collision(vehicle_center_x,vehicle_center_y,vehicle_radius,human_points)
                    # println("Collision: ",collision)
                    if(collision)
                        curr_action_sequence_safe = false
                        break
                    end
                end
                if(curr_action_sequence_safe==false || static_obstacle_collision(m,vehicle_center_x,vehicle_center_y))
                    action_safe = false
                    break
                end
                i += 1
                t += 1
            end
            if(curr_action_sequence_safe==true)
                #Check if the vehicle can reach the goal from this point
                if(vehicle_can_reach_goal(m, curr_vehicle_state))
                    action_safe = true
                    break
                end                
            end
        end

        safe_actions[a_index] = action_safe
    end
end
#=
su = ShieldUtils(exp_details, pomdp_details, veh_params.wheelbase)
t = 20.5
# nh = output.nearby_humans[t]
# nh = NearbyHumans(vsd.lidar_data,vsd.ids,vsd.belief)
vsd = output.sim_objects[t].vehicle_sensor_data
nh = vsd.lidar_data
v = output.sim_objects[t].vehicle
despot_tree_t = t-0.5
as = view(output.despot_trees[despot_tree_t][:tree].ba_action,output.despot_trees[despot_tree_t][:tree].children[1]) 
generate_safe_action_set(extended_space_pomdp, su, nh, v, as)
su.safe_actions
@benchmark generate_safe_action_set($extended_space_pomdp, $su, $nh, $v, $as)

=#


#=
Old version of the function for ExtendedSpacePOMDP planning.

function generate_safe_action_set(m, shield_utils, shield_radius_humans, vehicle, action_set)

    (;vehicle_wheelbase, max_vehicle_speed, one_time_step, vehicle_R, vehicle_D, 
        min_safe_distance_from_human, vehicle_action_delta_speed) = m
    # (;position_data) = nearby_humans

    max_next_step_vehicle_speed = clamp(vehicle.v+vehicle_action_delta_speed, 0.0, max_vehicle_speed)
    num_steps = 1 + Int(max_next_step_vehicle_speed/vehicle_action_delta_speed)
    generate_FRS_points_shield_radius_humans(shield_utils, shield_radius_humans, num_steps)
    (;ϕ_sequence_set,FRS_points_shield_radius_humans,safe_actions,
        num_human_goals,num_steering_angles) = shield_utils

    for a_index in 1:length(action_set)

        a = action_set[a_index]
        # println("Action: ",a)
        ϕ = a.steering_angle
        new_vehicle_speed = clamp(vehicle.v+a.delta_speed, 0.0, max_vehicle_speed)

        if(ϕ == -10.0 || new_vehicle_speed == 0.0)
            safe_actions[a_index] = true
            continue
        end

        #=
        1) Propogate vehicle.
        2) Get vehicle body at this new position.
        3) Propogate all nearby humans and compute the FRS.
        4) Check if the vehicle body is in the FRS of any human.
            a) If it is, mark that action as unsafe.
        5) If it is not, check if there is a viable path from that point to vehicle goal.
            a) If there is, mark that action as safe, else unsafe
        =#

        new_vehicle_state =  move_vehicle(vehicle.x,vehicle.y,vehicle.theta,vehicle_wheelbase,ϕ,
                                        new_vehicle_speed,one_time_step)
        num_steps_until_stop = Int(new_vehicle_speed/vehicle_action_delta_speed)
        num_sequences = num_steering_angles^num_steps_until_stop
        action_sequence_set = view(ϕ_sequence_set,num_steps_until_stop,1:num_sequences)
        action_safe = false

        #=
        TBD : Need to do a collision check right here between humans and the vehicle before 
        the deceleration starts.
        =# 

        for action_sequence in action_sequence_set
            # println("Action sequence set: ",action_sequence)  
            curr_vehicle_state = new_vehicle_state
            curr_vehicle_speed = new_vehicle_speed
            curr_action_sequence_safe = true
            for t in 1:num_steps_until_stop
                # println("Time step: ",t)
                curr_ϕ = action_sequence[t]
                @assert curr_ϕ!=Inf "Tried to access wrong index in the ϕ sequence. Find out why."
                curr_vehicle_speed = clamp(curr_vehicle_speed-vehicle_action_delta_speed, 0.0, max_vehicle_speed)
                curr_vehicle_state = move_vehicle(curr_vehicle_state...,vehicle_wheelbase,curr_ϕ,
                                        curr_vehicle_speed,one_time_step)
                vehicle_center_x = curr_vehicle_state[1] + vehicle_D*cos(curr_vehicle_state[3])
                vehicle_center_y = curr_vehicle_state[2] + vehicle_D*sin(curr_vehicle_state[3])
                vehicle_radius = vehicle_R + min_safe_distance_from_human
                num_human_points = num_human_goals^t
                for human_index in 1:length(shield_radius_humans)
                    # human_index = ids[i]
                    #These human points correspond to the verices of the polygon of the FRS of that human
                    human_points = view(FRS_points_shield_radius_humans,human_index,t,1:num_human_points)
                    #Check collision between this FRS polygon and the circular vehicle body
                    collision = check_collision(vehicle_center_x,vehicle_center_y,vehicle_radius,human_points)
                    # println("Collision: ",collision)
                    if(collision)
                        curr_action_sequence_safe = false
                        break
                    end
                end
                if(curr_action_sequence_safe==false)
                    break
                end
            end
            if(curr_action_sequence_safe==true)
                #Check if the vehicle can reach the goal from this point
                if(vehicle_can_reach_goal(m, curr_vehicle_state))
                    action_safe = true
                    break
                end                
            end
        end

        safe_actions[a_index] = action_safe
    end

end
#=
su = ShieldUtils(exp_details, pomdp_details, veh_params.wheelbase)
t = 32.0
# nh = output.nearby_humans[t]
# nh = NearbyHumans(vsd.lidar_data,vsd.ids,vsd.belief)
vsd = output.sim_objects[t].vehicle_sensor_data
nh = vsd.lidar_data
v = output.sim_objects[t].vehicle
as = view(output.despot_trees[t][:tree].ba_action,output.despot_trees[t][:tree].children[1]) 
generate_safe_action_set(extended_space_pomdp, su, nh, v, as)
@benchmark generate_safe_action_set($extended_space_pomdp, $su, $nh, $v, $as)

=#

=#
