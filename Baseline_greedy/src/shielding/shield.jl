include("shield_utils.jl")

# This is the function main code calls to find the safe action set and return the one with the best Q-value. 
function get_best_safe_action(m::ExtendedSpacePOMDP{P,N}, shield_utils, shield_radius_humans, vehicle, 
                    pomdp_tree, print_logs=false) where {P,N}

    action_set = view(pomdp_tree.ba_action,pomdp_tree.children[1])
    safe_action_found, best_q_value, num_best_safe_actions = identify_best_safe_actions(m, shield_utils, 
                        shield_radius_humans, vehicle, action_set, pomdp_tree)

    if(print_logs)
        println("Safe Action Flags : ", shield_utils.safe_actions)
    end
    if(!safe_action_found)
        println("ISSUE: no safe actions returned. THIS SHOULD NOT HAVE HAPPENED. BAAAADDDDD!")
        println("Vehicle : ", vehicle)
        return
    else
        if(print_logs)
            println("best_q_value = ", best_q_value)
            println("num_best_safe_actions = ", num_best_safe_actions)
        end
        (;best_safe_action_indices, shield_rng) = shield_utils
        best_action_index = rand(shield_rng, view(best_safe_action_indices,1:num_best_safe_actions))
        if(print_logs)
            println("best safe action = ", action_set[best_action_index])
        end
        return action_set[best_action_index]
    end
end

function get_best_safe_action(m::LimitedSpacePOMDP{P,N}, shield_utils, shield_radius_humans, vehicle, 
                    pomdp_tree, print_logs=false) where {P,N}

    action_set = view(pomdp_tree.ba_action,pomdp_tree.children[1])
    safe_action_found, best_q_value, num_best_safe_actions = identify_best_safe_actions(m, shield_utils, 
                        shield_radius_humans, vehicle, action_set, pomdp_tree)

    if(print_logs)
        println("Safe Action Flags : ", shield_utils.safe_actions)
    end
    if(!safe_action_found)
        if(print_logs)
            println("ISSUE: no safe actions returned. THIS SHOULD NOT HAVE HAPPENED. BAAAADDDDD!")
            println("Vehicle : ", vehicle)
        end
        return false,ActionLimitedSpacePOMDP(-m.vehicle_action_delta_speed)
    else
        if(print_logs)
            println("best_q_value = ", best_q_value)
            println("num_best_safe_actions = ", num_best_safe_actions)
        end
        (;best_safe_action_indices, shield_rng) = shield_utils
        best_action_index = rand(shield_rng, view(best_safe_action_indices,1:num_best_safe_actions))
        if(print_logs)
            println("best safe action = ", action_set[best_action_index])
        end
        return true,action_set[best_action_index]
    end
end

function identify_best_safe_actions(m, shield_utils, shield_radius_humans, vehicle, action_set, pomdp_tree)

    # Run the shielding code to identify safe actions
    num_actions = length(action_set) 
    generate_safe_action_set(m, shield_utils, shield_radius_humans, vehicle, action_set)
    
    if(sum(view(shield_utils.safe_actions,1:num_actions)) == 0)
        #No safe actions were found
        return false, -Inf, 0
    else
        # Find the safe action with best POMDP Q-value 
        (;safe_actions, best_safe_action_indices, shield_rng) = shield_utils
        best_q_value = -Inf
        num_best_safe_actions = 1

        for a_index in 1:num_actions
            if(safe_actions[a_index])
                curr_action_q_value = get_q_value(pomdp_tree, a_index)
                if curr_action_q_value > best_q_value
                    num_best_safe_actions = 1
                    best_safe_action_indices[num_best_safe_actions] = a_index
                    best_q_value = curr_action_q_value
                    num_best_safe_actions += 1
                elseif curr_action_q_value == best_q_value
                    best_safe_action_indices[num_best_safe_actions] = a_index
                    num_best_safe_actions += 1
                end
            end
        end
        return true, best_q_value, num_best_safe_actions-1
    end
end


#Same as ba_l in pomdps_glue.jl in ARDESPOT
function get_q_value(D, ba)
    #Sum of value from each child node
    q_value = 0.0
    for bnode in D.ba_children[ba]
        q_value += D.l[bnode]
    end
    q_value += D.ba_rho[ba]
    return q_value
end

#=
su = ShieldUtils(exp_details, pomdp_details, veh_params.wheelbase)
t = 43
so = output.sim_objects[t]
vsd = so.vehicle_sensor_data
# srh = output.nearby_humans[t]
srh = find_shield_radius_humans(su, extended_space_pomdp, so)
v = output.sim_objects[t].vehicle
tr = output.despot_trees[t-0.5][:tree]
get_best_safe_action(extended_space_pomdp, su, srh, v, tr)
@benchmark get_best_safe_action($extended_space_pomdp, $su, $srh, $v, $tr)
    
=#


#=
After profiling, I found two major functions that take up the most amount of time
1. check_collision
2. generate_FRS_points_nearby_humans

I can and should use multithreading to make both of them faster.
However, it is probably not necessary right now for the simulation code.
=#


#=
This is thew old version of the main shielding function for simulations with extended space planner

    
function get_best_safe_action(m, shield_utils, shield_radius_humans, vehicle, pomdp_tree, print_logs=false)
    
    # Run the shielding code to identify safe actions
    action_set = view(pomdp_tree.ba_action,pomdp_tree.children[1])
    num_actions = length(action_set) 
    generate_safe_action_set(m, shield_utils, shield_radius_humans, vehicle, action_set)
    if(print_logs)
        println("Safe Action Flags : ", shield_utils.safe_actions)
    end
    if(sum(view(shield_utils.safe_actions,1:num_actions)) == 0)
        println("ISSUE: no safe actions returned. THIS SHOULD NOT HAVE HAPPENED. BAAAADDDDD!")
        println("Vehicle : ", vehicle)
        return
    end

    # Find the safe action with best POMDP Q-value 
    (;safe_actions, best_safe_action_indices, shield_rng) = shield_utils
    best_q_value = -Inf
    num_best_safe_actions = 1

    for a_index in 1:num_actions
        if(safe_actions[a_index])
            curr_action_q_value = get_q_value(pomdp_tree, a_index)
            if curr_action_q_value > best_q_value
                num_best_safe_actions = 1
                best_safe_action_indices[num_best_safe_actions] = a_index
                best_q_value = curr_action_q_value
                num_best_safe_actions += 1
            elseif curr_action_q_value == best_q_value
                best_safe_action_indices[num_best_safe_actions] = a_index
                num_best_safe_actions += 1
            end
        end
    end
    println("best_q_value = ", best_q_value)
    println("best_safe_action_indices = ", best_safe_action_indices)
    best_action_index = rand(shield_rng, view(best_safe_action_indices,1:num_best_safe_actions-1))
    if(print_logs)
        println("best safe action = ", action_set[best_action_index])
    end
    return action_set[best_action_index]

end

=#