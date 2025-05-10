# shield_wrappers.jl

# main function for POMDP to call to return best safe action
function lsp_get_best_shielded_action(veh, nearby_humans, Dt_obs_to_k1, Dt_plan,
        get_actions::Function, veh_body, human_goal_positions, pomdp, despot,
        user_defined_rng)

    print_logs = false
    # reformat vehicle state
    x_k1 = SVector(veh.x, veh.y, wrap_between_negative_pi_to_pi(veh.theta), veh.v)

    # reformat human positions
    nbh_pos = Vector{Tuple{Float64,Float64}}()
    for h in nearby_humans
      push!(nbh_pos, (h.x, h.y))
    end

    # run shielding algorithm to produce shielded action set
    v_human_max = 1.25
    shield_actions, shield_action_indices = lsp_shield_action_set(x_k1, nbh_pos, Dt_obs_to_k1,
                    Dt_plan, get_actions, veh_body, human_goal_positions, v_human_max, pomdp)
                    
    if(print_logs)
        println("len(ia_safe_set) = ", length(shield_action_indices))
    end

    # TEST ONLY ---
    if length(shield_action_indices) == 0
        println("ISSUE: no safe actions returned. Need to use Old Path and Slow down on it")
        println("x_k1 = ", x_k1)
        return false, ActionLimitedSpacePOMDP(-0.5)
    end

    # find action with best POMDP Q-value in shielded action set
    best_q_value = -Inf
    best_action = []

    for i in axes(shield_action_indices, 1)
        l = get_q_value(despot, shield_action_indices[i])

        if l > best_q_value
            best_q_value = l
            best_action = [shield_actions[i]]
        elseif l == best_q_value
            push!(best_action, shield_actions[i])
        end
    end

    # draw random if multiple actions have same value
    user_defined_rng = MersenneTwister(77)
    best_a = rand(user_defined_rng, best_action)
    if(print_logs)
        println("best safe action = ", best_a)
    end

    return true,ActionLimitedSpacePOMDP(best_a)
end

# same as ba_l in pomdps_glue.jl in ARDESPOT
function get_q_value(D, ba)
    # sum value from each child node
    q_value = 0.0
    for bnode in D.ba_children[ba]
        q_value += D.l[bnode]
    end
    q_value += D.ba_rho[ba]

    return q_value
end


function lsp_shield_get_actions(x, Dt, veh, m)
    delta_speed = m.vehicle_action_delta_speed
    if(x[4] == 0.0)
        actions = (0.0,delta_speed)
        return actions, collect(1:length(actions))
    elseif(x[4] == m.max_vehicle_speed)
        actions = (-delta_speed,0.0)
        return actions, collect(1:length(actions))
    else
        actions = (-delta_speed,0.0,delta_speed)
        return actions, collect(1:length(actions))
    end
end

# t = 3.0;
# sv = output.sim_objects[t].vehicle;
# snbh = output.nearby_humans[t].position_data;
# s_Dt_plan = exp_details.one_time_step;
# s_veh_body = veh_body;
# sm = extended_space_pomdp;
# s_despot = output.despot_trees[t][:tree];
# s_rng = MersenneTwister(11);
#
# best_shield_action(sv,snbh,0.0,s_Dt_plan,get_shield_actions,s_veh_body,exp_details.human_goal_locations,sm,s_despot,s_rng)
