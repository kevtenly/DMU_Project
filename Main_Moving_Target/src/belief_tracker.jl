include("utils.jl")

#Functions for tracking belief

#This function returns the distance of a human's position to all the possible goal locations
function calculate_human_dist_from_all_goals(human_position,list_of_goals)
    distance_list = Float64[]
    for goal in list_of_goals
        distance = ((human_position.x-goal.x)^2 + (human_position.y-goal.y)^2)^0.5
        push!(distance_list,distance)
    end
    return distance_list
end

function is_human_present_in_lidar_data(human,old_lidar_data)
    for human_old_lidar_data_index in 1:length(old_lidar_data)
        if(human.id == old_lidar_data[human_old_lidar_data_index].id)
            return human_old_lidar_data_index
        end
    end
    return -1
end

function update_belief(old_sensor_data::VehicleSensor, new_human_states, new_ids, all_goals_list)
    new_belief = Array{HumanGoalsBelief,1}()
    old_human_states = old_sensor_data.lidar_data
    old_ids = old_sensor_data.ids
    old_belief = old_sensor_data.belief
    num_goals = length(all_goals_list)

    for i in 1:length(new_human_states)
        #=
        Check if new id is present in old ids list.
        If not, then initialize new belief to uniform distribution for this human.
        If yes, then find the index at which it is. Use that index to get the old belief and generate new belief.
        Push the new belief in the list!
        =#
        id_tbf = new_ids[i]
        id_position_in_old_ids = findall(x->x==id_tbf, old_ids)
        if(length(id_position_in_old_ids) == 0)
            belief_new_human = HumanGoalsBelief(repeat([1/num_goals], num_goals))
            push!(new_belief,belief_new_human)
        else
            required_index = id_position_in_old_ids[1]
            required_belief = old_belief[required_index].pdf
            required_human_from_old_list = old_human_states[required_index]
            required_human_from_new_list = new_human_states[i]
            old_distance_from_all_goals = calculate_human_dist_from_all_goals(required_human_from_old_list,all_goals_list)
            new_distance_from_all_goals = calculate_human_dist_from_all_goals(required_human_from_new_list,all_goals_list)
            human_prob_over_goals = old_distance_from_all_goals .- new_distance_from_all_goals
            minimum_unnormalized_value = abs(minimum(human_prob_over_goals))
            for i in 1:length(human_prob_over_goals)
                human_prob_over_goals[i] += (minimum_unnormalized_value + 1)
            end
            #human_prob_over_goals_list = broadcast(x-> x+1+abs(minimum(human_prob_over_goals_list)),human_prob_over_goals_list)
            updated_belief_for_current_human = required_belief.*human_prob_over_goals
            updated_belief_for_current_human = updated_belief_for_current_human/sum(updated_belief_for_current_human)
            #@show(updated_belief_for_current_human)
            push!(new_belief,HumanGoalsBelief(updated_belief_for_current_human))
        end
    end
    return new_belief
end

function update_belief(current_belief::Array{HumanGoalsBelief,1},all_goals_list,old_lidar_data,new_lidar_data)
    #@show("INSIDE",current_belief, old_cart_lidar_data, new_cart_lidar_data,"*****")
    new_belief = Array{HumanGoalsBelief,1}()
    human_positions_old = old_lidar_data[1]
    index_list_old = old_lidar_data[2]
    human_positions_new = new_lidar_data[1]
    index_list_new = new_lidar_data[2]

    for i in 1:length(index_list_new)

        #=
        Check if new index is present in old list.
        If not, then initialize new belief to uniform distribution for this human.
        If yes, then find the index at which it is. Use that index to get the old belief and generate new belief.
        Push the new belief in the list!
        =#
        index_new = index_list_new[i]
        index_new_position_in_old_list = findall(x->x==index_new, index_list_old)
        if(length(index_new_position_in_old_list) == 0)
            num_goals = length(all_goals_list)
            belief_over_new_human = belief_over_human_goals(repeat([1/num_goals], num_goals))
            push!(new_belief,belief_over_new_human)
        else
            required_index = index_new_position_in_old_list[1]
            required_belief = current_belief[required_index].pdf
            required_human_from_old_list = human_positions_old[required_index]
            required_human_from_new_list = human_positions_new[i]
            old_distance_from_all_goals = calculate_human_dist_from_all_goals(required_human_from_old_list,all_goals_list)
            new_distance_from_all_goals = calculate_human_dist_from_all_goals(required_human_from_new_list,all_goals_list)
            human_prob_over_goals = old_distance_from_all_goals .- new_distance_from_all_goals
            minimum_unnormalized_value = abs(minimum(human_prob_over_goals))
            for i in 1:length(human_prob_over_goals)
                human_prob_over_goals[i] += (minimum_unnormalized_value + 1)
            end
            #human_prob_over_goals_list = broadcast(x-> x+1+abs(minimum(human_prob_over_goals_list)),human_prob_over_goals_list)
            updated_belief_for_current_human = required_belief.*human_prob_over_goals
            updated_belief_for_current_human = updated_belief_for_current_human/sum(updated_belief_for_current_human)
            #@show(updated_belief_for_current_human)
            push!(new_belief,belief_over_human_goals(updated_belief_for_current_human))
        end
    end
    return new_belief
end

# temp_old_lidar_data = [env.humans[1], env.humans[3], env.humans[5]]
# temp_current_belief = [human_probability_over_goals([0.25, 0.25, 0.25, 0.25]),
# human_probability_over_goals([0.25, 0.25, 0.25, 0.25]),
# human_probability_over_goals([0.25, 0.25, 0.25, 0.25])]
# temp_new_lidar_data = [get_new_human_position_actual_environemnt(env.humans[1],env,1),
#     get_new_human_position_actual_environemnt(env.humans[2],env,1),
#     get_new_human_position_actual_environemnt(env.humans[3],env,1),
#     get_new_human_position_actual_environemnt(env.humans[4],env,1),
#     get_new_human_position_actual_environemnt(env.humans[5],env,1)]

# @code_warntype update_belief(temp_current_belief, env.goals,temp_old_lidar_data,temp_new_lidar_data)
#lala = update_belief(temp_current_belief, env.goals,temp_old_lidar_data,temp_new_lidar_data)
#lala = @code_warntype update_belief([], env.goals,[],temp_new_lidar_data)
