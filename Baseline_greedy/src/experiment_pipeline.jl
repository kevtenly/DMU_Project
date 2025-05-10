include("main.jl")
import Serialization as SER

mutable struct PipelineIndividualOutput
    number_sudden_stops::Int64
    number_risky_scenarios::Int64
    time_taken::Float64
    vehicle_body::VPolygon
    vehicle_ran_into_boundary_wall::Bool
    vehicle_ran_into_obstacle::Bool
    vehicle_reached_goal::Bool
    vehicle_expected_trajectory::OrderedDict
    nearby_humans::OrderedDict
    vehicle_actions::OrderedDict
    sim_objects::OrderedDict
    risky_scenarios::OrderedDict
    despot_trees::OrderedDict
end

function PipelineIndividualOutput(output,save_trees=false)
    if(save_trees)
        return PipelineIndividualOutput(
            output.number_sudden_stops,#::Int64
            output.number_risky_scenarios,#::Int64
            output.time_taken,#::Float64
            output.vehicle_body,#::VehicleBody
            output.vehicle_ran_into_boundary_wall,#::Bool
            output.vehicle_ran_into_obstacle,#::Bool
            output.vehicle_reached_goal,#::Bool
            output.vehicle_expected_trajectory,#::OrderedDict
            output.nearby_humans,#::OrderedDict
            output.vehicle_actions,#::OrderedDict
            output.sim_objects,#::OrderedDict
            output.risky_scenarios,#::OrderedDict
            output.despot_trees,#::OrderedDict
        )
    else
        return PipelineIndividualOutput(
            output.number_sudden_stops,#::Int64
            output.number_risky_scenarios,#::Int64
            output.time_taken,#::Float64
            output.vehicle_body,#::VehicleBody
            output.vehicle_ran_into_boundary_wall,#::Bool
            output.vehicle_ran_into_obstacle,#::Bool
            output.vehicle_reached_goal,#::Bool
            output.vehicle_expected_trajectory,#::OrderedDict
            output.nearby_humans,#::OrderedDict
            output.vehicle_actions,#::OrderedDict
            output.sim_objects,#::OrderedDict
            output.risky_scenarios,#::OrderedDict
            OrderedDict()#::OrderedDict
        )
    end
end

struct PipelineOutput{I,R}
    environment_name::String
    num_experiments::Int64
    num_humans::Int64
    sudden_break_flag::Bool
    run_shield_flag::Bool
    run_with_trials::Bool
    seeds::Array{UInt32,1}
    input_config::I
    rollout_guide::R
    results_folder::String
end

function PipelineOutput(env_name, num_experiments, num_humans, sudden_break, run_shield,
                        run_with_trials, config, rollout_guide, results_folder, seeds=nothing)

    if(isnothing(seeds))
        seeds = [rand(UInt32) for i in 1:num_experiments]
    else
        @assert length(seeds) == num_experiments
    end

    return PipelineOutput(
        env_name,
        num_experiments,
        num_humans,
        sudden_break,
        run_shield,
        run_with_trials,
        seeds,
        config,
        rollout_guide,
        results_folder
    )
end

function run_pipeline(output_obj; lsp=false, esp_r=false, esp_sl=false, esp_hjb=true, 
                        print_logs=true, run_parallel=false)

    (;environment_name,num_experiments,num_humans,sudden_break_flag,run_shield_flag, 
    run_with_trials,seeds,input_config,rollout_guide,results_folder) = output_obj
    input_config.num_humans_env = num_humans

    if(run_with_trials)
        results_folder = results_folder*"With_trials/"
    else
        results_folder = results_folder*"With_time/"
    end

    base_folder = results_folder*"$(environment_name)/SB_$(sudden_break_flag)_Shield_$(run_shield_flag)/"*
                        "humans_$(num_humans)/experiments_$(num_experiments)/"
    if(!isdir(base_folder))
        println("Folder didn't exist. Creating directory : ", base_folder)
        mkpath(base_folder)
    end

    try
        # Threads.@threads for i in 1:num_experiments
        for i in 1:num_experiments
            seed = seeds[i]
            # RG = deepcopy(rollout_guide)

            if(lsp)
                IC = deepcopy(input_config) #To ensure the rng doesn't get modified between experiments when multi-threading
                IC.rng = MersenneTwister(seed)
                println("************ Running Experiment Number : ", i, " with LS Planner ************")
                l = run_limited_space_planner_experiment(IC,sudden_break_flag,run_shield_flag,run_with_trials,print_logs);
                lsp_result = PipelineIndividualOutput(l);
                save_location = base_folder*"lsp_exp_$i.h5"
                # file_obj = HDF5.h5open(save_location, "w");
                # if(haskey(file_obj, "output"))
                #     println("File already exists. Overwriting the file : ", save_location)
                #     HDF5.delete_object(file_obj, "output")
                # end
                # HDF5.write(save_location, "output", lsp_result);
                file_obj = SER.open(save_location, "w");
                SER.serialize(file_obj, lsp_result);
                close(file_obj);
            end


            if(esp_r)
                IC = deepcopy(input_config) #To ensure the rng doesn't get modified between experiments when multi-threading
                IC.rng = MersenneTwister(seed)
                println("************ Running Experiment Number : ", i, " with ES Planner - Random Rollouts ************")
                e = run_extended_space_planner_experiment_random_rollout(IC, rollout_guide,sudden_break_flag,
                                                            run_shield_flag,run_with_trials,print_logs);
                esp_random_result = PipelineIndividualOutput(e)
                save_location = base_folder*"esp_r_exp_$i.h5"
                # file_obj = HDF5.h5open(save_location, "w");
                # if(haskey(file_obj, "output"))
                #     println("File already exists. Overwriting the file : ", save_location)
                #     HDF5.delete_object(file_obj, "output")
                # end
                # HDF5.write(save_location, "output", esp_random_result);
                # close(file_obj);
                file_obj = SER.open(save_location, "w");
                SER.serialize(file_obj, esp_random_result);
                close(file_obj);
            end


            if(esp_sl)
                IC = deepcopy(input_config) #To ensure the rng doesn't get modified between experiments when multi-threading
                IC.rng = MersenneTwister(seed)
                println("************ Running Experiment Number : ", i, " with ES Planner - Straight Line Rollouts ************")
                e = run_extended_space_planner_experiment_straight_line_rollout(IC,rollout_guide,sudden_break_flag,
                                                            run_shield_flag,run_with_trials,print_logs);
                esp_sl_result = PipelineIndividualOutput(e)
                save_location = base_folder*"esp_sl_exp_$i.h5"
                # file_obj = HDF5.h5open(save_location, "w");
                # if(haskey(file_obj, "output"))
                #     println("File already exists. Overwriting the file : ", save_location)
                #     HDF5.delete_object(file_obj, "output")
                # end
                # HDF5.write(save_location, "output", esp_sl_result);
                # close(file_obj);
                file_obj = SER.open(save_location, "w");
                SER.serialize(file_obj, esp_sl_result);
                close(file_obj);
            end


            if(esp_hjb)
                IC = deepcopy(input_config)
                IC.rng = MersenneTwister(seed)
                println("************ Running Experiment Number : ", i, " with ES Planner - HJB Rollouts ************")
                e = run_extended_space_planner_experiment_HJB_rollout(IC,rollout_guide,sudden_break_flag,
                                                            run_shield_flag,run_with_trials,print_logs)
                esp_hjb_result = PipelineIndividualOutput(e)
                save_location = base_folder*"esp_hjb_exp_$i.h5"
                # file_obj = HDF5.h5open(save_location, "w");
                # if(haskey(file_obj, "output"))
                #     println("File already exists. Overwriting the file : ", save_location)
                #     HDF5.delete_object(file_obj, "output")
                # end
                # HDF5.write(file_obj, "output", esp_hjb_result);
                # close(file_obj);
                file_obj = SER.open(save_location, "w");
                SER.serialize(file_obj, esp_hjb_result);
                close(file_obj);                
            end

            GC.gc()
        end
    catch ex
        println("The experiment pipeline failed :(")
        println(ex)
    end
end


function get_results(output_obj,experiment_type,result_struct_key,base_folder=nothing)

    if(isnothing(base_folder))
        (;environment_name,num_experiments,num_humans,sudden_break_flag,run_shield_flag, 
            run_with_trials,results_folder) = output_obj

        if(run_with_trials)
            results_folder = results_folder*"With_trials/"
        else
            results_folder = results_folder*"With_time/"
        end
        base_folder = results_folder*"$(environment_name)/SB_$(sudden_break_flag)_Shield_$(run_shield_flag)/"*
                        "humans_$(num_humans)/experiments_$(num_experiments)/"
        if(!isdir(base_folder))
            println("Folder doesn't exist. Something is wrong. Please check : ", base_folder)
            return 0.0
        end
        filename = base_folder*"$(experiment_type)_exp_"
    else
        indices = findfirst("experiments_", base_folder)
        string_num_experiments = base_folder[indices[end]+1:end-1]
        num_experiments = parse(Int,string_num_experiments)
        filename = base_folder*"$(experiment_type)_exp_"
    end

    if(result_struct_key == :sb)
        result_struct_key = :number_sudden_stops
    elseif(result_struct_key == :c)
        result_struct_key = :number_collisions
    elseif(result_struct_key == :t)
        result_struct_key = :time_taken
    end

    num_successful_trials = 0
    unsuccesful_trial_ids = Array{Int64,1}()
    num_collision_free_trials = 0
    collision_trial_ids = Array{Int64,1}()
    array_desired_value = ones(num_experiments)*-1.0

    for i in 1:num_experiments
        save_location = filename*"$i.h5"
        file_obj = SER.open(save_location, "r");
        result = SER.deserialize(file_obj);
        close(file_obj);
        # println("For experiment $i, Goal: $(result.vehicle_reached_goal), 
        #             Obstacle: $(result.vehicle_ran_into_obstacle), 
        #             Boundary: $(result.vehicle_ran_into_boundary_wall), 
        #             Collisions: $(result.number_risky_scenarios),
        #             Time: $(result.time_taken)"
        #             )
        println("Reading results for experiment $i")

        #Filter out the results where the vehicle reached the goal
        if(result.vehicle_reached_goal)
            num_successful_trials += 1
            #Filter out the results where the vehicle had no collisions and reached the goal
            if(result.number_risky_scenarios == 0)
                num_collision_free_trials += 1
                array_desired_value[i] = getfield(result, Symbol(result_struct_key))
            else
                push!(collision_trial_ids,i)
            end
        else
            push!(unsuccesful_trial_ids,i)
            println("For experiment $i, Goal: $(result.vehicle_reached_goal), 
            Obstacle: $(result.vehicle_ran_into_obstacle), 
            Boundary: $(result.vehicle_ran_into_boundary_wall), 
            Collisions: $(result.number_risky_scenarios),
            Time: $(result.time_taken)"
            )
        end
    end

    println("Number of Successful trials where the vehicle reached the goal : ", num_successful_trials)
    println("Unsuccessful Trials : ", unsuccesful_trial_ids)
    println("Number of successful and collision free trials : ", num_collision_free_trials)
    println("Collision Trials : ", collision_trial_ids)

    #Calculate mean and std error on desired value
    filtered_desired_values = Iterators.filter(x->!(x<0.0),array_desired_value)
    @assert length(collect(filtered_desired_values)) == num_collision_free_trials "Length of filtered desired values doesn't match with num_collision_free_trials"
    mean_value = mean(filtered_desired_values)
    std_error_value = std(filtered_desired_values)/sqrt(num_collision_free_trials)
    # mean_value = mean(view(array_desired_value,1:num_collision_free_trials))
    # std_error_value = std(view(array_desired_value,1:num_collision_free_trials))/sqrt(num_collision_free_trials)
    println("Value for $result_struct_key :  $mean_value +/- $std_error_value")
    return(mean_value,std_error_value)
end
#=
get_results(data,:esp_hjb,:t)
get_results(data,:lsp,:t)

get_results(data,:esp_hjb,:t,"./RESULTS/With_trials/many_small_obstacles_50x50/humans_200/experiments_100/")

=#

function get_num_outperformed(output_obj,experiment_type1,experiment_type2,base_folder=nothing)


    if(isnothing(base_folder))
        (;environment_name,num_experiments,num_humans,sudden_break_flag,run_shield_flag, 
            run_with_trials,results_folder) = output_obj

        if(run_with_trials)
            results_folder = results_folder*"With_trials/"
        else
            results_folder = results_folder*"With_time/"
        end

        base_folder = results_folder*"$(environment_name)/SB_$(sudden_break_flag)_Shield_$(run_shield_flag)/"*
                        "humans_$(num_humans)/experiments_$(num_experiments)/"
        if(!isdir(base_folder))
            println("Folder doesn't exist. Something is wrong. Please check : ", base_folder)
            return 0.0
        end
    else
        indices = findfirst("experiments_", base_folder)
        string_num_experiments = base_folder[indices[end]+1:end-1]
        num_experiments = parse(Int,string_num_experiments)
    end

    filename1 = base_folder*"$(experiment_type1)_exp_"
    filename2 = base_folder*"$(experiment_type2)_exp_"

    count_1_outperformed_2 = 0
    time_type1 = Array{Pair{Int,Float64},1}(undef,num_experiments)
    time_type2 = Array{Pair{Int,Float64},1}(undef,num_experiments)

    for i in 1:num_experiments

        save_location = filename1*"$i.h5"
        file_obj = SER.open(save_location, "r");
        result1 = SER.deserialize(file_obj);
        close(file_obj);

        save_location = filename2*"$i.h5"
        file_obj = SER.open(save_location, "r");
        result2 = SER.deserialize(file_obj);
        close(file_obj);


        #Filter out the results where the vehicle reached the goal
        if(result1.vehicle_reached_goal && result2.vehicle_reached_goal)
            #Filter out the results where the vehicle had no collisions and reached the goal
            if(result1.number_risky_scenarios == 0 && result2.number_risky_scenarios == 0)
                println("Results for experiment $i : ($experiment_type1 : $(result1.time_taken)  $(result1.number_sudden_stops)), ($experiment_type2 : $(result2.time_taken)  $(result2.number_sudden_stops)), Difference (2-1) : $(result2.time_taken-result1.time_taken)")
                time_type1[i] = (i=>result1.time_taken)
                time_type2[i] = (i=>result2.time_taken)
                if(result1.time_taken <= result2.time_taken)
                    count_1_outperformed_2 += 1
                end
            else
                println("Results for experiment $i : Collision happened in one of the experiments")
                time_type1[i] = (i=>-1.0)
                time_type2[i] = (i=>-1.0)
            end
        else
            println("Results for experiment $i : Vehicle didn't reach goal in one of the experiments")
            time_type1[i] = (i=>-1.0)
            time_type2[i] = (i=>-1.0)
        end            
    end

    # println("Time taken for $experiment_type1 : ", time_type1)
    # println("Time taken for $experiment_type2 : ", time_type2)
    println("Number of Experiments where $experiment_type1 outperformed $experiment_type2 : ", count_1_outperformed_2)
    return count_1_outperformed_2
end
#=
get_num_outperformed(data,:esp_hjb,:lsp)
get_num_outperformed(data,:esp_sl,:lsp)
get_num_outperformed(data,:esp_r,:lsp)

=#

function getfield(x,y)
    if(y==:idle_time)
        return get_idle_time(x)
    elseif(y==:number_collisions)
        return get_num_distinct_collisions(x)    
    else
        return Core.getfield(x,y)
    end
end

function get_all_results(output_obj,experiment_type,result_struct_keys,base_folder=nothing)

    if(isnothing(base_folder))
        (;environment_name,num_experiments,num_humans,sudden_break_flag,run_shield_flag, 
            run_with_trials,results_folder) = output_obj

        if(run_with_trials)
            results_folder = results_folder*"With_trials/"
        else
            results_folder = results_folder*"With_time/"
        end
        base_folder = results_folder*"$(environment_name)/SB_$(sudden_break_flag)_Shield_$(run_shield_flag)/"*
                        "humans_$(num_humans)/experiments_$(num_experiments)/"
        if(!isdir(base_folder))
            println("Folder doesn't exist. Something is wrong. Please check : ", base_folder)
            return 0.0
        end
        filename = base_folder*"$(experiment_type)_exp_"
    else
        indices = findfirst("experiments_", base_folder)
        string_num_experiments = base_folder[indices[end]+1:end-1]
        num_experiments = parse(Int,string_num_experiments)
        filename = base_folder*"$(experiment_type)_exp_"
    end
 
    num_keys = length(result_struct_keys)
    collision_key_id = -1
    for i in 1:num_keys
        key = result_struct_keys[i]
        if(key==:t)
            result_struct_keys[i] = :time_taken
        elseif(key==:sb)
            result_struct_keys[i] = :number_sudden_stops
        elseif(key==:c)
            result_struct_keys[i] = :number_collisions
            collision_key_id = i
        elseif(key==:it)
            result_struct_keys[i] = :idle_time        
        end
    end

    # collision_index = findfirst(x->x==:number_collisions,result_struct_keys)
    # @assert collision_index != collision_key_id "Error with Collision key id. Please debug."
    num_successful_trials = 0
    unsuccesful_trial_ids = Array{Int64,1}()
    num_collision_free_trials = 0
    collision_trial_ids = Array{Int64,1}()
    array_desired_value = ones(num_keys,num_experiments)*-1.0

    for i in 1:num_experiments
        save_location = filename*"$i.h5"
        file_obj = SER.open(save_location, "r");
        result = SER.deserialize(file_obj);
        close(file_obj);

        #Filter out the results where the vehicle reached the goal
        if(result.vehicle_reached_goal)
            num_successful_trials += 1
            #Filter out the results where the vehicle had no collisions and reached the goal
            if(result.number_risky_scenarios == 0)
                num_collision_free_trials += 1
                values = getfield.(Ref(result), Symbol.(result_struct_keys))
                array_desired_value[:,i] .= values
                println("Results for experiment $i : $values")
            else
                push!(collision_trial_ids,i)
                println("Results for experiment $i : Collision with human(s) -> $(result.number_risky_scenarios)")
                if(collision_key_id != -1)
                    array_desired_value[collision_key_id,i] = getfield(result, :number_collisions)
                end
            end
        else
            push!(unsuccesful_trial_ids,i)
            println("Results for experiment $i, Goal: $(result.vehicle_reached_goal), 
            Obstacle: $(result.vehicle_ran_into_obstacle), 
            Boundary: $(result.vehicle_ran_into_boundary_wall), 
            Collisions: $(result.number_risky_scenarios),
            Time: $(result.time_taken)"
            )
        end
    end

    println("Number of Successful trials where the vehicle reached the goal : ", num_successful_trials)
    println("Unsuccessful Trials : ", unsuccesful_trial_ids)
    println("Number of successful and collision free trials : ", num_collision_free_trials)
    println("Collision Trials : ", collision_trial_ids)

    #Calculate mean and std error on desired value
    for i in 1:num_keys
        desired_values = view(array_desired_value,i,:)
        filtered_desired_values = Iterators.filter(x->!(x<0.0),desired_values)
        # @assert length(collect(filtered_desired_values)) == num_collision_free_trials "Length of filtered desired values doesn't match with num_collision_free_trials"
        mean_value = mean(filtered_desired_values)
        std_error_value = std(filtered_desired_values)/sqrt(num_collision_free_trials)
        if(i==collision_key_id)
            filtered_desired_values = Iterators.filter(x->x>0.0,desired_values)
            mean_value = mean(filtered_desired_values)
            std_error_value = std(filtered_desired_values)/sqrt(num_successful_trials-num_collision_free_trials)
        end
        # mean_value = mean(desired_values)
        # std_error_value = std(desired_values)/sqrt(num_collision_free_trials)
        println("Value for $(result_struct_keys[i]) :  $mean_value +/- $std_error_value")
    end

    return array_desired_value
end
#=
get_all_results(data,:esp_hjb,[:t,:c,:sb,:it])
get_all_results(data,:lsp,[:t,:c,:sb,:it])

get_all_results(data,:esp_hjb,[:t,:c,:sb,:it],"./RESULTS/With_trials/many_small_obstacles_50x50/humans_200/experiments_100/")
=#

function get_idle_time(output,print_logs=false)

    (;sim_objects,vehicle_actions) = output

    timestep_keys = collect(keys(vehicle_actions))
    idle_time_step = timestep_keys[2] - timestep_keys[1]
    sim_time_step = sim_objects[timestep_keys[1]].one_time_step

    idle_time = 0.0
    for i in 1:length(timestep_keys)-1
        curr_k = timestep_keys[i]+sim_time_step
        (;vehicle,one_time_step) = sim_objects[curr_k]
        if(vehicle.v == 0.0)
            if(print_logs)
                println("Vehicle is idle at time $curr_k : ", vehicle)
            end
            idle_time += idle_time_step
        end
    end
    return idle_time
end

function get_num_distinct_collisions(output)
    min_dist = 1.0  #Same as max_risk_distance in exp_details. Modify it to not be hardcoded.
    return count_num_collisions(output.risky_scenarios,min_dist)
end

function count_num_collisions(risky_scenarios, min_safe_distance_from_human)

    distinct_human_ids = Set{Int}()
    num_distinct_collisions = 0

    for (time_stamp,scenario) in risky_scenarios
        vehicle = scenario.vehicle
        lidar_data = scenario.vehicle_sensor_data.lidar_data
        ids = scenario.vehicle_sensor_data.ids

        vehicle_center_x = vehicle.x # + vehicle_params.dist_origin_to_center*cos(vehicle.theta)
        vehicle_center_y = vehicle.y #+ vehicle_params.dist_origin_to_center*sin(vehicle.theta)

        for (index,human) in enumerate(lidar_data)
            euclidean_distance = sqrt( (human.x - vehicle_center_x)^2 + (human.y - vehicle_center_y)^2 )
            # if(euclidean_distance<=(vehicle_params.radius+min_safe_distance_from_human))
            if(euclidean_distance<=(min_safe_distance_from_human))
                colliding_human_id = ids[index]
                if( !(colliding_human_id in distinct_human_ids))
                    num_distinct_collisions += 1
                    push!(distinct_human_ids,colliding_human_id)
                end
            end
        end
    end

    @assert length(distinct_human_ids) == num_distinct_collisions
    return num_distinct_collisions
end


function extract_one_executed_path(output)

    path_x = Float64[]
    path_y = Float64[]
    for (i,sim_obj) in output.sim_objects
        vehicle = sim_obj.vehicle
        push!(path_x, vehicle.x)
        push!(path_y, vehicle.y)
    end
    return path_x,path_y
end

function extract_all_executed_paths(results)

    all_executed_paths = Vector{Pair{Int64, Tuple{Vector{Float64}, Vector{Float64}}}}()
    num_experiments = length(results)

    for i in 1:num_experiments
        if(results[i].vehicle_reached_goal)
            px,py = extract_one_executed_path(results[i])
            push!( all_executed_paths, i=> (px,py) )
        end
    end

    return all_executed_paths
end


function extract_all_executed_paths(output_obj,experiment_type,base_folder=nothing)

    if(isnothing(base_folder))
        (;environment_name,num_experiments,num_humans,sudden_break_flag,run_shield_flag, 
            run_with_trials,results_folder) = output_obj

        if(run_with_trials)
            results_folder = results_folder*"With_trials/"
        else
            results_folder = results_folder*"With_time/"
        end
        base_folder = results_folder*"$(environment_name)/SB_$(sudden_break_flag)_Shield_$(run_shield_flag)/"*
                        "humans_$(num_humans)/experiments_$(num_experiments)/"
        if(!isdir(base_folder))
            println("Folder doesn't exist. Something is wrong. Please check : ", base_folder)
            return 0.0
        end
        filename = base_folder*"$(experiment_type)_exp_"
    else
        indices = findfirst("experiments_", base_folder)
        string_num_experiments = base_folder[indices[end]+1:end-1]
        num_experiments = parse(Int,string_num_experiments)
        filename = base_folder*"$(experiment_type)_exp_"
    end
    all_executed_paths = Vector{Pair{Int64, Tuple{Vector{Float64}, Vector{Float64}}}}()
    # num_experiments = length(results)

    println("Extracting all executed paths for $experiment_type")
    for i in 1:num_experiments

        save_location = filename*"$i.h5"
        file_obj = SER.open(save_location, "r");
        result = SER.deserialize(file_obj);
        close(file_obj);

        if(result.vehicle_reached_goal)
            px,py = extract_one_executed_path(result)
            push!( all_executed_paths, i=> (px,py) )
        end
    end

    return all_executed_paths
end


function visualize_env(env, vehicle_goal, exp_details)

    l = env.length
    b = env.breadth
    snapshot = plot(aspect_ratio=:equal,size=(1000,1000), dpi=300,
        axis=([], false),
        # xticks=0:2:l, yticks=0:2:b,
        # xlabel="x-axis [m]", ylabel="y-axis [m]",
        # legend=:bottom,
        # legend=false
        )

    #Plot Workspace
    turf_color = :white
    plot!(snapshot, rectangleShape(0.0,0.0,env.length,env.breadth),opacity=0.1,color=turf_color,linewidth=2.0,linecolor=:black,label="")


    #Plot External Anchors
    anchor_dist = 0.5
    scatter!(snapshot,
        [0.0 - anchor_dist, env.length + anchor_dist, env.length + anchor_dist, 0.0 - anchor_dist],
        [0.0 - anchor_dist, 0.0 - anchor_dist, env.breadth + anchor_dist, env.breadth + anchor_dist],
        markeralpha=0.0, fillalpha=0.0, label="")

    #Plot Obstacles
    obstacle_color = :black
    for obs in env.obstacles
        plot!(snapshot, circleShape(obs.x,obs.y,obs.r), color=obstacle_color, linewidth=2.0, linecolor = obstacle_color,
            legend=false, fillalpha=0.4, aspect_ratio=1, label="", seriestype = [:shape,])
        # Plots.annotate!(snapshot,obs.x, obs.y, text("Obs", obstacle_color, :center, 10))
    end

    #Plot Vehicle Goal
    vehicle_goal_color = :yellow
    Plots.annotate!(snapshot,vehicle_goal.x, vehicle_goal.y, text("GOAL", :brown, :center, 30))

    return snapshot
end


function visualize_all_executed_paths( input_config, all_executed_paths, col)
    exp_details = ExperimentDetails(input_config)
    env = generate_environment(input_config.env_length,input_config.env_breadth,input_config.obstacles)
    veh_goal = Location(input_config.veh_goal_x,input_config.veh_goal_y)
    snapshot = visualize_env(env, veh_goal, exp_details)
    Plots.annotate!(snapshot,input_config.veh_start_x, input_config.veh_start_y,
                                    text("START", :brown, :center, 30))
    for (i,path) in all_executed_paths
        px,py = path[1],path[2]
        plot!(snapshot, px, py, linewidth=2,label="",color=col)
    end
    display(snapshot)
    return snapshot
end


function run_experiment(environment_name, num_experiments, num_humans;
                sudden_break, run_shield, run_with_trials, seeds=nothing,
                results_folder = "/home/himanshu/Documents/Research/human_aware_navigation/RESULTS/", 
                lsp=false, esp_r=false, esp_sl=false, esp_hjb=true, print_logs=true)

    filename = "src/configs/"*environment_name*".jl"
    include(filename)
    rollout_guide_filename = "./src/rollout_guides/HJB_rollout_guide_"*environment_name*".jld2"
    s = load(rollout_guide_filename)
    rollout_guide = s["rollout_guide"];
    data = PipelineOutput(environment_name,num_experiments,num_humans,sudden_break,
                        run_shield,run_with_trials,input_config,rollout_guide,
                        results_folder,seeds);
    run_pipeline(data,lsp=lsp,esp_r=esp_r,esp_sl=esp_sl,esp_hjb=esp_hjb,print_logs=print_logs)

    if(run_with_trials)
        results_folder = results_folder*"With_trials/"
    else
        results_folder = results_folder*"With_time/"
    end
    base_folder = results_folder*"$(environment_name)/SB_$(sudden_break)_Shield_$(run_shield)/"*
                        "humans_$(num_humans)/experiments_$(num_experiments)/"
    
    if(!isdir(base_folder))
        println("Folder didn't exist. That shouldn't have happened. PROBLEM. Please debug : ", base_folder)
        return data
    end

    save_location = base_folder*"pipeline_output_struct.h5"
    file_obj = SER.open(save_location, "w");
    SER.serialize(file_obj, data);
    close(file_obj);
    return data
end


#=
Old code to get results when everything was in memory

function get_time_results(experimental_data)

    # (;num_experiments,baseline,esp_random,esp_sl,esp_hjb) = output_obj

    #Filter out the results where the vehicle reached the goal
    successful_trials = filter(x->x.vehicle_reached_goal == true, experimental_data)
    num_successful_trials = length(successful_trials)
    println("Number of Successful trials where the vehicle reached the goal : ", num_successful_trials)

    #Filter out the results where the vehicle had no collisions and reached the goal
    collision_free_trials = filter(x->x.number_risky_scenarios == 0, successful_trials)
    num_collision_free_trials = length(collision_free_trials)
    println("Number of successful and collision free trials : ", num_collision_free_trials)

    #Calculate mean and std error on time
    mean_time = mean(d.time_taken for d in collision_free_trials)
    std_error_time = std(d.time_taken for d in collision_free_trials)/sqrt(num_collision_free_trials)
    println("Trajectory Time (in seconds) : ", mean_time, " +/- ", std_error_time)

    return(mean_time,std_error_time)
end

function get_sudden_break_results(experimental_data)

    # (;num_experiments,baseline,esp_random,esp_sl,esp_hjb) = output_obj

    #Filter out the results where the vehicle reached the goal
    successful_trials = filter(x->x.vehicle_reached_goal == true, experimental_data)
    num_successful_trials = length(successful_trials)
    println("Number of Successful trials where the vehicle reached the goal : ", num_successful_trials)

    #Filter out the results where the vehicle had no collisions and reached the goal
    collision_free_trials = filter(x->x.number_risky_scenarios == 0, successful_trials)
    num_collision_free_trials = length(collision_free_trials)
    println("Number of successful and collision free trials : ", num_collision_free_trials)

    #Calculate mean and std error on number of SB actions
    mean_SB = mean(d.number_sudden_stops for d in collision_free_trials)
    std_error_SB = std(d.number_sudden_stops for d in collision_free_trials)/sqrt(num_collision_free_trials)
    println("Number of Sudden Break actions : ", mean_SB, " +/- ", std_error_SB)

    return(mean_SB,std_error_SB)
end

function get_num_collisions(experimental_data, min_dist)

    #Filter out the results where the vehicle reached the goal
    successful_trials = filter(x->x.vehicle_reached_goal == true, experimental_data)
    num_successful_trials = length(successful_trials)
    println("Number of Successful trials where the vehicle reached the goal : ", num_successful_trials)

    collision_per_successful_experiment = [count_num_collisions(d.risky_scenarios,min_dist) for d in successful_trials]

    num_unsafe_experiments = 0
    for collision_num in collision_per_successful_experiment
        if(collision_num!=0)
            num_unsafe_experiments+=1
        end
    end

    println("Number of Experiments where collisions happened : ", num_unsafe_experiments)

    #Calculate mean and std error on number of collisions
    mean_collisions = mean(collision_per_successful_experiment)
    std_error_collisions = std(collision_per_successful_experiment)/sqrt(num_successful_trials)
    println("Number of collisions : ", mean_collisions, " +/- ", std_error_collisions)

    return(mean_collisions,std_error_collisions)
end

function get_num_outperformed(baseline, proposed_planner)

    num_experiments = length(baseline)
    num_outperformed = 0

    for i in 1:num_experiments
        if( baseline[i].vehicle_reached_goal && proposed_planner[i].vehicle_reached_goal &&
            baseline[i].number_risky_scenarios == 0 && proposed_planner[i].number_risky_scenarios == 0
            )
            if(baseline[i].time_taken > proposed_planner[i].time_taken)
                num_outperformed += 1
            end
        end
    end

    println("Number of Experiments where Proposed Approach outperformed the baseline : ", num_outperformed)
    return num_outperformed
end

=#