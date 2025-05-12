#=
Pipeline code

include("src/experiment_pipeline.jl")

environment_name = "no_obstacles_25x25"
environment_name = "small_obstacles_25x25"
environment_name = "big_obstacle_25x25"
environment_name = "L_shape_25x25"

environment_name = "no_obstacles_50x50"
environment_name = "small_obstacles_50x50"
environment_name = "many_small_obstacles_50x50"
environment_name = "big_obstacle_50x50"
environment_name = "L_shape_50x50"


filename = "src/configs/"*environment_name*".jl"
include(filename)
input_config = small_obstacles_50x50
rollout_guide_filename = "./src/rollout_guides/HJB_rollout_guide_"*environment_name*".jld2"
s = load(rollout_guide_filename)
rollout_guide = s["rollout_guide"];
sudden_break = false
run_shield = false
num_experiments = 1



config_fn = "./src/configs/"*env_name*".jl"
include(config_fn)

all_paths_hjb = extract_all_executed_paths(data.esp_hjb);
p = visualize_all_executed_paths(input_config,all_paths_hjb, :olive);
esp_fn = "esp_"*env_name*"_paths.svg"
Plots.savefig(p,esp_fn)

all_paths_baseline = extract_all_executed_paths(data.baseline);
p = visualize_all_executed_paths(input_config, all_paths_baseline, :steelblue);
lsp_fn = "lsp_"*env_name*"_paths.svg"
Plots.savefig(p,lsp_fn)



Threads.@theads for entry in tuples
    data = run_experiment(env_name, num_experiments, num_humans, sudden_break, run_shield )
    datafile_name = env_name*"_humans_"*string(num_humans)*"_suddenbreak_"*string(sudden_break)*"_runshield_"*string(run_shield)*".jld2"
    data_dict = Dict("data"=>data);
    save(datafile_name, data_dict)
end

=#


#=
Run Experiments for the environment for your choice, with desired number of humans, sudden break and run shield flags  

using Pkg
Pkg.activate(".") 
include("src/experiment_pipeline.jl")
num_experiments = 100
env_name = "small_obstacles_50x50"
num_humans = 200
trials_flag = true
rf = "/home/himanshu/Documents/Research/human_aware_navigation/RESULTS/"
rf = "/home/himanshu/Documents/Research/SNAP/human_aware_navigation/RESULTS/"

data = ""
for (sb,rs) in [ (false,false) ]
    data = run_experiment(env_name, num_experiments, num_humans, 
    sudden_break=sb,run_shield=rs,run_with_trials=trials_flag,
    results_folder = rf,
    lsp=true,esp_sl=true);
end

GC.gc();

data = ""
for (sb,rs) in [ (true,false) ]
    data = run_experiment(env_name, num_experiments, num_humans, 
    sudden_break=sb,run_shield=rs,run_with_trials=trials_flag,
    results_folder = rf,
    esp_sl=true,esp_hjb=false);
end

GC.gc();

data = ""
for (sb,rs) in [ (false,true) ]
    data = run_experiment(env_name, num_experiments, num_humans, 
    sudden_break=sb,run_shield=rs,run_with_trials=trials_flag,
    results_folder = rf,
    lsp=true,esp_hjb=true);
end

GC.gc();

=#


#=
Read an output file and visualize the path

file_location = "./RESULTS/With_trials/small_obstacles_50x50/SB_true_Shield_false/humans_50/experiments_2/lsp_exp_1.h5"
file_obj = SER.open(file_location, "r");
output = SER.deserialize(file_obj);
close(file_obj);
vehicle_executed_trajectory = []
anim = @animate for k ∈ keys(output.sim_objects)
    observe(output, exp_details, k, vehicle_executed_trajectory);
end

=#


#=
Run the experiment for all environments and all number of humans and with different sudden break and run shield flags


all_environment_names = ("no_obstacles_50x50", "small_obstacles_50x50", "L_shape_50x50","many_small_obstacles_50x50", "big_obstacle_50x50")
num_experiments = 100
all_humans = (50,100,200)

rwt = true
for env_name in all_environment_names
    for (sb,rs) in [ (false,false), (true,false), (false,true) ]
        for num_humans in all_humans
            data = run_experiment(env_name, num_experiments, num_humans, 
            sudden_break=sb,run_shield=rs,run_with_trials=rwt,
            results_folder = "/home/himanshu/Documents/Research/SNAP/human_aware_navigation/RESULTS/",
            lsp=true,esp_hjb=true);
        end
    end
end

=#



#=
Miscellaneous


num_experiments = 2
env_name = "small_obstacles_50x50"
num_humans = 50
data = run_experiment(env_name, num_experiments, num_humans, sudden_break=true, run_shield=false, run_with_trials=true,
        results_folder = "/home/himanshu/Documents/Research/human_aware_navigation/RESULTS/", lsp=true,esp_hjb=true);

data = run_experiment(env_name, num_experiments, num_humans, sudden_break=true, run_shield=false, run_with_trials=false, 
        results_folder = "/home/himanshu/Documents/Research/human_aware_navigation/RESULTS/", lsp=true,esp_hjb=true);


data = run_experiment(env_name, num_experiments, num_humans, sudden_break=false, run_shield=true, run_with_trials=false, 
results_folder = "/home/himanshu/Documents/Research/human_aware_navigation/RESULTS/", lsp=true,esp_hjb=true);


for num_humans in (50,100,200)
        data = run_experiment(env_name, num_experiments, num_humans, sudden_break=true, run_shield=false, run_with_trials=true,
        results_folder = "/home/himanshu/Documents/Research/human_aware_navigation/RESULTS/", lsp=true,esp_hjb=true);
    end
end

For OMAK

data = run_experiment(env_name, num_experiments, num_humans, sudden_break=true, run_shield=false, run_with_trials= true,
        results_folder = "/home/himanshu/Documents/Research/SNAP/human_aware_navigation/RESULTS/", lsp=true,esp_hjb=true);


=#


#=
Run the experiment for all environments and specific number of humans and with sudden break and run shield flags

using Pkg
Pkg.activate(".") 
include("src/experiment_pipeline.jl")

all_environment_names = ("no_obstacles_50x50", "small_obstacles_50x50", "L_shape_50x50","many_small_obstacles_50x50", "big_obstacle_50x50")
num_experiments = 100
num_humans = 50
trials_flag = false
data = "" 
rf = "/home/himanshu/Documents/Research/human_aware_navigation/RESULTS/"
rf = "/media/himanshu_storage/NavigationPOMDP/RESULTS/"


for env_name in all_environment_names
    for (sb,rs) in [ (true,false)]
        data = run_experiment(env_name, num_experiments, num_humans, 
        sudden_break=sb,run_shield=rs,run_with_trials=trials_flag,
        results_folder = rf,
        lsp=true,esp_hjb=true,esp_sl=true,esp_r=true);
        end
    end
end


for env_name in all_environment_names
    for (sb,rs) in [ (false,false)]
        data = run_experiment(env_name, num_experiments, num_humans, 
        sudden_break=sb,run_shield=rs,run_with_trials=trials_flag,
        results_folder = rf,
        lsp=true,esp_hjb=true,esp_sl=true,esp_r=true);
    end
end

for env_name in all_environment_names
    for (sb,rs) in [ (false,true)]
        data = run_experiment(env_name, num_experiments, num_humans, 
        sudden_break=sb,run_shield=rs,run_with_trials=trials_flag,
        results_folder = rf,
        lsp=true,esp_hjb=true);
    end
end


=#



#=

environment_name = "big_obstacle_50x50"
filename = "src/configs/"*environment_name*".jl"
include(filename)
pomdp_details = POMPDPlanningDetails(input_config)
exp_details = ExperimentDetails(input_config)

exp_name = "no_obs_esp_hjb_exp_68"
fn = "/home/himanshu/Desktop/Dhruv_JSONS/"*exp_name*".h5"
fo = open(fn);
output = SER.deserialize(fo);
close(fo)


vehicle_executed_trajectory = []
anim = @animate for k ∈ keys(output.sim_objects)
    observe(output, exp_details, k, vehicle_executed_trajectory);
end
gif_name = exp_name*".gif"
gif(anim, gif_name, fps = 20)


=#



#=
using JSON

json_dict = Dict()
v_dict = Dict()
h_dict = Dict()
obs_dict= Dict()

file_list = [
"big_obs_esp_hjb_exp_83",
"L_shape_esp_hjb_exp_26",
"L_shape_esp_hjb_exp_35",
"L_shape_esp_hjb_exp_44",
"L_shape_esp_hjb_exp_54",
"L_shape_esp_hjb_exp_64",
"many_small_obs_esp_hjb_exp_67",
"many_small_obs_esp_hjb_exp_84",
"no_obs_esp_hjb_exp_37",
"no_obs_esp_hjb_exp_68",
"small_obs_esp_hjb_exp_11",
"small_obs_esp_hjb_exp_62",
"small_obs_esp_hjb_exp_73"
]

for exp_name in file_list

    json_dict = Dict()
    v_dict = Dict()
    h_dict = Dict()
    obs_dict= Dict()

    # exp_name = "small_obs_lsp_exp_73"
    fn = "/home/himanshu/Desktop/Dhruv_JSONS/"*exp_name*".h5"
    fo = open(fn);
    output = SER.deserialize(fo);
    close(fo)

    sos = output.sim_objects
    for k in keys(sos)
        sim_obj = sos[k]
        v = sim_obj.vehicle
        v_dict[k] = Dict('x'=>v.x,'y'=>v.y,'t'=>v.theta,'v'=>v.v) 
    end


    for k in keys(sos)
        sim_obj = sos[k]
        h = sim_obj.humans
        hp = sim_obj.humans_params
        curr_humans_dict = Dict()
        for i in 1:length(h)
            curr_humans_dict[hp[i].id] = Dict('x'=>h[i].x,'y'=>h[i].y,'v'=>h[i].v,'g'=>(h[i].goal.x,h[i].goal.y))
        end
        h_dict[k] = curr_humans_dict
    end


    obs = sos[1].env.obstacles
    for i in 1:length(obs)
        obs_dict[i] = Dict('x'=>obs[i].x,'y'=>obs[i].y,'r'=>obs[i].r)
    end

    json_dict["vehicle"] = v_dict
    json_dict["humans"] = h_dict
    json_dict["obstacles"] = obs_dict


    new_fn = "/home/himanshu/Desktop/Dhruv_JSONS/"*exp_name*".json"
    open(new_fn, "w") do file
        JSON.print(file,json_dict)
    end

end

=#



#=

environment_name = "big_obstacle_50x50"
filename = "src/configs/"*environment_name*".jl"
include(filename)
pomdp_details = POMPDPlanningDetails(input_config)
exp_details = ExperimentDetails(input_config)

exp_type = "lsp"

file_list = [
"big_obs_"*exp_type*"_exp_83",
"L_shape_"*exp_type*"_exp_26",
"L_shape_"*exp_type*"_exp_35",
"L_shape_"*exp_type*"_exp_44",
"L_shape_"*exp_type*"_exp_54",
"L_shape_"*exp_type*"_exp_64",
"many_small_obs_"*exp_type*"_exp_67",
"many_small_obs_"*exp_type*"_exp_84",
"no_obs_"*exp_type*"_exp_37",
"no_obs_"*exp_type*"_exp_68",
"small_obs_"*exp_type*"_exp_11",
"small_obs_"*exp_type*"_exp_62",
"small_obs_"*exp_type*"_exp_73"
]

for exp_name in file_list

    fn = "/home/himanshu/Desktop/Dhruv_JSONS/"*exp_name*".h5"
    fo = open(fn);
    output = SER.deserialize(fo);
    close(fo)

    vehicle_executed_trajectory = []
    anim = @animate for k ∈ keys(output.sim_objects)
        observe(output, exp_details, k, vehicle_executed_trajectory);
    end
    gif_name = "/home/himanshu/Documents/Research/human_aware_navigation/src/some_amazing_gifs/"*exp_name*".gif"
    gif(anim, gif_name, fps = 20)

end



=#



#=

Code to generate and save all trajectory images!

all_environment_names = ("no_obstacles_50x50", "small_obstacles_50x50", "L_shape_50x50","many_small_obstacles_50x50", "big_obstacle_50x50")
num_experiments = 100
all_humans = (50,100,200)
rf = "/media/storage/himanshu_storage/NavigationPOMDP/RESULTS/With_time/"


for env_name in all_environment_names
    for (sb,rs) in [ (false,false), (true,false), (false,true) ]
        for num_humans in all_humans
            folder_name = env_name*"/SB_"*string(sb)*"_Shield_"*string(rs)*"/humans_"*string(num_humans)*"/experiments_"*string(num_experiments)*"/"
            base_folder = rf*folder_name
            filename = replace(folder_name,"/"=>"_")
            for exp_type in (:lsp, :esp_hjb)
                all_paths = extract_all_executed_paths([], exp_type, base_folder)
                include("src/configs/"*env_name*".jl")
                if(exp_type == :lsp)
                    col = :steelblue
                else
                    col = :olive
                end
                save_final_name = rf*"trajectory_images/"*filename*string(exp_type)*"_paths.svg"
                println(save_final_name)
                p = visualize_all_executed_paths(input_config,all_paths,col);
                Plots.savefig(p,save_final_name)    
            end
            print(filename)
            println(base_folder)
        end
    end
end



=#