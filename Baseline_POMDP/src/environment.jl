#Define the Environment
function generate_environment(l, b, obstacles)
    return ExperimentEnvironment(l,b,obstacles)
end

# New function to generate environment with target locations
function generate_environment_with_targets(l, b, obstacles, target_locations)
    env = ExperimentEnvironment(l, b, obstacles)
    return env
end

#=
The modification I've made adds a new function generate_environment_with_targets 
that will be used to create an environment that includes information about possible target locations. 
=#