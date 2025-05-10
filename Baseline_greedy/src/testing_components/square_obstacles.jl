import LazySets as LS

function get_rectangle1()
    c = SVector(0.0,0.0)
    r = SVector(2.0,1.0)
    H = LS.Hyperrectangle(c, r)
    return H
end    


function get_rectangle2()
    c = SVector(1.0,1.0)
    r = SVector(2.0,1.0)
    H = LS.Hyperrectangle(c, r)
    return H
end


function get_veh_body(x, veh)
    #First, rotate the body about origin by theta degrees
    theta = x[3]
    # rot_matrix = [cos(theta) -sin(theta); sin(theta) cos(theta)]
    #=
    Note, there is a transpose here so that the rotation matrix is correct
    =#
    rotation_matrix = SMatrix{2,2}(cos(theta), -sin(theta), sin(theta), cos(theta))'
    body = LS.linear_map(rotation_matrix, veh)

    #Secnond, translate body from origin by to the vehicle's position
    pos_vec = SVector(x[1],x[2])
    b = LS.translate(body, pos_vec)
    return b
end


H1 = get_rectangle1()
H2 = get_rectangle2()

LS.isdisjoint(H1, H2)


struct Rectangle
    c::Tuple{Float64,Float64}
    l::Float64
    b::Float64
end
#=
r1 = Rectangle((0.0,0.0), 2.0, 1.0)
r2 = Rectangle((5.0,5.0), 2.0, 1.0)
=#

function get_vertices(rectangle,∠=0.0)
    (;c,l,b) = rectangle
    θ = atan(b/l)
    r = sqrt(l^2 + b^2)/2
    point_A = (c[1] + r*cos(∠+θ), c[2] + r*sin(∠+θ))
    point_B = (c[1] + r*cos(∠+θ+pi/2), c[2] + r*sin(∠+θ+pi/2))
    point_C = (c[1] + r*cos(∠+θ+pi), c[2] + r*sin(∠+θ+pi))
    point_D = (c[1] + r*cos(∠+θ+3*pi/2), c[2] + r*sin(∠+θ+3*pi/2))
    return point_A, point_B, point_C, point_D
end

function get_vertices_circular(rectangle,∠=0.0)
    (;c,l,b) = rectangle
    θ = atan(b/l)
    r = sqrt(l^2 + b^2)/2
    point_A = (c[1] + r*cos(∠+θ), c[2] + r*sin(∠+θ))
    point_B = (c[1] + r*cos(∠+θ+pi/2), c[2] + r*sin(∠+θ+pi/2))
    point_C = (c[1] + r*cos(∠+θ+pi), c[2] + r*sin(∠+θ+pi))
    point_D = (c[1] + r*cos(∠+θ+3*pi/2), c[2] + r*sin(∠+θ+3*pi/2))
    return point_A, point_B, point_C, point_D, point_A
end

function CCW(point_A,point_B,point_C)
    return (point_C[2]-point_A[2])*(point_B[1]-point_A[1]) > (point_B[2]-point_A[2])*(point_C[1]-point_A[1])
end

function intersect(A,B,C,D)
    return CCW(A,C,D) != CCW(B,C,D) && CCW(A,B,C) != CCW(A,B,D)
end

function intersect(vertices_rect_a, vertices_rect_b)
    for i in 1:4
        if(i<4)
            end_index_i = i+1
            else
            end_index_i = 1
        end
        for j in 1:4
            if(j<4)
                end_index_j = j+1
                else
                end_index_j = 1
            end
            if intersect(vertices_rect_a[i], vertices_rect_a[end_index_i], vertices_rect_b[j], vertices_rect_b[end_index_j])
                return true
            end
        end
    end
    return false    
end

function better_intersect(vertices_rect_a, vertices_rect_b)
    for i in 1:4
        for j in 1:4
            if intersect(vertices_rect_a[i], vertices_rect_a[i+1], vertices_rect_b[j], vertices_rect_b[j+1])
                return true
            end
        end
    end
    return false    
end


function overall(r1,vertices)
    vertices_r = get_vertices_circular(r1)
    return better_intersect(vertices_r, vertices)
end

function lala()
    A = SVector(2.0, 1.0)
    B = SVector(2.0, -1.0)
    C = SVector(0.0, 0.0)   
    D = SVector(4.0, 0.0)

    for i in 1:16
        intersect(A,B,C,D)
    end
    return true
end


function lala2()
    A = SVector(2.0, 1.0)
    B = SVector(2.0, -1.0)
    C = SVector(2.0, 1.0)
    D = SVector(2.0, -1.0)
    # C = SVector(0.0, 0.0)   
    # D = SVector(4.0, 0.0)
    return intersect(A,B,C,D)
end


