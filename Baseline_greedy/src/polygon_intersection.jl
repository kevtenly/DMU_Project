using LinearAlgebra

# Define a function to calculate distance between two points
function distance(p1, p2)
    s = 0.0
    for i in 1:length(p1)
        s += (p1[i] - p2[i])^2
    end
    return sqrt(s)
end

# Define a function to check if a point is inside a circle
function point_inside_circle(point, center, radius)
    return distance(point, center) <= radius
end

#=
Define a function to check if a point is inside a polygon
From my understanding, the logic for this comes from the ray casting algorithm.
It can be found here:
https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/
=#
function point_inside_polygon(point, vertices)
    n = length(vertices)
    inside = false
    j = n

    for i in 1:n
        if ((vertices[i][2] > point[2]) != (vertices[j][2] > point[2])) &&
           (point[1] < (vertices[j][1] - vertices[i][1]) * (point[2] - vertices[i][2]) /
                        (vertices[j][2] - vertices[i][2]) + vertices[i][1])
            inside = !inside
        end
        j = i
    end
    
    return inside
end

# Define a function to check if a circle and a line segment intersect
function circle_line_segment_intersection(circle_center,circle_radius,line_segment)

    #=
    line_segment is a tuple of 2 points (start_point,end_point)
    =#

    start_point,end_point = line_segment
    cr = circle_radius

    dx = end_point[1]-start_point[1]
    dy = end_point[2]-start_point[2]
    fx = start_point[1]-circle_center[1]
    fy = start_point[2]-circle_center[2]

    #Quadratic equation is  t^2 ( d · d ) + 2t ( d · f ) +  (f · f - r^2) = 0
    #Refer to this link if needed - https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
    #Standard form is a.t^2 + b.t + c = 0

    a = (dx^2 + dy^2)
    b = 2*(dx*fx + dy*fy)
    c = (fx^2 + fy^2) - (cr^2)
    discriminant = (b^2 - 4*a*c)

    if(discriminant<0)
        return false
    elseif (discriminant == 0)
        t = -b/(2*a)
        if(t>=0 && t<=1)
            return true
        end
    else
        discriminant = sqrt(discriminant)
        t = (-b-discriminant)/(2*a)
        if(t>=0 && t<=1)
            return true
        end
        t = (-b+discriminant)/(2*a)
        if(t>=0 && t<=1)
            return true
        end
    end
    return false
end


# # Define a function to check if a circle and a polygon intersect
# function circle_polygon_intersection(circle_center, circle_radius, polygon_vertices)
#     # Check if any polygon vertex is inside the circle
#     for vertex in polygon_vertices
#         if point_inside_circle(vertex, circle_center, circle_radius)
#             # println("Polygon vertex is inside the circle.")
#             return true
#         end
#     end
    
#     # Check if the circle intersects with any polygon edge
#     n = length(polygon_vertices)
#     for i in 1:n
#         j = i % n + 1
#         edge = (polygon_vertices[i], polygon_vertices[j])
#         if( circle_line_segment_intersection(circle_center,circle_radius,edge) )
#             # println("Circle intersects with polygon edge.")
#             return true
#         end
#     end
    

#     #=
#     Earlier I was checking if the circle is entirely inside the polygon or vice versa.
#     To check if the polygon is entirely inside the circle, I would have to check 
#     if all the vertices of the polygon are inside the circle. 
#     But the first check in the function sort of already does that.
#     So, I should only check if the circle is inside the polygon or not.
    
#     Old code
#         if point_inside_polygon(circle_center, polygon_vertices) || 
#         all(point_inside_circle(vertex, circle_center, circle_radius) for vertex in polygon_vertices)
#             # print("Circle and polygon are entirely inside each other.")
#             return true
#         end
#     =#
#     # Check if the circle is entirely inside the polygon
#     if point_inside_polygon(circle_center, polygon_vertices)
#         # print("Circle and polygon are entirely inside each other.")
#         return true
#     end
    
#     return false
# end
############
function circle_polygon_intersection(circle_center, circle_radius, polygon_vertices)
    # Convert to tuple format for consistency
    cc = (circle_center[1], circle_center[2])
    poly = [(v[1], v[2]) for v in polygon_vertices]

    # 1. Check if any polygon point is inside the circle (existing check)
    for vertex in poly
        dx = vertex[1] - cc[1]
        dy = vertex[2] - cc[2]
        if dx^2 + dy^2 <= circle_radius^2
            return true
        end
    end

    # 2. Find closest polygon point to circle center
    closest_dist_sq = Inf
    closest_point = poly[1]
    for point in poly
        dx = point[1] - cc[1]
        dy = point[2] - cc[2]
        dist_sq = dx^2 + dy^2
        if dist_sq < closest_dist_sq
            closest_dist_sq = dist_sq
            closest_point = point
        end
    end

    # Early exit if circle contains closest point
    if sqrt(closest_dist_sq) <= circle_radius
        return true
    end

    # 3. SAT Test for polygon edges
    n = length(poly)
    for i in 1:n
        j = i % n + 1
        edge_start = poly[i]
        edge_end = poly[j]
        
        # Get edge normal
        edge = (edge_end[1] - edge_start[1], edge_end[2] - edge_start[2])
        normal = (-edge[2], edge[1])
        length_normal = sqrt(normal[1]^2 + normal[2]^2)
        axis = (normal[1]/length_normal, normal[2]/length_normal)

        # Project polygon onto axis
        poly_min, poly_max = Inf, -Inf
        for p in poly
            proj = p[1]*axis[1] + p[2]*axis[2]
            poly_min = min(poly_min, proj)
            poly_max = max(poly_max, proj)
        end

        # Project circle onto axis
        center_proj = cc[1]*axis[1] + cc[2]*axis[2]
        circle_min = center_proj - circle_radius
        circle_max = center_proj + circle_radius

        # Check for separation
        if circle_max < poly_min || circle_min > poly_max
            return false
        end
    end

    # 4. Check axis from circle center to closest point
    dx = closest_point[1] - cc[1]
    dy = closest_point[2] - cc[2]
    axis_length = sqrt(dx^2 + dy^2)
    axis = (dx/axis_length, dy/axis_length)

    # Project polygon
    poly_min, poly_max = Inf, -Inf
    for p in poly
        proj = p[1]*axis[1] + p[2]*axis[2]
        poly_min = min(poly_min, proj)
        poly_max = max(poly_max, proj)
    end

    # Project circle
    center_proj = cc[1]*axis[1] + cc[2]*axis[2]
    return !(poly_max < center_proj - circle_radius || center_proj + circle_radius < poly_min)
end


#=
Example

circle_center = SVector(2.0, 2.0)
circle_radius = 1.0
polygon_vertices = SVector( 
                        SVector(0., 0.),
                        SVector(12, 7),
                        SVector(12, 0.),
                        # SVector(0.2, 0.8)
                        )

circle_polygon_intersection(circle_center, circle_radius, polygon_vertices)
@benchmark circle_polygon_intersection($circle_center, $circle_radius, $polygon_vertices)
=#



