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


# Define a function to check if a circle and a polygon intersect
function circle_polygon_intersection(circle_center, circle_radius, polygon_vertices)
    # Check if any polygon vertex is inside the circle
    for vertex in polygon_vertices
        if point_inside_circle(vertex, circle_center, circle_radius)
            # println("Polygon vertex is inside the circle.")
            return true
        end
    end
    
    # Check if the circle intersects with any polygon edge
    n = length(polygon_vertices)
    for i in 1:n
        j = i % n + 1
        edge = (polygon_vertices[i], polygon_vertices[j])
        if( circle_line_segment_intersection(circle_center,circle_radius,edge) )
            # println("Circle intersects with polygon edge.")
            return true
        end
    end
    

    #=
    Earlier I was checking if the circle is entirely inside the polygon or vice versa.
    To check if the polygon is entirely inside the circle, I would have to check 
    if all the vertices of the polygon are inside the circle. 
    But the first check in the function sort of already does that.
    So, I should only check if the circle is inside the polygon or not.
    
    Old code
        if point_inside_polygon(circle_center, polygon_vertices) || 
        all(point_inside_circle(vertex, circle_center, circle_radius) for vertex in polygon_vertices)
            # print("Circle and polygon are entirely inside each other.")
            return true
        end
    =#
    # Check if the circle is entirely inside the polygon
    if point_inside_polygon(circle_center, polygon_vertices)
        # print("Circle and polygon are entirely inside each other.")
        return true
    end
    
    return false
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



