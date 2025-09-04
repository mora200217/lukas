using GLMakie

# -------------------------------------------------------------------
# Core math
# -------------------------------------------------------------------
function forward_kinematics(l1, l2, θ1, θ2)
    x = l1 * cos(θ1) + l2 * cos(θ1 + θ2)
    y = l1 * sin(θ1) + l2 * sin(θ1 + θ2)
    return Point2f(x, y)   # <- Point2f
end

function workspace_points(l1, l2; n=50_000,
                          θ1range=(-π, π), θ2range=(-π, π))
    nside = ceil(Int, sqrt(n))
    θ1s = range(θ1range[1], θ1range[2], length=nside)
    θ2s = range(θ2range[1], θ2range[2], length=nside)
    pts = Point2f[]
    sizehint!(pts, nside^2)
    for θ1 in θ1s, θ2 in θ2s
        push!(pts, forward_kinematics(l1, l2, θ1, θ2))
    end
    return pts
end

# -------------------------------------------------------------------
# Plot helpers
# -------------------------------------------------------------------
function reach_circles!(ax, l1, l2)
    rmax, rmin = l1 + l2, abs(l1 - l2)
    lines!(ax, Circle(Point2f(0, 0), rmax), linewidth=2, color=:gray)
    if rmin > 0
        lines!(ax, Circle(Point2f(0, 0), rmin), linewidth=2,
               color=:gray, linestyle=:dash)
    end
end

function plot_workspace(l1, l2; n=50_000, θ1range=(-π, π), θ2range=(-π, π),
                        show_arm=false, θ1=0.0, θ2=0.0)
    fig = Figure(size=(720, 720))   # <- size instead of resolution
    ax  = Axis(fig[1, 1], aspect=DataAspect(),
               xlabel="x", ylabel="y",
               title="Workspace 2R (l₁=$(l1), l₂=$(l2))")

    # Workspace
    pts = workspace_points(l1, l2; n=n, θ1range=θ1range, θ2range=θ2range)
    scatter!(ax, pts; markersize=1.0, color=(:blue, 0.25))

    # Alcance
    reach_circles!(ax, l1, l2)

    # Base
    scatter!(ax, [0.0], [0.0]; markersize=10, color=:red)

    # Brazo
    if show_arm
        p0 = Point2f(0, 0)
        p1 = Point2f(l1 * cos(θ1), l1 * sin(θ1))
        p2 = forward_kinematics(l1, l2, θ1, θ2)
        lines!(ax, [p0, p1, p2]; linewidth=3, color=:black)
        scatter!(ax, [p0, p1, p2]; markersize=8, color=:orange)
    end

    r = l1 + l2 + 0.2max(l1, l2)
    xlims!(ax, -r, r); ylims!(ax, -r, r)

    return fig
end


l1, l2 = 11.3, 11.3
    fig = plot_workspace(l1, l2; show_arm=true,
                         θ1=deg2rad(40), θ2=deg2rad(-60))
    display(fig)