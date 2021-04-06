### A Pluto.jl notebook ###
# v0.14.0

using Markdown
using InteractiveUtils

# This Pluto notebook uses @bind for interactivity. When running this notebook outside of Pluto, the following 'mock version' of @bind gives bound variables a default value (instead of an error).
macro bind(def, element)
    quote
        local el = $(esc(element))
        global $(esc(def)) = Core.applicable(Base.get, el) ? Base.get(el) : missing
        el
    end
end

# ╔═╡ 70cf1dbf-4ad9-440b-93ce-43f1f4358f04
using Meshes: Point

# ╔═╡ e1186a57-640f-476b-9589-a7a9a61e9046
using VegaLite: @vlplot

# ╔═╡ e09157b3-74ac-49bc-84f8-72627d6f58ff
using PlutoUI: Slider

# ╔═╡ b8dc8e1a-96e3-11eb-1fca-fdd392a23642
include("/home/lassepe/worktree/research/21_interactive_planning/TestPolylineTracking.jl/main.jl")

# ╔═╡ 1c043428-a962-44b5-8529-42f96e4f4486
@bind px0 Slider(0.0:0.1:3.0)

# ╔═╡ 1281a3bb-c371-432f-8400-395b6ccc44ed
@bind py0 Slider(0.0:0.1:3.0)

# ╔═╡ f3abc6ae-c9ec-463c-b3f3-164b4874871b
@bind progress Slider(0.0:0.1:3.0)

# ╔═╡ c4d52872-4ebe-4bcd-8b45-5a3626878855
p = next_waypoint(lane, Point(px0,py0), [1, 0], progress)

# ╔═╡ 1bbdf50a-0d24-4af2-978e-6d23cbf36419
@vlplot(width = 900, height = 400) +
lane_viz +
@vlplot(
	data = [
		(; px = px0, py = py0, type = "current position"),
		(; px = p.coords[1], py = p.coords[2], type = "waypoint"),
	],
	:point, "px:q", "py:q", color="type:n"
)

# ╔═╡ Cell order:
# ╠═70cf1dbf-4ad9-440b-93ce-43f1f4358f04
# ╠═e1186a57-640f-476b-9589-a7a9a61e9046
# ╠═e09157b3-74ac-49bc-84f8-72627d6f58ff
# ╠═b8dc8e1a-96e3-11eb-1fca-fdd392a23642
# ╠═1c043428-a962-44b5-8529-42f96e4f4486
# ╠═1281a3bb-c371-432f-8400-395b6ccc44ed
# ╠═f3abc6ae-c9ec-463c-b3f3-164b4874871b
# ╠═c4d52872-4ebe-4bcd-8b45-5a3626878855
# ╠═1bbdf50a-0d24-4af2-978e-6d23cbf36419
