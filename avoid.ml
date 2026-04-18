type state = { pos : V3.t; vel : V3.t }

type obstacle = {
  pos : V3.t;
  vel : V3.t;
  radius : float;
  responsibility : float;
}

type params = { max_speed : float; vo_margin : float; time_horizon : float }
type plane = { normal : V3.t; point : V3.t }
type projection_set = Speed_ball | Half_space of plane

let eps = 1e-9
let projection_iters = 80
let projection_tol = 1e-5
let default_params = { max_speed = 0.25; vo_margin = 0.02; time_horizon = 20.0 }
let clamp lo hi x = max lo (min hi x)

let limit_speed vmax v =
  let n = V3.norm v in
  if n <= vmax || n < eps then v else V3.(vmax /. n * v)

(* 计算一个垂直于 a b 的向量 *)
let cross (a : V3.t) (b : V3.t) : V3.t =
  V3.make
    ((a.y *. b.z) -. (a.z *. b.y))
    ((a.z *. b.x) -. (a.x *. b.z))
    ((a.x *. b.y) -. (a.y *. b.x))

let choose_orthogonal (axis : V3.t) : V3.t =
  let ax = abs_float axis.x in
  let ay = abs_float axis.y in
  let az = abs_float axis.z in
  let basis =
    if ax <= ay && ax <= az then V3.make 1.0 0.0 0.0
    else if ay <= az then V3.make 0.0 1.0 0.0
    else V3.make 0.0 0.0 1.0
  in
  let ortho = cross axis basis in
  let n = V3.norm ortho in
  if n > eps then V3.(1.0 /. n * ortho)
  else
    let fallback = cross axis (V3.make 0.0 1.0 0.0) in
    let nf = V3.norm fallback in
    if nf > eps then V3.(1.0 /. nf * fallback) else V3.make 1.0 0.0 0.0

let project_ball vmax v = limit_speed vmax v

let project_half_space (pl : plane) (v : V3.t) =
  let delta = V3.dot pl.normal V3.(v - pl.point) in
  if delta >= 0.0 then v else V3.(v + (-.delta * pl.normal))

let satisfies_half_space (pl : plane) (v : V3.t) =
  V3.dot pl.normal V3.(v - pl.point) >= -1e-6

let project_set vmax set v =
  match set with
  | Speed_ball -> project_ball vmax v
  | Half_space pl -> project_half_space pl v

let feasible vmax planes v =
  V3.norm v <= vmax +. 1e-6
  && List.for_all (fun pl -> satisfies_half_space pl v) planes

let dykstra_project vmax planes v_pref =
  let sets =
    Array.of_list (Speed_ball :: List.map (fun pl -> Half_space pl) planes)
  in
  let corrections = Array.make (Array.length sets) V3.zero in
  let x = ref v_pref in
  let converged = ref false in
  let iter = ref 0 in
  while (not !converged) && !iter < projection_iters do
    incr iter;
    let before = !x in
    for i = 0 to Array.length sets - 1 do
      let y = V3.(!x + corrections.(i)) in
      let z = project_set vmax sets.(i) y in
      corrections.(i) <- V3.(y - z);
      x := z
    done;
    converged :=
      V3.distance before !x <= projection_tol && feasible vmax planes !x
  done;
  project_ball vmax !x

let obstacle_radius_sum self_radius (obs : obstacle) =
  max eps (self_radius +. obs.radius)

let coincident_with_obstacle (uav : state) (obs : obstacle) =
  V3.distance obs.pos uav.pos < eps

let intrudes_safety_zone ~self_radius (uav : state) (obs : obstacle) =
  let radius = obstacle_radius_sum self_radius obs in
  V3.distance obs.pos uav.pos <= radius +. eps

let cone_candidate axis basis s radial cos_theta sin_theta =
  let t = (s *. cos_theta) +. (radial *. sin_theta) in
  let axial = t *. cos_theta in
  let lateral = t *. sin_theta in
  let point = V3.((axial * axis) + (lateral * basis)) in
  (t, point)

let pick_closest_point v candidates =
  match candidates with
  | [] -> None
  | q :: rest ->
    Some
      (List.fold_left
         (fun best q_cur ->
           if V3.distance v q_cur < V3.distance v best then q_cur else best )
         q rest )

let clamp_responsibility x = clamp 0.0 1.0 x

let orca_plane_for_obstacle params ~dt ~self_radius (uav : state)
  (obs : obstacle) =
  let radius = obstacle_radius_sum self_radius obs in
  let p = V3.(obs.pos - uav.pos) in
  let v_rel = V3.(uav.vel - obs.vel) in
  let dist = V3.norm p in
  let responsibility = clamp_responsibility obs.responsibility in
  if coincident_with_obstacle uav obs then
    let normal =
      let n = V3.norm v_rel in
      if n > eps then V3.(1.0 /. n * v_rel) else V3.make 1.0 0.0 0.0
    in
    let inv_dt = 1.0 /. max dt 1e-3 in
    let u = V3.(radius *. inv_dt * normal) in
    Some { normal; point = V3.(uav.vel + (responsibility * u)) }
  else if intrudes_safety_zone ~self_radius uav obs then
    let inv_dt = 1.0 /. max dt 1e-3 in
    let w = V3.(v_rel - (inv_dt * p)) in
    let unit_w =
      let nw = V3.norm w in
      if nw > eps then V3.(1.0 /. nw * w) else V3.(-1.0 /. dist * p)
    in
    let magnitude = (radius *. inv_dt) -. V3.norm w in
    let u = V3.(magnitude * unit_w) in
    let nu = V3.norm u in
    if nu < eps then None
    else
      let normal = V3.(1.0 /. nu * u) in
      Some { normal; point = V3.(uav.vel + (responsibility * u)) }
  else
    let horizon = max params.time_horizon dt in
    let center = V3.(1.0 /. horizon * p) in
    let cap_radius = radius /. horizon in
    let axis = V3.(1.0 /. dist * p) in
    let radial_base = sqrt (max 0.0 ((dist *. dist) -. (radius *. radius))) in
    let cos_theta = radial_base /. dist in
    let sin_theta = radius /. dist in
    let tan_theta = radius /. radial_base in
    let s_cap = radial_base *. radial_base /. (dist *. horizon) in
    let s = V3.dot v_rel axis in
    let v_perp = V3.(v_rel - (s * axis)) in
    let radial = V3.norm v_perp in
    let basis =
      if radial > eps then V3.(1.0 /. radial * v_perp)
      else choose_orthogonal axis
    in
    let inside_cap = V3.distance v_rel center <= cap_radius +. 1e-6 in
    let inside_cone =
      s >= s_cap -. 1e-6 && radial <= (s *. tan_theta) +. 1e-6
    in
    if not (inside_cap || inside_cone) then None
    else
      let sphere_point =
        let offset = V3.(v_rel - center) in
        let dir =
          let n = V3.norm offset in
          if n > eps then V3.(1.0 /. n * offset) else V3.(-1.0 * axis)
        in
        let q = V3.(center + (cap_radius * dir)) in
        if V3.dot q axis <= s_cap +. 1e-6 then Some q else None
      in
      let cone_point =
        let t, q = cone_candidate axis basis s radial cos_theta sin_theta in
        if t >= (radial_base /. horizon) -. 1e-6 then Some q else None
      in
      let candidates =
        List.filter_map (fun x -> x) [ sphere_point; cone_point ]
      in
      match pick_closest_point v_rel candidates with
      | None -> None
      | Some boundary ->
        let u = V3.(boundary - v_rel) in
        let nu = V3.norm u in
        if nu < eps then None
        else
          let normal = V3.(1.0 /. nu * u) in
          Some { normal; point = V3.(uav.vel + (responsibility * u)) }

let active_planes params ~dt ~self_radius uav obstacles =
  List.filter_map
    (orca_plane_for_obstacle params ~dt ~self_radius uav)
    obstacles

let has_collision_risk params ~dt ~self_radius uav obstacles =
  active_planes params ~dt ~self_radius uav obstacles <> []

let desired_velocity params ~dt ~self_radius ~target (uav : state)
  (obstacles : obstacle list) =
  let target_delta = V3.(target - uav.pos) in
  let v_pref = limit_speed params.max_speed target_delta in
  let planes = active_planes params ~dt ~self_radius uav obstacles in
  if planes = [] then v_pref else dykstra_project params.max_speed planes v_pref

let step ~dt (s : state) (v_cmd : V3.t) : state =
  { pos = V3.(s.pos + (dt * v_cmd)); vel = v_cmd }
