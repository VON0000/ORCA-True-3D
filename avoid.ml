type state = {
  pos : V3.t;
  vel : V3.t;
  acc : V3.t;
  psi : float;
  psi_rate : float;
}

type obstacle = {
  pos : V3.t;
  vel : V3.t;
  radius : float;
  responsibility : float;
}

type params = {
  max_speed : float;
  vo_margin : float;
  time_horizon : float;
  mass : float;
  g : float;
  thrust_max_total : float;
  theta_max : float;
  jerk_max : float;
  jxy_max : float;
  jz_max : float;
  vz_up_max : float;
  vz_down_max : float;
  az_up_max : float;
  az_down_max : float;
  yaw_rate_max : float;
  yaw_acc_max : float;
  dykstra_max_iter : int;
  dykstra_tol : float;
  residual_tol : float;
  use_exact_dynamic_projection : bool;
}

type plane = { normal : V3.t; point : V3.t }

type convex_set =
  | Ball of { center : V3.t; radius : float }
  | HalfSpace of plane
  | TiltCone of { apex : V3.t; tan_theta : float }
  | ZSlab of { z_min : float; z_max : float }

type projection_status =
  | ProjectionConverged
  | ProjectionDidNotConverge
  | ProjectionInfeasible

type projection_result = {
  x : V3.t;
  status : projection_status;
  iters : int;
  residual : float;
}

type solve_status =
  | Feasible
  | OrcaEmpty
  | ExactFeasibleSetEmpty
  | ProjectionDidNotConverge

type solve_result = {
  status : solve_status;
  v_next : V3.t;
  acc_next : V3.t;
  jerk_cmd : V3.t;
  psi_next : float;
  psi_rate_next : float;
  psi_acc_cmd : float;
  solver_iters : int;
  solver_residual : float;
  yaw_wedge_relaxed : bool;
}

let eps = 1e-9
let pi = 4.0 *. atan 1.0

let default_params =
  {
    max_speed = 0.25;
    vo_margin = 0.02;
    time_horizon = 20.0;
    mass = 1.5;
    g = 9.81;
    thrust_max_total = 30.0;
    theta_max = 0.6;
    jerk_max = 2.0;
    jxy_max = 2.0;
    jz_max = 1.5;
    vz_up_max = 0.4;
    vz_down_max = 0.4;
    az_up_max = 1.0;
    az_down_max = 1.0;
    yaw_rate_max = 1.5;
    yaw_acc_max = 2.0;
    dykstra_max_iter = 200;
    dykstra_tol = 1e-6;
    residual_tol = 1e-5;
    use_exact_dynamic_projection = true;
  }

let clamp lo hi x = max lo (min hi x)

let wrap_angle a =
  let two_pi = 2.0 *. pi in
  let rec loop x =
    if x > pi then loop (x -. two_pi)
    else if x <= -.pi then loop (x +. two_pi)
    else x
  in
  loop a

let angle_diff target source = wrap_angle (target -. source)

let limit_speed vmax v =
  let n = V3.norm v in
  if n <= vmax || n < eps then v else V3.(vmax /. n * v)

let choose_orthogonal (axis : V3.t) : V3.t =
  let ax = abs_float axis.x in
  let ay = abs_float axis.y in
  let az = abs_float axis.z in
  let basis =
    if ax <= ay && ax <= az then V3.make 1.0 0.0 0.0
    else if ay <= az then V3.make 0.0 1.0 0.0
    else V3.make 0.0 0.0 1.0
  in
  let ortho = V3.cross axis basis in
  let n = V3.norm ortho in
  if n > eps then V3.(1.0 /. n * ortho)
  else
    let fallback = V3.cross axis (V3.make 0.0 1.0 0.0) in
    let nf = V3.norm fallback in
    if nf > eps then V3.(1.0 /. nf * fallback) else V3.make 1.0 0.0 0.0

let project_ball ~center ~radius x =
  let radius = max 0.0 radius in
  let delta = V3.(x - center) in
  let n = V3.norm delta in
  if n <= radius || n < eps then x else V3.(center + (radius /. n * delta))

let project_half_space (pl : plane) (x : V3.t) =
  let delta = V3.dot pl.normal V3.(x - pl.point) in
  if delta >= 0.0 then x
  else
    let n2 = max eps (V3.norm2 pl.normal) in
    V3.(x + (-.delta /. n2 * pl.normal))

let project_z_slab ~z_min ~z_max x = V3.with_z x (clamp z_min z_max x.z)

let project_tilt_cone ~apex ~tan_theta p =
  let y = V3.(p - apex) in
  let rho = sqrt ((y.x *. y.x) +. (y.y *. y.y)) in
  let z = y.z in
  let k = max eps tan_theta in
  if rho <= k *. z then p
  else if z <= -.k *. rho then apex
  else
    let z_star = ((k *. rho) +. z) /. ((k *. k) +. 1.0) in
    let rho_star = k *. z_star in
    let scale = if rho < eps then 0.0 else rho_star /. rho in
    V3.make
      (apex.x +. (scale *. y.x))
      (apex.y +. (scale *. y.y))
      (apex.z +. z_star)

let project_convex_set set x =
  match set with
  | Ball { center; radius } -> project_ball ~center ~radius x
  | HalfSpace pl -> project_half_space pl x
  | TiltCone { apex; tan_theta } -> project_tilt_cone ~apex ~tan_theta x
  | ZSlab { z_min; z_max } -> project_z_slab ~z_min ~z_max x

let violation_of_set set x =
  match set with
  | Ball { center; radius } -> max 0.0 (V3.distance x center -. max 0.0 radius)
  | HalfSpace { normal; point } -> max 0.0 (-.V3.dot normal V3.(x - point))
  | ZSlab { z_min; z_max } -> max 0.0 (max (z_min -. x.z) (x.z -. z_max))
  | TiltCone { apex; tan_theta } ->
    let y = V3.(x - apex) in
    let rho = sqrt ((y.x *. y.x) +. (y.y *. y.y)) in
    max 0.0 (rho -. (tan_theta *. y.z))

let satisfies_set ?(tol = 1e-6) set x = violation_of_set set x <= tol

let max_residual sets x =
  List.fold_left (fun acc set -> max acc (violation_of_set set x)) 0.0 sets

let dykstra_project_sets ~params ~sets ~v_pref =
  let sets_arr = Array.of_list sets in
  let corrections = Array.make (Array.length sets_arr) V3.zero in
  let x = ref v_pref in
  let status : projection_status ref =
    ref (ProjectionDidNotConverge : projection_status)
  in
  let residual = ref (max_residual sets !x) in
  let iter = ref 0 in
  while !status = ProjectionDidNotConverge && !iter < params.dykstra_max_iter do
    incr iter;
    let before = !x in
    for i = 0 to Array.length sets_arr - 1 do
      let y = V3.(!x + corrections.(i)) in
      let z = project_convex_set sets_arr.(i) y in
      corrections.(i) <- V3.(y - z);
      x := z
    done;
    residual := max_residual sets !x;
    let movement = V3.distance before !x in
    if movement <= params.dykstra_tol && !residual <= params.residual_tol then
      status := ProjectionConverged
    else if movement <= params.dykstra_tol && !residual > params.residual_tol
    then status := ProjectionInfeasible
  done;
  { x = !x; status = !status; iters = !iter; residual = !residual }

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
  let radius = max eps (self_radius +. obs.radius +. params.vo_margin) in
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

let build_orca_sets params planes =
  Ball { center = V3.zero; radius = params.max_speed }
  :: List.map (fun pl -> HalfSpace pl) planes

let acceleration_next ~dt (state : state) u =
  V3.((2.0 /. dt * (u - state.vel)) - state.acc)

let integrate_constant_jerk ~dt (state : state) v_next =
  let jerk = V3.(2.0 /. (dt *. dt) * (v_next - state.vel - (dt * state.acc))) in
  let acc_next = V3.(state.acc + (dt * jerk)) in
  let pos_next =
    V3.(
      state.pos
      + (dt * state.vel)
      + (0.5 *. dt *. dt * state.acc)
      + (dt *. dt *. dt /. 6.0 * jerk) )
  in
  (jerk, acc_next, pos_next)

let yaw_acc_bounds params ~dt (state : state) =
  let psi_rate =
    clamp (-.params.yaw_rate_max) params.yaw_rate_max state.psi_rate
  in
  let psi_acc_min =
    max (-.params.yaw_acc_max) ((-.params.yaw_rate_max -. psi_rate) /. dt)
  in
  let psi_acc_max =
    min params.yaw_acc_max ((params.yaw_rate_max -. psi_rate) /. dt)
  in
  (psi_rate, psi_acc_min, psi_acc_max)

let yaw_wedge_sets params ~dt (state : state) =
  let psi_rate, psi_acc_min, psi_acc_max = yaw_acc_bounds params ~dt state in
  let psi_min =
    state.psi +. (psi_rate *. dt) +. (0.5 *. psi_acc_min *. dt *. dt)
  in
  let psi_max =
    state.psi +. (psi_rate *. dt) +. (0.5 *. psi_acc_max *. dt *. dt)
  in
  let width = psi_max -. psi_min in
  if width > pi then ([], true)
  else
    let e_min = V3.make (cos psi_min) (sin psi_min) 0.0 in
    let e_max = V3.make (cos psi_max) (sin psi_max) 0.0 in
    let lower = { normal = V3.make (-.e_min.y) e_min.x 0.0; point = V3.zero } in
    let upper = { normal = V3.make e_max.y (-.e_max.x) 0.0; point = V3.zero } in
    ([ HalfSpace lower; HalfSpace upper ], false)

let build_dynamic_sets params ~dt (state : state) ~yaw_wedge =
  let ez = V3.make 0.0 0.0 1.0 in
  let jerk_ball =
    Ball
      {
        center = V3.(state.vel + (dt * state.acc));
        radius = 0.5 *. params.jerk_max *. dt *. dt;
      }
  in
  let thrust_ball =
    Ball
      {
        center = V3.(state.vel + (0.5 *. dt * (state.acc - (params.g * ez))));
        radius = 0.5 *. dt *. (params.thrust_max_total /. params.mass);
      }
  in
  let tilt_cone =
    TiltCone
      {
        apex =
          V3.make
            (state.vel.x +. (0.5 *. dt *. state.acc.x))
            (state.vel.y +. (0.5 *. dt *. state.acc.y))
            (state.vel.z +. (0.5 *. dt *. (state.acc.z -. params.g)));
        tan_theta = tan params.theta_max;
      }
  in
  let vertical_speed =
    ZSlab { z_min = -.params.vz_down_max; z_max = params.vz_up_max }
  in
  let vertical_acc =
    ZSlab
      {
        z_min = state.vel.z +. (0.5 *. dt *. (state.acc.z -. params.az_down_max));
        z_max = state.vel.z +. (0.5 *. dt *. (state.acc.z +. params.az_up_max));
      }
  in
  [ jerk_ball; thrust_ball; tilt_cone; vertical_speed; vertical_acc ]
  @ yaw_wedge

let build_exact_velocity_sets params ~dt (state : state) planes =
  let yaw_wedge, yaw_wedge_relaxed = yaw_wedge_sets params ~dt state in
  let sets =
    build_orca_sets params planes
    @
    if params.use_exact_dynamic_projection then
      build_dynamic_sets params ~dt state ~yaw_wedge
    else []
  in
  (sets, yaw_wedge_relaxed)

let yaw_command params ~dt (state : state) u =
  let psi_rate, psi_acc_min, psi_acc_max = yaw_acc_bounds params ~dt state in
  if V3.norm_xy u < 1e-6 then
    let psi_rate_next =
      clamp (-.params.yaw_rate_max) params.yaw_rate_max psi_rate
    in
    (wrap_angle (state.psi +. (psi_rate_next *. dt)), psi_rate_next, 0.0)
  else
    let psi_des = atan2 u.y u.x in
    let raw_acc =
      2.0 *. angle_diff psi_des (state.psi +. (psi_rate *. dt)) /. (dt *. dt)
    in
    let psi_acc_cmd = clamp psi_acc_min psi_acc_max raw_acc in
    let psi_next =
      wrap_angle
        (state.psi +. (psi_rate *. dt) +. (0.5 *. psi_acc_cmd *. dt *. dt))
    in
    let psi_rate_next =
      clamp (-.params.yaw_rate_max) params.yaw_rate_max
        (psi_rate +. (psi_acc_cmd *. dt))
    in
    (psi_next, psi_rate_next, psi_acc_cmd)

let solve_desired_velocity ~params ~dt ~self_radius ~target ~(state : state)
  obstacles =
  let target_delta = V3.(target - state.pos) in
  let v_pref = limit_speed params.max_speed target_delta in
  let planes = active_planes params ~dt ~self_radius state obstacles in
  let orca_sets = build_orca_sets params planes in
  let orca_projection = dykstra_project_sets ~params ~sets:orca_sets ~v_pref in
  if orca_projection.residual > params.residual_tol then
    {
      status = OrcaEmpty;
      v_next = V3.zero;
      acc_next = V3.zero;
      jerk_cmd = V3.zero;
      psi_next = state.psi;
      psi_rate_next = 0.0;
      psi_acc_cmd = 0.0;
      solver_iters = orca_projection.iters;
      solver_residual = orca_projection.residual;
      yaw_wedge_relaxed = false;
    }
  else
    let exact_sets, yaw_wedge_relaxed =
      build_exact_velocity_sets params ~dt state planes
    in
    let projection = dykstra_project_sets ~params ~sets:exact_sets ~v_pref in
    let status =
      match projection.status with
      | ProjectionConverged when projection.residual <= params.residual_tol ->
        Feasible
      | ProjectionDidNotConverge -> ProjectionDidNotConverge
      | ProjectionConverged | ProjectionInfeasible -> ExactFeasibleSetEmpty
    in
    let v_next = if status = Feasible then projection.x else V3.zero in
    let jerk_cmd, acc_next, _ = integrate_constant_jerk ~dt state v_next in
    let psi_next, psi_rate_next, psi_acc_cmd =
      if status = Feasible then yaw_command params ~dt state v_next
      else (state.psi, 0.0, 0.0)
    in
    {
      status;
      v_next;
      acc_next = (if status = Feasible then acc_next else V3.zero);
      jerk_cmd = (if status = Feasible then jerk_cmd else V3.zero);
      psi_next;
      psi_rate_next;
      psi_acc_cmd;
      solver_iters = projection.iters;
      solver_residual = projection.residual;
      yaw_wedge_relaxed;
    }

let desired_velocity params ~dt ~self_radius ~target (uav : state)
  (obstacles : obstacle list) =
  let result =
    solve_desired_velocity ~params ~dt ~self_radius ~target ~state:uav obstacles
  in
  result.v_next

let step_with_result ~dt (s : state) (r : solve_result) : state =
  match r.status with
  | Feasible ->
    let _jerk, acc_next, pos_next = integrate_constant_jerk ~dt s r.v_next in
    {
      pos = pos_next;
      vel = r.v_next;
      acc = acc_next;
      psi = r.psi_next;
      psi_rate = r.psi_rate_next;
    }
  | OrcaEmpty | ExactFeasibleSetEmpty | ProjectionDidNotConverge ->
    { s with vel = V3.zero; acc = V3.zero; psi_rate = 0.0 }

let step ~dt (s : state) (v_cmd : V3.t) : state =
  let jerk, acc_next, pos_next = integrate_constant_jerk ~dt s v_cmd in
  let _ = jerk in
  { s with pos = pos_next; vel = v_cmd; acc = acc_next }
