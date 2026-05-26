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

val default_params : params
val clamp : float -> float -> float -> float
val coincident_with_obstacle : state -> obstacle -> bool
val intrudes_safety_zone : self_radius:float -> state -> obstacle -> bool
val project_convex_set : convex_set -> V3.t -> V3.t
val violation_of_set : convex_set -> V3.t -> float
val satisfies_set : ?tol:float -> convex_set -> V3.t -> bool

val dykstra_project_sets :
  params:params -> sets:convex_set list -> v_pref:V3.t -> projection_result

val build_orca_sets : params -> plane list -> convex_set list

val build_dynamic_sets :
  params -> dt:float -> state -> yaw_wedge:convex_set list -> convex_set list

val build_exact_velocity_sets :
  params -> dt:float -> state -> plane list -> convex_set list * bool

val has_collision_risk :
  params -> dt:float -> self_radius:float -> state -> obstacle list -> bool

val desired_velocity :
  params ->
  dt:float ->
  self_radius:float ->
  target:V3.t ->
  state ->
  obstacle list ->
  V3.t

val solve_desired_velocity :
  params:params ->
  dt:float ->
  self_radius:float ->
  target:V3.t ->
  state:state ->
  obstacle list ->
  solve_result

val acceleration_next : dt:float -> state -> V3.t -> V3.t
val step_with_result : dt:float -> state -> solve_result -> state
val step : dt:float -> state -> V3.t -> state
