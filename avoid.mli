type state = { pos : V3.t; vel : V3.t }

type obstacle = {
  pos : V3.t;
  vel : V3.t;
  radius : float;
  responsibility : float;
}

type params = { max_speed : float; vo_margin : float; time_horizon : float }

val default_params : params
val coincident_with_obstacle : state -> obstacle -> bool
val intrudes_safety_zone : self_radius:float -> state -> obstacle -> bool

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

val step : dt:float -> state -> V3.t -> state
