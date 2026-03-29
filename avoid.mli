type state = { pos : V3.t; vel : V3.t }
type obstacle = { pos : V3.t; vel : V3.t }

type params = {
  rpz : float;
  rpf : float;
  kpf : float;
  h : float;
  cpf : float;
  a : float;
  b : float;
  eps_sigma : float;
  tp_samples : int;
  phi_steps : int;
  phi_window : float;
  max_speed : float;
  vo_margin : float;
}

val default_params : params
val desired_velocity : params -> target:V3.t -> state -> obstacle list -> V3.t
val step : dt:float -> state -> V3.t -> state
