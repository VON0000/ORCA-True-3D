let in_conflict_zone (pa : V3.t) (pb : V3.t) (safe_range : float) : bool =
  if V3.norm2 V3.(pa - pb) < safe_range *. safe_range then true else false

let rvo (a : V3.t) (b : V3.t) (safe_range : float) : float =
  let distance2_ab = V3.distance2 a b in
  let distance_ab = V3.distance a b in
  safe_range *. sqrt (distance2_ab -. (safe_range *. safe_range)) /. distance_ab

let dvo (a : V3.t) (b : V3.t) (safe_range : float) : float =
  let distance2_ab = V3.distance2 a b in
  let distance_ab = V3.distance a b in
  (distance2_ab -. (safe_range *. safe_range)) /. distance_ab

let theta_vo (rvo : float) (dvo : float)  : float =
  atan (rvo /. dvo)