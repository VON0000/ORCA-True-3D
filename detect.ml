let detect_collision (pa : V3.t) (pb : V3.t) (safe_range : float) : bool =
  if V3.norm2 V3.(pa - pb) < safe_range *. safe_range then true else false
