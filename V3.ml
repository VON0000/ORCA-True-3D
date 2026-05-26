type t = { x : float; y : float; z : float }

let make x y z = { x; y; z }
let zero = { x = 0.; y = 0.; z = 0. }
let ( + ) a b = { x = a.x +. b.x; y = a.y +. b.y; z = a.z +. b.z }
let ( - ) a b = { x = a.x -. b.x; y = a.y -. b.y; z = a.z -. b.z }
let ( * ) k a = { x = k *. a.x; y = k *. a.y; z = k *. a.z }
let scale k a = k * a
let div a k = 1.0 /. k * a
let dot a b = (a.x *. b.x) +. (a.y *. b.y) +. (a.z *. b.z)

let cross a b =
  make
    ((a.y *. b.z) -. (a.z *. b.y))
    ((a.z *. b.x) -. (a.x *. b.z))
    ((a.x *. b.y) -. (a.y *. b.x))

let norm2 a = dot a a
let norm a = sqrt (norm2 a)
let norm_xy a = sqrt ((a.x *. a.x) +. (a.y *. a.y))
let xy a = { x = a.x; y = a.y; z = 0.0 }
let with_z a z = { a with z }

let normalize ?(eps = 1e-12) a =
  let n = norm a in
  if n < eps then zero else 1. /. n * a

let clamp_norm max_norm a =
  let n = norm a in
  if max_norm < 0.0 then zero
  else if n <= max_norm || n < 1e-12 then a
  else max_norm /. n * a

let distance a b = norm (a - b)

let is_finite a =
  match (classify_float a.x, classify_float a.y, classify_float a.z) with
  | (FP_nan | FP_infinite), _, _
  | _, (FP_nan | FP_infinite), _
  | _, _, (FP_nan | FP_infinite) ->
    false
  | _ -> true
