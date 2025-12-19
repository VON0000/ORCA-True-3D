type t = { x : float; y : float; z : float }

let make x y z = { x; y; z }
let zero = { x = 0.; y = 0.; z = 0. }
let ( + ) a b = { x = a.x +. b.x; y = a.y +. b.y; z = a.z +. b.z }
let ( - ) a b = { x = a.x -. b.x; y = a.y -. b.y; z = a.z -. b.z }
let ( * ) k a = { x = k *. a.x; y = k *. a.y; z = k *. a.z }
let dot a b = (a.x *. b.x) +. (a.y *. b.y) +. (a.z *. b.z)

let cross a b =
  {
    x = (a.y *. b.z) -. (a.z *. b.y);
    y = (a.z *. b.x) -. (a.x *. b.z);
    z = (a.x *. b.y) -. (a.y *. b.x);
  }

let norm2 a = dot a a
let norm a = sqrt (norm2 a)

let normalize ?(eps = 1e-12) a =
  let n = norm a in
  if n < eps then zero else 1. /. n * a
